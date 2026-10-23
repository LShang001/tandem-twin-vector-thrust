/**
 * @file    fw_sil_env_main.cpp
 * @module  固件/宿主 SIL（fw-tests：真实固件 + 6DOF + 传感器/环境注入）
 * @brief   在真实固件控制链外挂 6DOF 动力学、传感器模型与风扰/故障注入
 * @details 被测：真实固件 `src/flight_control.cpp`（控制链）与
 *          `lib/navigation-main/src/ekf_15_state.h`（`--ekf` 时）。
 *          输入：命令行开关——`--manual` 手动阶跃、`--ekf` 用真实 EKF 估计、
 *          `--fault 3|4|5` 单发推力损失/上摆卡死/双发推力损失、`--fault-at T` 注入时刻（默认 8 s）、
 *          `--loss F` 推力保留比例（默认 0.70）、`--alt T` 定高目标（默认 10 m）、
 *          `--seconds T` 仿真时长（默认 40 s）、
 *          `--mask N` 覆盖 `inertia_comp_mask`（0=关前馈 / 3=全开，用于 C-02 A/B）、
 *          `--coupled` 同时 roll/pitch 激励 + 油门脉冲、`--flow` 光流测速、
 *          `--flow-delay N` 光流延迟、`--mag` 磁航向、`--seed`、`--no-wind`；
 *          处理：本文件提供 6DOF 刚体、电机/舵机动态、IMU/GNSS/气压计注入、
 *          20–25 s 侧风 3 m/s、地面约束与 0.1% PWM 量化；
 *          输出：每 1 s 打印高度/估计高度/位置/姿态/舵机/电机/GNSS 有效位
 *          与 F3/F4 标志；结束打印 max/rms 姿态、故障标志、final_alt。
 *          运行：BIN_DIR=<临时目录> bash test_host/fw_sil/build_and_run_env.sh <开关>
 * @theory  6DOF 刚体：`I·ω̇ = M − ω×Iω`、`v̇ = F/m + g`；
 *          推进力矩/推力来自 `computeWrench()`（kT/kQ 口径）。
 *          @ref THY-002 §2–3、@ref CTL-001 §4（控制链）
 * @frame   NED（位置/速度/风）+ FRD（角速率/比力）；`--ekf` 时 EKF 内部保持 WGS84 LLA
 * @units   高度 m，姿态 deg，舵机 PWM 百分比，电机转速 rad/s，风 m/s，时间 s
 * @note    与实机差异：传感器为理想注入 + 白噪声，无振动谱、无磁干扰、无 GNSS 多径；
 *          `--mag` 默认关闭（多 seed 下 MeasurementUpdateYaw 会 NaN/发散，见 AGENTS.md）；
 *          `--flow` 是理想光流，不能当实机验证。AGENTS.md 记录的水平漂移 115–457 m
 *          正是本文件无外部参考时的失锁表现。
 * @test    S1 定高（直注/EKF）闭环；S2 --manual 阶跃；S3 --fault 3/4 故障降级；
 *          S4 --flow 光流辅助；S5 --seed 多 seed 稳定性（run_matrix.sh）；
 *          S6 --coupled + --mask 0/3 前馈 A/B（C-02）；S7 --fault 3 的 F3 标志（C-01）
 * @see     TandemVec_FCS/test_host/fw_sil/README.md、run_matrix.sh、run_acceptance.sh
 */
// ============================================================
// 真实源码：src/flight_control.cpp + src/state_data.cpp；
// 本文件只提供 plant、传感器注入与环境，不复制固件控制律。
// ============================================================
#include "state_data.h"
#include "flight_control.h"
#include "TandemVec_Propulsion.h"
#include "TandemVec_Config.h"
#include "Quat4f.h"
#include "QuaternionMath.h"
#include "ekf_15_state.h"
#include "transforms.h"
#include <Eigen/Dense>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <array>

static const TandemVecParams &P = kDefaultTandemVecParams;
static const ServoConfig &SC = kDefaultServoConfig;
static bfs::Ekf15State g_ekf;
static bool g_use_ekf = false;
static int g_flow_delay = 1;  // 10 Hz 样本数
static bool g_use_mag_yaw = false;
static float g_flow_scale = 0.95f; // 在线标定后的光流尺度
static const Eigen::Vector3d kHomeLla(28.2 * M_PI / 180.0,
                                      112.9 * M_PI / 180.0, 50.0);

struct Plant {
  Quat4f q;
  float omega[3];      // body rad/s
  float vel[3];        // NED m/s
  float pos[3];        // NED m
  float wind[3];       // NED m/s (currently force-equivalent)
  float acc_ned[3];    // last NED inertial acceleration
  float force_ned[3];  // last NED thrust/force (no gravity)
  float motor_wf, motor_wt;
  float servo_f, servo_t; // rad actual
  bool on_ground;

  Plant() : q(1,0,0,0), omega{0,0,0}, vel{0,0,0}, pos{0,0,0},
            wind{0,0,0}, acc_ned{0,0,0}, force_ned{0,0,0}, motor_wf(0), motor_wt(0),
            servo_f(0), servo_t(0), on_ground(true) {}

  void body_to_ned(const float v[3], float out[3]) const {
    const float w=q.w, x=q.x, y=q.y, z=q.z;
    out[0] = (1-2*(y*y+z*z))*v[0] + 2*(x*y-w*z)*v[1] + 2*(x*z+w*y)*v[2];
    out[1] = 2*(x*y+w*z)*v[0] + (1-2*(x*x+z*z))*v[1] + 2*(y*z-w*x)*v[2];
    out[2] = 2*(x*z-w*y)*v[0] + 2*(y*z+w*x)*v[1] + (1-2*(x*x+y*y))*v[2];
  }
  void ned_to_body(const float v[3], float out[3]) const {
    const float w=q.w, x=q.x, y=q.y, z=q.z;
    out[0] = (1-2*(y*y+z*z))*v[0] + 2*(x*y+w*z)*v[1] + 2*(x*z-w*y)*v[2];
    out[1] = 2*(x*y-w*z)*v[0] + (1-2*(x*x+z*z))*v[1] + 2*(y*z+w*x)*v[2];
    out[2] = 2*(x*z+w*y)*v[0] + 2*(y*z-w*x)*v[1] + (1-2*(x*x+y*y))*v[2];
  }
  void euler(float &roll, float &pitch, float &yaw) const {
    Quaternion qq{q.w,q.x,q.y,q.z};
    quaternionToEuler(qq, roll, pitch, yaw);
  }
  void step(const SixDOFWrench &w, float dt) {
    // model -> body: F=(Fz', Fy', -Fx'), M=(-Mz', My', -Mx')
    const float Fb[3] = {w.Fz, w.Fy, -w.Fx};
    const float M[3] = {-w.Mz, w.My, -w.Mx};

    const float p=omega[0], qq=omega[1], r=omega[2];
    omega[0] += (M[0] - (P.Iz-P.Iy)*qq*r) / P.Ix * dt;
    omega[1] += (M[1] - (P.Ix-P.Iz)*r*p) / P.Iy * dt;
    omega[2] += (M[2] - (P.Iy-P.Ix)*p*qq) / P.Iz * dt;

    const float wx=omega[0], wy=omega[1], wz=omega[2];
    Quat4f dq{-0.5f*(q.x*wx+q.y*wy+q.z*wz),
              0.5f*(q.w*wx+q.y*wz-q.z*wy),
              0.5f*(q.w*wy-q.x*wz+q.z*wx),
              0.5f*(q.w*wz+q.x*wy-q.y*wx)};
    q.w+=dq.w*dt; q.x+=dq.x*dt; q.y+=dq.y*dt; q.z+=dq.z*dt;
    q=qNorm(q);

    float Fn[3];
    body_to_ned(Fb, Fn);
    const float ax=(Fn[0]+wind[0])/P.m;
    const float ay=(Fn[1]+wind[1])/P.m;
    const float az=(Fn[2]+wind[2])/P.m + P.g;
    acc_ned[0]=ax; acc_ned[1]=ay; acc_ned[2]=az;
    force_ned[0]=Fn[0]+wind[0]; force_ned[1]=Fn[1]+wind[1]; force_ned[2]=Fn[2]+wind[2];
    vel[0]+=ax*dt; vel[1]+=ay*dt; vel[2]+=az*dt;
    pos[0]+=vel[0]*dt; pos[1]+=vel[1]*dt; pos[2]+=vel[2]*dt;

    if (pos[2] >= 0.0f) { // NED down: ground at 0
      pos[2]=0.0f;
      if (vel[2] > 0.0f) vel[2]=0.0f;
      on_ground = true;
    } else {
      on_ground = false;
    }
  }
};

struct Sensors {
  float gnss_n, gnss_e, gnss_d;
  float gnss_vn, gnss_ve, gnss_vd;
  float baro_alt;
  float gyro[3];
  float accel[3];
  float mag_yaw;
  bool gnss_valid;
  float flow_vn, flow_ve;
  bool flow_valid;
  int flow_quality;
  std::array<float, 32> flow_hist_vn{};
  std::array<float, 32> flow_hist_ve{};
  Sensors() : gnss_n(0),gnss_e(0),gnss_d(0),gnss_vn(0),gnss_ve(0),gnss_vd(0),
              baro_alt(0), gyro{0,0,0}, accel{0,0,0}, mag_yaw(0),
              gnss_valid(true), flow_vn(0), flow_ve(0), flow_valid(false),
              flow_quality(100) {}
};

static void set_rc(uint16_t throttle, uint16_t mode, bool unlock,
                   uint16_t roll = 1500, uint16_t pitch = 1500) {
  for (int i=0;i<9;++i) raw_rc_values[i]=1500;
  raw_rc_values[0]=roll; raw_rc_values[1]=pitch;
  raw_rc_values[2]=throttle; raw_rc_values[3]=1500;
  raw_rc_values[4]=unlock?1800:1000;
  raw_rc_values[5]=1800;
  raw_rc_values[6]=mode; raw_rc_values[7]=1000; raw_rc_values[8]=1000;
}

static void update_sensors(Plant &p, Sensors &s, std::mt19937 &rng, float t) {
  std::normal_distribution<float> gn(0.0f, 1.0f);
  // IMU: gyro + bias + noise; accel specific force in body
  const float gyro_bias[3] = {0.003f, -0.002f, 0.004f};
  for (int i=0;i<3;++i) s.gyro[i] = p.omega[i] + gyro_bias[i] + 0.0015f*gn(rng);
  const float accel_bias[3] = {0.05f, -0.03f, 0.08f};
  // 比力：地面静止时地面反力支撑，f_ned=(0,0,-g)；
  // 飞行时 f_ned=推力/质量（不含重力）。
  float f_ned[3];
  if (p.on_ground) {
    f_ned[0]=0; f_ned[1]=0; f_ned[2]=-P.g;
  } else {
    f_ned[0]=p.force_ned[0]/P.m;
    f_ned[1]=p.force_ned[1]/P.m;
    f_ned[2]=p.force_ned[2]/P.m;
  }
  float a_body[3];
  p.ned_to_body(f_ned, a_body);
  for (int i=0;i<3;++i) s.accel[i] = a_body[i] + accel_bias[i] + 0.05f*gn(rng);

  // GNSS 10 Hz, noise + 20-25s outage
  const bool outage = (t > 20.0f && t < 25.0f);
  s.gnss_valid = !outage;
  static int last_gnss_tick = -1;
  const int gnss_tick = int(t * 10.0f + 0.5f);
  if (!outage && gnss_tick != last_gnss_tick) {
    last_gnss_tick = gnss_tick;
    s.gnss_n = p.pos[0] + 0.8f*gn(rng);
    s.gnss_e = p.pos[1] + 0.8f*gn(rng);
    s.gnss_d = p.pos[2] + 1.2f*gn(rng);
    s.gnss_vn = p.vel[0] + 0.1f*gn(rng);
    s.gnss_ve = p.vel[1] + 0.1f*gn(rng);
    s.gnss_vd = p.vel[2] + 0.15f*gn(rng);
  }
  // Baro 25 Hz, noise + slow drift
  static int last_baro_tick = -1;
  const int baro_tick = int(t * 25.0f + 0.5f);
  if (baro_tick != last_baro_tick) {
    last_baro_tick = baro_tick;
    s.baro_alt = -p.pos[2] + 0.25f*gn(rng) + 0.02f*t;
  }
  // Mag yaw (truth + noise)
  float r,pitch,yaw; p.euler(r,pitch,yaw);
  s.mag_yaw = yaw + 0.005f*gn(rng);
  // 光流质量门控：大姿态角时地面纹理/尺度失效，质量降 0
  s.flow_quality = (fabsf(r) < 0.17f && fabsf(pitch) < 0.17f) ? 100 : 0;

  // 光流水平速度（真值+噪声，10 Hz；连续可用；无高度/垂直）
  static int last_flow_tick = -1;
  const int ft = int(t * 10.0f + 0.5f);
  if (ft != last_flow_tick) {
    last_flow_tick = ft;
    // 机体水平速度（真实光流是机体系，含 0.95 尺度误差），
    // 在采样时刻用 EKF 姿态转 NED 后存入延迟队列。这样延迟补偿的是
    // “测量时刻的 NED 速度”，避免用当前姿态旋转旧机体测量。
    float vb[3];
    p.ned_to_body(p.vel, vb);
    // GNSS 有效时在线估计光流尺度（真实固件也有双矢量测速/尺度标定）
    if (s.gnss_valid && !outage) {
      const float sf = std::hypot(vb[0], vb[1]);
      const float sg = std::hypot(s.gnss_vn, s.gnss_ve);
      if (sf > 1.0f && sg > 1.0f) {
        const float ratio = std::max(0.7f, std::min(1.3f, sg / sf));
        g_flow_scale = 0.98f * g_flow_scale + 0.02f * ratio;
      }
    }
    const float fvx = g_flow_scale * vb[0] + 0.12f * gn(rng);
    const float fvy = g_flow_scale * vb[1] + 0.12f * gn(rng);
    const Eigen::Vector3f vn = g_ekf.quat().toRotationMatrix() *
                               Eigen::Vector3f(fvx, fvy, 0.0f);
    const int hi = ft % 32;
    s.flow_hist_vn[hi] = vn(0);
    s.flow_hist_ve[hi] = vn(1);
    // 10 Hz 链路 0.1 s 延迟（1 个样本）
    const int di = (hi - g_flow_delay + 32) % 32;
    s.flow_vn = s.flow_hist_vn[di];
    s.flow_ve = s.flow_hist_ve[di];
    s.flow_valid = (ft > 0);
  }
}

static void feed_firmware(const Plant &p, const Sensors &s) {
  if (!g_use_ekf) {
    // 直注模式：AHRS/导航量由传感器直接注入（没有 EKF）
    Quaternion qq{p.q.w,p.q.x,p.q.y,p.q.z};
    float r,pitch,yaw; quaternionToEuler(qq,r,pitch,yaw);
    AHRS_Packet.Roll=r; AHRS_Packet.Pitch=pitch; AHRS_Packet.Heading=s.mag_yaw;
    AHRS_Packet.Qw=p.q.w; AHRS_Packet.Qx=p.q.x;
    AHRS_Packet.Qy=p.q.y; AHRS_Packet.Qz=p.q.z;
    icm_gyro_x=s.gyro[0]; icm_gyro_y=s.gyro[1]; icm_gyro_z=s.gyro[2];
    current_omega_dps_body_filtered=Vector3(s.gyro[0]*57.29578f,
                                             s.gyro[1]*57.29578f,
                                             s.gyro[2]*57.29578f);
    relative_north=s.gnss_n; relative_east=s.gnss_e;
    relative_down=-s.baro_alt;
    estimated_height=s.baro_alt;
    fused_north_pos=s.gnss_n; fused_east_pos=s.gnss_e;
    fused_north_vel=s.gnss_vn; fused_east_vel=s.gnss_ve;
    INS_GNSS_Packet.location_north=s.gnss_n;
    INS_GNSS_Packet.location_east=s.gnss_e;
    INS_GNSS_Packet.location_down=s.gnss_d;
    INS_GNSS_Packet.velocity_north=s.gnss_vn;
    INS_GNSS_Packet.velocity_east=s.gnss_ve;
    INS_GNSS_Packet.velocity_down=s.gnss_vd;
    return;
  }
  // 真实 EKF 输出（姿态、位置、速度）
  const Eigen::Vector3f v = g_ekf.ned_vel_mps();
  const Eigen::Vector3d lla(g_ekf.lat_rad(), g_ekf.lon_rad(), g_ekf.alt_m());
  const Eigen::Vector3d rel = bfs::lla2ned(lla, kHomeLla);
  const Eigen::Quaternionf q = g_ekf.quat();
  const float roll = g_ekf.roll_rad(), pitch = g_ekf.pitch_rad(), yaw = g_ekf.yaw_rad();

  AHRS_Packet.Roll = roll;
  AHRS_Packet.Pitch = pitch;
  AHRS_Packet.Heading = yaw;
  AHRS_Packet.Qw = q.w(); AHRS_Packet.Qx = q.x();
  AHRS_Packet.Qy = q.y(); AHRS_Packet.Qz = q.z();

  // 速率环用原始陀螺（与固件一致），姿态用 EKF
  icm_gyro_x = s.gyro[0];
  icm_gyro_y = s.gyro[1];
  icm_gyro_z = s.gyro[2];
  current_omega_dps_body_filtered = Vector3(s.gyro[0] * 57.29578f,
                                            s.gyro[1] * 57.29578f,
                                            s.gyro[2] * 57.29578f);

  // EKF 姿态 + 传感器高度/位置（诊断用混合：先验证高度链，不是最终 EKF 全闭环）
  relative_north = s.gnss_n;
  relative_east = s.gnss_e;
  relative_down = -s.baro_alt;
  estimated_height = s.baro_alt;
  fused_north_pos = rel(0);
  fused_east_pos = rel(1);
  fused_north_vel = v(0);
  fused_east_vel = v(1);

  INS_GNSS_Packet.location_north = rel(0);
  INS_GNSS_Packet.location_east = rel(1);
  INS_GNSS_Packet.location_down = rel(2);
  INS_GNSS_Packet.velocity_north = v(0);
  INS_GNSS_Packet.velocity_east = v(1);
  INS_GNSS_Packet.velocity_down = v(2);
}

int main(int argc, char** argv) {
  int seed=1;
  bool manual=false;
  bool coupled=false;   // 同时 roll/pitch 激励 + 油门脉冲，用于 C-02 前馈 A/B
  int fault=0; // 0=none, 3=thrust loss(单发), 4=servo jam, 5=both motors loss
  float fault_at=8.0f;  // 故障注入时刻 [s]（默认 8；定高需先爬升时可设 20）
  float loss=0.70f;     // fault 3/5 的推力保留比例（0.9=损失10%）
  float target_alt=10.0f; // AUTO_ALTITUDE 目标高度 [m]（F3 验证可调高）
  float seconds=40.0f;    // 仿真时长 [s]
  int mask=0x03;        // inertia_comp_mask，0 关闭前馈 / 3 全开（C-02 A/B）
  bool flow=false;
  bool wind_on=true;    // --no-wind 关闭 20–25 s 侧风（F3 洁净场景用）
  for (int i=1;i<argc;++i) {
    if (std::strcmp(argv[i],"--manual")==0) manual=true;
    else if (std::strcmp(argv[i],"--coupled")==0) coupled=true;
    else if (std::strcmp(argv[i],"--ekf")==0) g_use_ekf=true;
    else if (std::strcmp(argv[i],"--no-wind")==0) wind_on=false;
    else if (std::strcmp(argv[i],"--fault")==0 && i+1<argc) fault=std::atoi(argv[++i]);
    else if (std::strcmp(argv[i],"--fault-at")==0 && i+1<argc) fault_at=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--loss")==0 && i+1<argc) loss=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--alt")==0 && i+1<argc) target_alt=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--seconds")==0 && i+1<argc) seconds=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--mask")==0 && i+1<argc) mask=std::atoi(argv[++i]);
    else if (std::strcmp(argv[i],"--flow")==0) flow=true;
    else if (std::strcmp(argv[i],"--flow-delay")==0 && i+1<argc) g_flow_delay=std::atoi(argv[++i]);
    else if (std::strcmp(argv[i],"--mag")==0) g_use_mag_yaw=true;
    else if (std::strcmp(argv[i],"--seed")==0 && i+1<argc) seed=std::atoi(argv[++i]);
  }
  // 运行时覆盖前馈掩码（仅宿主机 SIL；生产固件由 0xE1 参数写入）
  kFlightCtrlParams.inertia_comp_mask = static_cast<uint8_t>(mask & 0x03);
  if (coupled) manual = true;  // 耦合激励走手动杆，便于直接看姿态响应
  Plant p; Sensors s; std::mt19937 rng(seed);
  g_ekf.Initialize(Eigen::Vector3f(0.0f, 0.0f, -P.g),
                   Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(),
                   Eigen::Vector3f::Zero(), kHomeLla);
  isLinkUp=true; failsafe_in_flight=false;
  nav_system_initialized=true; is_origin_lla_set=true;
  origin_lat_deg=28.2; origin_lon_deg=112.9; origin_alt_m=50.0;

  const float dt=0.005f;
  const int steps=int(seconds/dt);
  bool alt_cmd=false;
  float max_roll=0,max_pitch=0;
  double sum_roll2=0.0, sum_pitch2=0.0; int n_att=0;
  int last_gnss_update=-1, last_baro_update=-1, last_flow_update=-1;

  for (int k=0;k<steps;++k) {
    const float t=k*dt;
    if (coupled) {
      // 温和耦合激励：10–12 s 右滚 + 抬头，12–14 s 反向；小幅（±30 µs），
      // 让姿态保持在线性区，比较 mask=0/3 的跨轴响应。
      const uint16_t roll = (t>10 && t<12) ? 1530 : ((t>12 && t<14) ? 1470 : 1500);
      const uint16_t pitch = (t>10 && t<12) ? 1470 : ((t>12 && t<14) ? 1530 : 1500);
      set_rc(1500,1000,t>=0.5f,roll,pitch);
    } else if (manual) {
      const uint16_t roll=(t>10 && t<15)?1600:1500;
      const uint16_t pitch=(t>20 && t<25)?1600:1500;
      set_rc((t<1)?1000:1500,1000,t>=0.5f,roll,pitch);
    } else {
      const uint16_t thr = (t < 1.0f) ? 1000 : ((t >= 2.0f && t < 5.0f) ? 1600 : 1500);
      set_rc(thr,(t<2)?1300:1500,t>=0.5f);
    }
    update_sensors(p,s,rng,t);

    // 真实 EKF：IMU 传播 + GNSS/气压量测（--ekf 实验模式）
    if (g_use_ekf) g_ekf.TimeUpdate(Eigen::Vector3f(s.accel[0], s.accel[1], s.accel[2]),
                     Eigen::Vector3f(s.gyro[0], s.gyro[1], s.gyro[2]), dt);
    const int gt = int(t * 10.0f + 0.5f);
    if (g_use_ekf && s.gnss_valid && gt != last_gnss_update) {
      last_gnss_update = gt;
      const Eigen::Vector3d gnss_lla = bfs::ned2lla(
          Eigen::Vector3d(s.gnss_n, s.gnss_e, s.gnss_d), kHomeLla);
      g_ekf.MeasurementUpdate(
          Eigen::Vector3f(s.gnss_vn, s.gnss_ve, s.gnss_vd), gnss_lla, dt);
    }
    const int ft = int(t * 10.0f + 0.5f);
    if (g_use_ekf && g_use_mag_yaw && ft != last_flow_update) {
      // 磁航向（真实 EKF yaw 观测；默认关闭，见 README：多 seed 会失稳）
      g_ekf.MeasurementUpdateYaw(s.mag_yaw, 0.02f);
    }
    if (g_use_ekf && flow && s.flow_valid && s.flow_quality >= 50 &&
        ft != last_flow_update) {
      last_flow_update = ft;
      // 光流速度：只更新 N/E，D 填当前估计并给大噪声
      const Eigen::Vector3f vcur = g_ekf.ned_vel_mps();
      // 机体系光流速度 -> NED（用当前 EKF 姿态）；D 填当前估计并给大噪声
      g_ekf.MeasurementUpdateAidingVelocityDetailed(
          Eigen::Vector3f(s.flow_vn, s.flow_ve, vcur(2)), 0.20f, 1.0e3f);
    }
    const int bt = int(t * 25.0f + 0.5f);
    if (g_use_ekf && bt != last_baro_update) {
      last_baro_update = bt;
      g_ekf.MeasurementUpdateBaroAltitude(
          static_cast<float>(kHomeLla(2) + s.baro_alt), 0.3f);
    }
    feed_firmware(p,s);
    runGNCExecutive();
    if (!manual && !alt_cmd && t >= 3.0f) {
      target_altitude = target_alt;
      vertical_loiter_state = ALTITUDE_HOVERING;
      alt_cmd = true;
    }

    const float upper=(ch1_output-(50+SC.zero_roll_pct))*SC.half_travel_deg/
      (50*(SC.teeth_gimbal/SC.teeth_servo)*SC.dir_roll)*0.017453292f;
    const float lower=(ch2_output-(50+SC.zero_pitch_pct))*SC.half_travel_deg/
      (50*(SC.teeth_gimbal/SC.teeth_servo)*SC.dir_pitch)*0.017453292f;
    // 舵机一阶 + 300°/s 速率限幅 + 0.1% PWM 量化（物理执行器约束）
    const float rate_limit = 300.0f * 0.017453292f * dt;
    float u_cmd = upper;
    float l_cmd = lower;
    // 0.1% PWM -> 角度量化近似
    u_cmd = std::round(u_cmd / (0.001f * SC.half_travel_deg * 2.0f)) *
            (0.001f * SC.half_travel_deg * 2.0f);
    l_cmd = std::round(l_cmd / (0.001f * SC.half_travel_deg * 2.0f)) *
            (0.001f * SC.half_travel_deg * 2.0f);
    const float du = std::fmaxf(-rate_limit, std::fminf(rate_limit, u_cmd - p.servo_f));
    const float dl = std::fmaxf(-rate_limit, std::fminf(rate_limit, l_cmd - p.servo_t));
    p.servo_f += du * fminf(dt/0.05f,1.0f);
    p.servo_t += dl * fminf(dt/0.05f,1.0f);

    const float wf_t=(ch3_output/100)*P.wMax, wt_t=(ch4_output/100)*P.wMax;
    p.motor_wf += (wf_t-p.motor_wf)*fminf(dt/0.28f,1.0f);
    p.motor_wt += (wt_t-p.motor_wt)*fminf(dt/0.28f,1.0f);

    // 故障注入：F4 上摆卡死（冻结在 +8°）；F3/5 的推力损失在构造 PropulsionState 后
    // 乘到“有效转速”上（只作用于本拍 wrench，不改变电机状态，避免逐拍复利）。
    if (fault == 4 && t > fault_at) {
      p.servo_f = 8.0f * 0.017453292f;
    }

    // 阵风：20-25s 侧风 3 m/s（以 NED 侧向力体现）；--no-wind 时关闭
    p.wind[1]=(wind_on && t>20 && t<25)?3.0f*P.m:0.0f;
    PropulsionState ps{p.motor_wf,p.motor_wt,p.servo_f,p.servo_t};
    // 故障注入：fault 3 = 单发（前）推力保留 loss；fault 5 = 双发同时损失
    if ((fault == 3 || fault == 5) && t > fault_at) {
      ps.wf *= loss;
      if (fault == 5) ps.wt *= loss;
    }
    SixDOFWrench wr=computeWrench(ps,P);
    p.step(wr,dt);

    if ((k%200)==0) {
      float r,pitch,yaw; p.euler(r,pitch,yaw);
      if (fabsf(r*57.29578f)>max_roll) max_roll=fabsf(r*57.29578f);
      if (fabsf(pitch*57.29578f)>max_pitch) max_pitch=fabsf(pitch*57.29578f);
      if (t > 9.0f) {
        sum_roll2 += double(r*57.29578f)*double(r*57.29578f);
        sum_pitch2 += double(pitch*57.29578f)*double(pitch*57.29578f);
        ++n_att;
      }
      std::printf("t=%5.1f alt=%7.2f ekf_alt=%7.2f pos=(%6.2f,%6.2f) roll=%6.2f pitch=%6.2f "
                  "servo=(%5.1f,%5.1f) motor=(%5.1f,%5.1f) gnss=%d f3=%d f4=%d mask=%d\n",
                  t,-p.pos[2],g_ekf.alt_m()-kHomeLla(2),p.pos[0],p.pos[1],r*57.29578f,pitch*57.29578f,
                  ch1_output,ch2_output,ch3_output,ch4_output,(int)s.gnss_valid,
                  (int)g_actuator_thrust_loss,(int)g_actuator_servo_jam,(int)kFlightCtrlParams.inertia_comp_mask);
    }
  }
  const double rms_roll = n_att ? std::sqrt(sum_roll2/n_att) : 0.0;
  const double rms_pitch = n_att ? std::sqrt(sum_pitch2/n_att) : 0.0;
  std::printf("[fw_sil_env] seed=%d manual=%d coupled=%d fault=%d mask=%d "
              "max_roll=%.2f max_pitch=%.2f rms_roll=%.2f rms_pitch=%.2f "
              "f3=%d f4=%d final_alt=%.2f\n",
              seed,(int)manual,(int)coupled,fault,(int)kFlightCtrlParams.inertia_comp_mask,
              max_roll,max_pitch,rms_roll,rms_pitch,
              (int)g_actuator_thrust_loss,(int)g_actuator_servo_jam,-p.pos[2]);
  return 0;
}
