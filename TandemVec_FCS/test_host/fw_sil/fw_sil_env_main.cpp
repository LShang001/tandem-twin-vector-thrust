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
 *          `--real-nav` 下另有 `--mag-noise/--mag-disturb/--mag-dropout/
 *          --mag-cal-invalid`、`--imu-fault nan|stuck`/`--imu-dropout A:B`、
 *          `--gnss-consistency-reject`；
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
 * @note    与实机差异：传感器为注入模型 + 白噪声，无真实振动谱、无电机磁干扰、
 *          无 GNSS 多径；`--mag`/`mag_*` profile 才启用磁路径，基线默认关闭；
 *          `--flow` 是理想光流，不能当实机验证。40 s `--pos-hold` 的横滚/航向摆动
 *          在无磁基线也存在，属既有位置保持-偏航耦合边界，不是磁更新引入。
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
#include "mag_reference.h"
#include <Eigen/Dense>
#ifdef FW_SIL_REAL_NAV
#include "communication.h"
#include "navigation_task.h"
#include "sil_sensor_model.h"
#endif

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
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

#ifdef FW_SIL_REAL_NAV
// ============================================================
// 真实导航任务 SIL（--real-nav）：2 kHz IMU → 真实 handleNavigationSystem
// ============================================================
struct RealNavConfig
{
  bool enabled = false;
  int profile = 1;  // 0 clean / 1 nominal / 2 degraded
  float vib_g = 1.0f;       // 全油门振动幅值 [g]，按 (rpm/wMax)^2 缩放
  float vib_hz = 80.0f;
  int vib_mode = 0;
  bool anti_alias = true;
  float gnss_latency_s = 0.020f;
  float gnss_pos_ne_std = 0.8f;
  float gnss_pos_d_std = 1.2f;
  float gnss_vel_ne_std = 0.10f;
  float gnss_vel_d_std = 0.15f;
  float outage_start = 20.0f;
  float outage_end = 25.0f;
  float outlier_prob = 0.0f;
  bool gnss_spoof = false;
  float spoof_offset_m = 20.0f;
  float spoof_ramp_mps = 0.0f;
  float spoof_start_s = 15.0f;
  float baro_noise = 0.25f;
  float baro_drift = 0.02f;
  bool baro_lag = true;
  float wind_force_mps2 = 1.0f;  // 20–25 s 侧风等效加速度（历史 SIL 的 3 m/s² 作为 degraded）
  bool flow = false;
  int flow_delay = 1;
  bool pos_hold = false;
  bool rc_loss = false;
  float rc_loss_start = 15.0f;
  float rc_loss_end = 17.0f;
  float imu_drop_prob = 0.0f;
  int imu_fault_mode = 0;  // 0 none / 1 stuck / 2 NaN
  float imu_fault_start = 20.0f;
  float imu_fault_end = 21.0f;
  float imu_dropout_start = 1.0e9f;
  float imu_dropout_end = 1.0e9f;
  bool mag = false;
  bool gnss_consistency_reject = false;
  float mag_noise_ut = 0.5f;
  float mag_disturb_ut = 0.0f; // NED 正弦干扰幅值；0=干净
  float mag_dropout_start = 1.0e9f;
  float mag_dropout_end = 1.0e9f;
  bool mag_cal_invalid = false;
};

static void applyRealNavProfile(RealNavConfig &c, const char *name)
{
  if (std::strcmp(name, "clean") == 0)
  {
    c.profile = 0;
    c.vib_g = 0.0f;
    c.gnss_latency_s = 0.015f;
    c.gnss_pos_ne_std = 0.5f;
    c.gnss_pos_d_std = 0.8f;
    c.gnss_vel_ne_std = 0.05f;
    c.gnss_vel_d_std = 0.08f;
    c.outage_start = 1.0e9f;
    c.outage_end = 1.0e9f;
    c.outlier_prob = 0.0f;
    c.baro_noise = 0.10f;
    c.baro_drift = 0.0f;
    c.baro_lag = false;
    c.wind_force_mps2 = 0.0f;
    c.flow = false;
    c.rc_loss = false;
    c.imu_drop_prob = 0.0f;
  }
  else if (std::strcmp(name, "nomag_degraded") == 0)
  {
    // 无磁观测性降级场景：环境用 clean，但不注入磁；由 HeadingAidingGate
    // 强制 AUTO_POSITION→AUTO_ALTITUDE。不要复用 nominal 的 1g 振动/风，
    // 否则把“降级后自由漂移”误当成控制发散。
    applyRealNavProfile(c, "clean");
    c.mag = false;
  }
  else if (std::strcmp(name, "degraded") == 0)
  {
    c.profile = 2;
    c.vib_g = 2.0f;
    c.vib_hz = 120.0f;
    c.gnss_latency_s = 0.050f;
    c.gnss_pos_ne_std = 2.5f;
    c.gnss_pos_d_std = 4.0f;
    c.gnss_vel_ne_std = 0.20f;
    c.gnss_vel_d_std = 0.30f;
    // 传感器/环境退化门：5 s 失联 + 1.5 m/s² 阵风；RC 链路故障单独用
    // --rc-loss 注入，不能把“命令 disarm”混进传感器退化验收。
    c.outage_start = 20.0f;
    c.outage_end = 25.0f;
    c.outlier_prob = 0.02f;
    c.baro_noise = 0.50f;
    c.baro_drift = 0.10f;
    c.baro_lag = true;
    c.wind_force_mps2 = 1.5f;
    c.flow = true;
    c.rc_loss = false;
    c.rc_loss_start = 20.0f;
    c.rc_loss_end = 22.0f;
    c.imu_drop_prob = 0.001f;
  }
  else if (std::strcmp(name, "mag_clean") == 0)
  {
    applyRealNavProfile(c, "clean");
    c.mag = true;
    c.mag_noise_ut = 0.3f;
    c.mag_disturb_ut = 0.0f;
  }
  else if (std::strcmp(name, "mag_nominal") == 0)
  {
    applyRealNavProfile(c, "nominal");
    c.mag = true;
    c.mag_noise_ut = 0.6f;
    c.mag_disturb_ut = 0.0f;
  }
  else if (std::strcmp(name, "mag_disturbed") == 0)
  {
    applyRealNavProfile(c, "nominal");
    c.mag = true;
    c.mag_noise_ut = 0.8f;
    c.mag_disturb_ut = 12.0f;
  }
  else  // nominal
  {
    c.profile = 1;
    c.vib_g = 1.0f;
    c.vib_hz = 80.0f;
    c.gnss_latency_s = 0.020f;
    c.gnss_pos_ne_std = 0.8f;
    c.gnss_pos_d_std = 1.2f;
    c.gnss_vel_ne_std = 0.10f;
    c.gnss_vel_d_std = 0.15f;
    c.outage_start = 20.0f;
    c.outage_end = 25.0f;
    c.outlier_prob = 0.0f;
    c.baro_noise = 0.25f;
    c.baro_drift = 0.02f;
    c.baro_lag = true;
    c.wind_force_mps2 = 1.0f;
    // 室外 5 s GNSS 失联 + 侧风时，没有速度外部参照就无法保持水平位置；
    // nominal 启用光流测速（有该传感器的机型），否则只能验收定高。
    c.flow = true;
    c.rc_loss = false;
    c.imu_drop_prob = 0.0f;
  }
}

struct TruthSample
{
  float pos[3];
  float vel[3];
};

static void accumulateImuSample(const sil::ImuSample &s, float dt_s)
{
  acc_sum_x += s.accel[0];
  acc_sum_y += s.accel[1];
  acc_sum_z += s.accel[2];
  gyro_sum_x += s.gyro[0];
  gyro_sum_y += s.gyro[1];
  gyro_sum_z += s.gyro[2];
  ++imu_sample_count;

  if (imu_delta_sample_count < NAV_IMU_DELTA_BUFFER_SIZE)
  {
    const int idx = imu_delta_sample_count;
    imu_delta_theta_x[idx] = s.gyro[0] * dt_s;
    imu_delta_theta_y[idx] = s.gyro[1] * dt_s;
    imu_delta_theta_z[idx] = s.gyro[2] * dt_s;
    imu_delta_v_x[idx] = s.accel[0] * dt_s;
    imu_delta_v_y[idx] = s.accel[1] * dt_s;
    imu_delta_v_z[idx] = s.accel[2] * dt_s;
    imu_delta_dt_s[idx] = dt_s;
    imu_delta_sample_count++;
    imu_delta_time_sum_s += dt_s;
  }
  else
  {
    imu_delta_overflow = true;
  }
}

static int runRealNavLoop(const RealNavConfig &cfg, int seed, float seconds,
                          float target_alt, bool manual, bool coupled, int fault,
                          float fault_at, float loss, int mask, bool wind_on)
{
  Plant p;
  std::mt19937 rng(seed);
  sil::ImuConfig imu_cfg;
  sil::ImuModel imu(imu_cfg, static_cast<uint32_t>(seed) * 7919U + 11U);

  // 让固件自己完成 EKF 初始化/原点设置；不要沿用旧 SIL 的强制初始化。
  nav_system_initialized = false;
  is_origin_lla_set = false;
  nav_data_source = NavDataSource::INTERNAL;
  deta100_online = false;
  isLinkUp = true;
  failsafe_in_flight = false;
  g_sil_time_us = 0;

  const float dt_ctrl = 0.005f;
  const int sub_steps = 10;
  const float dt_imu = dt_ctrl / static_cast<float>(sub_steps);
  const int steps = static_cast<int>(seconds / dt_ctrl);
  const int kHist = 256;
  std::array<TruthSample, kHist> hist{};
  std::normal_distribution<float> n01(0.0f, 1.0f);

  Eigen::Vector3f gnss_corr = Eigen::Vector3f::Zero();
  float baro_state = 0.0f;
  bool baro_init = false;
  std::array<float, 32> flow_hist_vn{};
  std::array<float, 32> flow_hist_ve{};
  float servo_target_f = 0.0f, servo_target_t = 0.0f;
  float motor_target_wf = 0.0f, motor_target_wt = 0.0f;

  bool alt_cmd = false;
  float max_roll = 0.0f, max_pitch = 0.0f;
  float max_mag_kick = 0.0f;
  float max_mag_kick_rp = 0.0f;
  bool imu_fault_detected_any = false;
  double sum_roll2 = 0.0, sum_pitch2 = 0.0;
  double sum_pos2 = 0.0, max_pos = 0.0;
  int n_att = 0, n_pos = 0;
  double sum_nav_alt = 0.0;
  int n_nav_alt = 0;
  float hold_target[2] = {0.0f, 0.0f};
  bool hold_set = false;
  bool rc_lost_prev = false;
  float pending_imu_dt = 0.0f;  // 丢样时把时间并到下一个被接受的样本
  sil::ImuSample last_imu;
  bool have_last_imu = false;
  const float kImuNaN = std::numeric_limits<float>::quiet_NaN();
  double sum_hold2 = 0.0, max_hold = 0.0;
  double sum_hold_end2 = 0.0;
  int n_hold = 0, n_hold_end = 0;

  for (int k = 0; k < steps; ++k)
  {
    const float t = k * dt_ctrl;
    g_sil_time_us = static_cast<uint32_t>(t * 1.0e6f);

    // 真值历史（用于模拟 GNSS 链路延迟；按控制帧 5 ms 存储）。
    const int hi = k % kHist;
    hist[hi].pos[0] = p.pos[0]; hist[hi].pos[1] = p.pos[1]; hist[hi].pos[2] = p.pos[2];
    hist[hi].vel[0] = p.vel[0]; hist[hi].vel[1] = p.vel[1]; hist[hi].vel[2] = p.vel[2];

    // RC 50 Hz：链路丢失走真实 linkDownCallback（飞行中保持解锁 + AUTO_ALTITUDE），
    // 不再用“把 CH5 拉低”模拟——那会让固件执行 disarm 而不是 failsafe。
    const bool rc_lost = cfg.rc_loss &&
                         (t >= cfg.rc_loss_start && t < cfg.rc_loss_end);
    if (rc_lost && !rc_lost_prev)
    {
      linkDownCallback();
    }
    else if (!rc_lost && rc_lost_prev)
    {
      linkUpCallback();
    }
    rc_lost_prev = rc_lost;
    if (!rc_lost && (k % 4) == 0)
    {
      if (coupled)
      {
        const uint16_t roll = (t > 10 && t < 12) ? 1530 : ((t > 12 && t < 14) ? 1470 : 1500);
        const uint16_t pitch = (t > 10 && t < 12) ? 1470 : ((t > 12 && t < 14) ? 1530 : 1500);
        set_rc(1500, 1000, t >= 0.5f, roll, pitch);
      }
      else if (manual)
      {
        const uint16_t roll = (t > 10 && t < 15) ? 1600 : 1500;
        const uint16_t pitch = (t > 20 && t < 25) ? 1600 : 1500;
        set_rc((t < 1) ? 1000 : 1500, 1000, t >= 0.5f, roll, pitch);
      }
      else
      {
        const uint16_t thr = (t < 1.0f) ? 1000 : ((t >= 2.0f && t < 5.0f) ? 1600 : 1500);
        // 1700 落在 AUTO_POSITION 档（<1750 不触发 GUIDED 轨迹规划），
        // 1800 会因 ignition 打开被判成 GUIDED，超时后退回 MANUAL。
        const uint16_t mode = (cfg.pos_hold && t >= 5.0f) ? 1700 : ((t < 2) ? 1300 : 1500);
        set_rc(thr, mode, t >= 0.5f);
      }
    }

    // GNSS 10 Hz：真实 UBX NAV-PVT+EOE 注入 Serial4，由 ubx.Pump 解析。
    const bool gnss_outage = (t >= cfg.outage_start && t < cfg.outage_end);
    if (!gnss_outage && (k % 20) == 0 && t >= 0.1f)
    {
      const float t_meas = t - cfg.gnss_latency_s;
      int j = k - static_cast<int>(std::lround(cfg.gnss_latency_s / dt_ctrl));
      if (j < 0) j = 0;
      const TruthSample &h = hist[j % kHist];

      const float a_corr = std::exp(-0.1f / 20.0f);
      const float q_corr = std::sqrt(std::max(0.0f, 1.0f - a_corr * a_corr));
      gnss_corr(0) = a_corr * gnss_corr(0) + q_corr * 0.5f * n01(rng);
      gnss_corr(1) = a_corr * gnss_corr(1) + q_corr * 0.5f * n01(rng);
      gnss_corr(2) = a_corr * gnss_corr(2) + q_corr * 0.8f * n01(rng);

      float pn = h.pos[0] + gnss_corr(0) + cfg.gnss_pos_ne_std * n01(rng);
      float pe = h.pos[1] + gnss_corr(1) + cfg.gnss_pos_ne_std * n01(rng);
      float pd = h.pos[2] + gnss_corr(2) + cfg.gnss_pos_d_std * n01(rng);
      float vn = h.vel[0] + cfg.gnss_vel_ne_std * n01(rng);
      float ve = h.vel[1] + cfg.gnss_vel_ne_std * n01(rng);
      float vd = h.vel[2] + cfg.gnss_vel_d_std * n01(rng);

      bool outlier = false;
      if (cfg.outlier_prob > 0.0f)
      {
        std::uniform_real_distribution<float> u01(0.0f, 1.0f);
        if (u01(rng) < cfg.outlier_prob)
        {
          outlier = true;
          pn += 50.0f; pe -= 40.0f; pd += 30.0f;
          vn += 12.0f; ve -= 10.0f;
        }
      }
      // GNSS spoof：位置偏移但自报精度仍然良好（h_acc 正常），用于检验
      // 30 m 野值门/NIS 能否识别“慢偏/小台阶”欺骗；不是普通野值。
      if (cfg.gnss_spoof && t >= cfg.spoof_start_s)
      {
        const float dt_spoof = t - cfg.spoof_start_s;
        const float off = cfg.spoof_offset_m + cfg.spoof_ramp_mps * dt_spoof;
        pn += off;
        pe += 0.6f * off;
        vn += cfg.spoof_ramp_mps;
        ve += 0.6f * cfg.spoof_ramp_mps;
      }

      const Eigen::Vector3d lla = bfs::ned2lla(
          Eigen::Vector3d(pn, pe, pd), kHomeLla, bfs::AngPosUnit::RAD);
      sil::UbxPvtFields f;
      f.itow = 100000U + static_cast<uint32_t>(t_meas * 1000.0f);
      f.lon_1e7 = static_cast<int32_t>(std::llround(lla(1) * 57.29577951308232 * 1.0e7));
      f.lat_1e7 = static_cast<int32_t>(std::llround(lla(0) * 57.29577951308232 * 1.0e7));
      f.height_mm = static_cast<int32_t>(std::llround(lla(2) * 1000.0));
      f.h_msl_mm = f.height_mm;
      f.vel_n_mmps = static_cast<int32_t>(std::llround(vn * 1000.0f));
      f.vel_e_mmps = static_cast<int32_t>(std::llround(ve * 1000.0f));
      f.vel_d_mmps = static_cast<int32_t>(std::llround(vd * 1000.0f));
      f.g_speed_mmps = static_cast<int32_t>(std::llround(std::hypot(vn, ve) * 1000.0f));
      f.fix_type = 3;
      f.num_sv = outlier ? 7 : 14;
      f.h_acc_mm = outlier ? 20000U : static_cast<uint32_t>(
          std::max(0.5f, 0.8f + 0.5f * gnss_corr.head<2>().norm()) * 1000.0f);
      f.v_acc_mm = outlier ? 30000U : static_cast<uint32_t>(
          std::max(1.0f, 1.5f + 0.8f * std::fabs(gnss_corr(2))) * 1000.0f);
      f.s_acc_mmps = outlier ? 5000U : static_cast<uint32_t>(
          std::max(0.05f, 0.08f + 0.05f * gnss_corr.head<2>().norm()) * 1000.0f);
      f.p_dop_1e2 = outlier ? 500U : static_cast<uint16_t>(
          100 + 20 * std::min(4.0f, gnss_corr.norm()));

      uint8_t frame[128];
      size_t n = sil::BuildNavPvt(f, frame, sizeof(frame));
      Serial4.feed(frame, n);
      n = sil::BuildNavEoe(f.itow, frame, sizeof(frame));
      Serial4.feed(frame, n);
      last_gnss_data_ms = millis();
    }
    // 每帧泵一次 UART backlog；真实解析器负责帧同步/校验/epoch 排队。
    ubx.Pump(512, micros());

    // 气压计 25 Hz：一阶滞后 + 噪声 + 漂移 + 1 cm 量化。
    if ((k % 8) == 0)
    {
      const float true_alt = -p.pos[2];
      if (!baro_init) { baro_state = true_alt; baro_init = true; }
      if (cfg.baro_lag)
        baro_state += (true_alt - baro_state) * std::fmin(1.0f, 0.04f / 0.15f);
      else
        baro_state = true_alt;
      float b = baro_state + cfg.baro_noise * n01(rng) + cfg.baro_drift * t;
      b = std::round(b / 0.01f) * 0.01f;
      baro_altitude_raw = b;
      baro_altitude = b;
    }

    // 光流 10 Hz：机体系速度 + 尺度/噪声 + 姿态质量门控 + 链路延迟。
    if ((k % 20) == 0)
    {
      if (cfg.flow)
      {
        float vb[3];
        p.ned_to_body(p.vel, vb);
        float r_att, pitch_att, yaw_att;
        p.euler(r_att, pitch_att, yaw_att);
        const bool quality_ok = (std::fabs(r_att) < 0.35f && std::fabs(pitch_att) < 0.35f);
        const float fx = 0.95f * vb[0] + 0.12f * n01(rng);
        const float fy = 0.95f * vb[1] + 0.12f * n01(rng);
        const int fhi = (k / 20) % 32;
        flow_hist_vn[fhi] = fx;
        flow_hist_ve[fhi] = fy;
        const int fdi = (fhi - cfg.flow_delay + 32) % 32;
        flow_data.velocity_x_mps = flow_hist_vn[fdi];
        flow_data.velocity_y_mps = flow_hist_ve[fdi];
        flow_data.flow_quality = quality_ok ? 100 : 0;
        flow_data.is_flow_valid = quality_ok;
        flow_data.timestamp_ms = millis();
        flow_data.last_update_ms = millis();
        flow_data.distance_m = -p.pos[2] + 0.05f * n01(rng);
        flow_data.range_quality = 90;
        flow_data.is_range_valid = true;
        flow_data.last_range_update_ms = millis();
      }
      else
      {
        flow_data.is_flow_valid = false;
        flow_data.is_range_valid = false;
      }
    }

    if (cfg.pos_hold && !hold_set && t >= 5.0f)
    {
      hold_target[0] = p.pos[0];
      hold_target[1] = p.pos[1];
      hold_set = true;
    }

    // 磁力计注入（生产路径）：按真值姿态把站点参考场旋到机体系，
    // 加噪声/硬铁残差/慢变干扰后写入 MagSensor；导航任务自行做门控与 EKF 融合。
    // 100 Hz 注入（2 kHz 内环每 2 拍一次），更接近真实 MagSensor 轮询率。
    if (cfg.mag)
    {
      const bool mag_dropout =
          t >= cfg.mag_dropout_start && t <= cfg.mag_dropout_end;
      if (!mag_dropout && (k % 2) == 0)
      {
        const Eigen::Quaternionf q_true(p.q.w, p.q.x, p.q.y, p.q.z);
        Eigen::Vector3f mag_ned(
            bfs::kDefaultMagReference.field_ut[0],
            bfs::kDefaultMagReference.field_ut[1],
            bfs::kDefaultMagReference.field_ut[2]);
        if (cfg.mag_disturb_ut > 0.0f)
        {
          // NED 慢变干扰：同时破坏模长/倾角/方位，模拟电机/外部磁源。
          mag_ned.x() += cfg.mag_disturb_ut *
                         std::sin(2.0f * 3.14159265f * t / 7.0f + 0.3f);
          mag_ned.y() += cfg.mag_disturb_ut *
                         std::cos(2.0f * 3.14159265f * t / 11.0f + 0.7f);
          mag_ned.z() += 0.5f * cfg.mag_disturb_ut *
                         std::sin(2.0f * 3.14159265f * t / 13.0f);
        }
        Eigen::Vector3f mag_body = q_true.conjugate() * mag_ned;
        mag_body.x() += cfg.mag_noise_ut * n01(rng);
        mag_body.y() += cfg.mag_noise_ut * n01(rng);
        mag_body.z() += cfg.mag_noise_ut * n01(rng);
        magSensor.silInjectFrdSample(mag_body.data(), micros(),
                                     !cfg.mag_cal_invalid);
      }
    }

    // 真实导航任务 + 真实 GNC。导航任务消费 IMU 聚合量和 UBX epoch。
    handleNavigationSystem();
    runGNCExecutive();
    if (g_sil_nav_debug_imu_failed)
    {
      imu_fault_detected_any = true;
    }
    if (std::fabs(mag_status.last_kick_deg) > max_mag_kick)
    {
      max_mag_kick = std::fabs(mag_status.last_kick_deg);
    }
    if (mag_status.last_kick_rp_deg > max_mag_kick_rp)
    {
      max_mag_kick_rp = mag_status.last_kick_rp_deg;
    }

    if (!manual && !alt_cmd && t >= 3.0f)
    {
      target_altitude = target_alt;
      vertical_loiter_state = ALTITUDE_HOVERING;
      alt_cmd = true;
    }

    // 控制输出 -> 执行器目标（量化在控制帧，动态在 2 kHz 内环）。
    const float upper = (ch1_output - (50 + SC.zero_roll_pct)) * SC.half_travel_deg /
                        (50 * (SC.teeth_gimbal / SC.teeth_servo) * SC.dir_roll) * 0.017453292f;
    const float lower = (ch2_output - (50 + SC.zero_pitch_pct)) * SC.half_travel_deg /
                        (50 * (SC.teeth_gimbal / SC.teeth_servo) * SC.dir_pitch) * 0.017453292f;
    const float q = 0.001f * SC.half_travel_deg * 2.0f;
    servo_target_f = std::round(upper / q) * q;
    servo_target_t = std::round(lower / q) * q;
    motor_target_wf = (ch3_output / 100.0f) * P.wMax;
    motor_target_wt = (ch4_output / 100.0f) * P.wMax;

    // 2 kHz 内环：执行器动态 + plant + IMU 采样。
    for (int s = 0; s < sub_steps; ++s)
    {
      const float ts = t + static_cast<float>(s) * dt_imu;
      g_sil_time_us = static_cast<uint32_t>(ts * 1.0e6f);

      const float rate_limit = 300.0f * 0.017453292f * dt_imu;
      const float servo_step_f = std::fmax(-rate_limit, std::fmin(rate_limit,
          (servo_target_f - p.servo_f) * std::fmin(1.0f, dt_imu / 0.05f)));
      const float servo_step_t = std::fmax(-rate_limit, std::fmin(rate_limit,
          (servo_target_t - p.servo_t) * std::fmin(1.0f, dt_imu / 0.05f)));
      p.servo_f += servo_step_f;
      p.servo_t += servo_step_t;
      p.motor_wf += (motor_target_wf - p.motor_wf) * std::fmin(1.0f, dt_imu / 0.28f);
      p.motor_wt += (motor_target_wt - p.motor_wt) * std::fmin(1.0f, dt_imu / 0.28f);

      if (fault == 4 && t > fault_at) p.servo_f = 8.0f * 0.017453292f;
      p.wind[1] = (wind_on && t > 20 && t < 25) ? cfg.wind_force_mps2 * P.m : 0.0f;

      PropulsionState ps{p.motor_wf, p.motor_wt, p.servo_f, p.servo_t};
      if ((fault == 3 || fault == 5) && t > fault_at)
      {
        ps.wf *= loss;
        if (fault == 5) ps.wt *= loss;
      }
      p.step(computeWrench(ps, P), dt_imu);

      float f_ned[3];
      if (p.on_ground)
      {
        f_ned[0] = 0.0f; f_ned[1] = 0.0f; f_ned[2] = -P.g;
      }
      else
      {
        f_ned[0] = p.force_ned[0] / P.m;
        f_ned[1] = p.force_ned[1] / P.m;
        f_ned[2] = p.force_ned[2] / P.m;
      }
      float f_body[3];
      p.ned_to_body(f_ned, f_body);

      const float rpm_ratio = std::fmax(p.motor_wf, p.motor_wt) / P.wMax;
      const float vib_scale = rpm_ratio * rpm_ratio;
      std::uniform_real_distribution<float> u01(0.0f, 1.0f);
      const bool imu_dropout_window =
          t >= cfg.imu_dropout_start && t <= cfg.imu_dropout_end;
      const bool drop = imu_dropout_window ||
                        ((cfg.imu_drop_prob > 0.0f) &&
                         (u01(rng) < cfg.imu_drop_prob));
      const bool imu_fault =
          (cfg.imu_fault_mode != 0) &&
          (t >= cfg.imu_fault_start && t < cfg.imu_fault_end);
      if (drop)
      {
        pending_imu_dt += dt_imu;
      }
      else
      {
        const float sample_dt = dt_imu + pending_imu_dt;
        pending_imu_dt = 0.0f;
        sil::ImuSample sample;
        if (imu_fault && cfg.imu_fault_mode == 1 && have_last_imu)
        {
          sample = last_imu;  // 传感器输出冻结
        }
        else
        {
          const float omega[3] = {p.omega[0], p.omega[1], p.omega[2]};
          sample = imu.Step(
              omega, f_body, ts, sample_dt, cfg.vib_g * vib_scale, cfg.vib_hz,
              cfg.vib_mode, cfg.anti_alias);
          if (imu_fault && cfg.imu_fault_mode == 2)
          {
            for (int a = 0; a < 3; ++a)
            {
              sample.accel[a] = kImuNaN;
              sample.gyro[a] = kImuNaN;
            }
          }
          else
          {
            last_imu = sample;
            have_last_imu = true;
          }
        }
        accumulateImuSample(sample, sample_dt);
        icm_gyro_x = sample.gyro[0];
        icm_gyro_y = sample.gyro[1];
        icm_gyro_z = sample.gyro[2];
        icm_accel_x = sample.accel[0];
        icm_accel_y = sample.accel[1];
        icm_accel_z = sample.accel[2];
      }
    }

    // 每帧统计（避免 1 Hz 抽样漏掉尖峰）；打印仍保持 1 Hz。
    float r = 0.0f, pitch = 0.0f, yaw = 0.0f;
    p.euler(r, pitch, yaw);
    if (std::fabs(r * 57.29578f) > max_roll) max_roll = std::fabs(r * 57.29578f);
    if (std::fabs(pitch * 57.29578f) > max_pitch) max_pitch = std::fabs(pitch * 57.29578f);
    if (t > 9.0f)
    {
      sum_roll2 += double(r * 57.29578f) * double(r * 57.29578f);
      sum_pitch2 += double(pitch * 57.29578f) * double(pitch * 57.29578f);
      ++n_att;
    }
    if (nav_system_initialized && t > 10.0f)
    {
      const double en = double(relative_north) - double(p.pos[0]);
      const double ee = double(relative_east) - double(p.pos[1]);
      const double e2 = en * en + ee * ee;
      sum_pos2 += e2;
      if (std::sqrt(e2) > max_pos) max_pos = std::sqrt(e2);
      ++n_pos;
      sum_nav_alt += nav_ekf.alt_m() - origin_alt_m;
      ++n_nav_alt;
    }
    if (hold_set && t > 10.0f)
    {
      const double hx = double(p.pos[0]) - double(hold_target[0]);
      const double hy = double(p.pos[1]) - double(hold_target[1]);
      const double h2 = hx * hx + hy * hy;
      sum_hold2 += h2;
      if (std::sqrt(h2) > max_hold) max_hold = std::sqrt(h2);
      ++n_hold;
      if (t > (seconds - 5.0f))
      {
        sum_hold_end2 += h2;
        ++n_hold_end;
      }
    }

    // 1 Hz 诊断打印。
    if ((k % 200) == 0)
    {
      const long age_ms = (last_gnss_data_ms > 0)
                              ? static_cast<long>(millis() - last_gnss_data_ms)
                              : -1;
      Eigen::Vector3d obs_rel = Eigen::Vector3d::Zero();
      if (gnss_status.has_obs && is_origin_lla_set)
      {
        Eigen::Vector3d origin_lla(origin_lat_rad, origin_lon_rad, origin_alt_m);
        obs_rel = bfs::lla2ned(gnss_status.last_obs_lla_rad_m, origin_lla,
                               bfs::AngPosUnit::RAD);
      }
      std::printf("t=%5.1f alt=%7.2f nav_alt=%7.2f pos=(%6.2f,%6.2f) truth=(%6.2f,%6.2f) "
                  "obs=(%6.2f,%6.2f) nis=%.2f ep=%u pvt=%u eoe=%u pend=%u "
                  "roll=%6.2f pitch=%6.2f yaw=%6.2f navyaw=%6.2f "
                  "servo=(%5.1f,%5.1f) motor=(%5.1f,%5.1f) "
                  "nav=%d fix=%d gnss_age=%ld imu=%d acc=(%6.2f,%6.2f,%6.2f) "
                  "gyro=(%6.3f,%6.3f,%6.3f) st=%d/%.2f/%u epoch=%d initd=%d\n",
                  t, -p.pos[2],
                  nav_system_initialized ? (nav_ekf.alt_m() - origin_alt_m) : 0.0,
                  relative_north, relative_east, p.pos[0], p.pos[1],
                  obs_rel(0), obs_rel(1), gnss_status.last_test_ratio,
                  gnss_status.epoch_count, gnss_status.parser_nav_pvt_count,
                  gnss_status.parser_nav_eoe_count, gnss_status.pending_epoch_count,
                  r * 57.29578f, pitch * 57.29578f, yaw * 57.29578f,
                  nav_system_initialized ? nav_ekf.yaw_rad() * 57.29578f : 0.0f,
                  ch1_output, ch2_output, ch3_output, ch4_output,
                  nav_system_initialized ? 1 : 0,
                  static_cast<int>(ubx.fix()), age_ms,
                  g_sil_nav_debug_imu_count,
                  g_sil_nav_debug_accel[0], g_sil_nav_debug_accel[1], g_sil_nav_debug_accel[2],
                  g_sil_nav_debug_gyro[0], g_sil_nav_debug_gyro[1], g_sil_nav_debug_gyro[2],
                  g_sil_nav_debug_static ? 1 : 0,
                  g_sil_nav_debug_static_confidence,
                  g_sil_nav_debug_static_frames,
                  g_sil_nav_debug_gnss_epoch ? 1 : 0,
                  g_sil_nav_debug_initialized ? 1 : 0);
    }
  }

  const double rms_roll = n_att ? std::sqrt(sum_roll2 / n_att) : 0.0;
  const double rms_pitch = n_att ? std::sqrt(sum_pitch2 / n_att) : 0.0;
  const double pos_rms = n_pos ? std::sqrt(sum_pos2 / n_pos) : -1.0;
  const double nav_alt_mean = n_nav_alt ? (sum_nav_alt / n_nav_alt) : 0.0;
  const double hold_rms = n_hold ? std::sqrt(sum_hold2 / n_hold) : -1.0;
  const double hold_end_rms = n_hold_end ? std::sqrt(sum_hold_end2 / n_hold_end) : -1.0;
  std::printf("[sil_real] profile=%d seed=%d final_alt=%.2f nav_alt_mean=%.2f "
              "pos_rms=%.2f pos_max=%.2f hold_rms=%.2f hold_max=%.2f hold_end_rms=%.2f "
              "max_roll=%.2f max_pitch=%.2f rms_roll=%.2f rms_pitch=%.2f "
              "nav_init=%d fix3d=%u quality_fail=%u tow_mismatch=%u duplicate=%u "
              "reacquire_resets=%u outlier_rejects=%u mag_fused=%u mag_rej=%u "
              "mag_gate=%u mag_cal=%d mag_norm=%.2f mag_hdg=%.2f "
              "grav_att=%u grav_fused=%u mag_kick=%.2f mag_kickrp=%.2f "
              "imu_state=%d imu_frz=%.2f imu_drop=%.2f imu_failed=%d "
              "imu_detected=%d gnss_susp=%d gnss_crej=%u gnss_cres=%.3f\n",
              cfg.profile, seed, -p.pos[2], nav_alt_mean, pos_rms, max_pos,
              hold_rms, max_hold, hold_end_rms, max_roll, max_pitch, rms_roll, rms_pitch,
              nav_system_initialized ? 1 : 0,
              gnss_status.fix3d_count, gnss_status.quality_fail_count,
              gnss_status.tow_mismatch_count, gnss_status.duplicate_itow_count,
              nav_ekf.gnss_reacquire_reset_count(),
              nav_ekf.gnss_outlier_reject_count(),
              nav_ekf.mag_vector_fused_count(),
              nav_ekf.mag_vector_reject_count(),
              mag_status.gate_rejected, mag_status.cal_valid ? 1 : 0,
              mag_status.norm_ut, mag_status.heading_error_deg,
              aid_tracker.status(bfs::AidSource::Gravity).attempt_count,
              aid_tracker.status(bfs::AidSource::Gravity).fuse_count,
              max_mag_kick, max_mag_kick_rp,
              static_cast<int>(g_imu_health_state), g_imu_health_frozen_s,
              g_imu_health_no_sample_s, g_sil_nav_debug_imu_failed ? 1 : 0,
              imu_fault_detected_any ? 1 : 0,
              gnss_status.consistency_suspect ? 1 : 0,
              gnss_status.consistency_reject_count,
              gnss_status.consistency_accel_res);
  std::printf("SIL_REAL_JSON {\"profile\":%d,\"seed\":%d,\"final_alt\":%.3f,"
              "\"nav_alt_mean\":%.3f,\"pos_rms\":%.3f,\"pos_max\":%.3f,"
              "\"hold_rms\":%.3f,\"hold_max\":%.3f,\"hold_end_rms\":%.3f,"
              "\"max_roll\":%.3f,"
              "\"max_pitch\":%.3f,\"rms_roll\":%.3f,\"rms_pitch\":%.3f,"
              "\"nav_init\":%d,\"fix3d\":%u,\"quality_fail\":%u,"
              "\"tow_mismatch\":%u,\"duplicate\":%u,\"reacquire_resets\":%u,"
              "\"outlier_rejects\":%u,\"mag_fused\":%u,\"mag_rej\":%u,"
              "\"mag_gate\":%u,\"mag_cal\":%d,\"mag_norm\":%.3f,"
              "\"mag_hdg\":%.3f,\"imu_state\":%d,\"imu_frz\":%.3f,"
              "\"imu_drop\":%.3f,\"imu_failed\":%d,\"imu_detected\":%d,"
              "\"gnss_susp\":%d,\"gnss_crej\":%u,\"gnss_cres\":%.3f,"
              "\"heading_aiding\":%d,\"heading_age\":%.2f,"
              "\"pos_degraded\":%d}\n",
              cfg.profile, seed, -p.pos[2], nav_alt_mean, pos_rms, max_pos,
              hold_rms, max_hold, hold_end_rms, max_roll, max_pitch, rms_roll, rms_pitch,
              nav_system_initialized ? 1 : 0,
              gnss_status.fix3d_count, gnss_status.quality_fail_count,
              gnss_status.tow_mismatch_count, gnss_status.duplicate_itow_count,
              nav_ekf.gnss_reacquire_reset_count(),
              nav_ekf.gnss_outlier_reject_count(),
              nav_ekf.mag_vector_fused_count(),
              nav_ekf.mag_vector_reject_count(),
              mag_status.gate_rejected, mag_status.cal_valid ? 1 : 0,
              mag_status.norm_ut, mag_status.heading_error_deg,
              static_cast<int>(g_imu_health_state), g_imu_health_frozen_s,
              g_imu_health_no_sample_s, g_sil_nav_debug_imu_failed ? 1 : 0,
              imu_fault_detected_any ? 1 : 0,
              gnss_status.consistency_suspect ? 1 : 0,
              gnss_status.consistency_reject_count,
              gnss_status.consistency_accel_res,
              g_heading_aiding_valid ? 1 : 0,
              static_cast<double>(g_heading_aiding_age_s),
              g_mode_degraded_heading ? 1 : 0);
  return 0;
}
#endif  // FW_SIL_REAL_NAV

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
#ifdef FW_SIL_REAL_NAV
  RealNavConfig real_cfg;
#endif
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
#ifdef FW_SIL_REAL_NAV
    else if (std::strcmp(argv[i],"--mag-noise")==0 && i+1<argc) real_cfg.mag_noise_ut=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--mag-disturb")==0 && i+1<argc) real_cfg.mag_disturb_ut=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--mag-dropout")==0 && i+1<argc)
      std::sscanf(argv[++i], "%f:%f", &real_cfg.mag_dropout_start,
                  &real_cfg.mag_dropout_end);
    else if (std::strcmp(argv[i],"--mag-cal-invalid")==0) real_cfg.mag_cal_invalid=true;
    else if (std::strcmp(argv[i],"--real-nav")==0) real_cfg.enabled=true;
    else if (std::strcmp(argv[i],"--profile")==0 && i+1<argc)
      applyRealNavProfile(real_cfg, argv[++i]);
    else if (std::strcmp(argv[i],"--vib")==0 && i+1<argc) real_cfg.vib_g=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--vib-hz")==0 && i+1<argc) real_cfg.vib_hz=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--vib-mode")==0 && i+1<argc) real_cfg.vib_mode=std::atoi(argv[++i]);
    else if (std::strcmp(argv[i],"--no-anti-alias")==0) real_cfg.anti_alias=false;
    else if (std::strcmp(argv[i],"--anti-alias")==0) real_cfg.anti_alias=true;
    else if (std::strcmp(argv[i],"--gnss-latency-ms")==0 && i+1<argc)
      real_cfg.gnss_latency_s=std::atof(argv[++i])*0.001f;
    else if (std::strcmp(argv[i],"--gnss-pos-std")==0 && i+1<argc)
      real_cfg.gnss_pos_ne_std=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--gnss-vel-std")==0 && i+1<argc)
      real_cfg.gnss_vel_ne_std=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--gnss-outlier")==0 && i+1<argc)
      real_cfg.outlier_prob=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--gnss-spoof")==0 && i+1<argc)
    {
      real_cfg.gnss_spoof = true;
      real_cfg.spoof_offset_m = std::atof(argv[++i]);
    }
    else if (std::strcmp(argv[i],"--gnss-spoof-ramp")==0 && i+1<argc)
      real_cfg.spoof_ramp_mps=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--gnss-spoof-start")==0 && i+1<argc)
      real_cfg.spoof_start_s=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--gnss-outage")==0 && i+1<argc)
      std::sscanf(argv[++i], "%f:%f", &real_cfg.outage_start, &real_cfg.outage_end);
    else if (std::strcmp(argv[i],"--baro-noise")==0 && i+1<argc)
      real_cfg.baro_noise=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--baro-drift")==0 && i+1<argc)
      real_cfg.baro_drift=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--wind-force")==0 && i+1<argc)
      real_cfg.wind_force_mps2=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--no-baro-lag")==0) real_cfg.baro_lag=false;
    else if (std::strcmp(argv[i],"--pos-hold")==0) real_cfg.pos_hold=true;
    else if (std::strcmp(argv[i],"--no-flow")==0) real_cfg.flow=false;
    else if (std::strcmp(argv[i],"--rc-loss")==0 && i+1<argc)
    {
      real_cfg.rc_loss=true;
      std::sscanf(argv[++i], "%f:%f", &real_cfg.rc_loss_start, &real_cfg.rc_loss_end);
    }
    else if (std::strcmp(argv[i],"--imu-drop")==0 && i+1<argc)
      real_cfg.imu_drop_prob=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--imu-fault")==0 && i+1<argc)
    {
      const char *m = argv[++i];
      real_cfg.imu_fault_mode = (std::strcmp(m, "stuck") == 0) ? 1 :
                                ((std::strcmp(m, "nan") == 0) ? 2 : 0);
    }
    else if (std::strcmp(argv[i],"--imu-fault-window")==0 && i+1<argc)
      std::sscanf(argv[++i], "%f:%f", &real_cfg.imu_fault_start,
                  &real_cfg.imu_fault_end);
    else if (std::strcmp(argv[i],"--imu-dropout")==0 && i+1<argc)
      std::sscanf(argv[++i], "%f:%f", &real_cfg.imu_dropout_start,
                  &real_cfg.imu_dropout_end);
    else if (std::strcmp(argv[i],"--gnss-consistency-reject")==0)
      real_cfg.gnss_consistency_reject=true;
#endif
  }
#ifdef FW_SIL_REAL_NAV
  if (real_cfg.enabled)
  {
    real_cfg.flow = real_cfg.flow || flow;
    real_cfg.flow_delay = g_flow_delay;
    real_cfg.mag = real_cfg.mag || g_use_mag_yaw;
    g_gnss_consistency_reject_enabled = real_cfg.gnss_consistency_reject;
    kFlightCtrlParams.inertia_comp_mask = static_cast<uint8_t>(mask & 0x03);
    if (coupled) manual = true;
    return runRealNavLoop(real_cfg, seed, seconds, target_alt, manual, coupled,
                          fault, fault_at, loss, mask, wind_on);
  }
#endif
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
