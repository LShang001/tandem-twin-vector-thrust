/**
 * @file    fw_sil_guidance_main.cpp
 * @module  固件/宿主 SIL（fw-tests：制导算法联合仿真入口）
 * @brief   UDP 双向桥接的真实固件 SIL：外部制导算法 ↔ 真实 runGNCExecutive() 闭环
 * @details 被测：真实固件 `runGNCExecutive()` 及其控制链（不复制控制律）。
 *          输入：UDP 端口（默认 7001）收 14 B 指令帧 `0xAA + 3×float(U,E,N) + 0x55`；
 *          处理：本文件提供 6DOF 单刚体 plant（平动用四元数把推力旋到 NED）、
 *          电机一阶滞后、传感器直注与执行器解算；固件代码照常跑 200 Hz；
 *          飞控在每拍把状态打包为 74 B 帧
 *          `0xAA + 18×float(auto,fire,t,pn,pe,pu,vn,ve,vu,roll,pitch,yaw,gx,gy,gz,au,ae,an) + 0x55`
 *          发往 UDP 端口（默认 7002）。
 *          运行：BIN_DIR=T:/fw_sil bash test_host/fw_sil/build_and_run_guidance.sh [rx_port] [tx_port]
 * @theory  plant 侧为牛顿-欧拉刚性体；GUIDED 模式进入条件见
 *          `math_utils.h::getControlMode()`（CH7>1650 且 trajectoryPlanningStarted）
 * @frame   plant NED（n/e/d），姿态用机体 FRD 四元数；状态帧天向取 -pos_d（Up+）
 * @units   位置 m，速度 m/s，姿态 rad，角速度 rad/s，加速度 m/s²，时间 s
 * @note    与实机差异：无气动、无风、无振动、传感器直注无噪声；
 *          本入口**只用于算法联调**，不构成飞行品质证据。
 * @test    G1 制导算法给零指令时 plant 保持悬停；G2 收到加速度指令后按指令机动
 * @see     fw_sil_main.cpp（定高/手动入口）、test_host/fw_sil/README.md
 */
#include "state_data.h"
#include "flight_control.h"
#include "TandemVec_Propulsion.h"
#include "TandemVec_Config.h"
#include "Quat4f.h"
#include "QuaternionMath.h"

#include <cmath>
#include <random>
#include <cstdio>
#include <cstring>
#include <cstdlib>

#ifdef _WIN32
// 注意：stub/Arduino.h 把 INPUT 定义成宏（#define INPUT 0），与 winuser.h 的
// INPUT 结构体名冲突；此处先撤销宏再引入 winsock2，避免构建被平台头阻断。
#undef INPUT
#include <winsock2.h>
#include <ws2tcpip.h>
#include <mmsystem.h>
typedef int socklen_t;
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

static const TandemVecParams &P = kDefaultTandemVecParams;
// 模型不确定性注入（默认 1.0 = 名义模型）：
//   g_plant_mass   质量缩放（电池/载荷变化）
//   g_thrust_scale 推力缩放（电压下降 / ESC 个体差异）
static float g_plant_mass_scale = 1.0f;
static float g_thrust_scale = 1.0f;
static float g_plant_mass = 1.0f;
static float g_last_specific_force_up = 0.0f; // 上一拍机体比力（FRD z，向上为负）
// IMU 加速度噪声注入（真实振动/量化噪声），用于验证加速度闭环会不会被噪声带坏
static float g_accel_noise_rms = 0.0f;
static std::mt19937 g_rng(20261103u);
static std::normal_distribution<float> g_gauss(0.0f, 1.0f);
// 传感器延迟注入（ms）与陀螺零偏（rad/s），模拟真实 IMU 链路
static float g_accel_delay_s = 0.0f;
static float g_gyro_bias = 0.0f;
static const int kDelayBuf = 64;
static float g_accel_hist[kDelayBuf] = {0};
static int g_accel_hist_idx = 0;
static int g_delay_steps = 0;
static const ServoConfig &SC = kDefaultServoConfig;

struct Motor
{
  float w, target, tau;
  Motor(float w0 = 0.0f, float t = 0.28f) : w(w0), target(w0), tau(t) {}
  void set(float wt) { target = wt; }
  float step(float dt) { w += (target - w) * fminf(dt / tau, 1.0f); return w; }
};

/** @brief 高精度 6DOF 刚体：完整欧拉方程 + RK4 四元数 + 二次气动阻力 + 质量可时变
 *
 *  状态 y = [pn, pe, pd, vn, ve, vd, qw, qx, qy, qz, wx, wy, wz]
 *  平动：a = (T·(-z_b) + F_aero)/m + g_ned，F_aero = -½ρ·CdA·|v|·v
 *  转动：I·ω̇ = M_b - ω×(I·ω)（对角惯量下退化为欧拉方程三分量）
 *  积分：四阶 Runge–Kutta（与项目红线「四元数 RK4 + 每阶段保范」一致）
 */
#ifndef BFS_SIL_CDA
#define BFS_SIL_CDA 0.0f // 等效阻力面积 Cd·A (m²)；0 = 忽略气动（用户要求先用无气动模型）
#endif

struct Body
{
  Quat4f q;
  float omega[3];
  float vel_n, vel_e, vel_d;
  float pos_n, pos_e, pos_d;
  float mass_now;

  Body() : q(1, 0, 0, 0), omega{0, 0, 0},
           vel_n(0), vel_e(0), vel_d(0),
           pos_n(0), pos_e(0), pos_d(0), mass_now(P.m) {}

  void deriv(const float *y, float Fx, const float *Mb, float *dy) const
  {
    const float qw = y[6], qx = y[7], qy = y[8], qz = y[9];
    const float wx = y[10], wy = y[11], wz = y[12];
    const float vn = y[3], ve = y[4], vd = y[5];

    dy[0] = vn; dy[1] = ve; dy[2] = vd;

    // 推力沿机体 -z：R 的第三列是 z_b 在 NED 的分量
    const float R31 = 2.0f * (qx * qz + qw * qy);
    const float R32 = 2.0f * (qy * qz - qw * qx);
    const float R33 = 1.0f - 2.0f * (qx * qx + qy * qy);

    const float sp = sqrtf(vn * vn + ve * ve + vd * vd);
    const float kd = 0.5f * 1.225f * BFS_SIL_CDA;

    dy[3] = (-Fx * R31 - kd * sp * vn) / mass_now;
    dy[4] = (-Fx * R32 - kd * sp * ve) / mass_now;
    dy[5] = (-Fx * R33 - kd * sp * vd) / mass_now + P.g;

    // 四元数微分：q̇ = ½·q⊗ω
    dy[6] = 0.5f * (-wx * qx - wy * qy - wz * qz);
    dy[7] = 0.5f * (wx * qw + wz * qy - wy * qz);
    dy[8] = 0.5f * (wy * qw - wz * qx + wx * qz);
    dy[9] = 0.5f * (wz * qw + wy * qx - wx * qy);

    // 完整欧拉方程（对角惯量）
    dy[10] = (Mb[0] - (P.Iz - P.Iy) * wy * wz) / P.Ix;
    dy[11] = (Mb[1] - (P.Ix - P.Iz) * wz * wx) / P.Iy;
    dy[12] = (Mb[2] - (P.Iy - P.Ix) * wx * wy) / P.Iz;
  }

  void step(const SixDOFWrench &w, float dt)
  {
    // computeWrench 模型系 -> 机体 FRD（x'=-z_b, y'=+y_b, z'=+x_b）
    const float Mb[3] = {-w.Mz, w.My, -w.Mx};
    float y[13] = {pos_n, pos_e, pos_d, vel_n, vel_e, vel_d,
                   q.w, q.x, q.y, q.z, omega[0], omega[1], omega[2]};
    float k1[13], k2[13], k3[13], k4[13], tmp[13];

    deriv(y, w.Fx, Mb, k1);
    for (int i = 0; i < 13; ++i) tmp[i] = y[i] + 0.5f * dt * k1[i];
    deriv(tmp, w.Fx, Mb, k2);
    for (int i = 0; i < 13; ++i) tmp[i] = y[i] + 0.5f * dt * k2[i];
    deriv(tmp, w.Fx, Mb, k3);
    for (int i = 0; i < 13; ++i) tmp[i] = y[i] + dt * k3[i];
    deriv(tmp, w.Fx, Mb, k4);

    for (int i = 0; i < 13; ++i)
    {
      y[i] += (dt / 6.0f) * (k1[i] + 2.0f * k2[i] + 2.0f * k3[i] + k4[i]);
    }

    pos_n = y[0]; pos_e = y[1]; pos_d = y[2];
    vel_n = y[3]; vel_e = y[4]; vel_d = y[5];

    Quat4f qq{y[6], y[7], y[8], y[9]};
    qq = qNorm(qq); // 每步保范
    q = qq;

    omega[0] = y[10]; omega[1] = y[11]; omega[2] = y[12];
  }

  void euler(float &roll, float &pitch, float &yaw) const
  {
    Quaternion qq{q.w, q.x, q.y, q.z};
    quaternionToEuler(qq, roll, pitch, yaw);
  }
};

static void set_rc(uint16_t throttle, uint16_t mode, bool unlock,
                   uint16_t roll = 1500, uint16_t pitch = 1500,
                   uint16_t ignition = 1800)
{
  for (int i = 0; i < 9; ++i) raw_rc_values[i] = 1500;
  raw_rc_values[0] = roll;
  raw_rc_values[1] = pitch;
  raw_rc_values[2] = throttle;
  raw_rc_values[3] = 1500;
  raw_rc_values[4] = unlock ? 1800 : 1000;
  raw_rc_values[5] = ignition;
  raw_rc_values[6] = mode;
  raw_rc_values[7] = 1000;
  raw_rc_values[8] = 1000;
}

static void feed_sensors(const Body &b)
{
  float roll, pitch, yaw;
  b.euler(roll, pitch, yaw);
  AHRS_Packet.Roll = roll;
  AHRS_Packet.Pitch = pitch;
  AHRS_Packet.Heading = yaw;
  AHRS_Packet.Qw = b.q.w; AHRS_Packet.Qx = b.q.x;
  AHRS_Packet.Qy = b.q.y; AHRS_Packet.Qz = b.q.z;

  icm_gyro_x = b.omega[0];
  icm_gyro_y = b.omega[1];
  icm_gyro_z = b.omega[2];
  current_omega_dps_body_filtered = Vector3(b.omega[0] * 57.29578f,
                                            b.omega[1] * 57.29578f,
                                            b.omega[2] * 57.29578f);

  estimated_height = -b.pos_d;
  relative_north = b.pos_n;
  relative_east = b.pos_e;
  relative_down = b.pos_d;
  fused_north_pos = b.pos_n;
  fused_east_pos = b.pos_e;
  fused_north_vel = b.vel_n;
  fused_east_vel = b.vel_e;

  INS_GNSS_Packet.location_north = b.pos_n;
  INS_GNSS_Packet.location_east = b.pos_e;
  INS_GNSS_Packet.location_down = b.pos_d;
  INS_GNSS_Packet.velocity_north = b.vel_n;
  INS_GNSS_Packet.velocity_east = b.vel_e;
  INS_GNSS_Packet.velocity_down = b.vel_d;
  // EKF 天向速度估计注入（Up+）：速度差分闭环的输入；噪声按加速度噪声的
  // 1/20 折算（真实 EKF 输出比原始 IMU 干净），无噪声时为真值。
  estimated_velocity = -b.vel_d;
  if (g_accel_noise_rms > 0.0f)
  {
    estimated_velocity += (g_accel_noise_rms * 0.05f) * g_gauss(g_rng);
  }

  // IMU 比力注入（机体 FRD）：本 plant 只有推力，推力沿机体 -z，
  // 故比力 = -F/m（z 分量为负、向上）。加速度闭环修正用得到。
  icm_accel_x = 0.0f;
  icm_accel_y = 0.0f;
  float acc_z = g_last_specific_force_up;
  if (g_accel_noise_rms > 0.0f)
  {
    acc_z += g_accel_noise_rms * g_gauss(g_rng);
  }
  // 延迟：最近 N 步的环形缓冲
  g_accel_hist[g_accel_hist_idx] = acc_z;
  g_accel_hist_idx = (g_accel_hist_idx + 1) % kDelayBuf;
  if (g_delay_steps > 0)
  {
    int read_idx = (g_accel_hist_idx - 1 - g_delay_steps + kDelayBuf * 2) % kDelayBuf;
    acc_z = g_accel_hist[read_idx];
  }
  icm_accel_z = acc_z;
  icm_gyro_x += g_gyro_bias;
}

/** @brief 从一个 UDP 套接字非阻塞取一帧 14 B 制导指令，成功返回 true */
static bool try_recv_command(int sock, float &au, float &ae, float &an)
{
  uint8_t buf[64];
  const int n = (int)recv(sock, (char *)buf, sizeof(buf), 0);
  if (n < 14)
  {
    return false;
  }
  if (buf[0] != 0xAA || buf[13] != 0x55)
  {
    return false;
  }
  std::memcpy(&au, &buf[1], sizeof(float));
  std::memcpy(&ae, &buf[5], sizeof(float));
  std::memcpy(&an, &buf[9], sizeof(float));
  return true;
}

/** @brief 把本拍状态打包成 74 B 帧发往制导侧 */
static void send_state(int sock, const sockaddr_in &dst, float t, const Body &b,
                       float acc_u, float acc_e, float acc_n)
{
  float roll, pitch, yaw;
  b.euler(roll, pitch, yaw);
  float f[18] = {
      1.0f, 1.0f, t,
      b.pos_n, b.pos_e, -b.pos_d,
      b.vel_n, b.vel_e, -b.vel_d,
      roll, pitch, yaw,
      b.omega[0], b.omega[1], b.omega[2],
      acc_u, acc_e, acc_n};
  uint8_t frame[74];
  frame[0] = 0xAA;
  std::memcpy(&frame[1], f, sizeof(f));
  frame[73] = 0x55;
  sendto(sock, (const char *)frame, sizeof(frame), 0,
         (const sockaddr *)&dst, sizeof(dst));
}

int main(int argc, char **argv)
{
#ifdef _WIN32
  WSADATA wsa;
  WSAStartup(MAKEWORD(2, 2), &wsa);
  // 1 ms 定时器分辨率：否则 Sleep(4) 实际约 15.6 ms，仿真会比实时慢 3~4 倍，
  // 制导侧按墙钟发指令会被当成"超时"。
  timeBeginPeriod(1);
#endif
  const int rx_port = (argc > 1) ? std::atoi(argv[1]) : 7001;
  const int tx_port = (argc > 2) ? std::atoi(argv[2]) : 7002;
  g_plant_mass_scale = (argc > 3) ? (float)std::atof(argv[3]) : 1.0f;
  g_thrust_scale = (argc > 4) ? (float)std::atof(argv[4]) : 1.0f;
  if (g_plant_mass_scale <= 0.0f) g_plant_mass_scale = 1.0f;
  if (g_thrust_scale <= 0.0f) g_thrust_scale = 1.0f;
  g_accel_noise_rms = (argc > 5) ? (float)std::atof(argv[5]) : 0.0f;
  g_accel_delay_s = (argc > 6) ? (float)std::atof(argv[6]) : 0.0f;
  g_gyro_bias = (argc > 7) ? (float)std::atof(argv[7]) : 0.0f;
  const float pos_step_n = (argc > 8) ? (float)std::atof(argv[8]) : 0.0f;
  const char *test_mode = (argc > 9) ? argv[9] : "auto";
  const bool mode_fwd = (std::strcmp(test_mode, "fwd") == 0);
  const bool mode_stick_roll = (std::strcmp(test_mode, "stick_roll") == 0);
  const bool mode_stick_rate = (std::strcmp(test_mode, "stick_rate") == 0);
  std::printf("[guidance-sil] test_mode=%s pos_step=%.2f\n", test_mode, pos_step_n);
  g_delay_steps = (int)(g_accel_delay_s / 0.005f);
  if (g_delay_steps < 0) g_delay_steps = 0;
  if (g_delay_steps > kDelayBuf - 1) g_delay_steps = kDelayBuf - 1;
  g_plant_mass = P.m * g_plant_mass_scale;
  std::printf("[guidance-sil] mass_scale=%.3f thrust_scale=%.3f (m=%.3f kg)\n",
              g_plant_mass_scale, g_thrust_scale, g_plant_mass);

  int rx = (int)socket(AF_INET, SOCK_DGRAM, 0);
  int tx = (int)socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons((unsigned short)rx_port);
  if (bind(rx, (sockaddr *)&addr, sizeof(addr)) != 0)
  {
    std::printf("bind rx :%d failed\n", rx_port);
    return 1;
  }
  // 接收套接字必须非阻塞：每拍只取当前已有指令，没有指令时立刻返回
#ifdef _WIN32
  u_long nonblock = 1;
  ioctlsocket(rx, FIONBIO, &nonblock);
#else
  fcntl(rx, F_SETFL, fcntl(rx, F_GETFL, 0) | O_NONBLOCK);
#endif
  sockaddr_in dst{};
  dst.sin_family = AF_INET;
  dst.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  dst.sin_port = htons((unsigned short)tx_port);
  std::printf("[guidance-sil] rx=%d tx=%d\n", rx_port, tx_port);

  Body body;
  // 初始离地 10 m：制导侧轨迹规划器（vtvl_sim PLAN_R_X_MIN=-0.5）不允许初始
  // 高度低于 -0.5 m，而 plant 从 0 高度起跳会在解锁瞬间下沉重到 −10 m 以上，
  // 直接触发规划不可行。这里等价于"已起飞到 3 m 再交接制导"。
  body.pos_d = -10.0f;
  body.mass_now = g_plant_mass; // 质量偏差注入（含电池/载荷变化）
  const float w_hover = P.wMax * 0.5f;
  Motor mf(w_hover), mt(w_hover);

  isLinkUp = true;
  failsafe_in_flight = false;
  nav_system_initialized = true;
  is_origin_lla_set = true;
  origin_lat_deg = 28.2; origin_lon_deg = 112.9; origin_alt_m = 50.0;

  const float dt = 0.005f;
  const int steps = int(60.0f / dt);
  bool got_command = false;
  float acc_u = 0.0f, acc_e = 0.0f, acc_n = 0.0f;
  int cmd_count = 0;

  for (int k = 0; k < steps; ++k)
  {
    const float t = k * dt;

    // 制导指令：非阻塞收，收到即刷新时间戳（GUIDED 500 ms 超时保护）
    float au, ae, an;
    if (try_recv_command(rx, au, ae, an))
    {
      acc_u = au; acc_e = ae; acc_n = an;
      guidance_accel_U_cmd = au;
      guidance_accel_E_cmd = ae;
      guidance_accel_N_cmd = an;
      last_guidance_command_millis = millis();
      new_guidance_command_received = true;
      got_command = true;
      cmd_count++;
    }

    // 起飞段用 AUTO_ALTITUDE（CH7 中档）闭环稳定在 10 m；收到首帧制导指令后
    // 才切高档 + 开点火开关进 GUIDED。若一开始就挂高档，AUTO_POSITION 只有
    // 手动油门、中位不足以悬停，plant 会沉到 −8 m 以下，制导侧规划直接不可行。
    // pos_step 模式：轨迹规划（AUTO_POSITION）位置阶跃，用于速度/位置跟踪验证；
    // 非 pos_step 模式才走 GUIDED 制导链路。
    const bool pos_mode = (pos_step_n != 0.0f);
    const uint16_t ch7 = (pos_mode || got_command) ? 1800 : 1300;
    if (mode_stick_roll || mode_stick_rate)
    {
      const uint16_t roll_cmd = (t > 5.0f && t < 10.0f) ? 1600 : 1500;
      set_rc(1500, 1300, t >= 0.5f, roll_cmd, 1500, 1800);
      if (mode_stick_rate)
      {
        raw_rc_values[8] = 1800; // CH9 >= 1500 → 角速率模式
      }
    }
    else if (mode_fwd)
    {
      const uint16_t pitch_cmd = (t > 3.0f) ? 1600 : 1500;
      set_rc(1500, 1800, t >= 0.5f, 1500, pitch_cmd, 1000);
    }
    else
    {
      set_rc(1500, ch7, t >= 0.5f, 1500, 1500,
             (pos_mode ? 1000 : (got_command ? 1800 : 1000)));
    }
    static bool pos_step_applied = false;
    if (pos_mode && !pos_step_applied && t >= 3.0f)
    {
      targetNorth += pos_step_n;
      pos_step_applied = true;
    }
    static bool alt_cmd_sent = false;
    if (!got_command && !alt_cmd_sent && t >= 1.0f)
    {
      target_altitude = 10.0f;
      alt_cmd_sent = true;
    }
    trajectoryPlanningStarted = got_command;

    // 航向可观门控注入：本入口不链接 navigation_task.cpp，g_pos_hold_degraded
    // 保持 state_data.cpp 的默认 true，会把 AUTO_POSITION/GUIDED 一律降级成
    // AUTO_ALTITUDE，制导闭环无法成立。SIL 的 plant 直接注入真实姿态角，
    // 此处声明"航向可用"（等价于磁/航向源新鲜），仅用于算法联调。
    g_pos_hold_degraded = false;

    feed_sensors(body);
    runGNCExecutive();

    const float upper = (ch1_output - (50.0f + SC.zero_roll_pct)) *
                        SC.half_travel_deg /
                        (50.0f * (SC.teeth_gimbal / SC.teeth_servo) * SC.dir_roll) *
                        0.017453292f;
    const float lower = (ch2_output - (50.0f + SC.zero_pitch_pct)) *
                        SC.half_travel_deg /
                        (50.0f * (SC.teeth_gimbal / SC.teeth_servo) * SC.dir_pitch) *
                        0.017453292f;

    // 实机标定口径：油门控制量 → 推力近似线性（用户实测，大范围内符合）。
    // plant 推力为 kT·ω²，故转速取 √(油门) 才让 F 与油门成正比。
    // 注意：fw_sil_main.cpp（历史定高基线）仍用平方口径，不要改它。
    const float pct1 = ch3_output / 100.0f;
    const float pct2 = ch4_output / 100.0f;
    mf.set(sqrtf(pct1 > 0.0f ? pct1 : 0.0f) * P.wMax);
    mt.set(sqrtf(pct2 > 0.0f ? pct2 : 0.0f) * P.wMax);
    PropulsionState ps{mf.step(dt), mt.step(dt), upper, lower};
    SixDOFWrench wr = computeWrench(ps, P);
    // 推力缩放模拟电压下降/ESC 个体差异（力矩同比例缩放）
    wr.Fx *= g_thrust_scale;
    wr.My *= g_thrust_scale;
    wr.Mz *= g_thrust_scale;
    g_last_specific_force_up = -wr.Fx / g_plant_mass;
    body.step(wr, dt);

    // 每拍回传状态（200 Hz），制导侧自行降采样
    send_state(tx, dst, t, body, acc_u, acc_e, acc_n);

    // 实时节流：SIL 纯计算远快于墙钟，若不限速，飞控 500 ms 制导超时会以
    // 虚拟时间瞬间触发（外部算法来不及回包）。按 200 Hz 睡眠 ≈ dt 对齐实时。
#ifdef _WIN32
    Sleep(4);
#else
    usleep(4000);
#endif

    if ((k % 400) == 0)
    {
      float r, p, y;
      body.euler(r, p, y);
      std::printf("t=%5.1f alt=%7.2f vz=%6.2f n=%7.3f e=%7.3f roll=%6.2f pitch=%6.2f "
                  "rate_r=%6.1f mode=%d m1=%5.1f gU=%7.3f cmd=%d\n",
                  t, -body.pos_d, -body.vel_d, body.pos_n, body.pos_e,
                  r * 57.29578f, p * 57.29578f, body.omega[0] * 57.29578f,
                  (int)g_current_flight_mode, ch3_output, guidance_accel_U_cmd,
                  cmd_count);
      std::fflush(stdout);
    }
  }

#ifdef _WIN32
  closesocket(rx);
  closesocket(tx);
  timeEndPeriod(1);
  WSACleanup();
#else
  close(rx);
  close(tx);
#endif
  std::printf("[guidance-sil] done, commands received: %d (got_command=%d)\n",
              cmd_count, (int)got_command);
  return 0;
}
