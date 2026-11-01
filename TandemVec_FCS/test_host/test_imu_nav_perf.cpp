/**
 * @file    test_imu_nav_perf.cpp
 * @module  固件/宿主回归测试（fw-tests：IMU × GNSS 器件模型 × 导航性能）
 * @brief   按器件手册参数生成 IMU 样本与不同档次 GNSS 观测，驱动真实
 *          Ekf15State，评估导航算法在换型 + 不同 GNSS 条件下的精度与可靠性。
 * @details 被测：`lib/navigation-main/src/ekf_15_state.h`（生产 EKF，不重写）。
 *          输入矩阵：IMU{ICM42688P@2k, ICM45686@3.2k} ×
 *          GNSS{RTK 10Hz, RTK 10Hz+100ms 延迟, M8N 10Hz, M8N 5Hz, M8N 1Hz,
 *          低端 5Hz} × 场景{static, maneuver, cruise} × 4 seeds。
 *          处理：IMU 按 ODR 采样（噪声密度/零偏/量化/一阶 LPF/饱和）→ 5ms 内
 *          双子样累加 → TimeUpdateTwoSample；GNSS 按刷新率注入（延迟用
 *          t−latency 的真值）→ MeasurementUpdate；输出 CSV + 汇总表。
 *
 * @theory  白噪声密度 N [unit/√Hz] 在采样率 fs 下离散化 σ=N·√(fs/2)，经一阶
 *          LPF（fc）后 σ≈N·√(π/2·fc)。零偏按手册 ±max 取 1σ=max/3。
 *          手册：ICM-45686 DS-000489 v1.1（陀螺 0.0038 °/s/√Hz、ZRO ±0.4 °/s
 *          板级、加计 70 µg/√Hz @≤8g、零 g ±20 mg 板级）；ICM-42688-P 用
 *          TDK DS-000347 典型值（陀螺 0.0028 °/s/√Hz、ZRO ±0.5 °/s、加计
 *          70 µg/√Hz）——仓库未存该手册全文，数值待台架复核，仅作对照臂。
 * @frame   机体系 FRD、导航系 NED；比力 f_ned = a_ned − g_ned，g_ned=(0,0,+9.79)。
 * @units   加速度 m/s²、角速度 rad/s、姿态 °、位置/速度 m、m/s。
 * @note    yaw 在无磁航向时不可观，只输出漂移量、不设 PASS 阈值；
 *          姿态 PASS 只看 roll/pitch（重力可观），位置/速度阈值随 GNSS 档次缩放。
 * @test    static/maneuver/cruise × 6 GNSS 配置 × 2 IMU × 4 seeds
 * @see     lib/ICM45686/doc/DS-000489_ICM-45686_v1.1.md、src/imu_factory.cpp
 */
#include <Eigen/Dense>
#include "ins_test_gravity.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <random>
#include <vector>

#include "ekf_15_state.h"

namespace
{
const float kGravity = ins_test::GravityWgs84();  // WGS84 正常重力（与 EKF 同口径）
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kRad2Deg = 180.0 / kPi;
constexpr float kNavDt = 0.005f;    // 导航 200Hz
constexpr int kSeedCount = 4;
constexpr int kScenarioCount = 4; // 主矩阵场景数（static/maneuver/cruise/vibration）
constexpr double kDurationS = 30.0;

enum class Scenario
{
  Static = 0,
  Maneuver = 1,
  Cruise = 2,
  Vibration = 3,
  VibrationOutage = 4,
  Accel = 5,  // 正弦加速：5 m/s 峰值、0.2 Hz（评估速度延迟）
};

const char *scenarioName(Scenario s)
{
  switch (s)
  {
  case Scenario::Static:   return "static";
  case Scenario::Maneuver: return "maneuver";
  case Scenario::Vibration: return "vibration";
  case Scenario::VibrationOutage: return "vib_outage";
  case Scenario::Accel:            return "accel";
  default:                 return "cruise";
  }
}

/// IMU 器件参数（手册典型/板级值；偏置按 max/3 取 1σ）。
struct ImuSpec
{
  const char *name;
  double odr_hz;
  double accel_noise_density;  // m/s²/√Hz
  double gyro_noise_density;   // rad/s/√Hz
  double accel_bias_1sigma;    // m/s²
  double gyro_bias_1sigma;     // rad/s
  double lpf_accel_hz;
  double lpf_gyro_hz;
  double accel_fs;             // m/s²（±）
  double gyro_fs;              // rad/s（±）
};

/// GNSS 档次（位置/速度 1σ、刷新率、延迟）。
struct GnssSpec
{
  const char *name;
  double rate_hz;
  double pos_std_m;
  double vel_std_mps;
  double latency_ms;
};

const ImuSpec kIcm42688p = {
    "ICM42688P_2k", 2000.0,
    70e-6 * kGravity,
    0.0028 * kDeg2Rad,
    (20e-3 * kGravity) / 3.0,
    (0.5 * kDeg2Rad) / 3.0,
    50.0, 100.0,
    8.0 * kGravity, 2000.0 * kDeg2Rad};

const ImuSpec kIcm45686 = {
    "ICM45686_3k2", 3200.0,
    70e-6 * kGravity,
    0.0038 * kDeg2Rad,
    (20e-3 * kGravity) / 3.0,
    (0.4 * kDeg2Rad) / 3.0,
    50.0, 100.0,
    8.0 * kGravity, 2000.0 * kDeg2Rad};

const GnssSpec kGnssSpecs[] = {
    {"RTK_10Hz",        10.0, 0.02, 0.010,   0.0},
    {"RTK_10Hz_100ms",  10.0, 0.02, 0.010, 100.0},
    {"M8N_10Hz",        10.0, 1.50, 0.050,   0.0},
    {"M8N_5Hz",          5.0, 2.50, 0.100,   0.0},
    {"M8N_1Hz",          1.0, 2.50, 0.150,   0.0},
    {"LowEnd_5Hz",       5.0, 5.00, 0.300,   0.0},
    {"RTK_FLOAT_10Hz",  10.0, 0.20, 0.020,   0.0},
    {"M8N_20Hz",        20.0, 1.50, 0.050,   0.0},
    {"M8N_10Hz_50ms",   10.0, 1.50, 0.050,  50.0},
};
constexpr int kGnssCount = static_cast<int>(sizeof(kGnssSpecs) / sizeof(kGnssSpecs[0]));

struct Truth
{
  double roll, pitch, yaw;         // rad
  double p, q, r;                  // body rates rad/s
  Eigen::Vector3d pos_ned;         // m
  Eigen::Vector3d vel_ned;         // m/s
  Eigen::Vector3d acc_ned;         // m/s²
};

Truth truthAt(double t, Scenario sc)
{
  Truth tr{};
  if (sc == Scenario::Accel)
  {
    // 纯平动正弦加速：v_n = 5·sin(0.4πt)，a_n 峰值约 6.3 m/s²。
    const double w = 2.0 * kPi * 0.2;
    tr.vel_ned(0) = 5.0 * std::sin(w * t);
    tr.pos_ned(0) = (5.0 / w) * (1.0 - std::cos(w * t));
    tr.acc_ned(0) = 5.0 * w * std::cos(w * t);
    return tr;
  }
  if (sc == Scenario::Static || sc == Scenario::Vibration ||
      sc == Scenario::VibrationOutage)
  {
    return tr; // 全零；Vibration 的 80Hz 激励只加在 IMU 测量侧
  }
  if (sc == Scenario::Maneuver)
  {
    const double w1 = 2.0 * kPi * 0.2;
    const double w2 = 2.0 * kPi * 0.15;
    tr.roll = 10.0 * kDeg2Rad * std::sin(w1 * t);
    tr.pitch = 8.0 * kDeg2Rad * std::sin(w2 * t);
    tr.yaw = 3.0 * kDeg2Rad * t;
    const double roll_dot = 10.0 * kDeg2Rad * w1 * std::cos(w1 * t);
    const double pitch_dot = 8.0 * kDeg2Rad * w2 * std::cos(w2 * t);
    const double yaw_dot = 3.0 * kDeg2Rad;
    tr.p = roll_dot - yaw_dot * std::sin(tr.pitch);
    tr.q = pitch_dot * std::cos(tr.roll) + yaw_dot * std::sin(tr.roll) * std::cos(tr.pitch);
    tr.r = -pitch_dot * std::sin(tr.roll) + yaw_dot * std::cos(tr.roll) * std::cos(tr.pitch);
    return tr;
  }
  // Cruise：北向 1 m/s 匀速 + 东向 0.2 m/s 正弦；姿态简化保持水平。
  const double we = 0.1;
  tr.pos_ned = Eigen::Vector3d(1.0 * t, 0.2 * std::sin(we * t), 0.0);
  tr.vel_ned = Eigen::Vector3d(1.0, 0.2 * we * std::cos(we * t), 0.0);
  tr.acc_ned = Eigen::Vector3d(0.0, -0.2 * we * we * std::sin(we * t), 0.0);
  return tr;
}

Eigen::Matrix3d nedFromBody(const Truth &tr)
{
  const Eigen::AngleAxisd rx(tr.roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd ry(tr.pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd rz(tr.yaw, Eigen::Vector3d::UnitZ());
  return (rz * ry * rx).toRotationMatrix();
}

struct Lpf
{
  double alpha = 1.0;
  Eigen::Vector3d state = Eigen::Vector3d::Zero();
  void init(double fc_hz, double fs_hz)
  {
    const double tau = 1.0 / (2.0 * kPi * fc_hz);
    const double dt = 1.0 / fs_hz;
    alpha = dt / (tau + dt);
    state.setZero();
  }
  Eigen::Vector3d update(const Eigen::Vector3d &x)
  {
    state += alpha * (x - state);
    return state;
  }
};

struct RunResult
{
  double roll_rmse_deg = 0.0;
  double pitch_rmse_deg = 0.0;
  double yaw_drift_deg = 0.0;
  double vel_rmse = 0.0;
  double pos_rmse = 0.0;
  int gnss_updates = 0;
  double out_pos_rmse = 0.0;
  double out_vel_rmse = 0.0;
  double out_alt_rmse = 0.0;
  double alt_rmse = 0.0;
  double post_outage_rmse = 0.0;
  double max_att_jump_deg = 0.0;
  int reanchor_suspicious = 0;
  double nis_mean = 0.0;
  double nis_reject_rate = 0.0;
  double gyro_freeze_detect_t = -1.0;
  int gyro_freeze_hits = 0;
  double accel_freeze_detect_t = -1.0;
  int accel_freeze_hits = 0;
  double nan_t = -1.0;
  int out_count = 0;
  bool finite = true;
};

double wrapPi(double a)
{
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

constexpr double kEarthR = 6378137.0;

RunResult runCase(const ImuSpec &imu, const GnssSpec &gnss, Scenario sc,
                  int seed, bool verbose, std::vector<double> *trace_buf = nullptr,
                  bool compensate_delay = false, double vib_hz = 80.0, int gnss_mode = 0,
                  double hacc_gate_m = 1.0e9, int yaw_aid = 0, float q_accel = 2.0f,
                  int baro_mode = 0, double baro_std = 0.5, double vib_amp = 0.5,
                  double outage_start = 10.0, double outage_end = 20.0,
                  double baro_bias = 0.0, int baro_qnh_cal = 0, double baro_start_s = 0.0,
                  double baro_spike_m = 0.0, int baro_gate = 0, double qnh_alpha = 0.05, float q_accel_outage = 0.0f,
                  double reanchor_m = 0.0, int reanchor_streak = 1,
                  int adapt_r = 0, double adapt_thresh_m = 5.0,
                  double gnss_spike_m = 0.0, double gnss_spike_t = 15.0,
                  int gnss_spike_frames = 1, int reanchor_consist = 0,
                  double ekf_accel_std = 0.25, double ekf_gyro_std = 0.0015,
                  double ekf_accel_markov = 0.010, double ekf_gyro_markov = 0.00524,
                  double r_floor = 0.15, double gyro_tempc = 0.0,
                  int gyro_temp_comp = 0, double temp_comp_err = 0.0,
                  double misalign_deg = 0.0, double lever_x = 0.0,
                  double lever_y = 0.0, double lever_z = 0.0,
                  double vel_extra_delay_s = 0.0, double vert_noise_mult = 1.5,
                  double gyro_freeze_t = -1.0, double accel_freeze_t = -1.0,
                  double vel_noise_mult = 1.0, double accel_tempc = 0.0,
                  double pos_noise_mult = 1.0, double gnss_time_offset_s = 0.0,
                  double vert_spike_m = 0.0)
{
  std::mt19937_64 rng(static_cast<uint64_t>(seed) * 7919ULL +
                      static_cast<uint64_t>(imu.odr_hz) * 13ULL +
                      static_cast<uint64_t>(gnss.rate_hz * 100.0) +
                      static_cast<uint64_t>(sc));
  std::normal_distribution<double> n01(0.0, 1.0);
  // baro 噪声用独立随机源：避免开/关 baro 改变 GNSS 噪声序列，保证 A/B 公平。
  std::mt19937_64 baro_rng(static_cast<uint64_t>(seed) * 131ULL + 977ULL);
  std::normal_distribution<double> baro_n01(0.0, 1.0);

  Eigen::Vector3d accel_bias(imu.accel_bias_1sigma * n01(rng),
                             imu.accel_bias_1sigma * n01(rng),
                             imu.accel_bias_1sigma * n01(rng));
  Eigen::Vector3d gyro_bias(imu.gyro_bias_1sigma * n01(rng),
                            imu.gyro_bias_1sigma * n01(rng),
                            imu.gyro_bias_1sigma * n01(rng));

  const double fs = imu.odr_hz;
  const double accel_sigma = imu.accel_noise_density * std::sqrt(fs / 2.0);
  const double gyro_sigma = imu.gyro_noise_density * std::sqrt(fs / 2.0);
  Lpf accel_lpf[3], gyro_lpf[3];
  for (int i = 0; i < 3; ++i)
  {
    accel_lpf[i].init(imu.lpf_accel_hz, fs);
    gyro_lpf[i].init(imu.lpf_gyro_hz, fs);
  }
  const double accel_lsb = 2.0 * imu.accel_fs / 65536.0;
  const double gyro_lsb = 2.0 * imu.gyro_fs / 65536.0;

  const Truth tr0 = truthAt(0.0, sc);
  const Eigen::Matrix3d R0 = nedFromBody(tr0);
  const Eigen::Vector3d g_ned(0.0, 0.0, kGravity);
  const Eigen::Vector3d f_ned0 = tr0.acc_ned - g_ned;
  const Eigen::Vector3d f_body0 = R0.transpose() * f_ned0;

  const Eigen::Vector3d lla0(31.2304 * kDeg2Rad, 121.4737 * kDeg2Rad, 5.0);
  bfs::Ekf15State ekf;
  ekf.accel_std_mps2(static_cast<float>(ekf_accel_std));
  ekf.gyro_std_radps(static_cast<float>(ekf_gyro_std));
  ekf.accel_markov_bias_std_mps2(static_cast<float>(ekf_accel_markov));
  ekf.gyro_markov_bias_std_radps(static_cast<float>(ekf_gyro_markov));
  ekf.Initialize(f_body0.cast<float>(), Eigen::Vector3f::Zero(),
                 Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), lla0);
  ekf.set_q_accel_scale(q_accel);

  const int nav_steps = static_cast<int>(kDurationS / kNavDt);
  const double dt_imu = 1.0 / fs;
  const int samples_per_nav = static_cast<int>(std::lround(kNavDt / dt_imu));
  const int gnss_div = std::max(1, static_cast<int>(std::lround(200.0 / gnss.rate_hz)));
  const double latency_s = gnss.latency_ms / 1000.0;

  double sum_roll2 = 0.0, sum_pitch2 = 0.0, sum_vel2 = 0.0, sum_pos2 = 0.0;
  double sum_out_pos2 = 0.0, sum_out_vel2 = 0.0, sum_out_alt2 = 0.0;
  double sum_alt2 = 0.0;
  double sum_post_pos2 = 0.0;
  int post_count = 0;
  double nan_t_local = -1.0;
  double qnh_est = 0.0; // baro QNH 在线偏差估计（GNSS 高度差低通）
  bool qnh_inited = false;
  int baro_gate_rej = 0; // 连续被残差门控拒绝次数（防锁死）
  bool baro_suspect = false;  // gate=2：baro 健康 suspect（冻结融合）
  double last_good_baro = lla0(2);
  double max_att_jump_deg_local = 0.0;  // 重锚前后姿态跳变（安全校验）
  int reanchor_suspicious_local = 0;
  double nis_sum = 0.0;
  static constexpr int kHealthWin = 400;  // 0.2s @2kHz
  Eigen::Vector3d gyro_hist[kHealthWin];
  int gyro_hist_idx = 0;
  int gyro_hist_count = 0;
  double gyro_freeze_detect_local = -1.0;
  int gyro_freeze_hits_local = 0;
  Eigen::Vector3d accel_hist[kHealthWin];
  int accel_hist_idx = 0;
  int accel_hist_count = 0;
  double accel_freeze_detect_local = -1.0;
  int accel_freeze_hits_local = 0;
  Eigen::Vector3d frozen_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d frozen_accel = Eigen::Vector3d::Zero();
  bool have_frozen_accel = false;
  bool have_frozen_gyro = false;
  int nis_cnt = 0;
  int nis_rej = 0;
  int baro_ok_streak = 0;     // 连续正常次数
  int reanchor_cnt = 0;       // 连续大新息计数
  int gnss_update_seq = 0;     // GNSS 更新序号（野值按次数注入）
  int spike_seq0 = -1;         // 首次进入野值时间的更新序号
  double last_gnss_n = 0.0, last_gnss_e = 0.0;
  double last_gnss_t = -1.0;
  double last_re_dn = 0.0, last_re_de = 0.0;
  int out_count = 0;
  int count = 0, gnss_updates = 0;
  const int start_eval = static_cast<int>(0.10 * nav_steps);

  for (int k = 0; k < nav_steps; ++k)
  {
    const double t = k * kNavDt;
    Eigen::Vector3d dtheta1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d dtheta2 = Eigen::Vector3d::Zero();
    Eigen::Vector3d dv1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d dv2 = Eigen::Vector3d::Zero();
    const int half = samples_per_nav / 2;

    for (int i = 0; i < samples_per_nav; ++i)
    {
      const double ti = t + i * dt_imu;
      const Truth tr = truthAt(ti, sc);
      const Eigen::Matrix3d R = nedFromBody(tr);
      const Eigen::Vector3d f_body = R.transpose() * (tr.acc_ned - g_ned);

      Eigen::Vector3d accel_meas = f_body;
      Eigen::Vector3d gyro_meas(tr.p, tr.q, tr.r);
      // 安装失准：绕 z 轴小角度旋转（片内测量 = R_mis * 真实体轴）。
      if (misalign_deg != 0.0)
      {
        const double m = misalign_deg * kDeg2Rad;
        Eigen::Matrix3d Rm;
        // 绕 x 轴（水平轴）失准：静止重力会分到 y/z，能体现安装误差。
        Rm << 1.0, 0.0, 0.0,
              0.0, std::cos(m), -std::sin(m),
              0.0, std::sin(m),  std::cos(m);
        accel_meas = Rm * accel_meas;
        gyro_meas = Rm * gyro_meas;
      }
      for (int a = 0; a < 3; ++a)
      {
        // 加计温漂：同一升温曲线（0.3°C/s）。
        const double accel_temp_drift = accel_tempc * (25.0 + 0.3 * ti - 25.0);
        accel_meas(a) += accel_bias(a) + accel_temp_drift + accel_sigma * n01(rng);
        // 温度漂移：开机发热 0.3°C/s（30s +9°C），零偏随温度线性变化。
        const double temp_c = 25.0 + 0.3 * ti;
        const double temp_drift = gyro_tempc * (temp_c - 25.0);
        // 温补：片内温度 + 标定表（模拟；temp_comp_err 为系数误差比例）。
        const double temp_comp =
            (gyro_temp_comp != 0) ? temp_drift * (1.0 - temp_comp_err) : 0.0;
        gyro_meas(a) += gyro_bias(a) + (temp_drift - temp_comp) +
                        gyro_sigma * n01(rng);
      }
      if (sc == Scenario::Vibration || sc == Scenario::VibrationOutage)
      {
        // 80Hz、0.5g 机体 z 轴振动（电机/桨叶高频激励），验证 LPF 对振动泄漏的抑制。
        accel_meas(2) += vib_amp * kGravity * std::sin(2.0 * kPi * vib_hz * ti);
      }
      accel_meas = Eigen::Vector3d(accel_lpf[0].update(accel_meas).x(),
                                   accel_lpf[1].update(accel_meas).y(),
                                   accel_lpf[2].update(accel_meas).z());
      gyro_meas = Eigen::Vector3d(gyro_lpf[0].update(gyro_meas).x(),
                                  gyro_lpf[1].update(gyro_meas).y(),
                                  gyro_lpf[2].update(gyro_meas).z());
      for (int a = 0; a < 3; ++a)
      {
        accel_meas(a) = std::fmax(-imu.accel_fs, std::fmin(imu.accel_fs, accel_meas(a)));
        gyro_meas(a) = std::fmax(-imu.gyro_fs, std::fmin(imu.gyro_fs, gyro_meas(a)));
        accel_meas(a) = std::round(accel_meas(a) / accel_lsb) * accel_lsb;
        gyro_meas(a) = std::round(gyro_meas(a) / gyro_lsb) * gyro_lsb;
      }
      // 陀螺卡死：从 gyro_freeze_t 起输出冻结为最后一帧（故障注入）。
      if (gyro_freeze_t >= 0.0 && ti >= gyro_freeze_t)
      {
        if (!have_frozen_gyro)
        {
          frozen_gyro = gyro_meas;
          have_frozen_gyro = true;
        }
        gyro_meas = frozen_gyro;
      }
      // 加计卡死：从 accel_freeze_t 起输出冻结。
      if (accel_freeze_t >= 0.0 && ti >= accel_freeze_t)
      {
        if (!have_frozen_accel)
        {
          frozen_accel = accel_meas;
          have_frozen_accel = true;
        }
        accel_meas = frozen_accel;
      }
      // IMU 健康监测：0.2s 滑动窗口内陀螺三轴方差 < 1e-12 判卡死。
      gyro_hist[gyro_hist_idx] = gyro_meas;
      gyro_hist_idx = (gyro_hist_idx + 1) % kHealthWin;
      if (gyro_hist_count < kHealthWin)
      {
        ++gyro_hist_count;
      }
      if (gyro_hist_count == kHealthWin)
      {
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        for (int h = 0; h < kHealthWin; ++h)
        {
          mean += gyro_hist[h];
        }
        mean /= static_cast<double>(kHealthWin);
        double var_sum = 0.0;
        for (int h = 0; h < kHealthWin; ++h)
        {
          var_sum += (gyro_hist[h] - mean).squaredNorm();
        }
        var_sum /= static_cast<double>(kHealthWin);
        if (var_sum < 1.0e-12)
        {
          if (gyro_freeze_detect_local < 0.0)
          {
            gyro_freeze_detect_local = ti;
          }
          ++gyro_freeze_hits_local;
        }
      }
      // 加计健康监测：0.2s 窗口方差 < 1e-8（噪声方差约 1e-6）判卡死。
      accel_hist[accel_hist_idx] = accel_meas;
      accel_hist_idx = (accel_hist_idx + 1) % kHealthWin;
      if (accel_hist_count < kHealthWin)
      {
        ++accel_hist_count;
      }
      if (accel_hist_count == kHealthWin)
      {
        Eigen::Vector3d amean = Eigen::Vector3d::Zero();
        for (int h = 0; h < kHealthWin; ++h)
        {
          amean += accel_hist[h];
        }
        amean /= static_cast<double>(kHealthWin);
        double avar = 0.0;
        for (int h = 0; h < kHealthWin; ++h)
        {
          avar += (accel_hist[h] - amean).squaredNorm();
        }
        avar /= static_cast<double>(kHealthWin);
        if (avar < 1.0e-8)
        {
          if (accel_freeze_detect_local < 0.0)
          {
            accel_freeze_detect_local = ti;
          }
          ++accel_freeze_hits_local;
        }
      }
      if (i < half)
      {
        dtheta1 += gyro_meas * dt_imu;
        dv1 += accel_meas * dt_imu;
      }
      else
      {
        dtheta2 += gyro_meas * dt_imu;
        dv2 += accel_meas * dt_imu;
      }
    }

    // 失锁期过程噪声膨胀：让位置协方差增长更快，恢复时更信任新 GNSS。
    if (q_accel_outage > 0.0f)
    {
      const bool in_outage =
          (sc == Scenario::VibrationOutage && t >= outage_start && t < outage_end);
      ekf.set_q_accel_scale(in_outage ? q_accel_outage : q_accel);
    }
    ekf.TimeUpdateTwoSample(dtheta1.cast<float>(), dtheta2.cast<float>(),
                            dv1.cast<float>(), dv2.cast<float>(), kNavDt);

    // GNSS：按刷新率注入，测量对应 t−latency 的真值（体现定位延迟）。
    if ((k % gnss_div) == 0 &&
        !(sc == Scenario::VibrationOutage && t >= outage_start && t < outage_end))
    {
      const double tm = std::fmax(0.0, t - latency_s + gnss_time_offset_s);
      const Truth trm = truthAt(tm, sc);
      // 速度与位置的时间对齐：速度可能比位置晚/早到达，用独立延迟的真值。
      const Truth trm_vel = truthAt(std::fmax(0.0, tm - vel_extra_delay_s), sc);
      Eigen::Vector3d vel_meas = trm_vel.vel_ned;
      // 速度噪声倍数（评估速度观测质量对姿态/位置的影响）。
      for (int a = 0; a < 3; ++a)
      {
        vel_meas(a) += gnss.vel_std_mps * vel_noise_mult * n01(rng);
      }
      // 杆臂：GNSS 天线相对 IMU 的机体安装矢量，旋转到 NED 后加到真值位置。
      Eigen::Vector3d lever_ned = Eigen::Vector3d::Zero();
      if (lever_x != 0.0 || lever_y != 0.0 || lever_z != 0.0)
      {
        lever_ned = nedFromBody(trm) * Eigen::Vector3d(lever_x, lever_y, lever_z);
      }
      const double dn = trm.pos_ned(0) + lever_ned(0) +
                        gnss.pos_std_m * pos_noise_mult * n01(rng);
      const double de = trm.pos_ned(1) + lever_ned(1) +
                        gnss.pos_std_m * pos_noise_mult * n01(rng);
      const double dd = trm.pos_ned(2) + lever_ned(2) +
                       gnss.pos_std_m * vert_noise_mult * n01(rng);
      Eigen::Vector3d lla_meas = lla0;
      double dn_spike = dn;
      double dd_spike = dd;
      double de_spike = de;
      if (gnss_spike_m > 0.0 && t >= gnss_spike_t && spike_seq0 < 0)
      {
        spike_seq0 = gnss_update_seq;  // 第一次到达野值时间，记录起点序号
      }
      if (gnss_spike_m > 0.0 && spike_seq0 >= 0 &&
          gnss_update_seq >= spike_seq0 &&
          gnss_update_seq < spike_seq0 + gnss_spike_frames)
      {
        dn_spike += gnss_spike_m;  // 按 GNSS 更新帧数注入水平野值（多路径/欺骗）
        if (vert_spike_m != 0.0)
        {
          dd_spike += vert_spike_m;  // 同时注入垂直野值
        }
        if (std::getenv("IMUPERF_TRACE_REANCHOR") != nullptr)
        {
          std::printf("SPIKE,%.2f,%.1f,%.1f\n", t, dn, dn_spike);
        }
      }
      lla_meas(0) += dn_spike / kEarthR;
      lla_meas(1) += de_spike / (kEarthR * std::cos(lla0(0)));
      lla_meas(2) += -dd_spike;
      // GNSS 恢复硬重锚：位置新息超过阈值时直接把 EKF 位置/速度重置到 GNSS，
      // 避免长失锁后位置协方差与状态发散导致 NIS 拒绝所有更新。
      if (reanchor_m > 0.0)
      {
        static bool s_rcfg_printed = false;
        if (!s_rcfg_printed && std::getenv("IMUPERF_TRACE_REANCHOR") != nullptr)
        {
          std::printf("RCFG,%.1f,%d\n", reanchor_m, reanchor_streak);
          s_rcfg_printed = true;
        }
        const double est_n = (ekf.lat_rad() - lla0(0)) * kEarthR;
        const double est_e = (ekf.lon_rad() - lla0(1)) * kEarthR * std::cos(lla0(0));
        const double meas_n = (lla_meas(0) - lla0(0)) * kEarthR;
        const double meas_e = (lla_meas(1) - lla0(1)) * kEarthR * std::cos(lla0(0));
        const double dn_err = est_n - meas_n;
        const double de_err = est_e - meas_e;
        // 一致性门控：用“GNSS 位置变化率 vs GNSS 速度”识别位置跳变欺骗。
        // 失锁恢复时上一帧 GNSS 已陈旧（>0.5s）→ 跳过该门控，不阻碍真实恢复。
        bool consistent = true;
        if (std::getenv("IMUPERF_TRACE_REANCHOR") != nullptr)
        {
          const double speed_dbg = std::hypot(vel_meas(0), vel_meas(1));
          const double dpos_dbg = std::hypot(meas_n - last_gnss_n,
                                             meas_e - last_gnss_e);
          std::printf("RCHK,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", t, last_gnss_t,
                      t - last_gnss_t, speed_dbg, dpos_dbg, reanchor_consist);
        }
        if (reanchor_consist != 0 && last_gnss_t >= 0.0 &&
            (t - last_gnss_t) > 0.0 && (t - last_gnss_t) < 0.5)
        {
          const double pos_rate = std::hypot(meas_n - last_gnss_n,
                                             meas_e - last_gnss_e) /
                                  (t - last_gnss_t);
          const double speed = std::hypot(vel_meas(0), vel_meas(1));
          if (std::fabs(pos_rate - speed) > 5.0 + 0.5 * speed)
          {
            if (reanchor_consist == 2)
            {
              ++reanchor_suspicious_local;  // 只标记可疑，不阻断重锚
            }
            else
            {
              consistent = false;
            }
          }
        }
        if (std::hypot(dn_err, de_err) > reanchor_m && consistent)
        {
          if (std::getenv("IMUPERF_TRACE_REANCHOR") != nullptr)
          {
            std::printf("RCNT,%.2f,%d,%.1f,%.1f\n", t, reanchor_cnt, dn_err, de_err);
          }
          // 连续大新息且方向一致才重锚：单帧野值（多路径/欺骗）不会触发。
          if (reanchor_cnt > 0 &&
              (dn_err * last_re_dn + de_err * last_re_de) > 0.0)
          {
            ++reanchor_cnt;
          }
          else
          {
            reanchor_cnt = 1;
          }
          last_re_dn = dn_err;
          last_re_de = de_err;
          if (reanchor_cnt >= reanchor_streak)
          {
            const float roll_before = ekf.roll_rad();
            const float pitch_before = ekf.pitch_rad();
            std::printf("REANCHOR,%.2f,%.1f\n", t, std::hypot(dn_err, de_err));
            ekf.ResetPositionVelocityToGnss(vel_meas.cast<float>(), lla_meas);
            const double jump = std::fmax(
                std::fabs(static_cast<double>(ekf.roll_rad() - roll_before)),
                std::fabs(static_cast<double>(ekf.pitch_rad() - pitch_before)));
            max_att_jump_deg_local =
                std::fmax(max_att_jump_deg_local, jump * kRad2Deg);
            reanchor_cnt = 0;
          }
        }
        else
        {
          reanchor_cnt = 0;
        }
      }
      if (gnss_mode == 1 || gnss.pos_std_m >= hacc_gate_m)
      {
        // 只融合速度，不注入位置（模拟低质量 GNSS 下关闭位置观测）；
        // 速度 R 与 full 模式保持一致，差异只来自“是否融合位置”。
        // R 与噪声同步放大（否则滤波过度自信，2x 噪声就会发散）。
        const float v_std = static_cast<float>(
            std::fmax(gnss.vel_std_mps * vel_noise_mult, 0.05));
        (void)ekf.MeasurementUpdateVelocity(vel_meas.cast<float>(), v_std,
                                            static_cast<float>(v_std * 1.5));
      }
      else if (compensate_delay && gnss.latency_ms > 0.0)
      {
        // ★2026-11-01 改进验证：把真实观测年龄传给 EKF，触发延迟快照回放
        //   （四参数重载），而不是让 100ms 延迟当成“当前测量”。
        (void)ekf.MeasurementUpdateDetailed(vel_meas.cast<float>(), lla_meas, kNavDt,
                                           static_cast<float>(latency_s));
      }
      else
      {
        // 模拟固件动态 R（navigation_task.cpp:2106）：位置噪声按该 GNSS 档次的
        // hAcc 设置，并钳到固件同款下限 0.15m；此前用 EKF 默认 3m 会低估 RTK、
        // 高估低端 GNSS 的权重，与实机行为不一致。
        float h_acc = static_cast<float>(
            std::fmax(gnss.pos_std_m * pos_noise_mult, r_floor));
        if (adapt_r != 0)
        {
          // 中等新息自适应 R：新息 5-25m 时把位置 R 放大到新息的一半，
          // 让 EKF 接受部分修正，而不是被 NIS 全部拒绝。
          const double din = (lla_meas(0) - ekf.lat_rad()) * kEarthR;
          const double die = (lla_meas(1) - ekf.lon_rad()) * kEarthR *
                             std::cos(lla0(0));
          const double innov = std::hypot(din, die);
          if (innov > adapt_thresh_m)
          {
            h_acc = static_cast<float>(std::fmax(static_cast<double>(h_acc),
                                                 innov * 0.5));
          }
        }
        const float v_std = static_cast<float>(
            std::fmax(gnss.vel_std_mps * vel_noise_mult, 0.05));
        ekf.gnss_pos_ne_std_m(h_acc);
        ekf.gnss_pos_d_std_m(h_acc * 1.5f);
        ekf.gnss_vel_ne_std_mps(v_std);
        ekf.gnss_vel_d_std_mps(static_cast<float>(v_std * 1.5));
        const bfs::MeasurementUpdateResult res =
            ekf.MeasurementUpdateDetailed(vel_meas.cast<float>(), lla_meas, kNavDt);
        nis_sum += res.test_ratio;
        ++nis_cnt;
        if (!res.fused)
        {
          ++nis_rej;
        }
      }
      // ★GNSS 航迹角辅助：速度足够大时用 course 观测 yaw（无磁航向漂移的工程解）。
      if (yaw_aid != 0 && vel_meas.head<2>().norm() >= 1.0)
      {
        const float course_rad = static_cast<float>(
            std::atan2(vel_meas(1), vel_meas(0)));
        (void)ekf.MeasurementUpdateYaw(course_rad, 0.15f);
      }
      // QNH 在线校准：用 GNSS 高度与 baro 原始高度差低通估计常值偏差。
      if (baro_qnh_cal != 0)
      {
        const double baro_raw = lla0(2) - truthAt(t, sc).pos_ned(2) + baro_bias;
        if (std::getenv("IMUPERF_TRACE_QNH") != nullptr && seed == 2)
        {
          std::printf("QNHR,%.2f,%.3f,%.3f,%.3f,%.3f\n", t, baro_raw,
                      lla_meas(2), baro_raw - lla_meas(2), qnh_est);
        }
        if (!qnh_inited)
        {
          qnh_est = std::clamp(baro_raw - lla_meas(2), -15.0, 15.0);
          qnh_inited = true;
        }
        else
        {
          // 低通收敛到 resid 的均值：qnh += α·(resid − qnh)。
          // 旧实现写成 qnh += α·resid 是积分器，会一路积到 clamp 上限。
          const double resid = std::clamp(baro_raw - lla_meas(2), -15.0, 15.0);
          qnh_est = std::clamp(qnh_est + qnh_alpha * (resid - qnh_est), -15.0, 15.0);
        }
      }
      // 记录本帧 GNSS 位置，供下一帧做“位置变化率 vs 速度”一致性检查。
      last_gnss_n = (lla_meas(0) - lla0(0)) * kEarthR;
      last_gnss_e = (lla_meas(1) - lla0(1)) * kEarthR * std::cos(lla0(0));
      last_gnss_t = t;
      ++gnss_update_seq;
      ++gnss_updates;
    }

    // 气压计：25Hz 绝对高度观测（与固件 DPS310 节拍一致），仅用于高度通道。
    if (baro_mode != 0 && (k % 8) == 0 && t >= baro_start_s)
    {
      // 气流干扰：每 5 s 一个 0.5 s 尖峰（旋翼下洗/阵风），正负交替。
      double spike = 0.0;
      if (baro_spike_m != 0.0 && std::fmod(t, 5.0) < 0.5)
      {
        spike = ((static_cast<int>(t / 5.0) & 1) == 0) ? baro_spike_m : -baro_spike_m;
      }
      double alt_abs = lla0(2) - truthAt(t, sc).pos_ned(2) + baro_bias +
                             baro_std * baro_n01(baro_rng) + spike -
                             ((baro_qnh_cal != 0) ? qnh_est : 0.0);
      bool fuse = true;
      // gate=3：维护最近一次“正常”观测；尖峰期间强制接受时用最后正常值，
      // 避免把 3m 尖峰本身灌进 EKF。
      if (baro_gate == 3 &&
          std::fabs(alt_abs - static_cast<double>(ekf.alt_m())) <= 2.0)
      {
        last_good_baro = alt_abs;
      }
      if (baro_gate == 3 &&
          std::fabs(alt_abs - static_cast<double>(ekf.alt_m())) > 2.0)
      {
        if (++baro_gate_rej >= 25)
        {
          baro_gate_rej = 0;
          alt_abs = last_good_baro;  // 用最后正常值代替尖峰
        }
        else
        {
          fuse = false;
        }
      }
      else if (baro_gate != 0 &&
          std::fabs(alt_abs - static_cast<double>(ekf.alt_m())) > 2.0)
      {
        // 残差门控：尖峰/异常时跳过；连续 25 次（1 s）强制接受一次，
        // 避免 EKF 高度已偏时残差恒超限导致 baro 永久被拒（锁死）。
        if (++baro_gate_rej >= 25)
        {
          baro_gate_rej = 0;
        }
        else
        {
          fuse = false;
        }
      }
      else
      {
        baro_gate_rej = 0;
      }
      // gate=2：健康监测状态机。连续 2 次残差超限判 suspect 并冻结；
      // 连续 3 次残差正常才恢复。比单拍残差门控更能挡住气流尖峰串。
      if (baro_gate == 2)
      {
        const double resid = std::fabs(alt_abs - static_cast<double>(ekf.alt_m()));
        if (resid > 2.0)
        {
          ++baro_gate_rej;
          baro_ok_streak = 0;
        }
        else
        {
          ++baro_ok_streak;
          baro_gate_rej = 0;
        }
        if (baro_suspect)
        {
          fuse = (baro_ok_streak >= 3);
          if (fuse)
          {
            baro_suspect = false;
          }
        }
        else if (baro_gate_rej >= 2)
        {
          baro_suspect = true;
          fuse = false;
        }
      }
      // 高度保护：EKF 高度已发散（偏离初始 >100m）时不再喂 baro，
      // 防止数值爆炸继续被放大。
      if (fuse && std::fabs(static_cast<double>(ekf.alt_m()) - lla0(2)) > 100.0)
      {
        fuse = false;
      }
      if (fuse)
      {
        if (std::getenv("IMUPERF_TRACE_ALT") != nullptr && seed == 2)
        {
          std::printf("TRACE,%.2f,%.3f,%.3f,%.3f%s", t, ekf.alt_m(), qnh_est,
                      alt_abs, "\n");
        }
        (void)ekf.MeasurementUpdateBaroAltitudeDetailed(
            static_cast<float>(alt_abs), static_cast<float>(baro_std));
      }
    }

    // 每拍统一取一次估计值（trace 与指标共用）。★ 2026-11-01 踩坑：trace 分支
    // 里额外调用 roll_rad()/lat_rad() 等 getter 会让同一 seed 的结果发散（同二
    // 进制、仅环境变量不同），说明重复读取触发了状态相关路径；改为每拍只取一次。
    const float est_roll = ekf.roll_rad();
    const float est_pitch = ekf.pitch_rad();
    const Eigen::Vector3f est_vel = ekf.ned_vel_mps();
    const double est_n = (ekf.lat_rad() - lla0(0)) * kEarthR;
    const double est_e = (ekf.lon_rad() - lla0(1)) * kEarthR * std::cos(lla0(0));
    const double est_d = -(ekf.alt_m() - lla0(2));
    if (std::getenv("IMUPERF_TRACE_POS") != nullptr && seed == 1 &&
        sc == Scenario::VibrationOutage && t >= 24.9 && t <= 30.5)
    {
      const Truth tp = truthAt(t, sc);
      const Eigen::Vector3f v = ekf.ned_vel_mps();
      std::printf("POS,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d\n",
                  t, est_n, est_e, v(0), v(1), v(2), tp.pos_ned(0), tp.pos_ned(1),
                  tp.vel_ned(0), tp.vel_ned(1), reanchor_cnt);
    }
    if (!std::isfinite(est_d) || !std::isfinite(est_roll))
    {
      nan_t_local = t;
      break;
    }

    if (trace_buf != nullptr && (k % 20) == 0)
    {
      const Truth tr = truthAt(t, sc);
      trace_buf->push_back(t);
      trace_buf->push_back(tr.roll);
      trace_buf->push_back(est_roll);
      trace_buf->push_back(tr.pitch);
      trace_buf->push_back(est_pitch);
      trace_buf->push_back(tr.pos_ned(0));
      trace_buf->push_back(est_n);
      trace_buf->push_back(tr.pos_ned(1));
      trace_buf->push_back(est_e);
      trace_buf->push_back(tr.pos_ned(2));
      trace_buf->push_back(est_d);
      trace_buf->push_back(tr.vel_ned(0));
      trace_buf->push_back(est_vel(0));
      trace_buf->push_back(tr.vel_ned(1));
      trace_buf->push_back(est_vel(1));
    }

    if (k >= start_eval)
    {
      const Truth tr = truthAt(t, sc);
      sum_roll2 += std::pow(wrapPi(est_roll - tr.roll), 2);
      sum_pitch2 += std::pow(wrapPi(est_pitch - tr.pitch), 2);
      sum_vel2 += (est_vel.cast<double>() - tr.vel_ned).squaredNorm();
      const Eigen::Vector3d pos_err(est_n - tr.pos_ned(0), est_e - tr.pos_ned(1),
                                    est_d - tr.pos_ned(2));
      sum_pos2 += pos_err.squaredNorm();
      sum_alt2 += (est_d - tr.pos_ned(2)) * (est_d - tr.pos_ned(2));
      if (sc == Scenario::VibrationOutage && t >= outage_end && t < (outage_end + 5.0))
      {
        sum_post_pos2 += pos_err.squaredNorm();
        post_count++;
      }
      if (sc == Scenario::VibrationOutage && t >= outage_start && t < outage_end)
      {
        sum_out_pos2 += pos_err.squaredNorm();
        sum_out_vel2 += (est_vel.cast<double>() - tr.vel_ned).squaredNorm();
        const double alt_err = est_d - tr.pos_ned(2);
        sum_out_alt2 += alt_err * alt_err;
        out_count++;
      }
      count++;
    }
  }

  RunResult r{};
  r.gnss_updates = gnss_updates;
  if (count == 0)
  {
    r.finite = false;
    return r;
  }
  r.roll_rmse_deg = std::sqrt(sum_roll2 / count) * kRad2Deg;
  r.pitch_rmse_deg = std::sqrt(sum_pitch2 / count) * kRad2Deg;
  r.vel_rmse = std::sqrt(sum_vel2 / count);
  r.pos_rmse = std::sqrt(sum_pos2 / count);
  r.alt_rmse = std::sqrt(sum_alt2 / count);
  if (post_count > 0)
  {
    r.post_outage_rmse = std::sqrt(sum_post_pos2 / post_count);
  }
  r.nan_t = nan_t_local;
  r.max_att_jump_deg = max_att_jump_deg_local;
  r.reanchor_suspicious = reanchor_suspicious_local;
  r.gyro_freeze_detect_t = gyro_freeze_detect_local;
  r.gyro_freeze_hits = gyro_freeze_hits_local;
  r.accel_freeze_detect_t = accel_freeze_detect_local;
  r.accel_freeze_hits = accel_freeze_hits_local;
  if (nis_cnt > 0)
  {
    r.nis_mean = nis_sum / nis_cnt;
    r.nis_reject_rate = static_cast<double>(nis_rej) / nis_cnt;
  }
  r.out_count = out_count;
  if (out_count > 0)
  {
    r.out_pos_rmse = std::sqrt(sum_out_pos2 / out_count);
    r.out_vel_rmse = std::sqrt(sum_out_vel2 / out_count);
    r.out_alt_rmse = std::sqrt(sum_out_alt2 / out_count);
  }
  const Truth tr_end = truthAt((nav_steps - 1) * kNavDt, sc);
  r.yaw_drift_deg = wrapPi(ekf.yaw_rad() - tr_end.yaw) * kRad2Deg;
  r.finite = std::isfinite(r.roll_rmse_deg) && std::isfinite(r.pitch_rmse_deg) &&
             std::isfinite(r.vel_rmse) && std::isfinite(r.pos_rmse);
  if (verbose)
  {
    std::printf("  %-13s %-14s %-8s seed=%d roll=%.3f pitch=%.3f yaw=%+.1f "
                "vel=%.4f pos=%.3f\n",
                imu.name, gnss.name, scenarioName(sc), seed,
                r.roll_rmse_deg, r.pitch_rmse_deg, r.yaw_drift_deg,
                r.vel_rmse, r.pos_rmse);
  }
  return r;
}

/// 位置/速度 PASS 阈值随 GNSS 档次缩放（3σ 与实测噪声下限取大）。
bool passByGnss(const GnssSpec &g, const RunResult &r)
{
  if (!r.finite) return false;
  const double pos_limit = std::fmax(3.0 * g.pos_std_m, 0.5);
  const double vel_limit = std::fmax(3.0 * g.vel_std_mps, 0.10);
  return r.roll_rmse_deg < 1.5 && r.pitch_rmse_deg < 1.5 &&
         r.vel_rmse < vel_limit && r.pos_rmse < pos_limit;
}
} // namespace

int main()
{
  // ★连续参数优化入口：CMA-ES 等优化器通过环境变量注入参数，目标值走 OPT_RESULT。
  //   参数（7 个）：LPF 陀螺/加计、hAcc 门限、q_accel、EKF accel/gyro 白噪声、
  //   EKF 加计/陀螺零偏 Markov 驱动噪声（后 4 个是本次为“全面参数优化”新增）。
  if (const char *opt = std::getenv("IMUPERF_OPT"); opt != nullptr && opt[0] == '1')
  {
    auto envv = [](const char *k, double d) {
      const char *v = std::getenv(k);
      return v != nullptr ? std::atof(v) : d;
    };
    const double lpf_g = envv("IMUPERF_LPF_GYRO", 200.0);
    const double lpf_a = envv("IMUPERF_LPF_ACCEL", 100.0);
    const double hacc = envv("IMUPERF_HACC", 1.0e9);
    const double qa = envv("IMUPERF_Q_ACCEL", 2.0);
    const double eas = envv("IMUPERF_EKF_ACCEL_STD", 0.25);
    const double egs = envv("IMUPERF_EKF_GYRO_STD", 0.0015);
    const double eam = envv("IMUPERF_EKF_ACCEL_MARKOV", 0.010);
    const double egm = envv("IMUPERF_EKF_GYRO_MARKOV", 0.00524);
    ImuSpec spec = kIcm45686;
    spec.lpf_gyro_hz = lpf_g;
    spec.lpf_accel_hz = lpf_a;
    auto run = [&](Scenario sc, int gi, int seed) {
      return runCase(spec, kGnssSpecs[gi], sc, seed, false, nullptr, false, 80.0,
                     0, hacc, 0, static_cast<float>(qa), 0, 0.5, 0.5, 100.0,
                     101.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f, 0.0, 1, 0, 5.0, 0.0,
                     15.0, 1, 0, eas, egs, eam, egm);
    };
    double cost = 0.0;
    int n = 0;
    for (int seed = 1; seed <= 1; ++seed)
    {
      const RunResult rs = run(Scenario::Static, 2, seed);        // M8N 10Hz
      const RunResult rm = run(Scenario::Maneuver, 5, seed);      // 低端 5m 5Hz
      const RunResult rv = run(Scenario::Vibration, 2, seed);     // 0.5g/80Hz
      const RunResult ro = run(Scenario::VibrationOutage, 2, seed);  // 失锁（必含，否则过拟合）
      const RunResult ra = run(Scenario::Accel, 7, seed);  // M8N20 + 阶跃（硬约束场景）
      // 硬约束：M8N20 + accel 场景不得发散（历史教训：优化参数曾让该场景 0.2→157 m）
      if (ra.pos_rmse > 1.0 || !std::isfinite(ra.pos_rmse))
      {
        cost += 100.0;
      }
      cost += 0.5 * (rs.roll_rmse_deg + rs.pitch_rmse_deg) +
              0.5 * (rm.roll_rmse_deg + rm.pitch_rmse_deg) +
              2.0 * rs.pos_rmse + 2.0 * rm.pos_rmse +
              1.5 * 0.5 * (rv.roll_rmse_deg + rv.pitch_rmse_deg) +
              1.5 * rv.pos_rmse +
              0.5 * 0.5 * (ro.roll_rmse_deg + ro.pitch_rmse_deg) +
              0.5 * ro.pos_rmse;
      ++n;
    }
    std::printf("OPT_RESULT,%.6f\n", cost / n);
    std::exit(0);  // 优化模式：只跑 OPT 场景，跳过后续矩阵
  }

  std::printf("== IMU × GNSS 器件模型 × 真实 EKF 导航性能 ==\n");
  std::printf("imu,gnss,scenario,seed,roll_rmse_deg,pitch_rmse_deg,yaw_drift_deg,"
              "vel_rmse_mps,pos_rmse_m,gnss_updates\n");

  int tr_imu=-1, tr_gnss=-1, tr_sc=-1, tr_seed=-1;
  if (const char *e = std::getenv("IMUPERF_TRACE"))
    std::sscanf(e, "%d,%d,%d,%d", &tr_imu, &tr_gnss, &tr_sc, &tr_seed);

  const ImuSpec imus[2] = {kIcm42688p, kIcm45686};
  const Scenario scenarios[4] = {Scenario::Static, Scenario::Maneuver,
                                 Scenario::Cruise, Scenario::Vibration};
  int fail = 0, total = 0;

  // 汇总：每个 (imu, gnss, scenario) 的均值。
  std::vector<double> sum_pos(2 * kGnssCount * kScenarioCount, 0.0);
  std::vector<double> sum_vel(2 * kGnssCount * kScenarioCount, 0.0);
  std::vector<double> sum_att(2 * kGnssCount * kScenarioCount, 0.0);

  for (int si = 0; si < 2; ++si)
  {
    for (int gi = 0; gi < kGnssCount; ++gi)
    {
      for (int sc = 0; sc < 4; ++sc)
      {
        const int idx = (si * kGnssCount + gi) * kScenarioCount + sc;
        for (int seed = 1; seed <= kSeedCount; ++seed)
        {
          const bool trace_this = (si == tr_imu && gi == tr_gnss &&
                                   sc == tr_sc && seed == tr_seed);
          std::vector<double> trace_buf;
          const RunResult r = runCase(imus[si], kGnssSpecs[gi],
                                      scenarios[sc], seed, true,
                                      trace_this ? &trace_buf : nullptr);
          if (trace_this && !trace_buf.empty())
          {
            const char *out = std::getenv("IMUPERF_TRACE_OUT");
            std::ofstream of(out ? out : "trace_out.csv");
            for (size_t i = 0; i + 14 < trace_buf.size(); i += 15)
            {
              for (int c = 0; c < 15; ++c)
                of << (c ? "," : "") << trace_buf[i + c];
              of << "\n";
            }
          }
          std::printf("%s,%s,%s,%d,%.4f,%.4f,%+.2f,%.4f,%.4f,%d\n",
                      imus[si].name, kGnssSpecs[gi].name,
                      scenarioName(scenarios[sc]), seed, r.roll_rmse_deg,
                      r.pitch_rmse_deg, r.yaw_drift_deg, r.vel_rmse,
                      r.pos_rmse, r.gnss_updates);
          ++total;
          if (!passByGnss(kGnssSpecs[gi], r))
          {
            std::printf("[FAIL] %s %s %s seed=%d 超限\n", imus[si].name,
                        kGnssSpecs[gi].name, scenarioName(scenarios[sc]), seed);
            ++fail;
          }
          sum_pos[idx] += r.pos_rmse;
          sum_vel[idx] += r.vel_rmse;
          sum_att[idx] += 0.5 * (r.roll_rmse_deg + r.pitch_rmse_deg);
        }
      }
    }
  }

  std::printf("\n== 汇总（均值，%d seeds）==\n", kSeedCount);
  std::printf("%-13s %-14s %-8s %8s %8s %8s\n", "imu", "gnss", "scenario",
              "att_deg", "vel_mps", "pos_m");
  for (int si = 0; si < 2; ++si)
  {
    for (int gi = 0; gi < kGnssCount; ++gi)
    {
      for (int sc = 0; sc < 4; ++sc)
      {
        const int idx = (si * kGnssCount + gi) * kScenarioCount + sc;
        std::printf("%-13s %-14s %-8s %8.4f %8.4f %8.4f\n", imus[si].name,
                    kGnssSpecs[gi].name, scenarioName(scenarios[sc]),
                    sum_att[idx] / kSeedCount, sum_vel[idx] / kSeedCount,
                    sum_pos[idx] / kSeedCount);
      }
    }
  }

  // ★2026-11-01 延迟补偿 A/B：仅对有观测延迟的 GNSS 档跑 comp=off / comp=on。
  if (const char *ab = std::getenv("IMUPERF_DELAY_AB"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== 延迟补偿 A/B（age=latency，仅延迟档）==\n");
    std::printf("imu,gnss,scenario,seed,pos_off,pos_on,vel_off,vel_on\n");
    for (int si = 0; si < 2; ++si)
    {
      for (int gi = 0; gi < kGnssCount; ++gi)
      {
        if (kGnssSpecs[gi].latency_ms <= 0.0) { continue; }
        for (int sc = 0; sc < 4; ++sc)
        {
          double sum_off = 0.0, sum_on = 0.0, vel_off = 0.0, vel_on = 0.0;
          for (int seed = 1; seed <= kSeedCount; ++seed)
          {
            const RunResult ro = runCase(imus[si], kGnssSpecs[gi], scenarios[sc], seed, false);
            const RunResult rn = runCase(imus[si], kGnssSpecs[gi], scenarios[sc], seed, false, nullptr, true);
            std::printf("%s,%s,%s,%d,%.4f,%.4f,%.4f,%.4f\n", imus[si].name,
                        kGnssSpecs[gi].name, scenarioName(scenarios[sc]), seed,
                        ro.pos_rmse, rn.pos_rmse, ro.vel_rmse, rn.vel_rmse);
            sum_off += ro.pos_rmse; sum_on += rn.pos_rmse;
            vel_off += ro.vel_rmse; vel_on += rn.vel_rmse;
          }
          const double n = static_cast<double>(kSeedCount);
          std::printf("[AB] %s %s %s: pos %.4f -> %.4f (%+.1f%%), vel %.4f -> %.4f\n",
                      imus[si].name, kGnssSpecs[gi].name, scenarioName(scenarios[sc]),
                      sum_off / n, sum_on / n, 100.0 * (sum_on - sum_off) / sum_off,
                      vel_off / n, vel_on / n);
        }
      }
    }
  }

  // ★振动场景下的 LPF 带宽对比：80Hz/0.5g 激励，比较 50/100/400Hz 一阶 LPF 的残余误差。
  if (const char *lpf_env = std::getenv("IMUPERF_VIB_LPF_HZ"); lpf_env != nullptr)
  {
    const float lpf_hz = static_cast<float>(std::atof(lpf_env));
    ImuSpec wide = kIcm45686;
    wide.lpf_accel_hz = lpf_hz;
    wide.lpf_gyro_hz = lpf_hz;
    std::printf("== vibration 80Hz/0.5g, LPF %.0fHz, M8N_10Hz ==\n", lpf_hz);
    double sum_att = 0.0, sum_vel = 0.0, sum_pos = 0.0;
    for (int seed = 1; seed <= kSeedCount; ++seed)
    {
      const RunResult r = runCase(wide, kGnssSpecs[2], Scenario::Vibration, seed, false);
      std::printf("seed=%d roll=%.3f pitch=%.3f vel=%.4f pos=%.4f\n", seed,
                  r.roll_rmse_deg, r.pitch_rmse_deg, r.vel_rmse, r.pos_rmse);
      sum_att += 0.5 * (r.roll_rmse_deg + r.pitch_rmse_deg);
      sum_vel += r.vel_rmse;
      sum_pos += r.pos_rmse;
    }
    const double n = static_cast<double>(kSeedCount);
    std::printf("[VIB] LPF %.0fHz: att=%.3f deg vel=%.4f m/s pos=%.4f m\n",
                lpf_hz, sum_att / n, sum_vel / n, sum_pos / n);
  }

  if (const char *ab = std::getenv("IMUPERF_OUTAGE_AB"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== GNSS 10-20s outage + 80Hz/0.5g vibration, M8N_10Hz ==\n");
    std::printf("lpf_hz,seed,imu,out_pos_rmse,out_vel_rmse\n");
    for (float hz : {50.0f, 100.0f, 400.0f})
    {
      ImuSpec spec = kIcm45686;
      spec.lpf_accel_hz = hz;
      spec.lpf_gyro_hz = hz;
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(spec, kGnssSpecs[2], Scenario::VibrationOutage, seed, false);
        std::printf("%.0f,%d,45686,%.4f,%.4f\n", hz, seed, r.out_pos_rmse, r.out_vel_rmse);
      }
    }
  }

  // ★零偏敏感性：固定其余参数，只看陀螺 1σ 零偏对姿态/位置的影响（M8N_10Hz + 机动）。
  if (const char *ab = std::getenv("IMUPERF_BIAS_AB"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== gyro bias sensitivity (M8N_10Hz, maneuver) ==\n");
    std::printf("gyro_bias_dps,seed,att_rmse_deg,pos_rmse_m\n");
    for (float dps : {0.05f, 0.1f, 0.2f, 0.4f, 0.8f})
    {
      ImuSpec spec = kIcm45686;
      spec.gyro_bias_1sigma = dps * static_cast<double>(kDeg2Rad);
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(spec, kGnssSpecs[2], Scenario::Maneuver, seed, false);
        std::printf("%.2f,%d,%.3f,%.4f\n", dps, seed,
                    0.5 * (r.roll_rmse_deg + r.pitch_rmse_deg), r.pos_rmse);
      }
    }
  }

  // ★振动频率扫描：20/50/80/120/200/400Hz × 0.5g，M8N_10Hz + 45686。
  if (const char *ab = std::getenv("IMUPERF_VIB_SWEEP"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== vibration frequency sweep (0.5g, M8N_10Hz, 45686) ==\n");
    std::printf("vib_hz,seed,att_rmse_deg,pos_rmse_m\n");
    for (double hz : {20.0, 50.0, 80.0, 120.0, 200.0, 400.0})
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::Vibration,
                                    seed, false, nullptr, false, hz);
        std::printf("%.0f,%d,%.3f,%.4f\n", hz, seed,
                    0.5 * (r.roll_rmse_deg + r.pitch_rmse_deg), r.pos_rmse);
      }
    }
  }

  // ★GNSS 质量门控假设验证：低端 5m GNSS + 机动，比较全量更新与只更新速度。
  if (const char *ab = std::getenv("IMUPERF_GNSS_GATE"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== GNSS gate: LowEnd_5Hz maneuver (full vs velocity-only) ==\n");
    std::printf("mode,seed,att_rmse_deg,pos_rmse_m\n");
    for (int mode = 0; mode <= 1; ++mode)
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[5], Scenario::Maneuver,
                                    seed, false, nullptr, false, 80.0, mode);
        std::printf("%s,%d,%.3f,%.4f\n", mode == 0 ? "full" : "vel_only", seed,
                    0.5 * (r.roll_rmse_deg + r.pitch_rmse_deg), r.pos_rmse);
      }
    }
  }

  // ★hAcc 门控阈值扫描：低端 GNSS 位置标准差 5m，超过阈值时只融合速度。
  if (const char *ab = std::getenv("IMUPERF_HACC_SWEEP"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== hAcc gate sweep: LowEnd_5Hz maneuver ==\n");
    std::printf("gate_m,seed,att_rmse_deg,pos_rmse_m\n");
    for (double gate : {2.0, 3.0, 5.0, 6.0, 1.0e9})
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[5], Scenario::Maneuver,
                                    seed, false, nullptr, false, 80.0, 0, gate);
        std::printf("%.1f,%d,%.3f,%.4f\n", gate, seed,
                    0.5 * (r.roll_rmse_deg + r.pitch_rmse_deg), r.pos_rmse);
      }
    }
  }

  // ★阈值 3m 的边界验证：M8N_5Hz(2.5m) 不应被误关，LowEnd_5Hz(5m) 应关闭位置。
  if (const char *ab = std::getenv("IMUPERF_HACC_VALID"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== hAcc gate 3m validation (maneuver) ==\n");
    std::printf("gnss,gate_m,seed,att_rmse_deg,pos_rmse_m\n");
    for (int gi : {3, 5})
    {
      for (double gate : {3.0, 1.0e9})
      {
        for (int seed = 1; seed <= kSeedCount; ++seed)
        {
          const RunResult r = runCase(kIcm45686, kGnssSpecs[gi], Scenario::Maneuver,
                                      seed, false, nullptr, false, 80.0, 0, gate);
          std::printf("%s,%.0f,%d,%.3f,%.4f\n", kGnssSpecs[gi].name, gate, seed,
                      0.5 * (r.roll_rmse_deg + r.pitch_rmse_deg), r.pos_rmse);
        }
      }
    }
  }

  // ★LPF 降延迟 A/B：旧档 100/50Hz vs 新档 200/100Hz（45686 参数，M8N_10Hz）。
  if (const char *ab = std::getenv("IMUPERF_LPF_AB"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== LPF bandwidth A/B: old 100/50Hz vs new 200/100Hz ==\n");
    std::printf("lpf,scenario,seed,att_rmse_deg,pos_rmse_m\n");
    ImuSpec old_spec = kIcm45686;
    old_spec.lpf_gyro_hz = 100.0;
    old_spec.lpf_accel_hz = 50.0;
    ImuSpec new_spec = kIcm45686;
    new_spec.lpf_gyro_hz = 200.0;
    new_spec.lpf_accel_hz = 100.0;
    for (int sc = 0; sc <= 1; ++sc)
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult ro = runCase(old_spec, kGnssSpecs[2], scenarios[sc], seed, false);
        const RunResult rn = runCase(new_spec, kGnssSpecs[2], scenarios[sc], seed, false);
        std::printf("old,%s,%d,%.3f,%.4f\n", scenarioName(scenarios[sc]), seed,
                    0.5 * (ro.roll_rmse_deg + ro.pitch_rmse_deg), ro.pos_rmse);
        std::printf("new,%s,%d,%.3f,%.4f\n", scenarioName(scenarios[sc]), seed,
                    0.5 * (rn.roll_rmse_deg + rn.pitch_rmse_deg), rn.pos_rmse);
      }
    }
  }

  // ★GNSS 波特率实验：UBX-NAV-PVT 约 100B，115200≈8.7ms、921600≈1.1ms。
  if (const char *ab = std::getenv("IMUPERF_UART_AB"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== UART baud effect on GNSS age (NAV-PVT ~100B, cruise) ==\n");
    std::printf("gnss,solve_ms,baud,uart_ms,age_ms,comp,seed,pos_rmse_m\n");
    for (int gi : {0, 2}) // 0=RTK_10Hz(2cm)、2=M8N_10Hz(1.5m)
    {
      for (double solve : {10.0, 26.0})
      {
        for (double baud : {115200.0, 921600.0})
        {
          const double uart_ms = 100.0 * 10.0 / baud * 1000.0; // 10 bit/byte（8N1）
          for (int comp = 0; comp <= 1; ++comp)
          {
            for (int seed = 1; seed <= kSeedCount; ++seed)
            {
              ImuSpec spec = kIcm45686;
              GnssSpec g = kGnssSpecs[gi];
              g.latency_ms = solve + uart_ms;
              const RunResult r = runCase(spec, g, Scenario::Cruise, seed, false,
                                          nullptr, comp != 0);
              std::printf("%s,%.0f,%.0f,%.2f,%.2f,%d,%d,%.4f\n", kGnssSpecs[gi].name,
                          solve, baud, uart_ms, g.latency_ms, comp, seed, r.pos_rmse);
            }
          }
        }
      }
    }
  }

  // ★连续参数优化入口：CMA-ES 等优化器通过环境变量注入参数，目标值走 OPT_RESULT。
  //   参数（7 个）：LPF 陀螺/加计、hAcc 门限、q_accel、EKF accel/gyro 白噪声、
  //   EKF 加计/陀螺零偏 Markov 驱动噪声（后 4 个是本次为“全面参数优化”新增）。
  if (const char *opt = std::getenv("IMUPERF_OPT"); opt != nullptr && opt[0] == '1')
  {
    auto envv = [](const char *k, double d) {
      const char *v = std::getenv(k);
      return v != nullptr ? std::atof(v) : d;
    };
    const double lpf_g = envv("IMUPERF_LPF_GYRO", 200.0);
    const double lpf_a = envv("IMUPERF_LPF_ACCEL", 100.0);
    const double hacc = envv("IMUPERF_HACC", 1.0e9);
    const double qa = envv("IMUPERF_Q_ACCEL", 2.0);
    const double eas = envv("IMUPERF_EKF_ACCEL_STD", 0.25);
    const double egs = envv("IMUPERF_EKF_GYRO_STD", 0.0015);
    const double eam = envv("IMUPERF_EKF_ACCEL_MARKOV", 0.010);
    const double egm = envv("IMUPERF_EKF_GYRO_MARKOV", 0.00524);
    ImuSpec spec = kIcm45686;
    spec.lpf_gyro_hz = lpf_g;
    spec.lpf_accel_hz = lpf_a;
    auto run = [&](Scenario sc, int gi, int seed) {
      return runCase(spec, kGnssSpecs[gi], sc, seed, false, nullptr, false, 80.0,
                     0, hacc, 0, static_cast<float>(qa), 0, 0.5, 0.5, 100.0,
                     101.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f, 0.0, 1, 0, 5.0, 0.0,
                     15.0, 1, 0, eas, egs, eam, egm);
    };
    double cost = 0.0;
    int n = 0;
    for (int seed = 1; seed <= 1; ++seed)
    {
      const RunResult rs = run(Scenario::Static, 2, seed);        // M8N 10Hz
      const RunResult rm = run(Scenario::Maneuver, 5, seed);      // 低端 5m 5Hz
      const RunResult rv = run(Scenario::Vibration, 2, seed);     // 0.5g/80Hz
      const RunResult ro = run(Scenario::VibrationOutage, 2, seed);  // 失锁（必含，否则过拟合）
      cost += 0.5 * (rs.roll_rmse_deg + rs.pitch_rmse_deg) +
              0.5 * (rm.roll_rmse_deg + rm.pitch_rmse_deg) +
              2.0 * rs.pos_rmse + 2.0 * rm.pos_rmse +
              1.5 * 0.5 * (rv.roll_rmse_deg + rv.pitch_rmse_deg) +
              1.5 * rv.pos_rmse +
              0.5 * 0.5 * (ro.roll_rmse_deg + ro.pitch_rmse_deg) +
              0.5 * ro.pos_rmse;
      ++n;
    }
    std::printf("OPT_RESULT,%.6f\n", cost / n);
    std::exit(0);  // 优化模式：只跑 OPT 场景，跳过后续矩阵
  }

  // ★GNSS 航迹角辅助 yaw：cruise（1m/s）下对比无辅助/有辅助的 yaw 漂移与姿态。
  if (const char *ab = std::getenv("IMUPERF_YAW_AID"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== GNSS course yaw aiding (cruise, M8N_10Hz) ==\n");
    std::printf("aid,seed,att_rmse_deg,pos_rmse_m,yaw_drift_deg\n");
    for (int aid = 0; aid <= 1; ++aid)
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::Cruise,
                                    seed, false, nullptr, false, 80.0, 0, 1.0e9, aid);
        std::printf("%d,%d,%.3f,%.4f,%+.2f\n", aid, seed,
                    0.5 * (r.roll_rmse_deg + r.pitch_rmse_deg), r.pos_rmse,
                    r.yaw_drift_deg);
      }
    }
  }

  // ★EKF 过程噪声 q_accel_scale 敏感性：maneuver 与 vibration 两场景。
  if (const char *ab = std::getenv("IMUPERF_Q_AB"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== EKF q_accel_scale sensitivity (M8N_10Hz) ==\n");
    std::printf("q,scenario,seed,att_rmse_deg,pos_rmse_m\n");
    for (float q : {0.5f, 1.0f, 1.5f, 2.0f, 3.0f})
    {
      for (int sc = 1; sc <= 3; sc += 2) // maneuver=1, vibration=3
      {
        for (int seed = 1; seed <= kSeedCount; ++seed)
        {
          const RunResult r = runCase(kIcm45686, kGnssSpecs[2], scenarios[sc],
                                      seed, false, nullptr, false, 80.0, 0, 1.0e9, 0, q);
          std::printf("%.1f,%s,%d,%.3f,%.4f\n", q, scenarioName(scenarios[sc]), seed,
                      0.5 * (r.roll_rmse_deg + r.pitch_rmse_deg), r.pos_rmse);
        }
      }
    }
  }

  // ★气压计辅助：GNSS 10-20s 失锁窗口，比较无 baro / baro(0.5m,25Hz) 的高度与位置。
  if (const char *ab = std::getenv("IMUPERF_BARO_AB"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== baro aiding during GNSS outage (VibrationOutage) ==\n");
    std::printf("baro,seed,out_pos_rmse,out_vel_rmse,out_alt_rmse\n");
    for (int baro = 0; baro <= 1; ++baro)
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::VibrationOutage,
                                    seed, false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f, baro);
        std::printf("%d,%d,%.4f,%.4f,%.4f\n", baro, seed,
                    r.out_pos_rmse, r.out_vel_rmse, r.out_alt_rmse);
      }
    }
  }

  // ★baro NaN 定位：baro=1 下振动 0 / 0.5g × 失锁场景，输出 NaN 发生时刻。
  if (const char *ab = std::getenv("IMUPERF_BARO_NAN"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== baro NaN isolation (baro=1, VibrationOutage) ==\n");
    std::printf("vib_amp,seed,alt_rmse,nan_t\n");
    for (double amp : {0.0, 0.5})
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::VibrationOutage,
                                    seed, false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                    1, 0.5, amp);
        std::printf("%.1f,%d,%.4f,%.2f\n", amp, seed, r.alt_rmse, r.nan_t);
      }
    }
  }

  // ★长失锁 + baro：5-30s GNSS 失锁（25s），比较无 baro / baro(0.5m,25Hz)。
  if (const char *ab = std::getenv("IMUPERF_BARO_LONG"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== baro noise scan: long GNSS outage 5-30s ==\n");
    std::printf("baro_std,seed,alt_rmse,out_alt_rmse,out_vel_rmse,out_pos_rmse\n");
    for (double bstd : {0.2, 0.5, 1.0, 2.0})
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::VibrationOutage,
                                    seed, false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                    1, bstd, 0.5, 5.0, 30.0);
        std::printf("%.1f,%d,%.4f,%.4f,%.4f,%.4f\n", bstd, seed,
                    r.alt_rmse, r.out_alt_rmse, r.out_vel_rmse, r.out_pos_rmse);
      }
    }
  }

  // ★QNH 偏置：baro 常值偏差 0/2/5/8m 对在线与失锁高度的影响。
  if (const char *ab = std::getenv("IMUPERF_BARO_BIAS"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== baro QNH bias vs altitude (static & 25s outage) ==\n");
    std::printf("bias_m,scenario,seed,alt_rmse,out_alt_rmse\n");
    for (double bias : {0.0, 2.0, 5.0, 8.0})
    {
      for (int scn = 0; scn <= 1; ++scn)
      {
        const Scenario scn_e = (scn == 0) ? Scenario::Static : Scenario::VibrationOutage;
        const double os = (scn == 0) ? 100.0 : 5.0;
        const double oe = (scn == 0) ? 101.0 : 30.0;
        for (int seed = 1; seed <= kSeedCount; ++seed)
        {
          const RunResult r = runCase(kIcm45686, kGnssSpecs[2], scn_e, seed,
                                      false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                      1, 0.2, 0.5, os, oe, bias);
          std::printf("%.1f,%s,%d,%.4f,%.4f\n", bias, scenarioName(scn_e), seed,
                      r.alt_rmse, r.out_alt_rmse);
        }
      }
    }
  }

  // ★QNH 在线校准：bias=5m，cal=0/1 × static/失锁。
  if (const char *ab = std::getenv("IMUPERF_QNH_CAL"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== QNH online calibration (bias=5m) ==\n");
    std::printf("cal,scenario,seed,alt_rmse,out_alt_rmse\n");
    for (int cal = 0; cal <= 1; ++cal)
    {
      for (int scn = 0; scn <= 1; ++scn)
      {
        const Scenario scn_e = (scn == 0) ? Scenario::Static : Scenario::VibrationOutage;
        const double os = (scn == 0) ? 100.0 : 5.0;
        const double oe = (scn == 0) ? 101.0 : 30.0;
        for (int seed = 1; seed <= kSeedCount; ++seed)
        {
          const RunResult r = runCase(kIcm45686, kGnssSpecs[2], scn_e, seed,
                                      false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                      1, 0.2, 0.5, os, oe, 5.0, cal);
          std::printf("%d,%s,%d,%.4f,%.4f\n", cal, scenarioName(scn_e), seed,
                      r.alt_rmse, r.out_alt_rmse);
        }
      }
    }
  }

  // ★QNH 校准修正版：首次 GNSS 高度差直接初始化 + baro 延迟 2s 融合。
  if (const char *ab = std::getenv("IMUPERF_QNH_CAL2"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== QNH cal v2: init from first GNSS + baro delay 2s (bias=5m) ==\n");
    std::printf("cal,scenario,seed,alt_rmse,out_alt_rmse\n");
    for (int cal = 0; cal <= 1; ++cal)
    {
      for (int scn = 0; scn <= 1; ++scn)
      {
        const Scenario scn_e = (scn == 0) ? Scenario::Static : Scenario::VibrationOutage;
        const double os = (scn == 0) ? 100.0 : 5.0;
        const double oe = (scn == 0) ? 101.0 : 30.0;
        for (int seed = 1; seed <= kSeedCount; ++seed)
        {
          const RunResult r = runCase(kIcm45686, kGnssSpecs[2], scn_e, seed,
                                      false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                      1, 0.2, 0.5, os, oe, 5.0, cal, cal ? 2.0 : 0.0);
          std::printf("%d,%s,%d,%.4f,%.4f\n", cal, scenarioName(scn_e), seed,
                      r.alt_rmse, r.out_alt_rmse);
        }
      }
    }
  }

  // ★气流尖峰：3m/0.5s 每 5s，对比无门控 vs 残差门控（|残差|>2m 跳过）。
  if (const char *ab = std::getenv("IMUPERF_BARO_SPIKE"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== baro airflow spike 3m/0.5s: no gate vs residual gate ==\n");
    std::printf("gate,scenario,seed,alt_rmse,out_alt_rmse\n");
    for (int gate = 0; gate <= 1; ++gate)
    {
      for (int scn = 0; scn <= 1; ++scn)
      {
        const Scenario scn_e = (scn == 0) ? Scenario::Static : Scenario::VibrationOutage;
        const double os = (scn == 0) ? 100.0 : 5.0;
        const double oe = (scn == 0) ? 101.0 : 30.0;
        for (int seed = 1; seed <= kSeedCount; ++seed)
        {
          const RunResult r = runCase(kIcm45686, kGnssSpecs[2], scn_e, seed,
                                      false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                      1, 0.2, 0.5, os, oe, 0.0, 0, 0.0, 3.0, gate);
          std::printf("%d,%s,%d,%.4f,%.4f\n", gate, scenarioName(scn_e), seed,
                      r.alt_rmse, r.out_alt_rmse);
        }
      }
    }
  }

  // ★QNH α 扫描：低通系数对高度残余的影响（bias=5m）。
  if (const char *ab = std::getenv("IMUPERF_QNH_ALPHA"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== QNH alpha scan (bias=5m) ==\n");
    std::printf("alpha,scenario,seed,alt_rmse,out_alt_rmse\n");
    for (double alpha : {0.005, 0.01, 0.02, 0.05})
    {
      for (int scn = 0; scn <= 1; ++scn)
      {
        const Scenario scn_e = (scn == 0) ? Scenario::Static : Scenario::VibrationOutage;
        const double os = (scn == 0) ? 100.0 : 5.0;
        const double oe = (scn == 0) ? 101.0 : 30.0;
        for (int seed = 1; seed <= kSeedCount; ++seed)
        {
          const RunResult r = runCase(kIcm45686, kGnssSpecs[2], scn_e, seed,
                                      false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                      1, 0.2, 0.5, os, oe, 5.0, 1, 2.0, 0.0, 0, alpha);
          std::printf("%.3f,%s,%d,%.4f,%.4f\n", alpha, scenarioName(scn_e), seed,
                      r.alt_rmse, r.out_alt_rmse);
        }
      }
    }
  }

  // ★baro 门控对比：0=无、1=残差门控、2=健康监测冻结。
  if (const char *ab = std::getenv("IMUPERF_BARO_GATE"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== baro gate 0/1/2, airflow spike 3m/0.5s ==\n");
    std::printf("gate,scenario,seed,alt_rmse,out_alt_rmse\n");
    for (int gate : {0, 1, 3})
    {
      for (int scn = 0; scn <= 1; ++scn)
      {
        const Scenario scn_e = (scn == 0) ? Scenario::Static : Scenario::VibrationOutage;
        const double os = (scn == 0) ? 100.0 : 5.0;
        const double oe = (scn == 0) ? 101.0 : 30.0;
        for (int seed = 1; seed <= kSeedCount; ++seed)
        {
          const RunResult r = runCase(kIcm45686, kGnssSpecs[2], scn_e, seed,
                                      false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                      1, 0.2, 0.5, os, oe, 0.0, 0, 0.0, 3.0, gate);
          std::printf("%d,%s,%d,%.4f,%.4f\n", gate, scenarioName(scn_e), seed,
                      r.alt_rmse, r.out_alt_rmse);
        }
      }
    }
  }

  // ★失锁恢复：失锁期 Q 膨胀 vs 基准，比较恢复后 5s 位置误差。
  if (const char *ab = std::getenv("IMUPERF_OUTAGE_RECOVERY"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== GNSS outage recovery: q_accel_outage vs baseline ==\n");
    std::printf("q_out,seed,out_pos_rmse,post_outage_rmse\n");
    for (float qo : {0.0f, 5.0f, 10.0f})
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::VibrationOutage,
                                    seed, false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                    0, 0.5, 0.5, 5.0, 25.0, 0.0, 0, 0.0, 0.0, 0, 0.05, qo);
        std::printf("%.1f,%d,%.4f,%.4f\n", qo, seed,
                    r.out_pos_rmse, r.post_outage_rmse);
      }
    }
  }

  // ★长失锁恢复：硬重锚阈值 0/50/100m，25s 失锁。
  if (const char *ab = std::getenv("IMUPERF_REANCHOR"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== GNSS outage hard reanchor (25s outage) ==\n");
    std::printf("reanchor_m,seed,out_pos_rmse,post_outage_rmse\n");
    for (double rm : {0.0, 20.0, 30.0, 50.0})
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::VibrationOutage,
                                    seed, false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                    0, 0.5, 0.5, 5.0, 25.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f, rm);
        std::printf("%.0f,%d,%.4f,%.4f\n", rm, seed,
                    r.out_pos_rmse, r.post_outage_rmse);
      }
    }
  }

  // ★连续大新息重锚：失锁恢复 streak 1 vs 3。
  if (const char *ab = std::getenv("IMUPERF_REANCHOR2"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== reanchor consistency/streak (25s outage) ==\n");
    std::printf("cons,streak,seed,out_pos_rmse,post_outage_rmse,max_att_jump_deg\n");
    for (int cons : {0, 1})
    {
      for (int st : {1, 2})
      {
        for (int seed = 1; seed <= kSeedCount; ++seed)
        {
          const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::VibrationOutage,
                                      seed, false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                      0, 0.5, 0.5, 5.0, 25.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f,
                                      25.0, st, 0, 0.0, 0.0, 15.0, 1, cons);
          std::printf("%d,%d,%d,%.4f,%.4f,%.3f\n", cons, st, seed,
                      r.out_pos_rmse, r.post_outage_rmse, r.max_att_jump_deg);
        }
      }
    }
  }

  // ★野值鲁棒性：static + 单帧 100m 野值，streak 1 vs 3。
  if (const char *ab = std::getenv("IMUPERF_GNSS_SPIKE"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== GNSS spike 100m robustness (static) ==\n");
    std::printf("streak,seed,pos_rmse,alt_rmse\n");
    for (int st : {1, 2, 3})
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::Static,
                                    seed, false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                    0, 0.5, 0.5, 100.0, 101.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f,
                                    25.0, st, 100.0, 15.0);
        std::printf("%d,%d,%.4f,%.4f\n", st, seed, r.pos_rmse, r.alt_rmse);
      }
    }
  }

  // ★R 自适应：中等新息放大位置 R，对比固定 R（均关闭硬重锚）。
  if (const char *ab = std::getenv("IMUPERF_ADAPT_R"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== adaptive R (25s outage, no reanchor) ==\n");
    std::printf("adapt,seed,out_pos_rmse,post_outage_rmse\n");
    for (int ad : {0, 1})
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::VibrationOutage,
                                    seed, false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                    0, 0.5, 0.5, 5.0, 25.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f,
                                    0.0, 2, 0.0, 15.0, ad, 5.0);
        std::printf("%d,%d,%.4f,%.4f\n", ad, seed,
                    r.out_pos_rmse, r.post_outage_rmse);
      }
    }
  }

  // ★连续野值/欺骗：streak=2 在连续 1/3/5 帧 100m 偏移下是否误重锚。
  if (const char *ab = std::getenv("IMUPERF_GNSS_DECEPTION"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== GNSS deception: 3x100m spikes, consist mode ==\n");
    std::printf("cons,seed,pos_rmse,alt_rmse,suspicious\n");
    for (int cons : {0, 2})
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::Static,
                                    seed, false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                    0, 0.5, 0.5, 100.0, 101.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f,
                                    25.0, 2, 0, 5.0, 100.0, 15.0, 3, cons);
        std::printf("%d,%d,%.4f,%.4f,%d\n", cons, seed, r.pos_rmse,
                    r.alt_rmse, r.reanchor_suspicious);
      }
    }
  }

  // ★NIS 一致性：不同 GNSS 档次的 NIS 均值（6 维量测应≈6）与拒绝率。
  if (const char *ab = std::getenv("IMUPERF_NIS"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== NIS consistency by GNSS class (static) ==\n");
    std::printf("gnss,seed,nis_mean,reject_rate,pos_rmse\n");
    for (int gi : {0, 2, 5})
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[gi], Scenario::Static, seed,
                                    false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                    0, 0.5, 0.5, 100.0, 101.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f,
                                    0.0, 1, 0, 0.0, 0.0, 15.0, 1, 0);
        std::printf("%s,%d,%.3f,%.4f,%.4f\n", kGnssSpecs[gi].name, seed,
                    r.nis_mean, r.nis_reject_rate, r.pos_rmse);
      }
    }
  }

  // ★R 地板：0.05/0.10/0.15 × RTK/M8N，看 NIS 一致性与位置精度。
  if (const char *ab = std::getenv("IMUPERF_R_FLOOR"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== R floor scan (static) ==\n");
    std::printf("floor,gnss,seed,nis_mean,pos_rmse\n");
    for (double fl : {0.05, 0.10, 0.15})
    {
      for (int gi : {0, 2})
      {
        for (int seed = 1; seed <= kSeedCount; ++seed)
        {
          const RunResult r = runCase(kIcm45686, kGnssSpecs[gi], Scenario::Static, seed,
                                      false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                      0, 0.5, 0.5, 100.0, 101.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f,
                                      0.0, 1, 0, 0.0, 0.0, 15.0, 1, 0, fl);
          std::printf("%.2f,%s,%d,%.3f,%.4f\n", fl, kGnssSpecs[gi].name, seed,
                      r.nis_mean, r.pos_rmse);
        }
      }
    }
  }

  // ★温度零偏：0/0.001/0.002 rad/s/°C（30s +9°C），看姿态与零偏估计。
  if (const char *ab = std::getenv("IMUPERF_TEMP_BIAS"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== gyro temperature bias (static, +9C over 30s) ==\n");
    std::printf("tempc,seed,att_rmse_deg,pos_rmse\n");
    for (double tc : {0.0, 0.001, 0.002})
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::Static, seed,
                                    false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                    0, 0.5, 0.5, 100.0, 101.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f,
                                    0.0, 1, 0, 0.0, 0.0, 15.0, 1, 0, 0.15, tc);
        std::printf("%.3f,%d,%.3f,%.4f\n", tc, seed,
                    0.5 * (r.roll_rmse_deg + r.pitch_rmse_deg), r.pos_rmse);
      }
    }
  }

  // ★温度补偿验证：0.001 rad/s/°C 温漂，comp 0/1（系数误差 0/20%）。
  if (const char *ab = std::getenv("IMUPERF_TEMP_COMP"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== gyro temperature compensation (tc=0.001) ==\n");
    std::printf("comp,err,seed,att_rmse_deg,pos_rmse\n");
    for (int comp : {0, 1})
    {
      for (double err : {0.0, 0.2})
      {
        for (int seed = 1; seed <= kSeedCount; ++seed)
        {
          const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::Static, seed,
                                      false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                      0, 0.5, 0.5, 100.0, 101.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f,
                                      0.0, 1, 0, 0.0, 0.0, 15.0, 1, 0, 0.15, 0.001, comp, err);
          std::printf("%d,%.1f,%d,%.3f,%.4f\n", comp, err, seed,
                      0.5 * (r.roll_rmse_deg + r.pitch_rmse_deg), r.pos_rmse);
        }
      }
    }
  }

  // ★安装失准角：0/0.5/1/2° 绕 z，看姿态与位置误差。
  if (const char *ab = std::getenv("IMUPERF_MISALIGN"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== IMU misalignment (static) ==\n");
    std::printf("mis_deg,seed,att_rmse_deg,pos_rmse\n");
    for (double mis : {0.0, 0.5, 1.0, 2.0})
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::Static, seed,
                                    false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                    0, 0.5, 0.5, 100.0, 101.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f,
                                    0.0, 1, 0, 0.0, 0.0, 15.0, 1, 0, 0.15, 0.0, 0, 0.0, mis);
        std::printf("%.1f,%d,%.3f,%.4f\n", mis, seed,
                    0.5 * (r.roll_rmse_deg + r.pitch_rmse_deg), r.pos_rmse);
      }
    }
  }

  // ★杆臂效应：天线相对 IMU 0/0.5/1.0m（x），maneuver 场景。
  if (const char *ab = std::getenv("IMUPERF_LEVER"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== GNSS lever arm (maneuver, M8N_10Hz) ==\n");
    std::printf("lever_m,seed,pos_rmse,att_rmse_deg\n");
    for (double lv : {0.0, 0.5, 1.0})
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::Maneuver, seed,
                                    false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                    0, 0.5, 0.5, 100.0, 101.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f,
                                    0.0, 1, 0, 0.0, 0.0, 15.0, 1, 0, 0.15, 0.0, 0, 0.0, 0.0,
                                    lv, 0.0, 0.0);
        std::printf("%.1f,%d,%.4f,%.3f\n", lv, seed, r.pos_rmse,
                    0.5 * (r.roll_rmse_deg + r.pitch_rmse_deg));
      }
    }
  }

  // ★速度/位置时间对齐：速度额外延迟 0/50/100ms，maneuver 场景。
  if (const char *ab = std::getenv("IMUPERF_VEL_DELAY"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== GNSS velocity extra delay (maneuver) ==\n");
    std::printf("vel_delay_ms,seed,pos_rmse,att_rmse_deg\n");
    for (double vd : {0.0, 0.05, 0.10})
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::Maneuver, seed,
                                    false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                    0, 0.5, 0.5, 100.0, 101.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f,
                                    0.0, 1, 0, 0.0, 0.0, 15.0, 1, 0, 0.15, 0.0, 0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, vd);
        std::printf("%.0f,%d,%.4f,%.3f\n", vd * 1000.0, seed, r.pos_rmse,
                    0.5 * (r.roll_rmse_deg + r.pitch_rmse_deg));
      }
    }
  }

  // ★加速场景：正弦加速下速度延迟 0/50/100ms 的影响。
  if (const char *ab = std::getenv("IMUPERF_VEL_DELAY2"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== GNSS velocity delay on acceleration ==\n");
    std::printf("vel_delay_ms,seed,pos_rmse,att_rmse_deg\n");
    for (double vd : {0.0, 0.05, 0.10})
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::Accel, seed,
                                    false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                    0, 0.5, 0.5, 100.0, 101.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f,
                                    0.0, 1, 0, 0.0, 0.0, 15.0, 1, 0, 0.15, 0.0, 0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, vd);
        std::printf("%.0f,%d,%.4f,%.3f\n", vd * 1000.0, seed, r.pos_rmse,
                    0.5 * (r.roll_rmse_deg + r.pitch_rmse_deg));
      }
    }
  }

  // ★GNSS 垂直通道质量：垂直噪声 1.5x/2x/3x，看高度与位置。
  if (const char *ab = std::getenv("IMUPERF_VERT_NOISE"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== GNSS vertical noise (static, M8N_10Hz) ==\n");
    std::printf("mult,seed,alt_rmse,pos_rmse\n");
    for (double vm : {1.5, 2.0, 3.0})
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::Static, seed,
                                    false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                    0, 0.5, 0.5, 100.0, 101.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f,
                                    0.0, 1, 0, 0.0, 0.0, 15.0, 1, 0, 0.15, 0.0, 0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, vm);
        std::printf("%.1f,%d,%.4f,%.4f\n", vm, seed, r.alt_rmse, r.pos_rmse);
      }
    }
  }

  // ★陀螺卡死：freeze_t = -1/10/20s，看姿态与位置误差。
  if (const char *ab = std::getenv("IMUPERF_GYRO_FREEZE"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== gyro freeze fault (static) ==\n");
    std::printf("freeze_t,seed,att_rmse_deg,pos_rmse\n");
    for (double ft : {-1.0, 10.0, 20.0})
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::Maneuver, seed,
                                    false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                    0, 0.5, 0.5, 100.0, 101.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f,
                                    0.0, 1, 0, 0.0, 0.0, 15.0, 1, 0, 0.15, 0.0, 0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 1.5, ft);
        std::printf("%.0f,%d,%.3f,%.4f\n", ft, seed,
                    0.5 * (r.roll_rmse_deg + r.pitch_rmse_deg), r.pos_rmse);
      }
    }
  }

  // ★无重锚时 NIS 门控对单帧大野值：0/50/200m。
  if (const char *ab = std::getenv("IMUPERF_SPIKE_GATE"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== NIS gate vs single-frame spike (static, no reanchor) ==\n");
    std::printf("spike_m,seed,pos_rmse,reject_rate\n");
    for (double sp : {0.0, 50.0, 200.0})
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::Static, seed,
                                    false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                    0, 0.5, 0.5, 100.0, 101.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f,
                                    0.0, 1, 0, 5.0, sp, 15.0, 1, 0, 0.15, 0.0, 0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 1.5, -1.0);
        std::printf("%.0f,%d,%.4f,%.4f\n", sp, seed, r.pos_rmse,
                    r.nis_reject_rate);
      }
    }
  }

  // ★IMU 健康监测：陀螺卡死检测（0.2s 窗口方差）与误报。
  if (const char *ab = std::getenv("IMUPERF_IMU_HEALTH"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== IMU health: gyro freeze detection (maneuver) ==\n");
    std::printf("freeze_t,seed,detect_t,hits,att_rmse_deg\n");
    for (double ft : {-1.0, 10.0, 20.0})
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::Maneuver, seed,
                                    false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                    0, 0.5, 0.5, 100.0, 101.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f,
                                    0.0, 1, 0, 5.0, 0.0, 15.0, 1, 0, 0.15, 0.0, 0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 1.5, ft);
        std::printf("%.0f,%d,%.2f,%d,%.3f\n", ft, seed,
                    r.gyro_freeze_detect_t, r.gyro_freeze_hits,
                    0.5 * (r.roll_rmse_deg + r.pitch_rmse_deg));
      }
    }
  }

  // ★加计卡死检测：Accel 场景，freeze -1/10/20s。
  if (const char *ab = std::getenv("IMUPERF_ACCEL_FREEZE"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== accelerometer freeze detection (accel scenario) ==\n");
    std::printf("freeze_t,seed,detect_t,hits,pos_rmse\n");
    for (double ft : {-1.0, 10.0, 20.0})
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::Accel, seed,
                                    false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                    0, 0.5, 0.5, 100.0, 101.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f,
                                    0.0, 1, 0, 5.0, 0.0, 15.0, 1, 0, 0.15, 0.0, 0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 1.5, -1.0, ft);
        std::printf("%.0f,%d,%.2f,%d,%.4f\n", ft, seed,
                    r.accel_freeze_detect_t, r.accel_freeze_hits, r.pos_rmse);
      }
    }
  }

  // ★健康监测误报：强振动（80Hz/0.5g）下正常不应触发。
  if (const char *ab = std::getenv("IMUPERF_HEALTH_VIB"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== health monitor false positives (vibration 80Hz/0.5g) ==\n");
    std::printf("seed,gyro_hits,accel_hits\n");
    for (int seed = 1; seed <= kSeedCount; ++seed)
    {
      const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::Vibration, seed,
                                  false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                  0, 0.5, 0.5, 100.0, 101.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f,
                                  0.0, 1, 0, 5.0, 0.0, 15.0, 1, 0, 0.15, 0.0, 0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 1.5, -1.0, -1.0);
      std::printf("%d,%d,%d\n", seed, r.gyro_freeze_hits, r.accel_freeze_hits);
    }
  }

  // ★GNSS 速度噪声：1x/2x/3x，static 与 maneuver。
  if (const char *ab = std::getenv("IMUPERF_VEL_NOISE"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== GNSS velocity noise (M8N_10Hz) ==\n");
    std::printf("mult,scenario,seed,pos_rmse,att_rmse_deg\n");
    for (double vn : {1.0, 2.0, 3.0})
    {
      for (int sc : {0, 1})
      {
        for (int seed = 1; seed <= kSeedCount; ++seed)
        {
          const RunResult r = runCase(kIcm45686, kGnssSpecs[2],
                                      sc == 0 ? Scenario::Static : Scenario::Maneuver, seed,
                                      false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                      0, 0.5, 0.5, 100.0, 101.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f,
                                      0.0, 1, 0, 5.0, 0.0, 15.0, 1, 0, 0.15, 0.0, 0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0, 1.5, -1.0, -1.0, vn);
          std::printf("%.0f,%s,%d,%.4f,%.3f\n", vn,
                      sc == 0 ? "static" : "maneuver", seed, r.pos_rmse,
                      0.5 * (r.roll_rmse_deg + r.pitch_rmse_deg));
        }
      }
    }
  }

  // ★加计温漂：0/0.005/0.01 m/s²/°C（9°C → 0/0.045/0.09 m/s²）。
  if (const char *ab = std::getenv("IMUPERF_ACCEL_TEMP"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== accelerometer temperature drift (static) ==\n");
    std::printf("atc,seed,pos_rmse,att_rmse_deg\n");
    for (double at : {0.0, 0.005, 0.01})
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::Static, seed,
                                    false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                    0, 0.5, 0.5, 100.0, 101.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f,
                                    0.0, 1, 0, 5.0, 0.0, 15.0, 1, 0, 0.15, 0.0, 0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 1.5, -1.0, -1.0, 1.0, at);
        std::printf("%.3f,%d,%.4f,%.3f\n", at, seed, r.pos_rmse,
                    0.5 * (r.roll_rmse_deg + r.pitch_rmse_deg));
      }
    }
  }

  // ★GNSS 位置噪声：1x/2x/3x（R 同步匹配），static。
  if (const char *ab = std::getenv("IMUPERF_POS_NOISE"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== GNSS position noise (static, R matched) ==\n");
    std::printf("mult,seed,pos_rmse,alt_rmse\n");
    for (double pn : {1.0, 2.0, 3.0})
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::Static, seed,
                                    false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                    0, 0.5, 0.5, 100.0, 101.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f,
                                    0.0, 1, 0, 5.0, 0.0, 15.0, 1, 0, 0.15, 0.0, 0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 1.5, -1.0, -1.0, 1.0, 0.0, pn);
        std::printf("%.0f,%d,%.4f,%.4f\n", pn, seed, r.pos_rmse, r.alt_rmse);
      }
    }
  }

  // ★GNSS/IMU 时间同步误差：时间戳偏移 -20/0/+20ms（maneuver）。
  if (const char *ab = std::getenv("IMUPERF_TIME_SYNC"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== GNSS time sync error (maneuver) ==\n");
    std::printf("offset_ms,seed,pos_rmse,att_rmse_deg\n");
    for (double off : {-0.02, 0.0, 0.02})
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::Accel, seed,
                                    false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                    0, 0.5, 0.5, 100.0, 101.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f,
                                    0.0, 1, 0, 5.0, 0.0, 15.0, 1, 0, 0.15, 0.0, 0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 1.5, -1.0, -1.0, 1.0, 0.0, 1.0, off);
        std::printf("%.0f,%d,%.4f,%.3f\n", off * 1000.0, seed, r.pos_rmse,
                    0.5 * (r.roll_rmse_deg + r.pitch_rmse_deg));
      }
    }
  }

  // ★垂直野值：单帧高度跳变 0/20/100m（static）。
  if (const char *ab = std::getenv("IMUPERF_VERT_SPIKE"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== GNSS vertical spike (static) ==\n");
    std::printf("spike_m,seed,alt_rmse,pos_rmse\n");
    for (double vs : {0.0, 20.0, 100.0})
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::Static, seed,
                                    false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                    0, 0.5, 0.5, 100.0, 101.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f,
                                    25.0, 1, 0, 5.0, 0.001, 15.0, 1, 0, 0.15, 0.0, 0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 1.5, -1.0, -1.0, 1.0, 0.0, 1.0, 0.0, vs);
        std::printf("%.0f,%d,%.4f,%.4f\n", vs, seed, r.alt_rmse, r.pos_rmse);
      }
    }
  }

  // ★组合故障：GNSS 失锁 + 陀螺卡死（最坏场景）。
  if (const char *ab = std::getenv("IMUPERF_COMBO_FAULT"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== combo fault: GNSS outage + gyro freeze ==\n");
    std::printf("freeze_t,seed,detect_t,att_rmse_deg,pos_rmse\n");
    for (double ft : {-1.0, 10.0, 20.0})
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::VibrationOutage, seed,
                                    false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                    0, 0.5, 0.5, 5.0, 25.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f,
                                    0.0, 1, 0, 5.0, 0.0, 15.0, 1, 0, 0.15, 0.0, 0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 1.5, ft, -1.0, 1.0, 0.0, 1.0, 0.0, 0.0);
        std::printf("%.0f,%d,%.2f,%.3f,%.4f\n", ft, seed, r.gyro_freeze_detect_t,
                    0.5 * (r.roll_rmse_deg + r.pitch_rmse_deg), r.pos_rmse);
      }
    }
  }

  // ★振动幅值对失锁姿态影响：0.2/0.5/1.0 g。
  if (const char *ab = std::getenv("IMUPERF_VIB_AMP"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== vibration amplitude vs outage attitude ==\n");
    std::printf("vib_amp,seed,att_rmse_deg,pos_rmse\n");
    for (double va : {0.2, 0.5, 1.0})
    {
      for (int seed = 1; seed <= kSeedCount; ++seed)
      {
        const RunResult r = runCase(kIcm45686, kGnssSpecs[2], Scenario::VibrationOutage, seed,
                                    false, nullptr, false, 80.0, 0, 1.0e9, 0, 2.0f,
                                    0, 0.5, va, 5.0, 25.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f,
                                    0.0, 1, 0, 5.0, 0.0, 15.0, 1, 0, 0.15, 0.0, 0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 1.5, -1.0, -1.0, 1.0, 0.0, 1.0, 0.0, 0.0);
        std::printf("%.1f,%d,%.3f,%.4f\n", va, seed,
                    0.5 * (r.roll_rmse_deg + r.pitch_rmse_deg), r.pos_rmse);
      }
    }
  }

  // ★q_accel 扫描：1/2/3/4 × vibration/vib_outage。
  if (const char *ab = std::getenv("IMUPERF_QAC_SCAN"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== q_accel scan ==\n");
    std::printf("q_accel,scenario,seed,att_rmse_deg,pos_rmse\n");
    for (float qa : {1.0f, 2.0f, 3.0f, 4.0f})
    {
      for (int sc : {3, 4})
      {
        for (int seed = 1; seed <= kSeedCount; ++seed)
        {
          const RunResult r = runCase(kIcm45686, kGnssSpecs[2],
                                      sc == 3 ? Scenario::Vibration : Scenario::VibrationOutage,
                                      seed, false, nullptr, false, 80.0, 0, 1.0e9, 0, qa,
                                      0, 0.5, 0.5, 5.0, 25.0, 0.0, 0, 0.0, 0.0, 0, 0.05, 0.0f,
                                      0.0, 1, 0, 5.0, 0.0, 15.0, 1, 0, 0.15, 0.0, 0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0, 1.5, -1.0, -1.0, 1.0, 0.0, 1.0, 0.0, 0.0);
          std::printf("%.1f,%s,%d,%.3f,%.4f\n", qa,
                      sc == 3 ? "vibration" : "outage", seed,
                      0.5 * (r.roll_rmse_deg + r.pitch_rmse_deg), r.pos_rmse);
        }
      }
    }
  }

  // ★GRAND：超广覆盖矩阵（5 GNSS × 6 场景 × 3 振动 × 3 延迟 × 2 seeds）。
  if (const char *ab = std::getenv("IMUPERF_GRAND"); ab != nullptr && ab[0] == '1')
  {
    std::printf("\n== GRAND matrix ==\n");
    std::printf("gnss,scene,vib,delay,seed,pos,att,alt\n");
    auto ge = [](const char *k, double d) {
      const char *v = std::getenv(k);
      return v != nullptr ? std::atof(v) : d;
    };
    const double g_eas = ge("IMUPERF_EKF_ACCEL_STD", 0.25);
    const double g_egs = ge("IMUPERF_EKF_GYRO_STD", 0.0015);
    const double g_eam = ge("IMUPERF_EKF_ACCEL_MARKOV", 0.010);
    const double g_egm = ge("IMUPERF_EKF_GYRO_MARKOV", 0.00524);
    const double g_qa = ge("IMUPERF_Q_ACCEL", 2.0);
    const int gis[5] = {0, 2, 5, 7, 8};
    const Scenario scs[6] = {Scenario::Static, Scenario::Maneuver, Scenario::Cruise,
                             Scenario::Vibration, Scenario::VibrationOutage,
                             Scenario::Accel};
    const double vibs[3] = {0.2, 0.5, 1.0};
    const double delays[3] = {0.0, 0.03, 0.10};
    for (int gi : gis)
    for (Scenario sc : scs)
    for (double vb : vibs)
    for (double dl : delays)
    for (int seed = 1; seed <= 2; ++seed)
    {
      const bool comp = dl > 0.0;
      const RunResult r = runCase(kIcm45686, kGnssSpecs[gi], sc, seed,
                                  false, nullptr, comp, 80.0, 0, 1.0e9, 0,
                                  static_cast<float>(g_qa), 0, 0.5, vb, 5.0, 25.0, 0.0, 0,
                                  0.0, 0.0, 0, 0.05, 0.0f, 0.0, 1, 0, 5.0, 0.0, 15.0, 1, 0,
                                  g_eas, g_egs, g_eam, g_egm);
      std::printf("%s,%s,%.1f,%.3f,%d,%.4f,%.3f,%.4f\n", kGnssSpecs[gi].name,
                  scenarioName(sc), vb, dl, seed, r.pos_rmse,
                  0.5 * (r.roll_rmse_deg + r.pitch_rmse_deg), r.alt_rmse);
    }
  }

  std::printf("\n总计 %d 组，失败 %d 组\n", total, fail);
  if (fail == 0)
  {
    std::printf("全部通过\n");
    return 0;
  }
  return 1;
}
