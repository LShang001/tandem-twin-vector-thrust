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
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <random>
#include <vector>

#include "ekf_15_state.h"

namespace
{
constexpr float kGravity = 9.79f;   // 与 TandemVec_Config.h 一致
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kRad2Deg = 180.0 / kPi;
constexpr float kNavDt = 0.005f;    // 导航 200Hz
constexpr int kSeedCount = 4;
constexpr double kDurationS = 30.0;

enum class Scenario
{
  Static = 0,
  Maneuver = 1,
  Cruise = 2,
  Vibration = 3,
  VibrationOutage = 4,
};

const char *scenarioName(Scenario s)
{
  switch (s)
  {
  case Scenario::Static:   return "static";
  case Scenario::Maneuver: return "maneuver";
  case Scenario::Vibration: return "vibration";
  case Scenario::VibrationOutage: return "vib_outage";
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
                  double hacc_gate_m = 1.0e9)
{
  std::mt19937_64 rng(static_cast<uint64_t>(seed) * 7919ULL +
                      static_cast<uint64_t>(imu.odr_hz) * 13ULL +
                      static_cast<uint64_t>(gnss.rate_hz * 100.0) +
                      static_cast<uint64_t>(sc));
  std::normal_distribution<double> n01(0.0, 1.0);

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
  ekf.Initialize(f_body0.cast<float>(), Eigen::Vector3f::Zero(),
                 Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), lla0);

  const int nav_steps = static_cast<int>(kDurationS / kNavDt);
  const double dt_imu = 1.0 / fs;
  const int samples_per_nav = static_cast<int>(std::lround(kNavDt / dt_imu));
  const int gnss_div = std::max(1, static_cast<int>(std::lround(200.0 / gnss.rate_hz)));
  const double latency_s = gnss.latency_ms / 1000.0;

  double sum_roll2 = 0.0, sum_pitch2 = 0.0, sum_vel2 = 0.0, sum_pos2 = 0.0;
  double sum_out_pos2 = 0.0, sum_out_vel2 = 0.0;
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
      for (int a = 0; a < 3; ++a)
      {
        accel_meas(a) += accel_bias(a) + accel_sigma * n01(rng);
        gyro_meas(a) += gyro_bias(a) + gyro_sigma * n01(rng);
      }
      if (sc == Scenario::Vibration || sc == Scenario::VibrationOutage)
      {
        // 80Hz、0.5g 机体 z 轴振动（电机/桨叶高频激励），验证 LPF 对振动泄漏的抑制。
        accel_meas(2) += 0.5 * kGravity * std::sin(2.0 * kPi * vib_hz * ti);
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

    ekf.TimeUpdateTwoSample(dtheta1.cast<float>(), dtheta2.cast<float>(),
                            dv1.cast<float>(), dv2.cast<float>(), kNavDt);

    // GNSS：按刷新率注入，测量对应 t−latency 的真值（体现定位延迟）。
    if ((k % gnss_div) == 0 &&
        !(sc == Scenario::VibrationOutage && t >= 10.0 && t < 20.0))
    {
      const double tm = std::fmax(0.0, t - latency_s);
      const Truth trm = truthAt(tm, sc);
      Eigen::Vector3d vel_meas = trm.vel_ned;
      for (int a = 0; a < 3; ++a) vel_meas(a) += gnss.vel_std_mps * n01(rng);
      const double dn = trm.pos_ned(0) + gnss.pos_std_m * n01(rng);
      const double de = trm.pos_ned(1) + gnss.pos_std_m * n01(rng);
      const double dd = trm.pos_ned(2) + gnss.pos_std_m * 1.5 * n01(rng);
      Eigen::Vector3d lla_meas = lla0;
      lla_meas(0) += dn / kEarthR;
      lla_meas(1) += de / (kEarthR * std::cos(lla0(0)));
      lla_meas(2) += -dd;
      if (gnss_mode == 1 || gnss.pos_std_m >= hacc_gate_m)
      {
        // 只融合速度，不注入位置（模拟低质量 GNSS 下关闭位置观测）；
        // 速度 R 与 full 模式保持一致，差异只来自“是否融合位置”。
        const float v_std = static_cast<float>(std::fmax(gnss.vel_std_mps, 0.05));
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
        const float h_acc = static_cast<float>(std::fmax(gnss.pos_std_m, 0.15));
        const float v_std = static_cast<float>(std::fmax(gnss.vel_std_mps, 0.05));
        ekf.gnss_pos_ne_std_m(h_acc);
        ekf.gnss_pos_d_std_m(h_acc * 1.5f);
        ekf.gnss_vel_ne_std_mps(v_std);
        ekf.gnss_vel_d_std_mps(static_cast<float>(v_std * 1.5));
        ekf.MeasurementUpdate(vel_meas.cast<float>(), lla_meas, kNavDt);
      }
      ++gnss_updates;
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
      if (sc == Scenario::VibrationOutage && t >= 10.0 && t < 20.0)
      {
        sum_out_pos2 += pos_err.squaredNorm();
        sum_out_vel2 += (est_vel.cast<double>() - tr.vel_ned).squaredNorm();
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
  r.out_count = out_count;
  if (out_count > 0)
  {
    r.out_pos_rmse = std::sqrt(sum_out_pos2 / out_count);
    r.out_vel_rmse = std::sqrt(sum_out_vel2 / out_count);
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
  std::vector<double> sum_pos(2 * kGnssCount * 4, 0.0);
  std::vector<double> sum_vel(2 * kGnssCount * 4, 0.0);
  std::vector<double> sum_att(2 * kGnssCount * 4, 0.0);

  for (int si = 0; si < 2; ++si)
  {
    for (int gi = 0; gi < kGnssCount; ++gi)
    {
      for (int sc = 0; sc < 4; ++sc)
      {
        const int idx = (si * kGnssCount + gi) * 3 + sc;
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
        const int idx = (si * kGnssCount + gi) * 3 + sc;
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

  std::printf("\n总计 %d 组，失败 %d 组\n", total, fail);
  if (fail == 0)
  {
    std::printf("全部通过\n");
    return 0;
  }
  return 1;
}
