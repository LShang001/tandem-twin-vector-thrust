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
};

const char *scenarioName(Scenario s)
{
  switch (s)
  {
  case Scenario::Static:   return "static";
  case Scenario::Maneuver: return "maneuver";
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
  if (sc == Scenario::Static)
  {
    return tr; // 全零
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
                  int seed, bool verbose, std::vector<double> *trace_buf = nullptr)
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
    if ((k % gnss_div) == 0)
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
      ekf.MeasurementUpdate(vel_meas.cast<float>(), lla_meas, kNavDt);
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
  const Scenario scenarios[3] = {Scenario::Static, Scenario::Maneuver, Scenario::Cruise};
  int fail = 0, total = 0;

  // 汇总：每个 (imu, gnss, scenario) 的均值。
  std::vector<double> sum_pos(2 * kGnssCount * 3, 0.0);
  std::vector<double> sum_vel(2 * kGnssCount * 3, 0.0);
  std::vector<double> sum_att(2 * kGnssCount * 3, 0.0);

  for (int si = 0; si < 2; ++si)
  {
    for (int gi = 0; gi < kGnssCount; ++gi)
    {
      for (int sc = 0; sc < 3; ++sc)
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
      for (int sc = 0; sc < 3; ++sc)
      {
        const int idx = (si * kGnssCount + gi) * 3 + sc;
        std::printf("%-13s %-14s %-8s %8.4f %8.4f %8.4f\n", imus[si].name,
                    kGnssSpecs[gi].name, scenarioName(scenarios[sc]),
                    sum_att[idx] / kSeedCount, sum_vel[idx] / kSeedCount,
                    sum_pos[idx] / kSeedCount);
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
