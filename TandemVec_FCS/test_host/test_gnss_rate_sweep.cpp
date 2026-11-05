/**
 * @file    test_gnss_rate_sweep.cpp
 * @module   测试/宿主机
 * @brief   GNSS 更新率 × 飞行速度 × 加速度 对导航精度的影响（真实 EKF）。
 * @details 输入：北向正弦速度真值（v_amp 1/5/10/20 m/s、a_peak 0.5/2/5 m/s²）、
 *          ICM-45686 噪声与零偏（70 µg/√Hz、0.0038 °/s/√Hz）、F10N GNSS
 *          噪声（0.85 m、0.05 m/s，更新率 1–50 Hz，延迟 0–200 ms 可扫）；
 *          处理：真值投影为比力/角速度喂 Ekf15State 200 Hz，GNSS 位置+速度
 *          量测；输出：每组 pos_rmse、att_rmse_deg 的 CSV。判定：无断言，
 *          main 恒返回 0（扫描数据供选型与 Q/R/延迟参数评估）。
 * @frame   NED：轨迹与量测为北向位置/速度（东向 0），LLA 站点 31.2304°N/
 *          121.4737°E/5 m；机体系输入为比力 f_b=C_bn^T·f_ned 与陀螺 w_b
 *          （航向 0.1 °/s 慢转，水平无倾斜）。
 * @units   速度 m/s、加速度 m/s²、角速度 rad/s、角速率噪声 °/s/√Hz、GNSS 率 Hz、
 *          延迟 ms、位置 m、时长 s、姿态误差 deg。
 *
 * 目的（2026-11-01 用户要求）：量化“不同 GNSS 更新频率”在“不同运动轨迹/
 * 速度/加速度”下的导航效果，为 F10N（10 Hz 多星座 / 20 Hz 单星座）选型与
 * EKF 参数（GNSS R、延迟、hAcc 门控）提供依据。
 *
 * 方法：
 *   1) 真值轨迹：水平正弦速度 v(t)=v_amp·sin(2πf t)，加速度峰值
 *      a_peak = v_amp·2πf；由 (v_amp, a_peak) 反解 f，保证速度/加速度都可控；
 *   2) IMU 真值：比力 = a + g_ned（水平姿态，偏航缓慢转动 0.1 °/s）；
 *      陀螺 = 偏航率 + 地球自转投影（高精度口径）；
 *   3) 传感器：ICM-45686 噪声（手册 3.2 kHz、0.0038 °/s/√Hz、70 µg/√Hz）；
 *   4) GNSS：F10N 噪声（位置 1σ=0.85 m、速度 0.05 m/s），更新率 1/2/5/10/20/50 Hz；
 *   5) EKF 200 Hz；每 GNSS 周期做位置+速度量测（简单 R 对角）；
 *   6) 输出每组位置/姿态 RMSE，按（速度、加速度、率）聚合。
 */
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ekf_15_state.h"
#include "ins_gnss_delay_auto.h"
#include "ins_test_gravity.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kOmegaIe = 7.292115e-5;
constexpr double kLat = 31.2304 * kDeg2Rad;
const Eigen::Vector3d kLla(31.2304 * kDeg2Rad, 121.4737 * kDeg2Rad, 5.0);
constexpr double kImuDt = 0.005;  // 200 Hz

const float kG = ins_test::GravityWgs84();
const double kGd = static_cast<double>(kG);

// ICM-45686 手册噪声（1σ/√Hz → 采样噪声）
constexpr double kAccelNd = 70e-6 * 9.80665;
constexpr double kGyroNd = 0.0038 * kDeg2Rad;
constexpr double kAccelBias = 20e-3 * 9.80665 / 3.0;
constexpr double kGyroBias = 0.4 * kDeg2Rad / 3.0;

// F10N 噪声
constexpr double kGnssPosStd = 0.85;
constexpr double kGnssVelStd = 0.05;

struct Result
{
  double pos_rmse = 0.0;
  double att_rmse_deg = 0.0;
  int gnss_updates = 0;
};

/** 正弦速度轨迹：v(t)=v_amp·sin(2πf t)（北向），偏航慢转。 */
Result run(const double gnss_hz, const double v_amp, const double a_peak,
           const unsigned seed, const double ekf_accel_std = 0.25,
           const double gnss_delay_s = 0.0, const bool auto_delay = false)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> n01(0.0, 1.0);

  const double f = a_peak / (2.0 * kPi * v_amp);  // 由速度/加速度反解频率
  const double T = 120.0;                        // 仿真时长
  const int steps = static_cast<int>(T / kImuDt);

  bfs::Ekf15State ekf;
  Eigen::Vector3f f0(0.0f, 0.0f, -kG);
  Eigen::Vector3f w0(0.0f, 0.0f,
                     static_cast<float>(-kOmegaIe * std::sin(kLat)));
  // ★ EKF 过程噪声参数必须在 Initialize 之前设置：Initialize 会缓存 gs_/rw_，
  //   之后改 accel_std/q_accel 对完整协方差路径不生效（2026-11-01 排查结论）。
  ekf.accel_std_mps2(static_cast<float>(ekf_accel_std));
  ekf.set_q_accel_scale(2.0f);
  ekf.Initialize(f0, w0, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(),
                 kLla);


  // 位置/速度观测噪声
  std::normal_distribution<double> np(0.0, kGnssPosStd);
  std::normal_distribution<double> nv(0.0, kGnssVelStd);

  const double gnss_dt = 1.0 / gnss_hz;
  // 延迟量测需要历史真值（pos_n / vel_n）
  InsGnssDelayAutoEstimator delay_est;
  double delay_est_next = 1.0;
  float applied_age = static_cast<float>(gnss_delay_s);
  std::vector<double> hist_pos, hist_vel;
  hist_pos.reserve(steps + 1);
  hist_vel.reserve(steps + 1);
  double next_gnss = 0.0;
  int gnss_updates = 0;

  double pos_err_sq = 0.0;
  double att_err_sq = 0.0;
  int samples = 0;

  // 真值状态
  double pos_n = 0.0, pos_e = 0.0;
  double vel_n = 0.0;
  double yaw = 0.0;  // 缓慢转动

  for (int k = 0; k < steps; ++k)
  {
    const double t = k * kImuDt;
    // 真值：速度/加速度/位置
    const double omega = 2.0 * kPi * f;
    const double v = v_amp * std::sin(omega * t);
    const double a = v_amp * omega * std::cos(omega * t);
    vel_n = v;
    pos_n += v * kImuDt;
    yaw += 0.1 * kDeg2Rad * kImuDt;  // 0.1 °/s
    hist_pos.push_back(pos_n);
    hist_vel.push_back(v);
    if (auto_delay)
    {
      delay_est.PushReference(t, Eigen::Vector3d(pos_n, 0.0, 0.0),
                              Eigen::Vector3d(v, 0.0, 0.0));
    }

    const Eigen::Matrix3d C_bn =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d a_ned(a, 0.0, 0.0);
    const Eigen::Vector3d f_ned = a_ned + Eigen::Vector3d(0.0, 0.0, -kGd);
    const Eigen::Vector3d f_body = C_bn.transpose() * f_ned;
    const Eigen::Vector3d w_ned(0.0, 0.0, 0.1 * kDeg2Rad);
    const Eigen::Vector3d w_body = C_bn.transpose() * w_ned;

    // 传感器噪声（采样率 200 Hz → 密度 / √fs）
    const double fs = 1.0 / kImuDt;
    Eigen::Vector3f accel = f_body.cast<float>();
    Eigen::Vector3f gyro = w_body.cast<float>();
    accel(0) += static_cast<float>(kAccelBias * n01(rng));
    accel(1) += static_cast<float>(kAccelBias * n01(rng));
    accel(2) += static_cast<float>(kAccelBias * n01(rng));
    gyro(0) += static_cast<float>(kGyroBias * n01(rng));
    gyro(1) += static_cast<float>(kGyroBias * n01(rng));
    gyro(2) += static_cast<float>(kGyroBias * n01(rng));
    accel(0) += static_cast<float>(kAccelNd * std::sqrt(fs) * n01(rng));
    accel(1) += static_cast<float>(kAccelNd * std::sqrt(fs) * n01(rng));
    accel(2) += static_cast<float>(kAccelNd * std::sqrt(fs) * n01(rng));
    gyro(0) += static_cast<float>(kGyroNd * std::sqrt(fs) * n01(rng));
    gyro(1) += static_cast<float>(kGyroNd * std::sqrt(fs) * n01(rng));
    gyro(2) += static_cast<float>(kGyroNd * std::sqrt(fs) * n01(rng));

    ekf.TimeUpdate(accel, gyro, static_cast<float>(kImuDt));

    // 自动延迟估计：每秒尝试一次，有效则用 tau_hat 作为 age
    if (auto_delay && t >= delay_est_next)
    {
      delay_est_next = t + 1.0;
      if (delay_est.TryEstimate())
      {
        applied_age = delay_est.last_result().tau_s;
      }
    }

    // GNSS 量测（位置+速度，F10N 噪声）
    if (t >= next_gnss)
    {
      next_gnss += gnss_dt;
      // 延迟量测：取 delay 秒前的真值（历史索引），age=delay 交给 EKF 回放
      const int lag = static_cast<int>(std::round(gnss_delay_s / kImuDt));
      const int hi = std::max(0, static_cast<int>(hist_pos.size()) - 1 - lag);
      const double pos_delayed = hist_pos[hi];
      const double vel_delayed = hist_vel[hi];
      const double r_north = 6378137.0;
      const double r_east = 6378137.0 * std::cos(kLla(0));
      const Eigen::Vector3d lla_meas(
          kLla(0) + (pos_delayed + np(rng)) / r_north,
          kLla(1) + np(rng) / r_east,
          kLla(2) + np(rng));
      const Eigen::Vector3f vel_ned(static_cast<float>(vel_delayed + nv(rng)),
                                    static_cast<float>(nv(rng)),
                                    static_cast<float>(nv(rng)));
      if (auto_delay)
      {
        delay_est.PushGnss(
            t - gnss_delay_s,
            Eigen::Vector3d(pos_delayed + np(rng), np(rng), np(rng)),
            Eigen::Vector3d(vel_delayed + nv(rng), nv(rng), nv(rng)));
      }
      ekf.gnss_pos_ne_std_m(static_cast<float>(kGnssPosStd));
      ekf.gnss_pos_d_std_m(static_cast<float>(kGnssPosStd));
      ekf.gnss_vel_ne_std_mps(static_cast<float>(kGnssVelStd));
      ekf.gnss_vel_d_std_mps(static_cast<float>(kGnssVelStd));
      const auto res = ekf.MeasurementUpdateDetailed(
          vel_ned, lla_meas, kImuDt, applied_age);
      if (seed == 1 && gnss_hz == 1.0 && v_amp == 5.0 && a_peak == 2.0 && gnss_updates < 3)
      {
        std::printf("[gnss] input_valid=%d attempted=%d fused=%d nis=%.2f\n",
                    res.input_valid ? 1 : 0, res.attempted ? 1 : 0,
                    res.fused ? 1 : 0,
                    static_cast<double>(res.test_ratio));
      }
      ++gnss_updates;
    }

    // 误差统计（每 0.5 s 采一次，跳过前 5 s 对准段）
    if (t > 5.0 && k % 100 == 0)
    {
      const double dn = ekf.lat_rad() - (kLla(0) + pos_n / 6378137.0);
      const double de = ekf.lon_rad() - kLla(1);
      const double pos_err = std::hypot(dn * 6378137.0,
                                        de * 6378137.0 * std::cos(kLla(0)));
      const double att_err = std::fmax(std::fabs(ekf.roll_rad()),
                                       std::fabs(ekf.pitch_rad())) *
                             180.0 / kPi;
      pos_err_sq += pos_err * pos_err;
      att_err_sq += att_err * att_err;
      ++samples;
    }
  }

  Result r;
  r.pos_rmse = samples > 0 ? std::sqrt(pos_err_sq / samples) : 0.0;
  r.att_rmse_deg = samples > 0 ? std::sqrt(att_err_sq / samples) : 0.0;
  r.gnss_updates = gnss_updates;
  return r;
}
}  // namespace

int main()
{
  std::printf("== GNSS 更新率 × 速度 × 加速度 对导航精度影响 ==\n");
  std::printf("gnss_hz,v_amp,a_peak,seed,pos_rmse,att_rmse_deg,updates\n");

  const double rates[] = {1.0, 2.0, 5.0, 10.0, 20.0, 50.0};
  const double v_amps[] = {1.0, 5.0, 10.0, 20.0};
  const double a_peaks[] = {0.5, 2.0, 5.0};
  // Q 扫描：验证“更新率越高误差越大”是否由 Q 与 R 不匹配造成
  const double qs[3] = {0.05, 0.25, 0.5};  // accel_std（Q 的另一个因子）
  std::printf("\n-- Q 扫描（v=5, a=2, seed=1）--\n");
  std::printf("q,gnss_hz,pos_rmse,att_rmse_deg\n");
  for (double q : qs)
  {
    for (double hz2 : {1.0, 10.0, 20.0})
    {
      const Result rq = run(hz2, 5.0, 2.0, 1, q);
      std::printf("%.2f,%.0f,%.4f,%.4f\n", q, hz2, rq.pos_rmse, rq.att_rmse_deg);
    }
  }
  std::printf("\n");
  // 延迟扫描：延迟 {0,10,20,50,100,200} ms × 更新率 {10,20} × 速度 {1,10}
  std::printf("-- 延迟扫描 --\n");
  std::printf("delay_ms,gnss_hz,v,pos_rmse,att_rmse_deg\n");
  for (double dl : {0.0, 0.01, 0.02, 0.05, 0.1, 0.2})
  {
    for (double hz3 : {10.0, 20.0})
    {
      for (double vv : {1.0, 10.0})
      {
        const Result rd = run(hz3, vv, 2.0, 1, 0.25, dl);
        std::printf("%.0f,%.0f,%.0f,%.4f,%.4f\n", dl * 1000.0, hz3, vv,
                    rd.pos_rmse, rd.att_rmse_deg);
      }
    }
  }
  std::printf("\n");
  for (double hz : rates)
  {
    for (double va : v_amps)
    {
      for (double ap : a_peaks)
      {
        for (unsigned seed = 1; seed <= 2; ++seed)
        {
          const Result r = run(hz, va, ap, seed);
          std::printf("%.0f,%.1f,%.1f,%u,%.4f,%.4f,%d\n", hz, va, ap, seed,
                      r.pos_rmse, r.att_rmse_deg, r.gnss_updates);
        }
      }
    }
  }
  return 0;
}
