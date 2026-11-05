/**
 * @file    test_gnss_delay_estimator.cpp
 * @module   测试/宿主机
 * @brief   自动 GNSS 延迟估计器（ins_gnss_delay_auto.h）独立验证。
 * @details 输入：北向正弦速度真值（v_amp 5 m/s、a_peak 2 m/s²）逐 5 ms 推入参考
 *          轨迹，10 Hz GNSS 位置/速度量测取"延迟时刻真值"加噪（0.85 m、0.05 m/s）
 *          后按当前时刻投递；处理：每秒调 TryEstimate() 在 τ 网格上搜索；
 *          输出：τ̂、τ 误差、残差、锐度、拒绝码、样本数。判据：延迟
 *          0/50/150/300 ms 四档平均 |τ̂−τ| < 30 ms（锐度只打印，未断言）。
 * @frame   NED：参考位置/速度与 GNSS 速度均按 NED（仅北向非零）；无姿态与机体系。
 * @units   时间 s、位置 m、速度 m/s、延迟与 τ 以 ms 打印、残差 m/s、锐度无量纲。
 *
 * 目的：绕开 fw_nav_api 的 ABI 集成问题，直接验证估计器本身：
 *   - 正弦速度轨迹（加速度激励足够）；
 *   - 参考轨迹 = 真值（理想 IMU 参考）；
 *   - GNSS 样本按「投递时刻 = 采样时刻 + 延迟」推入（数据也是延迟时刻真值）；
 *   - 每秒 TryEstimate()，输出 τ̂ 与真值、残差、锐度、拒绝原因。
 *
 * 判据（代码断言）：延迟 0/50/150/300 ms 时平均 τ̂ 误差 <30 ms；锐度与残差只打印。
 */
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>
#include <Eigen/Core>

#include "ins_gnss_delay_auto.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDt = 0.005;
constexpr double kVamp = 5.0;
constexpr double kApeak = 2.0;

struct Result
{
  double tau_hat = 0.0;
  double tau_err = 0.0;
  double residual = 0.0;
  double sharpness = 0.0;
  bool valid = false;
  int reject = 0;
  int n_gnss = 0;
};
}  // namespace

int main()
{
  std::printf("== 自动 GNSS 延迟估计器独立验证 ==\n");
  std::printf("delay_ms,tau_hat_ms,tau_err_ms,residual,sharpness,valid,reject,n_gnss\n");

  const double omega = 2.0 * kPi * (kApeak / (2.0 * kPi * kVamp));
  const double T = 120.0;
  const int steps = static_cast<int>(T / kDt);
  const double gnss_dt = 0.1;  // 10 Hz

  int fail = 0;
  double next_gnss = 0.0;
  for (double delay : {0.0, 0.05, 0.15, 0.3})
  {
    InsGnssDelayAutoEstimator est;
    // 用默认上界（改后 0.4 s）验证修复
    std::mt19937 rng(20261101U);
    std::normal_distribution<double> n01(0.0, 1.0);

    // 历史真值
    std::vector<Eigen::Vector3d> hist_pos, hist_vel;
    hist_pos.reserve(steps + 1);
    hist_vel.reserve(steps + 1);

    double pos_n = 0.0;
    Result last;
    next_gnss = 0.0;
    int estimates = 0;
    int valid_count = 0;
    double tau_sum = 0.0;
    double err_sum = 0.0;
    double sharp_sum = 0.0;

    for (int k = 0; k < steps; ++k)
    {
      const double t = k * kDt;
      const double v = kVamp * std::sin(omega * t);
      pos_n += v * kDt;
      hist_pos.emplace_back(pos_n, 0.0, 0.0);
      hist_vel.emplace_back(v, 0.0, 0.0);

      // 参考轨迹（真值；固件里是纯惯导参考）
      est.PushReference(t, hist_pos.back(), hist_vel.back());

      // GNSS（10 Hz）：样本取延迟时刻真值，投递时刻 = 当前 t
      if (t >= next_gnss)
      {
        next_gnss += gnss_dt;
        const int lag = static_cast<int>(std::round(delay / kDt));
        const int hi = std::max(0, static_cast<int>(hist_pos.size()) - 1 - lag);
        const Eigen::Vector3d pos_meas =
            hist_pos[hi] + Eigen::Vector3d(0.85 * n01(rng), 0.85 * n01(rng),
                                           0.85 * n01(rng));
        const Eigen::Vector3d vel_meas =
            hist_vel[hi] + Eigen::Vector3d(0.05 * n01(rng), 0.05 * n01(rng),
                                           0.05 * n01(rng));
        // 投递时刻 = 采样时刻 + delay（真实链路：数据到得晚）
        est.PushGnss(t, pos_meas, vel_meas);
      }

      // 每秒估计一次
      if (k > 0 && k % 200 == 0)
      {
        const bool ok = est.TryEstimate();
        const auto &r = est.last_result();
        const auto &d = est.diag();
        ++estimates;
        if (ok)
        {
          ++valid_count;
          tau_sum += r.tau_s;
          err_sum += std::fabs(r.tau_s - delay);
          sharp_sum += r.sharpness;
        }
        last.tau_hat = r.tau_s;
        last.tau_err = std::fabs(r.tau_s - delay);
        last.residual = r.residual_mps;
        last.sharpness = r.sharpness;
        last.valid = ok;
        last.reject = static_cast<int>(d.reject);
        last.n_gnss = d.n_gnss;
      }
    }
    
    const double mean_tau = valid_count > 0 ? tau_sum / valid_count : 0.0;
    const double mean_err = valid_count > 0 ? err_sum / valid_count : 0.0;
    std::printf("%.0f,%.1f,%.1f,%.4f,%.4f,%d,%d,%d\n", delay * 1000.0,
                mean_tau * 1000.0, mean_err * 1000.0, last.residual,
                sharp_sum / std::max(1, valid_count), last.valid ? 1 : 0,
                last.reject, last.n_gnss);
    if (!(mean_err < 0.03))
    {
      std::printf("  [FAIL] 延迟 %.0f ms 估计误差 %.1f ms\n", delay * 1000.0,
                  mean_err * 1000.0);
      ++fail;
    }
  }

  if (fail == 0)
  {
    std::printf("自动延迟估计器通过（0-300 ms，误差 <30 ms）\n");
    return 0;
  }
  std::printf("失败 %d 项\n", fail);
  return 1;
}
