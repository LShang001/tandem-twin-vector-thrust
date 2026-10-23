/**
 * @file    test_gnss_delay_auto.cpp
 * @module  固件/宿主回归测试（fw-tests：无 PPS 的 GNSS 延时在线估计器）
 * @brief   机动窗口内验证延时估计收敛、野值剔除与 GNSS 失锁拒绝
 * @details 被测：`include/ins_gnss_delay_auto.h`（`InsGnssDelayAutoEstimator`：
 *          `PushGnss()`/`PushReference()`/`TryEstimate()`/`last_result()`）。
 *          输入：60 s、200 Hz 参考轨迹（真值 + 0.196 m/s² 加计零偏），
 *          GNSS 10 Hz、位置 1.5 m / 速度 0.05 m/s 噪声，真实延时默认 50 ms，
 *          机动段为 0–5 s 北向 +2、10–15 s 东向 +2、20–25 s 天向 -1、25–30 s 北向 -2；
 *          处理：只喂 0–30 s 机动窗口（之后匀速不可观测），TryEstimate(200, 20)。
 *          输出：估计有效、|τ̂−τ|<20 ms、sharpness≥0.003，且野值/失锁分支正确拒绝。
 *          运行：g++ -std=c++17 -Iinclude -Ilib/eigen/src test_host/test_gnss_delay_auto.cpp
 *          可选 argv：seed、true_delay、outlier_frac（默认 20261022/0.050/0.0）。
 * @theory  批处理对齐：对候选 τ 用“参考轨迹在 t−τ 的预测”拟合 GNSS 量测，
 *          残差最小者即延时；曲率（机动）是 τ 可观测的必要条件。
 *          @ref REF-120 §9.6 (9.6.9) p.298（速度/位置递推）、
 *          @ref REF-121 §2.2.1 p.33（量测时标假设）
 * @frame   NED（位置 m、速度 m/s），参考轨迹为纯 IMU 航位推算
 * @units   延时 s/ms，位置 m，速度 m/s，加计零偏 m/s²，sharpness 为残差曲率量
 * @note    20 ms 门限 = 候选网格 5 ms 的 4 倍；sharpness 门限 0.003 用于拒绝
 *          “平底残差”（匀速段所有 τ 残差相同，估计不可信）。
 *          历史事故：固件开关 `DISABLE_GNSS_DELAY_REPLAY` 会拒绝全部 GNSS，
 *          与本估计器不是同一路径，不能互相替代。
 * @test    T1 机动窗口估计收敛；T2 100 m/50 m/s 野值被剔除后仍有效；
 *          T3 GNSS 停在 20 s（失锁）时 TryEstimate 返回 false
 * @see     TandemVec_FCS/include/ins_gnss_delay_auto.h、src/navigation_task.cpp（默认开关）
 */
// 无 PPS 延时估计模块（ins_gnss_delay_auto.h）宿主回归
#include <cstdio>
#include <cmath>
#include <random>
#include <vector>
#include <Eigen/Dense>
#include "ins_gnss_delay_auto.h"

int main(int argc, char **argv)
{
  constexpr double dt = 0.005;
  constexpr double T = 60.0;
  const unsigned seed = (argc > 1) ? static_cast<unsigned>(std::atoi(argv[1])) : 20261022;
  const double true_delay = (argc > 2) ? std::atof(argv[2]) : 0.050;
  const double outlier_frac = (argc > 3) ? std::atof(argv[3]) : 0.0;
  constexpr double bias = 0.196;
  const int steps = static_cast<int>(T / dt);
  std::mt19937 rng(seed);
  std::normal_distribution<double> npos(0.0, 1.5);
  std::normal_distribution<double> nvel(0.0, 0.05);
  std::uniform_real_distribution<double> unif(0.0, 1.0);

  std::vector<Eigen::Vector3d> p_true(steps + 1), v_true(steps + 1);
  std::vector<Eigen::Vector3d> p_ref(steps + 1), v_ref(steps + 1);
  Eigen::Vector3d p(0, 0, 0), v(30, 0, 0), pr = p, vr = v;

  for (int k = 0; k < steps; ++k)
  {
    const double t = k * dt;
    Eigen::Vector3d a(0, 0, 0);
    if (t < 5.0) a(0) = 2.0;
    else if (t >= 10.0 && t < 15.0) a(1) = 2.0;
    else if (t >= 20.0 && t < 25.0) a(2) = -1.0;
    else if (t >= 25.0 && t < 30.0) a(0) = -2.0;
    v += a * dt;
    p += v * dt;
    Eigen::Vector3d am = a + Eigen::Vector3d(bias, 0, 0);
    vr += am * dt;
    pr += vr * dt;
    p_true[k + 1] = p;
    v_true[k + 1] = v;
    p_ref[k + 1] = pr;
    v_ref[k + 1] = vr;
  }

  InsGnssDelayAutoEstimator est;
  const int every = 20; // 10 Hz
  for (int k = 0; k < steps; k += every)
  {
    const double t = k * dt;
    if (t > 30.0) break; // 只在机动窗口内估计；50–60 s 已匀速、不可观测
    const double tm = std::max(0.0, t - true_delay);
    const int ti = static_cast<int>(std::lround(tm / dt));
    Eigen::Vector3d z = p_true[ti] +
                        Eigen::Vector3d(npos(rng), npos(rng), npos(rng));
    Eigen::Vector3d zv = v_true[ti] +
                         Eigen::Vector3d(nvel(rng), nvel(rng), nvel(rng));
    if (unif(rng) < outlier_frac) {
      z += Eigen::Vector3d(100.0, 100.0, 100.0);
      zv += Eigen::Vector3d(50.0, 50.0, 50.0);
    }
    est.PushGnss(t, z, zv);
  }

  // 参考轨迹按 200 Hz 缓存（与固件钩子一致）
  for (int k = 0; k <= steps; ++k) {
    if (k * dt > 30.0) break;
    est.PushReference(k * dt, p_ref[k], v_ref[k]);
  }

  const bool ok_est = est.TryEstimate(200, 20);
  const auto &r = est.last_result();
  std::printf("valid=%d tau=%.0f ms residual=%.3f sharpness=%.3f\n",
              ok_est ? 1 : 0, r.tau_s * 1000.0f, r.residual_mps, r.sharpness);
  // 野值：在正常估计后注入一个 100 m / 50 m/s 的坏样本，应被剔除。
  InsGnssDelayAutoEstimator outlier_est = est;
  outlier_est.PushReference(30.05, p_ref[static_cast<int>(30.05 / dt)],
                            v_ref[static_cast<int>(30.05 / dt)]);
  outlier_est.PushGnss(30.05, p_true[static_cast<int>(30.05 / dt)] +
                                  Eigen::Vector3d(100, 100, 100),
                       v_true[static_cast<int>(30.05 / dt)] +
                           Eigen::Vector3d(50, 50, 50));
  const bool outlier_ok = outlier_est.TryEstimate(200, 20);
  std::printf("outlier_ok=%d\n", outlier_ok ? 1 : 0);

  // GNSS 失锁：参考继续、GNSS 停在 20 s，估计必须拒绝。
  InsGnssDelayAutoEstimator stale;
  for (int k = 0; k <= steps; ++k) {
    if (k * dt > 30.0) break;
    stale.PushReference(k * dt, p_ref[k], v_ref[k]);
  }
  for (int k = 0; k < steps; k += every) {
    const double t = k * dt;
    if (t > 20.0) break;
    const double tm = std::max(0.0, t - true_delay);
    const int ti = static_cast<int>(std::lround(tm / dt));
    stale.PushGnss(t, p_true[ti], v_true[ti]);
  }
  const bool stale_rejected = !stale.TryEstimate(200, 20);
  std::printf("stale_rejected=%d\n", stale_rejected ? 1 : 0);

  const bool pass = ok_est && outlier_ok && stale_rejected &&
                    std::fabs(r.tau_s - true_delay) <= 0.02f &&
                    r.sharpness >= 0.003f;
  std::printf("[%s] ins_gnss_delay_auto\n", pass ? "PASS" : "FAIL");
  return pass ? 0 : 1;
}
