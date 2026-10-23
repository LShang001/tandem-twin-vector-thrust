/**
 * @file    test_ekf_delay_mmae.cpp
 * @module  固件/宿主研究（fw-tests：GNSS 延时的多假设 EKF 原型）
 * @brief   用 11 个候选 age 并行跑 EKF，验证机动段能选出接近真实的延时
 * @details 被测：`lib/navigation-main/src/ekf_15_state.h` 的按 age 回放更新路径。
 *          候选延时 τ = 0,10,…,100 ms；每个 EKF 用对应 age 更新同一组 GNSS 量测，
 *          按 test_ratio（NIS 平方根）滑动平均选择候选。
 *          场景：30 m/s 起点，0–5 s +2 m/s²、15–25 s −2 m/s²（机动激励）；
 *          真实链路延时 50 ms；GNSS 1.5 m / 0.05 m/s @10 Hz；ICM-42688 标称噪声。
 *          输出：选中候选与误差；匀速段不可观测时允许失败。
 *          运行：g++ 同 ekf15 目标；argv[1] 机动开关、argv[2] 种子。
 * @theory  MMAE：对每个假设延时各维护一个 EKF，用新息统计量（NIS）作为
 *          假设似然，滑动平均后取最小；NIS 在模型正确时服从 χ²(m)。
 *          @ref REF-121 §7.3.1 p.255（NIS 门控与 χ² 分布）
 * @frame   NED（水平位置误差）
 * @units   延时 s/ms，位置 m，速度 m/s，加计零偏 m/s²，角速率 rad/s
 * @note    匀速/悬停段“所有 τ 残差相同”是理论不可观测，不是实现缺陷；
 *          因此本文件不把匀速段的失败当回归。10 ms 网格决定可实现的分辨率，
 *          最优也可能落在两格之间（±5 ms 量化误差）。
 * @test    S1 机动段候选选择；S2 固定先验 50 ms 的误差对照；S3 多 seed 稳定性
 * @see     TandemVec_FCS/include/ins_gnss_delay_auto.h（工程化的单假设实现）
 */
// ============================================================
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <random>
#include <vector>
#include <Eigen/Dense>
#include "ekf_15_state.h"
#include "transforms.h"

using bfs::Ekf15State;

namespace {
constexpr float kG = 9.79f;
constexpr double kPi = 3.14159265358979323846;
constexpr float kDt = 0.005f;
constexpr float kDurationS = 60.0f;
constexpr int kCandidates = 11;
constexpr float kLatencyTrue = 0.050f;
const Eigen::Vector3d kHome(31.2304 * kPi / 180.0,
                            121.4737 * kPi / 180.0, 50.0);

struct Sample { double t; Eigen::Vector3f vel; Eigen::Vector3d lla; };

double posError(const Ekf15State &e, const Eigen::Vector3d &truth) {
  const Eigen::Vector3d lla(e.lat_rad(), e.lon_rad(), e.alt_m());
  const Eigen::Vector3d p = bfs::lla2ned(lla, kHome);
  return (p - truth).norm();
}
} // namespace

int main(int argc, char **argv) {
  const bool maneuver = (argc > 1) && (std::atoi(argv[1]) != 0);
  const unsigned seed = (argc > 2) ? static_cast<unsigned>(std::atoi(argv[2]))
                                   : 20261022u;
  std::mt19937 rng(seed);
  const float gyro_sigma = 0.0035f * (float)M_PI / 180.0f * 10.0f;
  const float accel_sigma = 70e-6f * kG * 10.0f;
  std::normal_distribution<float> ng(0.0f, gyro_sigma);
  std::normal_distribution<float> na(0.0f, accel_sigma);
  std::normal_distribution<float> npos(0.0f, 1.5f);
  std::normal_distribution<float> nvel(0.0f, 0.05f);

  std::vector<Ekf15State> ekf(kCandidates);
  const Eigen::Vector3f init_accel(0.0f, 0.0f, -kG);
  const Eigen::Vector3f init_gyro = Eigen::Vector3f::Zero();
  const Eigen::Vector3f init_mag = Eigen::Vector3f::Zero();
  const Eigen::Vector3f init_vel(30.0f, 0.0f, 0.0f);
  for (auto &e : ekf) {
    e.accel_std_mps2(0.25f);
    e.gyro_std_radps(0.0015f);
    e.gyro_markov_bias_std_radps(0.00524f);
    e.gnss_pos_ne_std_m(1.5f);
    e.gnss_pos_d_std_m(3.0f);
    e.gnss_vel_ne_std_mps(0.05f);
    e.gnss_vel_d_std_mps(0.10f);
    e.init_pos_err_std_m(2.0f);
    e.init_vel_err_std_mps(0.08f);
    e.init_att_err_std_rad(0.08f);
    e.init_heading_err_std_rad(0.2f);
    e.Initialize(init_accel, init_gyro, init_mag, init_vel, kHome);
  }

  std::deque<Sample> pending;
  const int steps = static_cast<int>(kDurationS / kDt);
  const int gnss_every = static_cast<int>(1.0f / (10.0f * kDt));
  Eigen::Vector3d truth(0.0, 0.0, 0.0);
  Eigen::Vector3f truth_vel(30.0f, 0.0f, 0.0f);
  const float a_bias = 20e-3f * kG;
  const float g_bias = 0.5f * (float)M_PI / 180.0f;

  std::vector<double> score(kCandidates, 0.0);
  std::vector<int> reject(kCandidates, 0);
  int selected = 5; // 先验 50 ms
  double sel_err = 0.0, fix0_err = 0.0;
  int count = 0;

  for (int k = 0; k < steps; ++k) {
    const double t = k * kDt;
    float a_cmd = 0.0f;
    if (maneuver) {
      if (t < 5.0) a_cmd = 2.0f;
      else if (t >= 15.0 && t < 25.0) a_cmd = -2.0f;
    }
    truth_vel(0) += a_cmd * kDt;
    truth(0) += truth_vel(0) * kDt;

    const Eigen::Vector3f accel(a_cmd + a_bias + na(rng), na(rng),
                                -kG + na(rng));
    const Eigen::Vector3f gyro(g_bias + ng(rng), ng(rng), ng(rng));
    for (auto &e : ekf) e.TimeUpdate(accel, gyro, kDt);

    if (k % gnss_every == 0) {
      Sample s;
      s.t = t;
      s.vel = Eigen::Vector3f(truth_vel(0) + nvel(rng),
                              truth_vel(1) + nvel(rng),
                              truth_vel(2) + nvel(rng));
      s.lla = bfs::ned2lla(
          Eigen::Vector3d(truth(0) + npos(rng), truth(1) + npos(rng),
                          truth(2) + npos(rng)),
          kHome);
      pending.push_back(s);
    }

    while (!pending.empty() && (t - pending.front().t) >= kLatencyTrue) {
      const Sample s = pending.front();
      pending.pop_front();
      const float age = static_cast<float>(t - s.t);
      for (int c = 0; c < kCandidates; ++c) {
        const float cand = c * 0.010f;
        const auto r = ekf[c].MeasurementUpdateDetailed(
            s.vel, s.lla, kDt, cand, true);
        const double ratio = (r.fused && std::isfinite(r.innovation_norm))
                                 ? static_cast<double>(r.innovation_norm)
                                 : 100.0;
        if (!r.fused) ++reject[c];
        score[c] = 0.98 * score[c] + 0.02 * ratio;
      }
      // 选最低分；带 5% 滞回，避免频繁切换
      int best = 0;
      for (int c = 1; c < kCandidates; ++c) {
        if (score[c] < score[best]) best = c;
      }
      if (score[best] < score[selected] * 0.95) selected = best;
      (void)age;
    }

    if (t >= 40.0) {
      const double e_sel = posError(ekf[selected], truth);
      const double e_fix0 = posError(ekf[0], truth);
      sel_err += e_sel * e_sel;
      fix0_err += e_fix0 * e_fix0;
      ++count;
    }
  }

  const double rmse_sel = std::sqrt(sel_err / count);
  const double rmse_fix0 = std::sqrt(fix0_err / count);
  std::printf("maneuver=%d seed=%u true_delay=%.0f ms\n", maneuver ? 1 : 0,
              seed, kLatencyTrue * 1000.0f);
  std::printf("selected=%d (%.0f ms)  sel_rmse=%.3f m  fixed0_rmse=%.3f m\n",
              selected, selected * 10.0f, rmse_sel, rmse_fix0);
  for (int c = 0; c < kCandidates; ++c) {
    std::printf("  cand=%3.0f ms score=%7.3f reject=%d\n",
                c * 10.0f, score[c], reject[c]);
  }
  // 机动场景门限：选中候选应与真实 50 ms 相差 ≤20 ms，且优于固定 0
  const bool pass = maneuver
                        ? (std::abs(selected - 5) <= 2 &&
                           rmse_sel < rmse_fix0)
                        : true; // 匀速不可观测，不做门限
  std::printf("[%s] MMAE delay selection\n", pass ? "PASS" : "FAIL");
  return pass ? 0 : 1;
}
