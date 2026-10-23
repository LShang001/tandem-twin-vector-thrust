/**
 * @file    test_ekf_delay_highspeed.cpp
 * @module  固件/宿主研究（fw-tests：GNSS/INS 延时补偿 × 高速运动）
 * @brief   扫描速度/延时/GNSS 噪声，验证时间戳补偿对高速位置精度的增益
 * @details 被测：`lib/navigation-main/src/ekf_15_state.h` 的
 *          `MeasurementUpdateDetailed(..., age=latency)` 回放融合路径。
 *          场景：匀速北向、GNSS 5 Hz、可配置链路延迟/速度/噪声；对比
 *          A 无时间戳补偿（延迟量测按“当前时刻”融合）与
 *          B 真实时间戳补偿（回放到量测时刻、融合后重传播到当前）。
 *          指标：最后 20 s 的水平位置 RMSE / 速度 RMSE。
 *          输出：三组扫描表 + 关键场景 5 seeds；回归门为
 *          “30 m/s、50 ms、0.30 m 噪声下位置 RMSE 至少改善 30%”。
 *          运行：见 run_all.sh 的 delayhs 目标（-DEKF_HOST_REGRESSION + convang 源）。
 * @theory  量测年龄补偿：EKF 把状态回放到量测时刻，融合后再传播到当前；
 *          高速时“用当前状态融旧量测”会引入约 v·τ 的位置偏差（30 m/s × 50 ms = 1.5 m），
 *          与 GNSS 本身 1.5 m 噪声同量级。@ref REF-121 §2.2.1 p.33（量测时标）
 * @frame   NED（水平位置误差按北/东 2D 范数）
 * @units   速度 m/s，延时 s，位置噪声 m，RMSE m / m/s
 * @note    算法级基线，不是实机/录包；不改固件默认参数。
 *          30% 门限取自报告中的实测改善量级（1.458→0.164 m ≈ 88.7%），留足余量
 *          以防随机种子波动，同时仍能拒掉“补偿无效”的实现。
 * @test    S1 延时扫描 0/15/30/50/100 ms；S2 速度扫描 5/15/30/50 m/s；
 *          S3 GNSS 位置噪声扫描 0.1/0.3/1.0/2.0 m；S4 关键场景 5 seeds 回归门
 * @see     TandemVec-EKF-Report/卫惯组合导航-延时补偿-高速精度-2026-10-22.md
 */
// ============================================================
#include <cmath>
#include <cstdio>
#include <deque>
#include <random>
#include <Eigen/Dense>
#include "ekf_15_state.h"
#include "transforms.h"

using bfs::Ekf15State;

namespace {
constexpr float kG = 9.79f;
constexpr double kPi = 3.14159265358979323846;
constexpr float kDt = 0.005f;       // 200 Hz
constexpr float kDurationS = 60.0f; // 最后 20 s 统计
constexpr float kGnssHz = 5.0f;
const Eigen::Vector3d kHome(31.2304 * kPi / 180.0,
                            121.4737 * kPi / 180.0, 50.0);

struct CaseResult {
  double pos_rmse_base = 0.0;
  double pos_rmse_comp = 0.0;
  double vel_rmse_base = 0.0;
  double vel_rmse_comp = 0.0;
  int fused_comp = 0;
  int rejected_comp = 0;
  double gainPct() const {
    return (pos_rmse_base > 1e-9) ? (1.0 - pos_rmse_comp / pos_rmse_base) * 100.0 : 0.0;
  }
};

struct GnssSample {
  double t;
  Eigen::Vector3f vel;
  Eigen::Vector3d lla;
};

double horizontalErrorM(const Ekf15State &ekf,
                        const Eigen::Vector3d &truth_ned) {
  const Eigen::Vector3d lla(ekf.lat_rad(), ekf.lon_rad(), ekf.alt_m());
  const Eigen::Vector3d est = bfs::lla2ned(lla, kHome);
  return std::hypot(est(0) - truth_ned(0), est(1) - truth_ned(1));
}

CaseResult runCase(float speed_mps, float latency_s, float pos_sigma_m,
                   unsigned seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<float> pos_noise(0.0f, pos_sigma_m);
  std::normal_distribution<float> vel_noise(0.0f, 0.05f);

  Ekf15State ekf_comp;
  Ekf15State ekf_base;
  const Eigen::Vector3f init_accel(0.0f, 0.0f, -kG);
  const Eigen::Vector3f init_gyro = Eigen::Vector3f::Zero();
  const Eigen::Vector3f init_mag = Eigen::Vector3f::Zero();
  const Eigen::Vector3f init_vel(speed_mps, 0.0f, 0.0f);
  ekf_comp.Initialize(init_accel, init_gyro, init_mag, init_vel, kHome);
  ekf_base.Initialize(init_accel, init_gyro, init_mag, init_vel, kHome);

  std::deque<GnssSample> pending;
  const int steps = static_cast<int>(kDurationS / kDt);
  const int gnss_every = static_cast<int>(1.0f / (kGnssHz * kDt));
  Eigen::Vector3d truth_pos(0.0, 0.0, 0.0);
  const Eigen::Vector3f truth_vel(speed_mps, 0.0f, 0.0f);

  CaseResult out;
  double sum_comp_pos2 = 0.0, sum_base_pos2 = 0.0;
  double sum_comp_vel2 = 0.0, sum_base_vel2 = 0.0;
  int metric_count = 0;

  for (int k = 0; k < steps; ++k) {
    const double t = k * kDt;
    truth_pos(0) += truth_vel(0) * kDt;

    ekf_comp.TimeUpdate(init_accel, init_gyro, kDt);
    ekf_base.TimeUpdate(init_accel, init_gyro, kDt);

    if (k % gnss_every == 0) {
      GnssSample s;
      s.t = t;
      s.vel = Eigen::Vector3f(truth_vel(0) + vel_noise(rng),
                              truth_vel(1) + vel_noise(rng),
                              truth_vel(2) + vel_noise(rng));
      s.lla = bfs::ned2lla(
          Eigen::Vector3d(truth_pos(0) + pos_noise(rng),
                          truth_pos(1) + pos_noise(rng), 0.0),
          kHome);
      pending.push_back(s);
    }

    while (!pending.empty() && (t - pending.front().t) >= latency_s) {
      const GnssSample s = pending.front();
      pending.pop_front();
      const float age = static_cast<float>(t - s.t);
      const auto rc = ekf_comp.MeasurementUpdateDetailed(
          s.vel, s.lla, kDt, age, true);
      if (rc.fused) ++out.fused_comp; else ++out.rejected_comp;
      ekf_base.MeasurementUpdateDetailed(s.vel, s.lla, kDt, 0.0f, false);
    }

    if (t >= 40.0) {
      const double ep_c = horizontalErrorM(ekf_comp, truth_pos);
      const double ep_b = horizontalErrorM(ekf_base, truth_pos);
      const double ev_c = (ekf_comp.ned_vel_mps() - truth_vel).norm();
      const double ev_b = (ekf_base.ned_vel_mps() - truth_vel).norm();
      sum_comp_pos2 += ep_c * ep_c;
      sum_base_pos2 += ep_b * ep_b;
      sum_comp_vel2 += ev_c * ev_c;
      sum_base_vel2 += ev_b * ev_b;
      ++metric_count;
    }
  }

  out.pos_rmse_comp = std::sqrt(sum_comp_pos2 / metric_count);
  out.pos_rmse_base = std::sqrt(sum_base_pos2 / metric_count);
  out.vel_rmse_comp = std::sqrt(sum_comp_vel2 / metric_count);
  out.vel_rmse_base = std::sqrt(sum_base_vel2 / metric_count);
  return out;
}

void printRow(const char *label, float speed, float latency, float sigma,
              const CaseResult &r) {
  std::printf("%-18s v=%5.1f m/s tau=%5.1f ms sig=%4.2f m | "
              "pos %6.3f -> %6.3f m (%+5.1f%%) | vel %5.3f -> %5.3f\n",
              label, speed, latency * 1000.0f, sigma,
              r.pos_rmse_base, r.pos_rmse_comp, r.gainPct(),
              r.vel_rmse_base, r.vel_rmse_comp);
}
} // namespace

int main() {
  std::printf("GNSS/INS delay compensation high-speed regression\n");
  std::printf("duration=%.0f s, stat window last 20 s, GNSS %.0f Hz\n\n",
              kDurationS, kGnssHz);

  // 1) 延迟扫描（30 m/s，0.30 m GNSS 噪声）
  std::printf("-- latency sweep --\n");
  const float lats[] = {0.0f, 0.015f, 0.030f, 0.050f, 0.100f};
  for (float tau : lats) {
    const auto r = runCase(30.0f, tau, 0.30f, 20261022u);
    char label[64];
    std::snprintf(label, sizeof(label), "latency");
    printRow(label, 30.0f, tau, 0.30f, r);
  }

  // 2) 速度扫描（50 ms，0.30 m 噪声）
  std::printf("\n-- speed sweep --\n");
  const float speeds[] = {5.0f, 15.0f, 30.0f, 50.0f};
  for (float v : speeds) {
    const auto r = runCase(v, 0.050f, 0.30f, 20261022u);
    printRow("speed", v, 0.050f, 0.30f, r);
  }

  // 3) GNSS 噪声扫描（30 m/s，50 ms）
  std::printf("\n-- GNSS position noise sweep --\n");
  const float sigmas[] = {0.10f, 0.30f, 1.00f, 2.00f};
  for (float s : sigmas) {
    const auto r = runCase(30.0f, 0.050f, s, 20261022u);
    printRow("sigma", 30.0f, 0.050f, s, r);
  }

  // 4) 关键场景 5 seeds
  std::printf("\n-- key case seeds (30 m/s, 50 ms, sigma=0.30 m) --\n");
  const auto key = runCase(30.0f, 0.050f, 0.30f, 20261022u);
  printRow("seed-20261022", 30.0f, 0.050f, 0.30f, key);
  for (unsigned seed = 1; seed <= 4; ++seed) {
    const auto r = runCase(30.0f, 0.050f, 0.30f, seed);
    char label[64];
    std::snprintf(label, sizeof(label), "seed-%u", seed);
    printRow(label, 30.0f, 0.050f, 0.30f, r);
  }

  // 回归门：50 ms / 30 m/s / 0.30 m 下位置 RMSE 至少改善 30%
  const bool pass = key.pos_rmse_comp < key.pos_rmse_base * 0.70;
  std::printf("\n[%s] 30 m/s, 50 ms: pos %.3f -> %.3f m (%+.1f%%), fused=%d rejected=%d\n",
              pass ? "PASS" : "FAIL", key.pos_rmse_base, key.pos_rmse_comp,
              key.gainPct(), key.fused_comp, key.rejected_comp);
  return pass ? 0 : 1;
}
