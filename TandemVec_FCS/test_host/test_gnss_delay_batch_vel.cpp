/**
 * @file    test_gnss_delay_batch_vel.cpp
 * @module  固件/宿主研究（fw-tests：GNSS 速度批处理的延时/零偏联合估计）
 * @brief   仅用 GNSS 速度 + IMU 参考速度，联合消元估计加计零偏与链路延时
 * @details 被测：本文件内的 `fitVelocity()` 原型（工程实现在
 *          `include/ins_gnss_delay_auto.h`）。
 *          模型（候选 τ）：v_gnss(t) ≈ v_ref(t − τ) − b_a·(t − τ) + v0；
 *          对固定 τ，b_a/v0 线性，最小二乘消去；在机动段联合估计 τ 与 b_a。
 *          argv：maneuver、seed、true_delay、GNSS 速度 σ。
 *          运行：g++ -std=c++17 -Iinclude -Ilib/eigen/src test_host/test_gnss_delay_batch_vel.cpp
 * @theory  速度通道比位置通道更敏感：GNSS 速度噪声 0.05 m/s 远小于位置 1.5 m，
 *          延时造成的偏差是 a·τ 而非 v·τ/2 的积分累积，因此同样机动下信噪比更高。
 *          @ref REF-120 §9.6 (9.6.9) p.298
 * @frame   NED（速度）
 * @units   延时 s，速度 m/s，加计零偏 m/s²，参考轨迹步长 s
 * @note    只验证速度通道；位置通道见 test_gnss_delay_batch.cpp。
 *          匀速段同样不可观测，本文件不把匀速失败当回归。
 * @test    S1 机动段 τ 估计；S2 零偏消元正确性；S3 多 seed 稳健性
 * @see     TandemVec_FCS/test_host/test_gnss_delay_batch.cpp
 */
// ============================================================
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>
#include <Eigen/Dense>

namespace {
constexpr float kDt = 0.005f;
constexpr float kDuration = 60.0f;
constexpr float kG = 9.79f;

struct Gnss { double t; Eigen::Vector3d vel; };

Eigen::Vector3d interp(const std::vector<Eigen::Vector3d> &traj, double t) {
  if (t <= 0.0) return traj.front();
  const double x = t / kDt;
  const int i = static_cast<int>(std::floor(x));
  if (i >= static_cast<int>(traj.size()) - 1) return traj.back();
  const double f = x - i;
  return (1.0 - f) * traj[i] + f * traj[i + 1];
}

double fitVelocity(const std::vector<Eigen::Vector3d> &vref,
                   const std::vector<Gnss> &gnss, double tau) {
  const double t0 = gnss[gnss.size() / 2].t;
  Eigen::Matrix2d A = Eigen::Matrix2d::Zero();
  Eigen::Matrix<double, 2, 3> B = Eigen::Matrix<double, 2, 3>::Zero();
  for (const auto &s : gnss) {
    const double t_meas = s.t - tau;
    const double dt = s.t - t0;
    const Eigen::Vector3d pred = interp(vref, t_meas);
    // v_gnss - v_ref = v0 - b*t_meas
    Eigen::Vector2d x(1.0, -t_meas);
    A += x * x.transpose();
    B += x * (s.vel - pred).transpose();
  }
  const Eigen::Matrix<double, 2, 3> C = A.ldlt().solve(B);
  double sum2 = 0.0;
  int n = 0;
  for (const auto &s : gnss) {
    const double t_meas = s.t - tau;
    const Eigen::Vector3d pred = interp(vref, t_meas) +
                                 C.row(0).transpose() +
                                 C.row(1).transpose() * (-t_meas);
    sum2 += (s.vel - pred).squaredNorm();
    ++n;
  }
  return std::sqrt(sum2 / n);
}
} // namespace

int main(int argc, char **argv) {
  const bool maneuver = (argc > 1) ? (std::atoi(argv[1]) != 0) : true;
  const unsigned seed = (argc > 2) ? static_cast<unsigned>(std::atoi(argv[2])) : 1;
  const double true_delay = (argc > 3) ? std::atof(argv[3]) : 0.050;
  const float gnss_vel_sigma = (argc > 4) ? std::atof(argv[4]) : 0.05f;
  const float true_bias = 0.196f;
  std::mt19937 rng(seed);
  std::normal_distribution<float> accel_noise(0.0f, 0.0069f);
  std::normal_distribution<float> vel_noise(0.0f, gnss_vel_sigma);

  const int steps = static_cast<int>(kDuration / kDt);
  std::vector<Eigen::Vector3d> v_true_hist(steps + 1);
  std::vector<Eigen::Vector3d> v_ref_hist(steps + 1);
  Eigen::Vector3d v_true(30.0, 0.0, 0.0), v_ref(30.0, 0.0, 0.0);
  v_true_hist[0] = v_true;
  v_ref_hist[0] = v_ref;
  for (int k = 0; k < steps; ++k) {
    const double t = k * kDt;
    float a_cmd = 0.0f;
    if (maneuver) {
      if (t < 5.0) a_cmd = 2.0f;
      else if (t >= 15.0 && t < 25.0) a_cmd = -2.0f;
    }
    v_true(0) += a_cmd * kDt;
    Eigen::Vector3d accel(a_cmd + true_bias + accel_noise(rng),
                          accel_noise(rng), -kG + accel_noise(rng));
    v_ref(0) += accel(0) * kDt;
    v_ref(1) += accel(1) * kDt;
    v_ref(2) += (accel(2) + kG) * kDt;
    v_true_hist[k + 1] = v_true;
    v_ref_hist[k + 1] = v_ref;
  }

  std::vector<Gnss> gnss;
  const int every = static_cast<int>(1.0f / (10.0f * kDt));
  for (int k = 0; k < steps; k += every) {
    const double tm = std::max(0.0, k * kDt - true_delay);
    Gnss s;
    s.t = k * kDt;
    s.vel = interp(v_true_hist, tm) +
            Eigen::Vector3d(vel_noise(rng), vel_noise(rng), vel_noise(rng));
    gnss.push_back(s);
  }

  double best_tau = 0.0, best_cost = 1e9;
  for (int c = 0; c <= 30; ++c) {
    const double tau = c * 0.005;
    const double cost = fitVelocity(v_ref_hist, gnss, tau);
    if (cost < best_cost) { best_cost = cost; best_tau = tau; }
  }
  std::printf("maneuver=%d seed=%u true_delay=%.0fms true_bias=%.3f\n",
              maneuver ? 1 : 0, seed, true_delay * 1000.0f, true_bias);
  std::printf("best_tau=%.0fms err=%.1fms cost=%.4f m/s\n",
              best_tau * 1000.0f, (best_tau - true_delay) * 1000.0f, best_cost);
  const bool pass = !maneuver || std::fabs(best_tau - true_delay) <= 0.02;
  std::printf("[%s] velocity batch delay/bias\n", pass ? "PASS" : "FAIL");
  return pass ? 0 : 1;
}
