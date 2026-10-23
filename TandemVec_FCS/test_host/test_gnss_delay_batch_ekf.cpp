/**
 * @file    test_gnss_delay_batch_ekf.cpp
 * @module  固件/宿主研究（fw-tests：用 EKF 在线零偏支撑批处理延时估计）
 * @brief   验证“不输入真值零偏”时，EKF 估出的加计零偏能否让 τ 仍可辨识
 * @details 被测：`lib/navigation-main/src/ekf_15_state.h` 的零偏估计 +
 *          批处理拟合原型（`fitResidual()`）。
 *          与 test_gnss_delay_batch.cpp 的区别：不输入真值零偏；先用 15 状态 EKF
 *          在线估计 accel bias（GNSS 使用真实年龄）；再用
 *          `accel_bias_mps2()` 修正 IMU 参考轨迹；最后批处理拟合延时。
 *          argv：seed、maneuver 开关。
 *          运行：同 ekf15 目标（-DEKF_HOST_REGRESSION + convang 源）。
 * @theory  延时与零偏强耦合：参考轨迹的二次项由零偏贡献，若不先估计零偏，
 *          拟合会把零偏误当延时（或反之）。EKF 的零偏状态提供该解耦信息。
 *          @ref REF-120 §9.6 (9.6.9) p.298、@ref REF-121 §2.2.1 p.33
 * @frame   NED（位置/速度），零偏在机体系
 * @units   延时 s，位置 m，速度 m/s，加计零偏 m/s²，陀螺零偏 rad/s
 * @note    只在机动段可信；本文件的失败首先应怀疑零偏可观测性，
 *          而不是拟合器本身。参考轨迹固定 200 Hz 采样（与固件钩子一致）。
 * @test    S1 EKF 零偏修正后的 τ 估计；S2 机动/匀速对照；S3 多 seed 稳健性
 * @see     TandemVec_FCS/test_host/test_gnss_delay_batch.cpp、include/ins_gnss_delay_auto.h
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
constexpr float kDt = 0.005f;
constexpr float kDuration = 60.0f;
constexpr float kG = 9.79f;
constexpr double kPi = 3.14159265358979323846;
const double kTrueDelay = 0.050;
const Eigen::Vector3d kHome(31.2304 * kPi / 180.0,
                            121.4737 * kPi / 180.0, 50.0);

struct Gnss { double t; Eigen::Vector3f vel; Eigen::Vector3d lla; };

Eigen::Vector3d interp(const std::vector<Eigen::Vector3d> &traj, double t) {
  if (t <= 0.0) return traj.front();
  const double x = t / kDt;
  const int i = static_cast<int>(std::floor(x));
  if (i >= static_cast<int>(traj.size()) - 1) return traj.back();
  const double f = x - i;
  return (1.0 - f) * traj[i] + f * traj[i + 1];
}

double fitResidual(const std::vector<Eigen::Vector3d> &traj,
                   const std::vector<Gnss> &gnss, double tau) {
  const double t0 = gnss[gnss.size() / 2].t;
  Eigen::Matrix2d A = Eigen::Matrix2d::Zero();
  Eigen::Matrix<double, 2, 3> B = Eigen::Matrix<double, 2, 3>::Zero();
  for (const auto &s : gnss) {
    const double dt = s.t - t0;
    const Eigen::Vector3d p = bfs::lla2ned(s.lla, kHome);
    const Eigen::Vector3d pred = interp(traj, s.t - tau);
    Eigen::Vector2d x(1.0, dt);
    A += x * x.transpose();
    B += x * (p - pred).transpose();
  }
  const Eigen::Matrix<double, 2, 3> C = A.ldlt().solve(B);
  double sum2 = 0.0;
  int n = 0;
  for (const auto &s : gnss) {
    const double dt = s.t - t0;
    const Eigen::Vector3d p = bfs::lla2ned(s.lla, kHome);
    const Eigen::Vector3d pred = interp(traj, s.t - tau) +
                                 C.row(0).transpose() +
                                 C.row(1).transpose() * dt;
    sum2 += (p - pred).squaredNorm();
    ++n;
  }
  return std::sqrt(sum2 / n);
}
} // namespace

int main(int argc, char **argv) {
  const unsigned seed = (argc > 1) ? static_cast<unsigned>(std::atoi(argv[1])) : 1;
  const bool maneuver = (argc > 2) ? (std::atoi(argv[2]) != 0) : true;
  std::mt19937 rng(seed);
  std::normal_distribution<float> accel_noise(0.0f, 0.0f);
  std::normal_distribution<float> gnss_noise(0.0f, 1.5f);
  std::normal_distribution<float> vel_noise(0.0f, 0.05f);
  const float accel_bias = 0.196f;
  const float gyro_bias = 0.5f * (float)kPi / 180.0f;

  const int steps = static_cast<int>(kDuration / kDt);
  std::vector<Eigen::Vector3d> true_pos(steps + 1);
  std::vector<Eigen::Vector3d> ref_pos(steps + 1);
  std::vector<Eigen::Vector3f> accel_hist(steps);
  std::vector<Eigen::Vector3f> gyro_hist(steps);
  Eigen::Vector3d p_true(0.0, 0.0, 0.0), v_true(30.0, 0.0, 0.0);
  Eigen::Vector3d p_ref = p_true, v_ref = v_true;
  true_pos[0] = p_true;
  ref_pos[0] = p_ref;

  for (int k = 0; k < steps; ++k) {
    const double t = k * kDt;
    float a_cmd = 0.0f;
    if (maneuver) {
      if (t < 5.0) a_cmd = 2.0f;
      else if (t >= 15.0 && t < 25.0) a_cmd = -2.0f;
    }
    v_true(0) += a_cmd * kDt;
    p_true += v_true * kDt;
    Eigen::Vector3f accel(a_cmd + accel_bias + accel_noise(rng),
                          accel_noise(rng), -kG + accel_noise(rng));
    Eigen::Vector3f gyro(gyro_bias, 0.0f, 0.0f);
    accel_hist[k] = accel;
    gyro_hist[k] = gyro;
    // 原始 IMU 参考（含零偏），后续用 EKF 零偏修正
    v_ref(0) += accel(0) * kDt;
    v_ref(1) += accel(1) * kDt;
    v_ref(2) += (accel(2) + kG) * kDt;
    p_ref += v_ref * kDt;
    true_pos[k + 1] = p_true;
    ref_pos[k + 1] = p_ref;
  }

  std::vector<Gnss> gnss;
  const int every = static_cast<int>(1.0f / (10.0f * kDt));
  for (int k = 0; k < steps; k += every) {
    const double tm = std::max(0.0, k * kDt - kTrueDelay);
    Gnss s;
    s.t = k * kDt;
    s.vel = Eigen::Vector3f(v_true(0) + vel_noise(rng),
                            vel_noise(rng), vel_noise(rng));
    const Eigen::Vector3d p = interp(true_pos, tm) +
                              Eigen::Vector3d(gnss_noise(rng),
                                              gnss_noise(rng),
                                              gnss_noise(rng));
    s.lla = bfs::ned2lla(p, kHome);
    gnss.push_back(s);
  }

  // ---- EKF 在线估计零偏；GNSS 使用真实年龄 ----
  Ekf15State ekf;
  ekf.accel_std_mps2(0.25f);
  ekf.gyro_std_radps(0.0015f);
  ekf.gyro_markov_bias_std_radps(0.00524f);
  ekf.gnss_pos_ne_std_m(1.5f);
  ekf.gnss_pos_d_std_m(3.0f);
  ekf.gnss_vel_ne_std_mps(0.05f);
  ekf.gnss_vel_d_std_mps(0.10f);
  ekf.init_pos_err_std_m(2.0f);
  ekf.init_vel_err_std_mps(0.08f);
  ekf.init_att_err_std_rad(0.08f);
  ekf.init_heading_err_std_rad(0.2f);
  ekf.Initialize(Eigen::Vector3f(0.0f, 0.0f, -kG),
                 Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(),
                 Eigen::Vector3f(30.0f, 0.0f, 0.0f), kHome);
  std::deque<Gnss> pending;
  size_t next = 0;
  for (int k = 0; k < steps; ++k) {
    const double t = k * kDt;
    ekf.TimeUpdate(accel_hist[k], gyro_hist[k], kDt);
    while (next < gnss.size() && gnss[next].t <= t + 1e-9) {
      pending.push_back(gnss[next]);
      ++next;
    }
    while (!pending.empty() && (t - pending.front().t) >= kTrueDelay) {
      const Gnss s = pending.front();
      pending.pop_front();
      ekf.MeasurementUpdateDetailed(s.vel, s.lla, kDt,
                                    static_cast<float>(kTrueDelay), true);
    }
  }
  const Eigen::Vector3f bias_vec = ekf.accel_bias_mps2();
  const float bias_est = bias_vec(0);
  std::printf("seed=%u maneuver=%d EKF_bias=(%.4f,%.4f,%.4f) true=%.4f\n",
              seed, maneuver ? 1 : 0, bias_vec(0), bias_vec(1), bias_vec(2),
              accel_bias);

  // ---- 用 EKF 零偏修正参考轨迹，再做延时批处理 ----
  std::vector<Eigen::Vector3d> ref_corr = ref_pos;
  for (int k = 0; k <= steps; ++k) {
    const double t = k * kDt;
    ref_corr[k](0) -= 0.5 * bias_est * t * t;
  }

  double best_tau = 0.0, best_cost = 1e9;
  for (int c = 0; c <= 30; ++c) {
    const double tau = c * 0.005;
    const double cost = fitResidual(ref_corr, gnss, tau);
    if (cost < best_cost) { best_cost = cost; best_tau = tau; }
  }
  std::printf("best_tau=%.0f ms true=%.0f ms err=%.0f ms cost=%.3f m\n",
              best_tau * 1000.0, kTrueDelay * 1000.0,
              (best_tau - kTrueDelay) * 1000.0, best_cost);
  const bool pass = std::fabs(best_tau - kTrueDelay) <= 0.02;
  std::printf("[%s] EKF-bias batch delay\n", pass ? "PASS" : "FAIL");
  return pass ? 0 : 1;
}
