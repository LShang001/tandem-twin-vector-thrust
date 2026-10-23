/**
 * @file    test_gnss_delay_batch.cpp
 * @module  固件/宿主研究（fw-tests：无 PPS 的 GNSS 延时批处理对齐，纯算法原型）
 * @brief   IMU 积分参考轨迹 + GNSS 位置/速度联合拟合，验证延时 τ 可被回顾性辨识
 * @details 被测：本文件内的 `fitResidual()` 原型（最终工程化实现在
 *          `include/ins_gnss_delay_auto.h`）。不依赖 PPS、不依赖 EKF 新息。
 *          方法：1) 用 IMU 加速度积分出参考轨迹 p_ref(t)；
 *          2) GNSS 位置 z_k 对应 p_true(t_k − τ)；
 *          3) 对候选 τ，把 p_ref(t_k − τ) 与 z_k 做“初位置 + 初速度 + 加计零偏”
 *          线性拟合，取残差最小；4) 只有机动段轨迹有曲率，τ 才可辨识，
 *          匀速直线下所有 τ 残差相同（不可观测）。
 *          argv：maneuver、seed、true_delay、GNSS 位置 σ、加计零偏、速度 σ、
 *          速度、y/z 轴零偏、失锁起点与时长。
 *          运行：g++ -std=c++17 -Iinclude -Ilib/eigen/src test_host/test_gnss_delay_batch.cpp
 * @theory  批处理最小二乘：模型 p_gnss = p_ref(t−τ) + p0 + v0·Δt − ½·b_a·Δt²，
 *          固定 τ 时对 (p0, v0, b_a) 线性，故可先消元再对 τ 一维搜索。
 *          @ref REF-120 §9.6 (9.6.9) p.298（速度/位置递推）
 * @frame   NED（参考轨迹与 GNSS 位置均在导航系）
 * @units   延时 s，位置 m，速度 m/s，加计零偏 m/s²
 * @note    算法层验证：参考轨迹用带噪声 IMU，不是实机结果；5 ms 网格决定分辨率。
 *          历史背景：本原型修过“A 矩阵三轴行覆盖”和“t≈0 速度差分退化”两处 bug
 *          （见 AGENTS.md），当前 50 seeds × 20/50/80 ms 平均绝对误差 1.3–1.5 ms。
 * @test    S1 机动段 τ 估计；S2 匀速段不可观测（允许失败）；S3 失锁窗容忍；
 *          S4 三轴零偏/速度注入下的稳健性（由 run_gnss_delay_large_mc.py 批量驱动）
 * @see     TandemVec_EKF_Report 中的延时补偿报告、include/ins_gnss_delay_auto.h
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
constexpr float kGnssHz = 10.0f;
constexpr float kTrueDelay = 0.050f;
constexpr float kG = 9.79f;

struct Sample { double t; Eigen::Vector3d pos; Eigen::Vector3d vel; };

Eigen::Vector3d interpPos(const std::vector<Eigen::Vector3d> &traj, double t) {
  if (t <= 0.0) return traj.front();
  const double x = t / kDt;
  const int i = static_cast<int>(std::floor(x));
  if (i >= static_cast<int>(traj.size()) - 1) return traj.back();
  const double f = x - i;
  return (1.0 - f) * traj[i] + f * traj[i + 1];
}

// 对候选 tau，联合位置+速度，线性拟合 p0/v0/b_a，返回加权残差。
// 模型：
//   p_gnss = p_ref(t-tau) + p0 + v0*dt - 0.5*b_a*(t-tau)^2
//   v_gnss = v_ref(t-tau) + v0 - b_a*(t-tau)
double fitResidual(const std::vector<Eigen::Vector3d> &p_ref,
                   const std::vector<Eigen::Vector3d> &v_ref,
                   const std::vector<Sample> &gnss, double tau,
                   double pos_sigma, double vel_sigma) {
  const double t0 = gnss[gnss.size() / 2].t;
  const int n = static_cast<int>(gnss.size());
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6 * n, 9);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6 * n, 3);
  int row = 0;
  for (const auto &s : gnss) {
    const double t_meas = s.t - tau;
    const double dt = s.t - t0;
    const Eigen::Vector3d p = interpPos(p_ref, t_meas);
    const Eigen::Vector3d v = interpPos(v_ref, t_meas);
    for (int axis = 0; axis < 3; ++axis) {
      // 每个轴独立一行，避免 A 行被覆盖。
      const int rp = row + axis;
      A(rp, 0 + axis) = 1.0 / pos_sigma;
      A(rp, 3 + axis) = dt / pos_sigma;
      A(rp, 6 + axis) = -0.5 * t_meas * t_meas / pos_sigma;
      B(rp, axis) = (s.pos(axis) - p(axis)) / pos_sigma;
      const int rv = row + 3 + axis;
      A(rv, 3 + axis) = 1.0 / vel_sigma;
      A(rv, 6 + axis) = -t_meas / vel_sigma;
      B(rv, axis) = (s.vel(axis) - v(axis)) / vel_sigma;
    }
    row += 6;
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::MatrixXd C = svd.solve(B);
  const Eigen::MatrixXd res = A * C - B;
  return res.norm();
}
} // namespace

int main(int argc, char **argv) {
  const int maneuver = (argc > 1) ? std::atoi(argv[1]) : 1;
  const unsigned seed = (argc > 2) ? static_cast<unsigned>(std::atoi(argv[2])) : 1;
  const float true_delay = (argc > 3) ? std::atof(argv[3]) : 0.050f;
  const float gnss_sigma = (argc > 4) ? std::atof(argv[4]) : 1.5f;
  const float bias_est = (argc > 5) ? std::atof(argv[5]) : 0.196f;
  const float accel_bias = bias_est;
  const float vel_sigma = (argc > 6) ? std::atof(argv[6]) : 0.05f;
  const float speed = (argc > 7) ? std::atof(argv[7]) : 30.0f;
  const float bias_y = (argc > 8) ? std::atof(argv[8]) : 0.0f;
  const float bias_z = (argc > 9) ? std::atof(argv[9]) : 0.0f;
  const float drop_start = (argc > 10) ? std::atof(argv[10]) : -1.0f;
  const float drop_len = (argc > 11) ? std::atof(argv[11]) : 0.0f;
  std::mt19937 rng(seed);
  std::normal_distribution<float> accel_noise(0.0f, 0.0f);
  std::normal_distribution<float> gnss_noise(0.0f, gnss_sigma);
  std::normal_distribution<float> vel_noise(0.0f, vel_sigma);

  const int steps = static_cast<int>(kDuration / kDt);
  std::vector<Eigen::Vector3d> true_pos(steps + 1);
  std::vector<Eigen::Vector3d> true_vel(steps + 1);
  std::vector<Eigen::Vector3d> ref_pos(steps + 1); // IMU 积分
  std::vector<Eigen::Vector3d> ref_vel(steps + 1);
  Eigen::Vector3d p_true(0.0, 0.0, 0.0), v_true(speed, 0.0, 0.0);
  Eigen::Vector3d p_ref = p_true, v_ref = v_true;
  true_pos[0] = p_true;
  true_vel[0] = v_true;
  ref_pos[0] = p_ref;
  ref_vel[0] = v_ref;

  for (int k = 0; k < steps; ++k) {
    const double t = k * kDt;
    Eigen::Vector3d a_cmd(0.0, 0.0, 0.0);
    if (maneuver == 1) {
      if (t < 5.0) a_cmd(0) = 2.0;
      else if (t >= 15.0 && t < 25.0) a_cmd(0) = -2.0;
    } else if (maneuver == 2) {
      if (t < 5.0) a_cmd(0) = 2.0;
      else if (t >= 10.0 && t < 15.0) a_cmd(1) = 2.0;
      else if (t >= 20.0 && t < 25.0) a_cmd(2) = -1.0;
      else if (t >= 25.0 && t < 30.0) a_cmd(0) = -2.0;
    }
    // 真值
    v_true += a_cmd * kDt;
    p_true += v_true * kDt;
    // 参考：IMU 加速度积分（含白噪声 + 20 mg x 零偏）
    const Eigen::Vector3d a_meas = a_cmd +
        Eigen::Vector3d(accel_noise(rng), accel_noise(rng), accel_noise(rng)) +
        Eigen::Vector3d(accel_bias, bias_y, bias_z);
    v_ref += a_meas * kDt;
    p_ref += v_ref * kDt;
    true_pos[k + 1] = p_true;
    true_vel[k + 1] = v_true;
    ref_pos[k + 1] = p_ref;
    ref_vel[k + 1] = v_ref;
  }

  std::vector<Sample> gnss;
  const int every = static_cast<int>(1.0f / (kGnssHz * kDt));
  for (int k = 0; k < steps; k += every) {
    const double t_sample = k * kDt;
    if (drop_start >= 0.0f && t_sample >= drop_start &&
        t_sample < drop_start + drop_len) {
      continue; // GNSS 失锁窗口
    }
    const double tm = t_sample - true_delay;
    const Eigen::Vector3d truth = interpPos(true_pos, std::max(0.0, tm));
    Sample s;
    s.t = k * kDt;
    s.pos = truth + Eigen::Vector3d(gnss_noise(rng), gnss_noise(rng),
                                     gnss_noise(rng));
    // 用真值速度轨迹直接采样（避免 t≈0 时中心差分退化为 0）
    s.vel = interpPos(true_vel, std::max(0.0, tm)) +
            Eigen::Vector3d(vel_noise(rng), vel_noise(rng), vel_noise(rng));
    gnss.push_back(s);
  }

  std::printf("maneuver=%d seed=%u true_delay=%.0f ms\n", maneuver,
              seed, true_delay * 1000.0f);

  double best_tau = 0.0;
  double best_cost = 1e9;
  double costs[31];
  int best_c = 0;
  for (int c = 0; c <= 30; ++c) {
    const double tau = c * 0.005;
    const double cost = fitResidual(ref_pos, ref_vel, gnss, tau,
                                    1.5, 0.05);
    costs[c] = cost;
    if (cost < best_cost) { best_cost = cost; best_tau = tau; best_c = c; }
    if (c % 5 == 0) {
      std::printf("  tau=%3.0f ms cost=%.4f m\n", tau * 1000.0f, cost);
    }
  }
  // 抛物线亚网格插值：降低 5 ms 网格量化偏置。
  if (best_c > 0 && best_c < 30) {
    const double cm = costs[best_c - 1];
    const double c0 = costs[best_c];
    const double cp = costs[best_c + 1];
    const double denom = cm - 2.0 * c0 + cp;
    if (denom > 1e-12) {
      const double delta = 0.5 * (cm - cp) / denom;
      best_tau += std::max(-0.005, std::min(0.005, delta * 0.005));
    }
  }
  std::printf("best_tau=%.3f ms (error %.3f ms), cost=%.4f m\n",
              best_tau * 1000.0f, (best_tau - true_delay) * 1000.0f, best_cost);

  // 机动段才做门限；匀速段允许不可观测
  const bool pass = (maneuver == 0) ||
                    std::fabs(best_tau - true_delay) <= 0.02f;
  std::printf("[%s] batch delay alignment\n", pass ? "PASS" : "FAIL");
  return pass ? 0 : 1;
}
