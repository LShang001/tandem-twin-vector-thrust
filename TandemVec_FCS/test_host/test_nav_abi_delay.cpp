/**
 * @file    test_nav_abi_delay.cpp
 * @brief   真实固件导航链路（fw_nav_api）验证 GNSS 延迟与自动估计。
 *
 * 目的（2026-11-01 用户要求）：尽量模拟真实固件飞行环境——直接调用固件
 * `handleNavigationSystem()`（fw_nav_api，含真实 UBX 解析器、门控、重锚、
 * 延迟快照回放、自动延迟估计），注入：
 *   - 正弦速度轨迹（v_amp、a_peak 可控）的 IMU 真值 + 手册噪声；
 *   - AHRS 四元数（姿态真值，等效 Madgwick 输出）；
 *   - GNSS 历元（lat/lon/alt/vel + iTOW），可把 iTOW 设为延迟时刻；
 *   - 气压高度（含噪声）。
 * 三种模式：
 *   A) 无延迟（iTOW=现在）；
 *   B) 真延迟 150 ms（iTOW=现在-150 ms，数据也是 150 ms 前）→ 固件应自动估计 τ̂≈0.15；
 *   C) 假时间标签（数据延迟 150 ms 但 iTOW=现在）→ 固件以为无延迟，自动估计无激励。
 *
 * 输出：位置 RMSE、`gnss_age_s`、`auto_delay_tau_s`、`auto_delay_valid`。
 */
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "fw_nav_api.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kLat = 31.2304 * kDeg2Rad;
constexpr double kLon = 121.4737 * kDeg2Rad;
constexpr double kAlt = 5.0;
constexpr double kG = 9.7938;
constexpr double kDt = 0.005;
constexpr double kOmegaIe = 7.292115e-5;
constexpr double kVamp = 5.0;
constexpr double kApeak = 2.0;

struct RunResult
{
  double pos_rmse = 0.0;
  double att_rmse_deg = 0.0;
  double mean_gnss_age_s = 0.0;
  double last_tau_s = 0.0;
  double mean_tau_s = 0.0;
  int tau_valid_count = 0;
  int reject = 0;
  int diag_n = 0;
  int diag_erased = 0;
  double diag_speed = 0.0;
  double diag_resid = 0.0;
  int steps = 0;
};

fw_nav_input_t make_input()
{
  fw_nav_input_t in;
  std::memset(&in, 0, sizeof(in));
  in.struct_size = sizeof(in);
  in.abi_version = FW_NAV_ABI_VERSION;
  in.dt_s = static_cast<float>(kDt);
  in.sample_count = 1;
  in.delta_count = 0;
  return in;
}

RunResult run(const bool delay_known, const double delay_s,
              const bool fake_itow, const unsigned seed)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> n01(0.0, 1.0);

  fw_nav *nav = nullptr;
  if (fw_nav_create(&nav) != 0)
  {
    std::printf("[FATAL] fw_nav_create 失败\n");
    return {};
  }
  fw_nav_reset(nav);
  std::printf("[diag] in_size=%zu out_size=%zu create_ok=1\n", sizeof(fw_nav_input_t), sizeof(fw_nav_output_t));

  const double omega = 2.0 * kPi * (kApeak / (2.0 * kPi * kVamp));
  const double T = 90.0;
  const int steps = static_cast<int>(T / kDt);
  const double gnss_dt = 0.1;  // 10 Hz
  double next_gnss = 0.0;
  int gnss_updates = 0;

  double pos_n = 0.0;
  double yaw = 0.0;
  double pos_err_sq = 0.0;
  double att_err_sq = 0.0;
  int samples = 0;
  double age_sum = 0.0;
  int last_reject = 0;
  int last_n = 0, last_erased = 0;
  double last_speed = 0.0, last_resid = 0.0;
  double tau_sum = 0.0;
  int tau_valid = 0;

  // 历史真值（供延迟量测）
  std::vector<double> hist_pos;
  std::vector<double> hist_vel;
  hist_pos.reserve(steps + 1);
  hist_vel.reserve(steps + 1);

  fw_nav_output_t out;
  std::memset(&out, 0, sizeof(out));
  out.struct_size = sizeof(out);

  for (int k = 0; k < steps; ++k)
  {
    const double t = k * kDt;
    const double v = kVamp * std::sin(omega * t);
    const double a = kVamp * omega * std::cos(omega * t);
    pos_n += v * kDt;
    yaw += 0.1 * kDeg2Rad * kDt;
    hist_pos.push_back(pos_n);
    hist_vel.push_back(v);

    const Eigen::Matrix3d C_bn =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d f_ned(a, 0.0, -kG);
    const Eigen::Vector3d w_ned(0.0, 0.0, 0.1 * kDeg2Rad);
    const Eigen::Vector3d f_b = C_bn.transpose() * f_ned;
    const Eigen::Vector3d w_b = C_bn.transpose() * w_ned;

    fw_nav_input_t in = make_input();
    // IMU（求和口径：sample_count=1 → 本拍单样本）
    in.accel_sum_body_mps2[0] = static_cast<float>(f_b(0) + 20e-3 * kG / 3.0 * n01(rng) + 70e-6 * kG / std::sqrt(kDt) * n01(rng));
    in.accel_sum_body_mps2[1] = static_cast<float>(f_b(1) + 20e-3 * kG / 3.0 * n01(rng) + 70e-6 * kG / std::sqrt(kDt) * n01(rng));
    in.accel_sum_body_mps2[2] = static_cast<float>(f_b(2) + 20e-3 * kG / 3.0 * n01(rng) + 70e-6 * kG / std::sqrt(kDt) * n01(rng));
    in.gyro_sum_body_radps[0] = static_cast<float>(w_b(0) + 0.4 * kDeg2Rad / 3.0 * n01(rng) + 0.0038 * kDeg2Rad / std::sqrt(kDt) * n01(rng));
    in.gyro_sum_body_radps[1] = static_cast<float>(w_b(1) + 0.4 * kDeg2Rad / 3.0 * n01(rng) + 0.0038 * kDeg2Rad / std::sqrt(kDt) * n01(rng));
    in.gyro_sum_body_radps[2] = static_cast<float>(w_b(2) + 0.4 * kDeg2Rad / 3.0 * n01(rng) + 0.0038 * kDeg2Rad / std::sqrt(kDt) * n01(rng));

    // AHRS 姿态真值
    const Eigen::Quaterniond q(C_bn);
    in.ahrs_quat_wxyz[0] = static_cast<float>(q.w());
    in.ahrs_quat_wxyz[1] = static_cast<float>(q.x());
    in.ahrs_quat_wxyz[2] = static_cast<float>(q.y());
    in.ahrs_quat_wxyz[3] = static_cast<float>(q.z());
    in.ahrs_valid = 1;

    // 气压
    in.baro_alt_up_m = static_cast<float>(0.0 + n01(rng));
    in.baro_valid = 1;
    in.unlocked = 1;
    in.throttle_pct = 50.0f;

    // GNSS（10 Hz）
    if (t >= next_gnss)
    {
      next_gnss += gnss_dt;
      const int lag = static_cast<int>(std::round((delay_known ? delay_s : (fake_itow ? 0.0 : 0.0)) / kDt));
      const int hi = std::max(0, static_cast<int>(hist_pos.size()) - 1 - lag);
      const double gnss_t = delay_known ? (t - delay_s) : (fake_itow ? (t - delay_s) : t);
      const std::uint32_t itow_ms =
          static_cast<std::uint32_t>(std::llround(gnss_t * 1000.0));
      const double r_n = 6378137.0;
      const double r_e = 6378137.0 * std::cos(kLat);
      in.gnss_valid = 1;
      in.gnss_lat_1e7 = static_cast<std::int32_t>(
          std::llround(((kLat + (hist_pos[hi] + 0.85 * n01(rng)) / r_n) / kDeg2Rad) * 1e7));
      in.gnss_lon_1e7 = static_cast<std::int32_t>(
          std::llround(((kLon + (0.85 * n01(rng)) / r_e) / kDeg2Rad) * 1e7));
      in.gnss_alt_mm = static_cast<std::int32_t>(std::llround((kAlt + n01(rng)) * 1000.0));
      in.gnss_vel_ned_mmps[0] = static_cast<std::int32_t>(std::llround((hist_vel[hi] + 0.05 * n01(rng)) * 1000.0));
      in.gnss_vel_ned_mmps[1] = static_cast<std::int32_t>(std::llround(0.05 * n01(rng) * 1000.0));
      in.gnss_vel_ned_mmps[2] = static_cast<std::int32_t>(std::llround(0.05 * n01(rng) * 1000.0));
      in.gnss_h_acc_mm = 850;
      in.gnss_v_acc_mm = 850;
      in.gnss_s_acc_mmps = 50;
      in.gnss_itow_ms = itow_ms;
      ++gnss_updates;
    }

    const int step_rc = fw_nav_step(nav, &in, &out);
    if (step_rc != 0)
    {
      std::printf("[FATAL] fw_nav_step rc=%d k=%d\n", step_rc, k);
      break;
    }

    if (t > 5.0 && k % 100 == 0)
    {
      // 位置误差：out.pos_ned_m 与真值（北向 pos_n）
      const double dn = out.pos_ned_m[0] - pos_n;
      const double de = out.pos_ned_m[1];
      const double pos_err = std::hypot(dn, de);
      const double att_err =
          std::fmax(std::fabs(out.euler_rpy_deg[0]), std::fabs(out.euler_rpy_deg[1]));
      pos_err_sq += pos_err * pos_err;
      att_err_sq += att_err * att_err;
      age_sum += out.gnss_age_s;
      last_reject = out.auto_delay_reject;
      last_n = out.auto_delay_n_gnss;
      last_erased = out.auto_delay_n_erased;
      last_speed = out.auto_delay_speed_max_mps;
      last_resid = out.auto_delay_residual_mps;
      if (out.auto_delay_valid)
      {
        tau_sum += out.auto_delay_tau_s;
        ++tau_valid;
      }
      ++samples;
    }
  }

  RunResult r;
  r.pos_rmse = samples > 0 ? std::sqrt(pos_err_sq / samples) : 0.0;
  r.att_rmse_deg = samples > 0 ? std::sqrt(att_err_sq / samples) : 0.0;
  r.mean_gnss_age_s = samples > 0 ? age_sum / samples : 0.0;
  r.last_tau_s = out.auto_delay_tau_s;
  r.mean_tau_s = tau_valid > 0 ? tau_sum / tau_valid : 0.0;
  r.tau_valid_count = tau_valid;
  r.reject = last_reject;
  r.diag_n = last_n;
  r.diag_erased = last_erased;
  r.diag_speed = last_speed;
  r.diag_resid = last_resid;
  r.steps = steps;
  fw_nav_destroy(nav);
  return r;
}
}  // namespace

int main(int argc, char **argv)
{
  // 单模式单进程：避免 fw_nav 单例跨 run 的初始化状态污染。
  const double delay_ms = (argc > 1) ? std::atof(argv[1]) : 0.0;
  const bool fake = (argc > 2) && (std::atoi(argv[2]) != 0);
  const bool known = !fake;
  const double d = delay_ms * 1e-3;
  std::printf("mode,pos_rmse,att_rmse_deg,age,tau,valid%,reject,n_gnss,n_erased,speed_max,resid\n");
  double pos = 0.0, att = 0.0, age = 0.0, tau = 0.0, tv = 0.0;
  int rej = 0;
  int diag_n = 0, diag_erased = 0;
  double diag_speed = 0.0, diag_resid = 0.0;
  const int seeds = 2;
  for (int s = 1; s <= seeds; ++s)
  {
    const RunResult r = run(known, d, fake, static_cast<unsigned>(s));
    pos += r.pos_rmse;
    att += r.att_rmse_deg;
    age += r.mean_gnss_age_s;
    tau += r.mean_tau_s;
    tv += r.tau_valid_count > 0 ? 1.0 : 0.0;
    rej = r.reject;
    diag_n = r.diag_n;
    diag_erased = r.diag_erased;
    diag_speed = r.diag_speed;
    diag_resid = r.diag_resid;
  }
  std::printf("%s_%.0fms,%.4f,%.3f,%.4f,%.4f,%.0f%%,%d\n",
              fake ? "fake_itow" : "known", delay_ms, pos / seeds,
              att / seeds, age / seeds, tau / seeds, tv / seeds * 100.0, rej,
              diag_n, diag_erased, diag_speed, diag_resid);
  return 0;
}
