/**
 * @file    test_nav_abi_delay.cpp
 * @module   测试/宿主机
 * @brief   真实固件导航链路（fw_nav_api）验证 GNSS 延迟与自动估计。
 * @details 输入：北向正弦速度真值（v_amp 5 m/s、a_peak 2 m/s²）、ICM-45686
 *          噪声（70 µg/√Hz、0.0038 °/s/√Hz + 零偏）、F10N GNSS 量测
 *          （0.85 m/0.05 m/s，10 Hz）；argv[1]=延迟 ms、argv[2]=fake_itow；
 *          处理：逐 5 ms 调 fw_nav_step 驱动真实固件导航，GNSS 取延迟时刻真值
 *          注入并按 age 回放；输出：pos RMSE、att、gnss_age、τ̂、valid%、
 *          reject、n_gnss、n_erased、speed_max、residual。判定：只打印 CSV，
 *          无断言（供延迟/自动估计 A/B 对比）。
 * @frame   NED：位置/速度取北向（LLA 站点 31.2304°N/121.4737°E/5 m）；机体系
 *          输入为比力/角速度（C_b^n 由航向 0.1 °/s 慢转）；AHRS 四元数按
 *          FRD←NED（C_n^b）传入，与固件 ABI 约定一致。
 * @units   位置 m、速度 m/s、比力 m/s²、角速度 rad/s、姿态 deg（输出接口）、
 *          延迟与 τ 以 ms 打印、时间 s；GNSS 整型接口用 1e-7 deg、mm、mm/s。
 *
 * 单模式单进程（argv: delay_ms [fake_itow]），避免 fw_nav 单例跨 run 污染。
 * 输出：pos RMSE、gnss_age、τ̂、valid、reject、n_gnss、n_erased、speed_max、residual。
 */
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ins_test_gravity.h"

#include "fw_nav_api.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kLat = 31.2304 * kDeg2Rad;
constexpr double kLon = 121.4737 * kDeg2Rad;
constexpr double kAlt = 5.0;
const double kG = static_cast<double>(ins_test::GravityWgs84());  // WGS84 正常重力，与 EKF 同口径
constexpr double kDt = 0.005;
constexpr double kVamp = 5.0;
constexpr double kApeak = 2.0;

struct Result
{
  double pos_rmse = 0.0;
  double att_deg = 0.0;
  double age = 0.0;
  double tau = 0.0;
  int valid = 0;
  int reject = 0;
  int n_gnss = 0;
  int n_erased = 0;
  double speed_max = 0.0;
  double residual = 0.0;
};

Result run(const bool fake_itow, const double delay_s, const unsigned seed)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> n01(0.0, 1.0);

  fw_nav *nav = nullptr;
  if (fw_nav_create(&nav) != 0)
  {
    std::printf("[FATAL] create\n");
    return {};
  }
  fw_nav_reset(nav);

  const double omega = 2.0 * kPi * (kApeak / (2.0 * kPi * kVamp));
  const double T = 90.0;
  const int steps = static_cast<int>(T / kDt);
  const double gnss_dt = 0.1;
  double next_gnss = 0.0;

  double pos_n = 0.0;
  double yaw = 0.0;
  double pos_err_sq = 0.0;
  double att_sq = 0.0;
  double age_sum = 0.0;
  double tau_sum = 0.0;
  int samples = 0;
  int valid_cnt = 0;
  int n_gnss = 0, n_erased = 0;
  double speed_max = 0.0, residual = 0.0;
  int reject = 0;

  std::vector<double> hist_pos, hist_vel;
  hist_pos.reserve(steps + 1);
  hist_vel.reserve(steps + 1);

  fw_nav_output_t out;
  std::memset(&out, 0, sizeof(out));
  out.struct_size = sizeof(out);

  for (int k = 0; k < steps; ++k)
  {
    const double t = k * kDt;
    // 前 1 s 静止：让 EKF Initialize 得到正确的水平姿态（TiltCompass 用比力）
    const bool still = (t < 1.0);
    const double v = still ? 0.0 : kVamp * std::sin(omega * (t - 1.0));
    const double a = still ? 0.0 : kVamp * omega * std::cos(omega * (t - 1.0));
    pos_n += v * kDt;
    if (!still) yaw += 0.1 * kDeg2Rad * kDt;
    hist_pos.push_back(pos_n);
    hist_vel.push_back(v);

    const Eigen::Matrix3d C_bn =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d f_ned(a, 0.0, -kG);
    const Eigen::Vector3d w_ned(0.0, 0.0, 0.1 * kDeg2Rad);
    const Eigen::Vector3d f_b = C_bn.transpose() * f_ned;
    const Eigen::Vector3d w_b = C_bn.transpose() * w_ned;

    fw_nav_input_t in;
    std::memset(&in, 0, sizeof(in));
    in.struct_size = sizeof(in);
    in.abi_version = FW_NAV_ABI_VERSION;
    in.dt_s = static_cast<float>(kDt);
    in.sample_count = 1;
    const double fs = 1.0 / kDt;
    for (int ax = 0; ax < 3; ++ax)
    {
      in.accel_sum_body_mps2[ax] = static_cast<float>(
          f_b(ax) + 20e-3 * kG / 3.0 * n01(rng) +
          70e-6 * kG * std::sqrt(fs) * n01(rng));
      in.gyro_sum_body_radps[ax] = static_cast<float>(
          w_b(ax) + 0.4 * kDeg2Rad / 3.0 * n01(rng) +
          0.0038 * kDeg2Rad * std::sqrt(fs) * n01(rng));
    }
    // AHRS 四元数约定 FRD←NED（C_n^b），不是 body→NED
    const Eigen::Quaterniond q(C_bn.transpose());
    in.ahrs_quat_wxyz[0] = static_cast<float>(q.w());
    in.ahrs_quat_wxyz[1] = static_cast<float>(q.x());
    in.ahrs_quat_wxyz[2] = static_cast<float>(q.y());
    in.ahrs_quat_wxyz[3] = static_cast<float>(q.z());
    in.ahrs_valid = 1;
    in.baro_alt_up_m = static_cast<float>(n01(rng));
    in.baro_valid = 1;
    in.unlocked = 1;
    in.throttle_pct = 50.0f;

    if (t >= next_gnss)
    {
      next_gnss += gnss_dt;
      const int lag = static_cast<int>(std::llround(delay_s / kDt));
      const int hi = std::max(0, static_cast<int>(hist_pos.size()) - 1 - lag);
      const double gnss_t = t - delay_s;
      const double r_n = 6378137.0;
      const double r_e = 6378137.0 * std::cos(kLat);
      in.gnss_valid = 1;
      in.gnss_lat_1e7 = static_cast<std::int32_t>(std::llround(
          ((kLat + (hist_pos[hi] + 0.85 * n01(rng)) / r_n) / kDeg2Rad) * 1e7));
      in.gnss_lon_1e7 = static_cast<std::int32_t>(std::llround(
          ((kLon + (0.85 * n01(rng)) / r_e) / kDeg2Rad) * 1e7));
      in.gnss_alt_mm = static_cast<std::int32_t>(std::llround((kAlt + n01(rng)) * 1000.0));
      in.gnss_vel_ned_mmps[0] = static_cast<std::int32_t>(std::llround((hist_vel[hi] + 0.05 * n01(rng)) * 1000.0));
      in.gnss_vel_ned_mmps[1] = static_cast<std::int32_t>(std::llround(0.05 * n01(rng) * 1000.0));
      in.gnss_vel_ned_mmps[2] = static_cast<std::int32_t>(std::llround(0.05 * n01(rng) * 1000.0));
      in.gnss_h_acc_mm = 850;
      in.gnss_v_acc_mm = 850;
      in.gnss_s_acc_mmps = 50;
      in.gnss_itow_ms = fake_itow
                            ? static_cast<std::uint32_t>(std::llround(t * 1000.0))
                            : static_cast<std::uint32_t>(std::llround(gnss_t * 1000.0));
    }

    if (fw_nav_step(nav, &in, &out) != 0)
    {
      std::printf("[FATAL] step k=%d\n", k);
      break;
    }

    if (t > 5.0 && k % 100 == 0)
    {
      const double dn = out.pos_ned_m[0] - pos_n;
      const double de = out.pos_ned_m[1];
      const double pos_err = std::hypot(dn, de);
      const double att_err =
          std::fmax(std::fabs(out.euler_rpy_deg[0]), std::fabs(out.euler_rpy_deg[1]));
      pos_err_sq += pos_err * pos_err;
      att_sq += att_err * att_err;
      age_sum += out.gnss_age_s;
      n_gnss = out.auto_delay_n_gnss;
      n_erased = out.auto_delay_n_erased;
      speed_max = out.auto_delay_speed_max_mps;
      residual = out.auto_delay_residual_mps;
      reject = out.auto_delay_reject;
      if (out.auto_delay_valid)
      {
        tau_sum += out.auto_delay_tau_s;
        ++valid_cnt;
      }
      ++samples;
    }
  }

  Result r;
  r.pos_rmse = samples > 0 ? std::sqrt(pos_err_sq / samples) : 0.0;
  r.att_deg = samples > 0 ? std::sqrt(att_sq / samples) : 0.0;
  r.age = samples > 0 ? age_sum / samples : 0.0;
  r.tau = valid_cnt > 0 ? tau_sum / valid_cnt : 0.0;
  r.valid = valid_cnt * 100 / std::max(1, samples);
  r.reject = reject;
  r.n_gnss = n_gnss;
  r.n_erased = n_erased;
  r.speed_max = speed_max;
  r.residual = residual;
  fw_nav_destroy(nav);
  return r;
}
}  // namespace

int main(int argc, char **argv)
{
  const double delay_ms = (argc > 1) ? std::atof(argv[1]) : 0.0;
  const bool fake = (argc > 2) && (std::atoi(argv[2]) != 0);
  const double d = delay_ms * 1e-3;

  std::printf("mode,pos_rmse,att_deg,age,tau,valid%%,reject,n_gnss,n_erased,speed_max,resid\n");
  double pos = 0.0, att = 0.0, age = 0.0, tau = 0.0, valid = 0.0;
  int rej = 0, ng = 0, ne = 0;
  double spd = 0.0, res = 0.0;
  const int seeds = 2;
  for (int s = 1; s <= seeds; ++s)
  {
    const Result r = run(fake, d, static_cast<unsigned>(s));
    pos += r.pos_rmse;
    att += r.att_deg;
    age += r.age;
    tau += r.tau;
    valid += r.valid;
    rej = r.reject;
    ng = r.n_gnss;
    ne = r.n_erased;
    spd = r.speed_max;
    res = r.residual;
  }
  std::printf("%s_%.0fms,%.4f,%.4f,%.4f,%.4f,%.0f%%,%d,%d,%d,%.2f,%.3f\n",
              fake ? "fake_itow" : "known", delay_ms, pos / seeds, att / seeds,
              age / seeds, tau / seeds, valid / seeds, rej, ng, ne, spd, res);
  return 0;
}
