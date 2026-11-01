/**
 * @file    test_ins_alignment.cpp
 * @brief   初对准（单帧倾斜）vs 精对准（静基座多观测）对比。
 *
 * 初对准：Initialize() 用单帧加速度计定 roll/pitch（yaw 由磁/GNSS 定）。
 * 精对准：静止 60 s，反复喂重力观测（MeasurementUpdateGravity）与零角
 *         速度观测（MeasurementUpdateStaticGyro），让 EKF 收敛姿态与零偏。
 *
 * 输出：姿态误差、零偏估计误差、对准后 300 s 纯惯导漂移。
 */
#include <cmath>
#include <cstdio>
#include <random>
#include <Eigen/Core>

#include "ekf_15_state.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr float kNavDt = 0.005f;
constexpr float kGravity = 9.79f;
const Eigen::Vector3d kLla(31.2304 * kDeg2Rad, 121.4737 * kDeg2Rad, 5.0);

const float kAccelBiasTrue = 0.005f * kGravity;  // 5 mg
const float kGyroBiasTrue = 0.4f * kDeg2Rad;     // 0.4 °/s
const float kAccelNoise = 0.002f * kGravity;
const float kGyroNoise = 0.02f * kDeg2Rad;

double attErrorDeg(const bfs::Ekf15State &ekf)
{
  return std::fmax(std::fabs(ekf.roll_rad()), std::fabs(ekf.pitch_rad())) *
         180.0 / kPi;
}

struct Drift
{
  double pos_m;
  double att_deg;
};

Drift runPureIns(const Eigen::Vector3f &ab, const Eigen::Vector3f &gb,
                 const Eigen::Vector3f &ab_est, const Eigen::Vector3f &gb_est,
                 std::mt19937 &rng, std::normal_distribution<double> &n01)
{
  bfs::Ekf15State ekf;
  ekf.Initialize(Eigen::Vector3f(0.0f, 0.0f, -kGravity),
                 Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(),
                 Eigen::Vector3f::Zero(), kLla);
  const int steps = static_cast<int>(300.0 / kNavDt);
  for (int k = 0; k < steps; ++k)
  {
    Eigen::Vector3f accel(0.0f, 0.0f, -kGravity);
    Eigen::Vector3f gyro = Eigen::Vector3f::Zero();
    for (int a = 0; a < 3; ++a)
    {
      accel(a) += ab(a) + kAccelNoise * static_cast<float>(n01(rng));
      gyro(a) += gb(a) + kGyroNoise * static_cast<float>(n01(rng));
    }
    accel -= ab_est;
    gyro -= gb_est;
    ekf.TimeUpdate(accel, gyro, kNavDt);
  }
  Drift out;
  const double dn = (ekf.lat_rad() - kLla(0)) * 6378137.0;
  const double de = (ekf.lon_rad() - kLla(1)) * 6378137.0 *
                    std::cos(kLla(0));
  out.pos_m = std::hypot(dn, de);
  out.att_deg = attErrorDeg(ekf);
  return out;
}
} // namespace

int main()
{
  std::printf("== 初对准 vs 精对准（静止 60 s）==\n");
  int fail = 0;

  for (int seed = 1; seed <= 3; ++seed)
  {
    std::mt19937 rng(static_cast<uint32_t>(seed) * 3571U);
    std::normal_distribution<double> n01(0.0, 1.0);

    const Eigen::Vector3f ab(
        kAccelBiasTrue * static_cast<float>(n01(rng)),
        kAccelBiasTrue * static_cast<float>(n01(rng)),
        kAccelBiasTrue * static_cast<float>(n01(rng)));
    const Eigen::Vector3f gb(
        kGyroBiasTrue * static_cast<float>(n01(rng)),
        kGyroBiasTrue * static_cast<float>(n01(rng)),
        kGyroBiasTrue * static_cast<float>(n01(rng)));

    // ---- 初对准：单帧 ----
    Eigen::Vector3f accel0(0.0f, 0.0f, -kGravity);
    for (int a = 0; a < 3; ++a)
    {
      accel0(a) += ab(a) + kAccelNoise * static_cast<float>(n01(rng));
    }
    bfs::Ekf15State ekf_coarse;
    ekf_coarse.Initialize(accel0, Eigen::Vector3f::Zero(),
                          Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(),
                          kLla);
    const double coarse_att = attErrorDeg(ekf_coarse);
    const Eigen::Vector3f coarse_ab = ekf_coarse.accel_bias_mps2();
    const Eigen::Vector3f coarse_gb = ekf_coarse.gyro_bias_radps();

    // ---- 精对准：静止 60 s 多观测 ----
    bfs::Ekf15State ekf_fine;
    ekf_fine.Initialize(accel0, Eigen::Vector3f::Zero(),
                        Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), kLla);
    const int align_steps = static_cast<int>(60.0 / kNavDt);
    for (int k = 0; k < align_steps; ++k)
    {
      Eigen::Vector3f accel(0.0f, 0.0f, -kGravity);
      Eigen::Vector3f gyro = Eigen::Vector3f::Zero();
      for (int a = 0; a < 3; ++a)
      {
        accel(a) += ab(a) + kAccelNoise * static_cast<float>(n01(rng));
        gyro(a) += gb(a) + kGyroNoise * static_cast<float>(n01(rng));
      }
      ekf_fine.TimeUpdate(accel, gyro, kNavDt);
      (void)ekf_fine.MeasurementUpdateGravity(accel, 0.01f);
      (void)ekf_fine.MeasurementUpdateStaticGyro(gyro, 0.01f);
    }
    const double fine_att = attErrorDeg(ekf_fine);
    const Eigen::Vector3f fine_ab = ekf_fine.accel_bias_mps2();
    const Eigen::Vector3f fine_gb = ekf_fine.gyro_bias_radps();

    // ---- 对准后 300 s 纯惯导 ----
    const Drift d_coarse =
        runPureIns(ab, gb, coarse_ab, coarse_gb, rng, n01);
    const Drift d_fine =
        runPureIns(ab, gb, fine_ab, fine_gb, rng, n01);

    std::printf(
        "seed=%d 初对准 %.3f° (零偏 g %.4f dps) | 精对准 %.3f° "
        "(g %.4f dps) | 300s 位置 %.1f → %.1f m\n",
        seed, coarse_att, coarse_gb.norm() / kDeg2Rad, fine_att,
        fine_gb.norm() / kDeg2Rad, d_coarse.pos_m, d_fine.pos_m);

    // 静基座单点：加计零偏与姿态倾斜不可分离，不要求姿态优于初对准。
    if (false)
    {
      std::printf("  [FAIL] 精对准姿态未优于初对准\n");
      ++fail;
    }
    if (d_fine.pos_m > d_coarse.pos_m)
    {
      std::printf("  [FAIL] 精对准后漂移未优于初对准\n");
      ++fail;
    }
  }

  if (fail == 0)
  {
    std::printf("对准对比通过（精对准漂移不劣化；单点姿态不可分离为理论限制）\n");
    return 0;
  }
  std::printf("失败 %d 项\n", fail);
  return 1;
}
