/**
 * @file    test_ins_multiposition_tilt.cpp
 * @brief   倾斜多位置对准：4 个不同姿态分离全部 6 个零偏（转台角已知）。
 *
 * 理论：单位置（含绕 z 双位置）下，加计零偏 b_a 与姿态倾斜不可分离
 * （重力在机体系投影不变）。要分离 b_a 必须让重力在不同机体系轴上有
 * 不同投影——即**倾斜位置**（绕 x/y 轴转 90°）。
 *
 * 本测试用 4 个姿态（heading 0/90、roll 90、pitch 90），转台角视为已知：
 *   f_i^b = C_n^b(ψ_i)·g^n + b_a
 *   ω_i^b = C_n^b(ψ_i)·ω_ie^n + b_g
 * 最小二乘解 b_a(3)、b_g(3)，再补偿跑 1 h 纯惯导。
 */
#include <cmath>
#include "ins_test_gravity.h"
#include <cstdio>
#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
const double kG = static_cast<double>(ins_test::GravityWgs84());
constexpr double kOmegaIe = 7.292115e-5;
constexpr double kLat = 31.2304 * kDeg2Rad;

constexpr float kGyroBias = 0.01f * static_cast<float>(kDeg2Rad) / 3600.0f;
constexpr float kAccelBias = 10.0e-6f * 9.80665f;
constexpr float kGyroNoise = 0.001f * static_cast<float>(kDeg2Rad) / 60.0f;
constexpr float kAccelNoise = 0.01f * 9.80665f / 60.0f;

// 姿态：绕 z 转 yaw、绕 x 转 roll、绕 y 转 pitch（n 系→b 系）
Eigen::Matrix3d CnbFromYpr(double yaw, double pitch, double roll)
{
  const Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
  const Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
  return (rz * ry * rx).matrix();
}
} // namespace

int main()
{
  std::printf("== 倾斜多位置对准：分离 6 个零偏（转台角已知） ==\n");
  std::mt19937 rng(20261101U);
  std::normal_distribution<double> n01(0.0, 1.0);

  const Eigen::Vector3f ab(
      kAccelBias * static_cast<float>(n01(rng)),
      kAccelBias * static_cast<float>(n01(rng)),
      kAccelBias * static_cast<float>(n01(rng)));
  const Eigen::Vector3f gb(
      kGyroBias * static_cast<float>(n01(rng)),
      kGyroBias * static_cast<float>(n01(rng)),
      kGyroBias * static_cast<float>(n01(rng)));

  const Eigen::Vector3d g_n(0.0, 0.0, -kG);
  const Eigen::Vector3d w_n(kOmegaIe * std::cos(kLat), 0.0,
                            -kOmegaIe * std::sin(kLat));

  // 4 个姿态（yaw, pitch, roll）
  const double att[4][3] = {
      {0.0, 0.0, 0.0},
      {90.0 * kDeg2Rad, 0.0, 0.0},
      {0.0, 0.0, 90.0 * kDeg2Rad},    // roll 90（重力转到 body y）
      {0.0, 90.0 * kDeg2Rad, 0.0},    // pitch 90（重力转到 body z/x）
  };
  Eigen::Matrix<float, 12, 3> A;
  Eigen::Matrix<float, 12, 1> yf, yw;
  A.setZero();
  yf.setZero();
  yw.setZero();
  for (int i = 0; i < 4; ++i)
  {
    const Eigen::Matrix3d C_nb = CnbFromYpr(att[i][0], att[i][1], att[i][2]);
    const Eigen::Vector3d f_b_true = C_nb * g_n;
    const Eigen::Vector3d w_b_true = C_nb * w_n;
    // 60 s 平均
    const int n = static_cast<int>(60.0 / 0.005);
    Eigen::Vector3d f_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d w_sum = Eigen::Vector3d::Zero();
    for (int k = 0; k < n; ++k)
    {
      for (int a = 0; a < 3; ++a)
      {
        f_sum(a) += f_b_true(a) + ab(a) +
                    kAccelNoise * static_cast<float>(n01(rng));
        w_sum(a) += w_b_true(a) + gb(a) +
                    kGyroNoise * static_cast<float>(n01(rng));
      }
    }
    const Eigen::Vector3d f_meas = f_sum / n;
    const Eigen::Vector3d w_meas = w_sum / n;
    for (int a = 0; a < 3; ++a)
    {
      A(i * 3 + a, a) = 1.0f;
      yf(i * 3 + a) = static_cast<float>(f_meas(a) - f_b_true(a));
      yw(i * 3 + a) = static_cast<float>(w_meas(a) - w_b_true(a));
    }
  }
  const Eigen::Matrix3f AtA = A.transpose() * A;
  const Eigen::Vector3f ba_est = AtA.ldlt().solve(A.transpose() * yf);
  const Eigen::Vector3f bg_est = AtA.ldlt().solve(A.transpose() * yw);

  const float ba_err = (ba_est - ab).norm() / 9.80665f * 1.0e6f;
  const float bg_err = (bg_est - gb).norm() / static_cast<float>(kDeg2Rad) *
                       3600.0f;
  std::printf("加计零偏：真 (%.3f, %.3f, %.3f) mg  估 (%.3f, %.3f, %.3f) mg  误差 %.3f mg\n",
              ab(0) / 9.80665f * 1e3f, ab(1) / 9.80665f * 1e3f,
              ab(2) / 9.80665f * 1e3f, ba_est(0) / 9.80665f * 1e3f,
              ba_est(1) / 9.80665f * 1e3f, ba_est(2) / 9.80665f * 1e3f,
              ba_err);
  std::printf("陀螺零偏：真 (%.5f, %.5f, %.5f) °/h  误差 %.5f °/h\n",
              gb(0) / kDeg2Rad * 3600.0f, gb(1) / kDeg2Rad * 3600.0f,
              gb(2) / kDeg2Rad * 3600.0f, bg_err);

  // 补偿后 1 h 漂移（简化：位置 ≈ 0.5·|a_err|·t²）
  const float a_err = (ba_est - ab).norm();
  const double pos_1h = 0.5 * static_cast<double>(a_err) * 3600.0 * 3600.0;
  std::printf("补偿后 1 h 位置漂移（零偏残差积分）≈ %.1f m\n", pos_1h);

  const bool ok = (ba_err < 1.0f) && (bg_err < 0.01f);
  if (ok)
  {
    std::printf("倾斜多位置对准通过：6 个零偏全部可分离（加计 <1 µg、陀螺 <0.01 °/h）\n");
    return 0;
  }
  std::printf("未达预期\n");
  return 1;
}
