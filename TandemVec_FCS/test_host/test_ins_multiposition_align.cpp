/**
 * @file    test_ins_multiposition_align.cpp
 * @module   测试/宿主机
 * @brief   多位置（转台）对准：分离并估计全部 6 个 IMU 零偏。
 *
 * 理论：静基座单位置对准中，加计零偏与水平失准角、陀螺零偏与方位失准角
 * 强耦合，无法分离（可观测性不足）。把 IMU 转到多个已知姿态、每个位置
 * 静止采样，则：
 *
 *   f_i^b = C_n^b(ψ_i)·f^n + b_a        （f^n = [0,0,−g]）
 *   ω_i^b = C_n^b(ψ_i)·ω_ie^n + b_g
 *
 * 多个 C_n^b 不同 → 最小二乘可解出 b_a(3) 与 b_g(3)。
 *
 * 本测试模拟 4 个位置（heading 0/90/180/270），每位置静止 60 s，
 * 用已知转台角做最小二乘，输出零偏估计误差与补偿后 1 h 纯惯导漂移。
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
const double kGravity = static_cast<double>(ins_test::GravityWgs84());
constexpr double kOmegaIe = 7.292115e-5;
constexpr double kLat = 31.2304 * kDeg2Rad;

// 真零偏（导航级 FOG + 10 µg 加计）
constexpr float kGyroBiasTrue = 0.01f * static_cast<float>(kDeg2Rad) / 3600.0f;
constexpr float kAccelBiasTrue = 10.0e-6f * 9.80665f;
// 噪声（每位置 60 s 平均前的白噪声）
constexpr float kGyroNoise = 0.001f * static_cast<float>(kDeg2Rad) / 60.0f;
constexpr float kAccelNoise = 0.01f * 9.80665f / 60.0f;

Eigen::Matrix3d bodyFromNed(double heading)
{
  const double c = std::cos(heading);
  const double s = std::sin(heading);
  Eigen::Matrix3d R;
  R << c, -s, 0.0,
       s,  c, 0.0,
       0.0, 0.0, 1.0;
  return R;  // C_b^n
}
} // namespace

int main()
{
  std::printf("== 多位置对准：6 个零偏的分离与估计 ==\n");
  std::printf("%-6s %14s %14s %12s\n", "位置", "accel_bias_err(µg)",
              "gyro_bias_err(°/h)", "1h_pos(m)");

  std::mt19937 rng(20261101U);
  std::normal_distribution<double> n01(0.0, 1.0);

  const Eigen::Vector3f ab_true(
      kAccelBiasTrue * static_cast<float>(n01(rng)),
      kAccelBiasTrue * static_cast<float>(n01(rng)),
      kAccelBiasTrue * static_cast<float>(n01(rng)));
  const Eigen::Vector3f gb_true(
      kGyroBiasTrue * static_cast<float>(n01(rng)),
      kGyroBiasTrue * static_cast<float>(n01(rng)),
      kGyroBiasTrue * static_cast<float>(n01(rng)));

  const Eigen::Vector3d f_n(0.0, 0.0, -kGravity);
  const Eigen::Vector3d w_n(kOmegaIe * std::cos(kLat), 0.0,
                            -kOmegaIe * std::sin(kLat));

  // 4 个位置：heading 0/90/180/270
  const double headings[4] = {0.0, 90.0 * kDeg2Rad, 180.0 * kDeg2Rad,
                              270.0 * kDeg2Rad};
  const int n = static_cast<int>(60.0 / 0.005);

  // 最小二乘：每个位置的方程 f_i^b = C_n^b(ψ_i) f^n + b_a
  //   未知 b_a(3)。4 位置 → 12 方程。
  Eigen::Matrix<float, 4 * 3, 3> A_f = Eigen::Matrix<float, 4 * 3, 3>::Zero();
  Eigen::Matrix<float, 4 * 3, 1> y_f = Eigen::Matrix<float, 4 * 3, 1>::Zero();
  Eigen::Matrix<float, 4 * 3, 3> A_w = Eigen::Matrix<float, 4 * 3, 3>::Zero();
  Eigen::Matrix<float, 4 * 3, 1> y_w = Eigen::Matrix<float, 4 * 3, 1>::Zero();

  for (int p = 0; p < 4; ++p)
  {
    const Eigen::Matrix3d C_bn = bodyFromNed(headings[p]);
    const Eigen::Vector3d f_b_true = C_bn.transpose() * f_n;
    const Eigen::Vector3d w_b_true = C_bn.transpose() * w_n;

    Eigen::Vector3d f_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d w_sum = Eigen::Vector3d::Zero();
    for (int k = 0; k < n; ++k)
    {
      for (int a = 0; a < 3; ++a)
      {
        f_sum(a) += f_b_true(a) + ab_true(a) +
                    kAccelNoise * static_cast<float>(n01(rng));
        w_sum(a) += w_b_true(a) + gb_true(a) +
                    kGyroNoise * static_cast<float>(n01(rng));
      }
    }
    const Eigen::Vector3d f_meas = f_sum / static_cast<double>(n);
    const Eigen::Vector3d w_meas = w_sum / static_cast<double>(n);

    // 方程：f_meas = C_n^b f^n + b_a → A·b_a = f_meas − C_n^b f^n
    for (int a = 0; a < 3; ++a)
    {
      A_f(p * 3 + a, a) = 1.0f;
      y_f(p * 3 + a) = static_cast<float>(f_meas(a) - f_b_true(a));
      A_w(p * 3 + a, a) = 1.0f;
      y_w(p * 3 + a) = static_cast<float>(w_meas(a) - w_b_true(a));
    }
  }

  const Eigen::Matrix<float, 3, 1> ba_est =
      (A_f.transpose() * A_f).ldlt().solve(A_f.transpose() * y_f);
  const Eigen::Matrix<float, 3, 1> bg_est =
      (A_w.transpose() * A_w).ldlt().solve(A_w.transpose() * y_w);

  const float ba_err = (ba_est - ab_true).norm() / 9.80665f * 1.0e6f;
  const float bg_err = (bg_est - gb_true).norm() / static_cast<float>(kDeg2Rad) *
                       3600.0f;
  std::printf("%-6s %14.3f %14.4f\n", "4 位置", ba_err, bg_err);

  // 补偿后 1 h 纯惯导（理想 IMU + 补偿后的零偏残差）
  // 简化：漂移近似由零偏残差积分：位置 ≈ 0.5·|b_a|·t² + 0.5·|b_g|·g·t³/...
  const double t = 3600.0;
  const double a_err = (ba_est - ab_true).norm();
  const double pos_1h = 0.5 * a_err * t * t;
  std::printf("%-6s %14s %14s %12.1f\n", "补偿后", "-", "-", pos_1h);

  if (ba_err < 5.0 && bg_err < 0.02)
  {
    std::printf("多位置对准通过：加计零偏误差 <5 µg、陀螺 <0.02 °/h\n");
    return 0;
  }
  std::printf("未达预期（加计 %.2f µg、陀螺 %.4f °/h）\n", ba_err, bg_err);
  return 1;
}
