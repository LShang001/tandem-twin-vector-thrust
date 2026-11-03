/**
 * @file    test_ins_param_align_module.cpp
 * @module   测试/宿主机
 * @brief   ins_param_align.h 模块回归：书本参数辨识法（§9.9.2）。
 */
#include <cmath>
#include "ins_test_gravity.h"
#include <cstdio>
#include <random>

#include "ins_param_align.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
const double kG = static_cast<double>(ins_test::GravityWgs84());
constexpr double kOmega = 7.292115e-5;
constexpr double kLat = 31.2304 * kDeg2Rad;

constexpr double kPhiE0 = 0.5 * kDeg2Rad;
constexpr double kPhiN0 = 0.3 * kDeg2Rad;
constexpr double kPhiU0 = 1.0 * kDeg2Rad;
constexpr double kGradE = 10.0e-6 * 9.80665;
constexpr double kGradN = -5.0e-6 * 9.80665;
constexpr double kEpsE = 0.01 * kDeg2Rad / 3600.0;
constexpr double kEpsN = -0.02 * kDeg2Rad / 3600.0;
constexpr double kEpsU = 0.005 * kDeg2Rad / 3600.0;
} // namespace

int main()
{
  std::printf("== ins_param_align 模块回归（书 §9.9.2）==\n");
  std::mt19937 rng(20261101U);
  std::normal_distribution<double> n01(0.0, 1.0);

  const double u_e = kPhiN0 * kOmega * std::sin(kLat) -
                     kPhiU0 * kOmega * std::cos(kLat) - kEpsE;
  const double u_n = -kPhiE0 * kOmega * std::sin(kLat) - kEpsN;
  const double u_u = kPhiE0 * kOmega * std::cos(kLat) - kEpsU;

  const double a[6] = {
      kGradE - kG * kPhiN0,
      -0.5 * kG * u_n,
      (1.0 / 6.0) * kG * kOmega * u_e * std::sin(kLat),
      kGradN + kG * kPhiE0,
      0.5 * kG * u_e,
      (1.0 / 6.0) * kG * kOmega * (u_n * std::sin(kLat) - u_u * std::cos(kLat)),
  };

  InsParamAlign align;
  const int n = 1200;  // 600 s @2 Hz
  for (int k = 0; k < n; ++k)
  {
    const double t = 0.5 * static_cast<double>(k + 1);
    const double de = a[0] * t + a[1] * t * t + a[2] * t * t * t +
                      0.01 * n01(rng);
    const double dn = a[3] * t + a[4] * t * t + a[5] * t * t * t +
                      0.01 * n01(rng);
    if (!align.Update(static_cast<float>(t), static_cast<float>(de),
                      static_cast<float>(dn)))
    {
      std::printf("[FAIL] Update 拒绝样本\n");
      return 1;
    }
  }

  const InsParamAlign::Result r =
      align.Solve(static_cast<float>(kLat), static_cast<float>(kG));
  if (!r.valid)
  {
    std::printf("[FAIL] Solve 无效\n");
    return 1;
  }

  const double d_e = std::fabs(r.phi_e_rad - kPhiE0) / kDeg2Rad;
  const double d_n = std::fabs(r.phi_n_rad - kPhiN0) / kDeg2Rad;
  const double d_u = std::fabs(r.phi_u_rad - kPhiU0) / kDeg2Rad;
  std::printf("φE %.4f° (真 %.4f) φN %.4f° (真 %.4f) φU %.4f° (真 %.4f)\n",
              r.phi_e_rad / kDeg2Rad, kPhiE0 / kDeg2Rad,
              r.phi_n_rad / kDeg2Rad, kPhiN0 / kDeg2Rad,
              r.phi_u_rad / kDeg2Rad, kPhiU0 / kDeg2Rad);
  std::printf("误差 δφE %.4f° δφN %.4f° δφU %.4f°；拟合残差 %.4f m/s\n",
              d_e, d_n, d_u, r.fit_residual_rms);
  std::printf("理论 δφE=∇N/g %.4f° δφN=−∇E/g %.4f° δφU=εE/(ωcosL) %.4f°\n",
              kGradN / kG / kDeg2Rad, -kGradE / kG / kDeg2Rad,
              kEpsE / (kOmega * std::cos(kLat)) / kDeg2Rad);

  // 双位置测漂（式 9.9.50，用第一位置 a：ε_I,N = εx = 0.01°/h）
  const double eps_x = 0.01 * kDeg2Rad / 3600.0;
  InsParamAlign::Result r2 = r;
  InsParamAlign::ApplyDualPosition(&r2, static_cast<float>(eps_x),
                                   static_cast<float>(kLat));
  const double d_u2 = std::fabs(r2.phi_u_rad - kPhiU0) / kDeg2Rad;
  std::printf("双位置修正后 δφU %.4f°（修正前 %.4f°）\n", d_u2, d_u);

  if (d_e < 0.01 && d_n < 0.01 && d_u2 < 0.05)
  {
    std::printf("模块回归通过\n");
    return 0;
  }
  std::printf("模块回归未达预期\n");
  return 1;
}
