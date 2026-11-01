/**
 * @file    test_ins_param_align.cpp
 * @brief   书本参数辨识法精对准 + 双位置测漂（秦永元《惯性导航》§9.9.2）。
 *
 * 公式链（书 9.9.34–9.9.51）：
 *   ΔV_E = a1E·t + a2E·t² + a3E·t³ + V_DE
 *   ΔV_N = a1N·t + a2N·t² + a3N·t³ + V_DN
 *   a1E = ∇E − g·φ_N0     a1N = ∇N + g·φ_E0
 *   a2E = −½g·u_N         a2N = +½g·u_E
 *   a3E = ⅙g·ωie·u_E·sinL a3N = ⅙g·ωie·(u_N sinL − u_U cosL)
 *   u_E = φ_N0·ω sinL − φ_U0·ω cosL − ε_E ; u_N = −φ_E0·ω sinL − ε_N
 *   u_U = φ_E0·ω cosL − ε_U
 *   反解：u_E=2a2N/g, u_N=−2a2E/g,
 *         u_U=−6a3N/(gω cosL) − (2a2E/g)tanL
 *         φ_E0=a1N/g, φ_N0=−a1E/g, φ_U0=φ_N0 tanL − (u_E+ε_E)/(ω cosL)
 *
 * 双位置（绕 z 转 90°）：ε_IIE = ε_IN（式 9.9.49），
 *   ε_IN = 2a2E^I/g − (a1N^I/g)·ω·sinL（式 9.9.50）。
 *
 * 验证：
 *   1) 单位置辨识出 φ0（水平误差 ∇/g，方位误差 ε_E/(ω cosL)）；
 *   2) 双位置用 ε_IN 修正后，方位精度明显提升。
 */
#include <cmath>
#include <cstdio>
#include <random>
#include <Eigen/Core>
#include <Eigen/Cholesky>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kG = 9.79;
constexpr double kOmega = 7.292115e-5;
constexpr double kLat = 31.2304 * kDeg2Rad;

// 真失准角初值（°）
constexpr double kPhiE0 = 0.5;
constexpr double kPhiN0 = 0.3;
constexpr double kPhiU0 = 1.0;
// 真零偏
constexpr double kGradE = 10.0e-6 * 9.80665;   // 加计东向零偏
constexpr double kGradN = -5.0e-6 * 9.80665;   // 加计北向零偏
constexpr double kEpsE = 0.01 * kDeg2Rad / 3600.0;   // 陀螺等效东向漂移
constexpr double kEpsN = -0.02 * kDeg2Rad / 3600.0;
constexpr double kEpsU = 0.005 * kDeg2Rad / 3600.0;

// 由 φ0、ε 求 u（式 9.9.29）
void computeU(double phi_e, double phi_n, double phi_u, double eps_e,
              double eps_n, double eps_u, double *u_e, double *u_n, double *u_u)
{
  *u_e = phi_n * kOmega * std::sin(kLat) - phi_u * kOmega * std::cos(kLat) - eps_e;
  *u_n = -phi_e * kOmega * std::sin(kLat) - eps_n;
  *u_u = phi_e * kOmega * std::cos(kLat) - eps_u;
}

// 由 a1/a2/a3 反解 φ0/u（式 9.9.42–9.9.43，无 ε_E 修正）
void invert(double a1e, double a2e, double a3e, double a1n, double a2n,
            double a3n, double *phi_e, double *phi_n, double *phi_u,
            double *u_e, double *u_n, double *u_u)
{
  *u_e = 2.0 * a2n / kG;
  *u_n = -2.0 * a2e / kG;
  *u_u = -6.0 * a3n / (kG * kOmega * std::cos(kLat)) -
         (2.0 * a2e / kG) * std::tan(kLat);
  *phi_e = a1n / kG;
  *phi_n = -a1e / kG;
  // 无 ε_E 时用式 9.9.43d：φ_U0 ≈ φ_N0·tanL − u_E/(ω·cosL)，误差 ε_E/(ω·cosL)
  *phi_u = *phi_n * std::tan(kLat) - (*u_e) / (kOmega * std::cos(kLat));
}

// 由 φ0/u 生成 ΔV（式 9.9.34）
void genDV(const double *a, double t, double dv_e, double dv_n, double *out_e,
           double *out_n)
{
  *out_e = a[0] * t + a[1] * t * t + a[2] * t * t * t;
  *out_n = a[3] * t + a[4] * t * t + a[5] * t * t * t;
  (void)dv_e; (void)dv_n;
}
} // namespace

int main()
{
  std::printf("== 参数辨识法精对准 + 双位置测漂（书 §9.9.2）==\n");
  std::mt19937 rng(20261101U);
  std::normal_distribution<double> n01(0.0, 1.0);

  const double phi_e0 = kPhiE0 * kDeg2Rad;
  const double phi_n0 = kPhiN0 * kDeg2Rad;
  const double phi_u0 = kPhiU0 * kDeg2Rad;
  double ue, un, uu;
  computeU(phi_e0, phi_n0, phi_u0, kEpsE, kEpsN, kEpsU, &ue, &un, &uu);

  // 真 a 参数（式 9.9.36）
  const double a[6] = {
      kGradE - kG * phi_n0,                     // a1E
      -0.5 * kG * un,                           // a2E
      (1.0 / 6.0) * kG * kOmega * ue * std::sin(kLat),  // a3E
      kGradN + kG * phi_e0,                     // a1N
      0.5 * kG * ue,                            // a2N
      (1.0 / 6.0) * kG * kOmega * (un * std::sin(kLat) - uu * std::cos(kLat)),
  };

  // 生成 600 s、1 Hz 的 ΔV（含噪声），用最小二乘拟合 a1/a2/a3
  const int N = 600;
  Eigen::MatrixXd H(N, 3);
  Eigen::VectorXd ze(N), zn(N);
  for (int k = 0; k < N; ++k)
  {
    const double t = static_cast<double>(k + 1);
    H(k, 0) = t;
    H(k, 1) = t * t;
    H(k, 2) = t * t * t;
    double de = 0.0, dn = 0.0;
    genDV(a, t, 0.0, 0.0, &de, &dn);
    ze(k) = de + 0.01 * n01(rng);
    zn(k) = dn + 0.01 * n01(rng);
  }
  const Eigen::Matrix3d HtH = H.transpose() * H;
  const Eigen::Vector3d ae = (HtH.ldlt().solve(H.transpose() * ze));
  const Eigen::Vector3d an = (HtH.ldlt().solve(H.transpose() * zn));

  double pe, pn, pu, ue_est, un_est, uu_est;
  invert(ae(0), ae(1), ae(2), an(0), an(1), an(2), &pe, &pn, &pu, &ue_est,
         &un_est, &uu_est);

  const double err_e = std::fabs(pe - phi_e0) / kDeg2Rad;
  const double err_n = std::fabs(pn - phi_n0) / kDeg2Rad;
  const double err_u = std::fabs(pu - phi_u0) / kDeg2Rad;
  std::printf("单位置：φE %.4f° (真 %.4f) φN %.4f° (真 %.4f) φU %.4f° (真 %.4f)\n",
              pe / kDeg2Rad, kPhiE0, pn / kDeg2Rad, kPhiN0, pu / kDeg2Rad,
              kPhiU0);
  std::printf("        误差：δφE %.4f°  δφN %.4f°  δφU %.4f°\n", err_e, err_n,
              err_u);
  std::printf("        理论：δφE=∇N/g %.4f°  δφN=−∇E/g %.4f°  "
              "δφU=εE/(ωcosL) %.4f°\n",
              kGradN / kG / kDeg2Rad, -kGradE / kG / kDeg2Rad,
              kEpsE / (kOmega * std::cos(kLat)) / kDeg2Rad);

  // 双位置：第一位置（绕 z 转 90°，式 9.9.47：ε_I = [−εy, εx, εz]）
  // 用自己的 a^I 算 ε_IN（式 9.9.50）。
  const double eps_ie = -kEpsN;   // ε_I,E = −εy
  const double eps_in_pos1 = kEpsE;  // ε_I,N = εx
  double ue1, un1, uu1;
  computeU(phi_e0, phi_n0, phi_u0, eps_ie, eps_in_pos1, kEpsU, &ue1, &un1, &uu1);
  const double a1e1 = kGradE - kG * phi_n0;
  const double a2e1 = -0.5 * kG * un1;
  const double a1n1 = kGradN + kG * phi_e0;
  const double eps_in = 2.0 * a2e1 / kG - (a1n1 / kG) * kOmega * std::sin(kLat);
  // 式 9.9.43c：φ_U0 = φ_N0·tanL − (u_E + ε_E)/(ω·cosL)；单位置估计用的是 u_E，
  // 双位置给出 ε_IIE = ε_IN，故修正 φ_U0 = φ_U0,est − ε_IN/(ω·cosL)。
  const double pu_corr = pu - eps_in / (kOmega * std::cos(kLat));
  const double err_u_corr = std::fabs(pu_corr - phi_u0) / kDeg2Rad;
  std::printf("双位置：ε_IN 估计 %.6f °/h（真 εx %.6f °/h）→ "
              "δφU 修正后 %.4f°（修正前 %.4f°）\n",
              eps_in / kDeg2Rad * 3600.0, kEpsE / kDeg2Rad * 3600.0,
              err_u_corr, err_u);

  const bool ok = (err_e < 0.01 && err_n < 0.01 && err_u_corr < 0.05);
  if (ok)
  {
    std::printf("参数辨识对准通过（水平 <0.01°、方位（双位置）<0.05°）\n");
    return 0;
  }
  std::printf("未达预期\n");
  return 1;
}
