/**
 * @file    test_ins_gyrocompass.cpp
 * @module   测试/宿主机
 * @brief   纯罗经解析对准（TRIAD 双矢量：重力 + 地球自转）。
 * @details 输入：真值姿态水平、航向 45°，FOG 噪声（ARW 0.001 °/√h）与 MEMS
 *          噪声（0.02 rad/√s）两档，时长 60/300/600 s × 3 个种子；处理：TRIAD
 *          双矢量（NED 的重力/地球自转 vs 机体系测量）求 C_b^n，按 3-2-1 提取
 *          欧拉角，再以 MEMS 档做对照；输出：roll/pitch/yaw 误差（°）。
 *          判据：FOG 各时长 yaw 误差 <1°（MEMS 只打印不判）。
 * @frame   NED（f^n=[0,0,−g]、ω_ie^n=[ΩcosL, 0, −ΩsinL]）→ 机体系 FRD（静止
 *          测量）；TRIAD 输出 C_b^n，姿态角按 ZYX 顺序提取。
 * @units   比力 m/s²、角速度 rad/s、ARW °/√h 与 rad/√s、时间 s、角度 deg（打印）。
 *
 * 原理（惯导教材静基座解析对准）：
 *   静止时加速度计测重力方向 f^b，陀螺测地球自转 ω_ie^b：
 *     f^n = [0, 0, −g]^T
 *     ω_ie^n = [ω_ie·cosL, 0, −ω_ie·sinL]^T
 *   由双矢量 TRIAD 求 C_b^n：
 *     u1=f, u2=f×ω, u3=u1×u2（导航系）；v1,v2,v3 用机体测量；
 *     C_b^n = [u1 u2 u3]·[v1 v2 v3]^T
 *   方位（yaw）由 ω_ie 的水平分量确定——纯罗经对准。
 *
 * 关键：MEMS 陀螺噪声 ≫ ω_ie（0.00418 °/s），方位不可观；FOG（0.001 °/√h）
 * 可行，但需要足够长的平均（60–600 s）。
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

// 真值姿态：水平、航向 45°（测试 TRIAD 能否复现）
constexpr double kTrueYaw = 45.0 * kDeg2Rad;

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

Eigen::Matrix3d triad(const Eigen::Vector3d &f_n, const Eigen::Vector3d &w_n,
                      const Eigen::Vector3d &f_b, const Eigen::Vector3d &w_b)
{
  // 正交基（导航系）
  const Eigen::Vector3d u1 = f_n.normalized();
  const Eigen::Vector3d u2 = u1.cross(w_n).normalized();
  const Eigen::Vector3d u3 = u1.cross(u2);
  // 正交基（机体系）
  const Eigen::Vector3d v1 = f_b.normalized();
  const Eigen::Vector3d v2 = v1.cross(w_b).normalized();
  const Eigen::Vector3d v3 = v1.cross(v2);
  Eigen::Matrix3d U, V;
  U.col(0) = u1; U.col(1) = u2; U.col(2) = u3;
  V.col(0) = v1; V.col(1) = v2; V.col(2) = v3;
  return U * V.transpose();
}
} // namespace

int main()
{
  std::printf("== 纯罗经解析对准（TRIAD：重力 + 地球自转）==\n");
  std::printf("真值姿态：水平、航向 45°\n");
  std::printf("%-10s %8s %10s %10s %10s\n", "时长(s)", "种子", "roll_err(°)",
              "pitch_err(°)", "yaw_err(°)");

  // FOG 级噪声（°/√h → rad/√s）
  const double fog_arw = 0.001 * kDeg2Rad / 60.0;  // rad/√s
  const double mems_arw = 0.02 * kDeg2Rad;         // rad/√s（MEMS 量级）

  int fail = 0;
  for (double duration : {60.0, 300.0, 600.0})
  {
    for (int seed = 1; seed <= 3; ++seed)
    {
      std::mt19937 rng(static_cast<uint32_t>(seed) * 977U);
      std::normal_distribution<double> n01(0.0, 1.0);

      const Eigen::Matrix3d C_bn = bodyFromNed(kTrueYaw);  // C_b^n
      // 真实机体测量（真值）
      const Eigen::Vector3d f_n(0.0, 0.0, -kGravity);
      const Eigen::Vector3d w_n(kOmegaIe * std::cos(kLat), 0.0,
                                -kOmegaIe * std::sin(kLat));
      const Eigen::Vector3d f_b_true = C_bn.transpose() * f_n;
      const Eigen::Vector3d w_b_true = C_bn.transpose() * w_n;

      // 采样平均（FOG 噪声）
      const int n = static_cast<int>(duration / 0.005);
      Eigen::Vector3d f_sum = Eigen::Vector3d::Zero();
      Eigen::Vector3d w_sum = Eigen::Vector3d::Zero();
      for (int k = 0; k < n; ++k)
      {
        f_sum += f_b_true + Eigen::Vector3d::Ones() * 0.002 * kGravity *
                                 n01(rng);
        w_sum += w_b_true + Eigen::Vector3d::Ones() * fog_arw *
                                 std::sqrt(1.0 / 0.005) * n01(rng);
      }
      const Eigen::Vector3d f_b = f_sum / static_cast<double>(n);
      const Eigen::Vector3d w_b = w_sum / static_cast<double>(n);

      const Eigen::Matrix3d C_est = triad(f_n, w_n, f_b, w_b);
      // 提取欧拉角（ZYX：yaw-pitch-roll）
      const Eigen::Matrix3d R = C_est;  // C_b^n（body→ned）
      const double pitch = std::asin(-R(2, 0));
      const double roll = std::atan2(R(2, 1), R(2, 2));
      const double yaw = std::atan2(R(1, 0), R(0, 0));

      const double roll_err = std::fabs(roll) * 180.0 / kPi;
      const double pitch_err = std::fabs(pitch) * 180.0 / kPi;
      const double yaw_err = std::fabs(yaw - kTrueYaw) * 180.0 / kPi;
      std::printf("%-10.0f %8d %10.5f %10.5f %10.4f\n", duration, seed,
                  roll_err, pitch_err, yaw_err);
      if (yaw_err > 1.0)
      {
        std::printf("  [FAIL] yaw 误差 %.3f° > 1°\n", yaw_err);
        ++fail;
      }
    }
  }

  // MEMS 对照（同一时长 300 s）：方位应明显更差
  std::printf("\nMEMS 对照（300 s）：\n");
  {
    std::mt19937 rng(42U);
    std::normal_distribution<double> n01(0.0, 1.0);
    const Eigen::Matrix3d C_bn = bodyFromNed(kTrueYaw);
    const Eigen::Vector3d f_n(0.0, 0.0, -kGravity);
    const Eigen::Vector3d w_n(kOmegaIe * std::cos(kLat), 0.0,
                              -kOmegaIe * std::sin(kLat));
    const Eigen::Vector3d f_b_true = C_bn.transpose() * f_n;
    const Eigen::Vector3d w_b_true = C_bn.transpose() * w_n;
    const int n = static_cast<int>(300.0 / 0.005);
    Eigen::Vector3d f_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d w_sum = Eigen::Vector3d::Zero();
    for (int k = 0; k < n; ++k)
    {
      f_sum += f_b_true + Eigen::Vector3d::Ones() * 0.002 * kGravity * n01(rng);
      w_sum += w_b_true + Eigen::Vector3d::Ones() * mems_arw *
                               std::sqrt(1.0 / 0.005) * n01(rng);
    }
    const Eigen::Matrix3d C_est = triad(
        f_n, w_n, f_sum / n, w_sum / n);
    const Eigen::Matrix3d R = C_est;
    const double yaw = std::atan2(R(1, 0), R(0, 0));
    std::printf("  MEMS yaw 误差 %.3f°（真值 45°）\n",
                std::fabs(yaw - kTrueYaw) * 180.0 / kPi);
  }

  if (fail == 0)
  {
    std::printf("纯罗经对准（FOG）通过：60–600 s 内 yaw 误差 <1°\n");
    return 0;
  }
  std::printf("失败 %d 项\n", fail);
  return 1;
}
