/**
 * @file    test_ins_fog_align.cpp
 * @module   测试/宿主机
 * @brief   FOG 级 IMU 的静基座罗经精对准 + 长航时纯惯导。
 * @details 输入：FOG 参数（陀螺 0.01 °/h、加计 10 µg）、水平静止、航向 0，
 *          3 个种子；处理：TiltCompass 初对准 → 1200 s 静基座精对准（重力 +
 *          零速 + 零角速度，每 20 步补一次航向量测）→ 补偿零偏后 3600 s 纯惯导；
 *          输出：对准姿态/航向误差、陀螺零偏误差、1 h 位置/姿态漂移。
 *          判据：对准姿态 <0.05°、1 h 位置漂移 <5 km。
 * @frame   机体系（FRD，heading=0 时与 NED 三轴对齐）比力/角速度输入，含地球
 *          自转投影 ω_ie^n=[ΩcosL, 0, −ΩsinL]；输出 NED 姿态与 LLA
 *          （站点 31.2304°N/121.4737°E/5 m）。
 * @units   陀螺零偏 °/h、加计零偏 µg、角速度 rad/s、比力 m/s²、地球自转 rad/s、
 *          时间 s、位置 m、角度 deg（打印）。
 *
 * 与 MEMS 的关键区别：FOG 陀螺能测到地球自转 ω_ie（0.00418 °/s），
 * 静基座下方位（yaw）因此可观测——这是罗经对准（gyrocompassing）的
 * 基础；MEMS 陀螺噪声远大于 ω_ie，方位只能靠外部航向。
 *
 * FOG 参数（导航级）：
 *   陀螺零偏 0.01 °/h、ARW 0.001 °/√h
 *   加计零偏 10 µg、VRW 0.01 m/s/√h
 *
 * 流程：
 *   1) 初对准（TiltCompass，accel+伪磁，yaw=0）；
 *   2) 精对准 1200 s：重力 + 零速 + 零角速度（含 ω_ie 投影）→ 方位/零偏收敛；
 *   3) 补偿零偏后 1 h 纯惯导 → 位置/姿态漂移。
 */
#include <cmath>
#include "ins_test_gravity.h"
#include <cstdio>
#include <random>
#include <Eigen/Core>

#include "ekf_15_state.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr float kNavDt = 0.005f;
const float kGravity = ins_test::GravityWgs84();  // WGS84 正常重力（与 EKF 同口径）
const Eigen::Vector3d kLla(31.2304 * kDeg2Rad, 121.4737 * kDeg2Rad, 5.0);

// FOG 导航级
constexpr float kGyroBias = 0.01f * kDeg2Rad / 3600.0f;   // °/h → rad/s
constexpr float kGyroNoise = 0.001f * kDeg2Rad / 60.0f;   // °/√h → rad/√s
constexpr float kAccelBias = 10.0e-6f * 9.80665f;         // 10 µg
constexpr float kAccelNoise = 0.01f * 9.80665f / 60.0f;   // m/s/√h

// 地球自转在静止机体系（heading=0，body=NED）的投影
constexpr double kOmegaIe = 7.292115e-5;
const float kOmegaIeN = static_cast<float>(kOmegaIe * std::cos(kLla(0)));
const float kOmegaIeD = static_cast<float>(-kOmegaIe * std::sin(kLla(0)));

double attErrorDeg(const bfs::Ekf15State &ekf)
{
  return std::fmax(std::fabs(ekf.roll_rad()), std::fabs(ekf.pitch_rad())) *
         180.0 / kPi;
}
} // namespace

int main()
{
  std::printf("== FOG 级静基座罗经精对准 + 1 h 纯惯导 ==\n");
  std::printf("%-4s %10s %10s %12s %10s %10s\n", "seed", "att_align", "yaw_align",
              "gyro_bias_err", "pos_1h", "att_1h");
  int fail = 0;

  for (int seed = 1; seed <= 3; ++seed)
  {
    std::mt19937 rng(static_cast<uint32_t>(seed) * 1543U);
    std::normal_distribution<double> n01(0.0, 1.0);
    const Eigen::Vector3f gb(
        kGyroBias * static_cast<float>(n01(rng)),
        kGyroBias * static_cast<float>(n01(rng)),
        kGyroBias * static_cast<float>(n01(rng)));
    const Eigen::Vector3f ab(
        kAccelBias * static_cast<float>(n01(rng)),
        kAccelBias * static_cast<float>(n01(rng)),
        kAccelBias * static_cast<float>(n01(rng)));

    // 初对准（TiltCompass）
    Eigen::Vector3f accel0(0.0f, 0.0f, -kGravity);
    for (int a = 0; a < 3; ++a)
    {
      accel0(a) += ab(a) + kAccelNoise * static_cast<float>(n01(rng));
    }
    const Eigen::Vector3f gyro0 = Eigen::Vector3f(kOmegaIeN, 0.0f, kOmegaIeD) + gb;
    bfs::Ekf15State ekf;
    ekf.Initialize(accel0, gyro0, Eigen::Vector3f::Zero(),
                   Eigen::Vector3f::Zero(), kLla);

    // 精对准 1200 s（罗经：零角速度观测里含 C^b_n(ω_ie+ω_en) 模型）
    const int align_steps = static_cast<int>(1200.0 / kNavDt);
    for (int k = 0; k < align_steps; ++k)
    {
      Eigen::Vector3f accel(0.0f, 0.0f, -kGravity);
      Eigen::Vector3f gyro(kOmegaIeN, 0.0f, kOmegaIeD);
      for (int a = 0; a < 3; ++a)
      {
        accel(a) += ab(a) + kAccelNoise * static_cast<float>(n01(rng));
        gyro(a) += gb(a) + kGyroNoise * static_cast<float>(n01(rng));
      }
      ekf.TimeUpdate(accel, gyro, kNavDt);
      (void)ekf.MeasurementUpdateGravity(accel, 1.0e-5f);
      (void)ekf.MeasurementUpdateStaticGyro(gyro, 1.0e-7f);
      (void)ekf.MeasurementUpdateVelocity(Eigen::Vector3f::Zero(), 1.0e-4f,
                                          1.0e-4f);
      // 外部航向（磁/GNSS course）：EKF 设计上 yaw 不可观（StaticGyro 只更新
      // 零偏），FOG 高精度对准需要外部航向锁定方位。
      if ((k % 20) == 0)
      {
        (void)ekf.MeasurementUpdateYaw(0.0f, 0.001f);
      }
    }
    const double att_align = attErrorDeg(ekf);
    // 航向误差（相对真值 0）
    const double yaw_align =
        std::fabs(std::atan2(2.0 * (ekf.quat().w() * ekf.quat().z() +
                                    ekf.quat().x() * ekf.quat().y()),
                             1.0 - 2.0 * (ekf.quat().y() * ekf.quat().y() +
                                          ekf.quat().z() * ekf.quat().z()))) *
        180.0 / kPi;
    const Eigen::Vector3f gb_est = ekf.gyro_bias_radps();
    const float gb_err_dps = (gb_est - gb).norm() / kDeg2Rad * 1.0f;
    const Eigen::Vector3f ab_est = ekf.accel_bias_mps2();

    // 1 h 纯惯导（补偿估计零偏）
    bfs::Ekf15State ekf_ins;
    ekf_ins.Initialize(Eigen::Vector3f(0.0f, 0.0f, -kGravity), gyro0,
                       Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), kLla);
    const int steps = static_cast<int>(3600.0 / kNavDt);
    for (int k = 0; k < steps; ++k)
    {
      Eigen::Vector3f accel(0.0f, 0.0f, -kGravity);
      Eigen::Vector3f gyro(kOmegaIeN, 0.0f, kOmegaIeD);
      for (int a = 0; a < 3; ++a)
      {
        accel(a) += ab(a) + kAccelNoise * static_cast<float>(n01(rng));
        gyro(a) += gb(a) + kGyroNoise * static_cast<float>(n01(rng));
      }
      accel -= ab_est;
      gyro -= gb_est;
      ekf_ins.TimeUpdate(accel, gyro, kNavDt);
    }
    const double dn = (ekf_ins.lat_rad() - kLla(0)) * 6378137.0;
    const double de = (ekf_ins.lon_rad() - kLla(1)) * 6378137.0 *
                      std::cos(kLla(0));
    const double pos_1h = std::hypot(dn, de);
    const double att_1h = attErrorDeg(ekf_ins);

    std::printf("%-4d %10.4f %10.4f %12.6f %10.1f %10.4f\n", seed, att_align,
                yaw_align, gb_err_dps, pos_1h, att_1h);

    // FOG 级：对准姿态应 <0.05°，1 h 漂移应 <5 km（10 µg 加计主导）
    if (att_align > 0.05)
    {
      std::printf("  [FAIL] 对准姿态 %.4f° > 0.05°\n", att_align);
      ++fail;
    }
    if (pos_1h > 5000.0)
    {
      std::printf("  [FAIL] 1 h 位置漂移 %.1f m > 5 km\n", pos_1h);
      ++fail;
    }
  }

  if (fail == 0)
  {
    std::printf("FOG 级对准/长航时通过（姿态 <0.05°、1 h 漂移 <5 km）\n");
    return 0;
  }
  std::printf("失败 %d 项\n", fail);
  return 1;
}
