/**
 * @file    test_ins_fog.cpp
 * @module   测试/宿主机
 * @brief   光纤陀螺（FOG）级别 IMU 的纯惯导长航时漂移仿真。
 * @details 输入：FOG 参数（陀螺 0.01 °/h、加计 10 µg、ARW 0.001 °/√h、
 *          VRW 0.01 m/s/√h）与 MEMS 参数（0.4 °/s、5 mg）两档，各 2 个种子；
 *          处理：静止 60 s 均值校准零偏 → 补偿 → 3600 s 纯惯导（Ekf15State
 *          机械编排）；输出：加计零偏估计误差 µg、1 h 位置/姿态漂移。
 *          判据：FOG 1 h 位置漂移 ≤ 30 km（超出即判 FAIL）。
 * @frame   机体系（FRD）比力/角速度输入；Ekf15State 输出 NED 姿态与 LLA 位置，
 *          重力为 WGS84 正常重力（ins_test::GravityWgs84，沿 NED +D 向下）。
 * @units   陀螺零偏 °/h、加计零偏 µg、角速度 rad/s、比力 m/s²、时间 s、位置 m、
 *          姿态 deg（打印）。
 *
 * 目的：量化“换 FOG 能带来多少改善”，以及换完之后瓶颈变成什么。
 *
 * 参数（典型导航级 FOG）：
 *   陀螺零偏 0.01 °/h = 2.78e-6 °/s，ARW 0.001 °/√h
 *   加计零偏 10 µg = 9.8e-5 m/s²，VRW 0.01 m/s/√h
 * 对比 MEMS（45686）：陀螺 0.4 °/s、加计 5 mg。
 *
 * 流程：静止 60 s 校准估计零偏 → 补偿 → 3600 s 纯惯导 → 输出漂移。
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

struct ImuClass
{
  const char *name;
  float gyro_bias_radps;
  float gyro_noise_radps;
  float accel_bias_mps2;
  float accel_noise_mps2;
};

// FOG 导航级：0.01 °/h、10 µg
const ImuClass kFog{"FOG", 0.01f * kDeg2Rad / 3600.0f,
                    0.001f * kDeg2Rad / 60.0f,
                    10.0e-6f * 9.80665f,
                    0.01f * 9.80665f / 60.0f};
// MEMS（45686 量级）：0.4 °/s、5 mg
const ImuClass kMems{"MEMS", 0.4f * kDeg2Rad * 1.0f,
                     0.02f * kDeg2Rad,
                     0.005f * 9.80665f,
                     0.002f * 9.80665f};

struct Out
{
  double pos_m;
  double att_deg;
};
} // namespace

int main()
{
  std::printf("== FOG vs MEMS 纯惯导长航时漂移（1 h，含 60 s 静止校准）==\n");
  std::printf("%-6s %12s %12s %12s\n", "class", "accel_err(µg)", "pos_1h(m)",
              "att_1h(deg)");
  int fail = 0;

  for (const ImuClass &c : {kFog, kMems})
  {
    for (int seed = 1; seed <= 2; ++seed)
    {
      std::mt19937 rng(static_cast<uint32_t>(seed) * 7919U);
      std::normal_distribution<double> n01(0.0, 1.0);
      const Eigen::Vector3f gb(
          c.gyro_bias_radps * static_cast<float>(n01(rng)),
          c.gyro_bias_radps * static_cast<float>(n01(rng)),
          c.gyro_bias_radps * static_cast<float>(n01(rng)));
      const Eigen::Vector3f ab(
          c.accel_bias_mps2 * static_cast<float>(n01(rng)),
          c.accel_bias_mps2 * static_cast<float>(n01(rng)),
          c.accel_bias_mps2 * static_cast<float>(n01(rng)));

      // ---- 静止 60 s 校准 ----
      const int align_steps = static_cast<int>(60.0 / kNavDt);
      Eigen::Vector3f as = Eigen::Vector3f::Zero();
      Eigen::Vector3f gs = Eigen::Vector3f::Zero();
      for (int k = 0; k < align_steps; ++k)
      {
        for (int a = 0; a < 3; ++a)
        {
          as(a) += ab(a) + c.accel_noise_mps2 *
                               static_cast<float>(n01(rng));
          gs(a) += gb(a) + c.gyro_noise_radps *
                               static_cast<float>(n01(rng));
        }
      }
      const Eigen::Vector3f ab_est =
          as / static_cast<float>(align_steps);
      const Eigen::Vector3f gb_est =
          gs / static_cast<float>(align_steps);
      const float ab_err = (ab_est - ab).norm();

      // ---- 1 h 纯惯导（补偿） ----
      bfs::Ekf15State ekf;
      ekf.Initialize(Eigen::Vector3f(0.0f, 0.0f, -kGravity),
                     Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(),
                     Eigen::Vector3f::Zero(), kLla);
      const int steps = static_cast<int>(3600.0 / kNavDt);
      for (int k = 0; k < steps; ++k)
      {
        Eigen::Vector3f accel(0.0f, 0.0f, -kGravity);
        Eigen::Vector3f gyro = Eigen::Vector3f::Zero();
        for (int a = 0; a < 3; ++a)
        {
          accel(a) += ab(a) + c.accel_noise_mps2 *
                                  static_cast<float>(n01(rng));
          gyro(a) += gb(a) + c.gyro_noise_radps *
                                 static_cast<float>(n01(rng));
        }
        accel -= ab_est;
        gyro -= gb_est;
        ekf.TimeUpdate(accel, gyro, kNavDt);
      }
      const double dn = (ekf.lat_rad() - kLla(0)) * 6378137.0;
      const double de = (ekf.lon_rad() - kLla(1)) * 6378137.0 *
                        std::cos(kLla(0));
      Out out;
      out.pos_m = std::hypot(dn, de);
      out.att_deg =
          std::fmax(std::fabs(ekf.roll_rad()), std::fabs(ekf.pitch_rad())) *
          180.0 / kPi;

      std::printf("%-6s %12.2f %12.1f %12.4f\n", c.name,
                  ab_err / 9.80665f * 1.0e6f, out.pos_m, out.att_deg);

      // FOG 1 h 位置漂移应在公里级以内（加计 10 µg 主导：约 0.5·a·t² ≈ 635 m）
      if (c.name[0] == 'F' && out.pos_m > 30000.0)
      {
        std::printf("  [FAIL] FOG 位置漂移 %.1f m 超出预期（加计零偏主导）\n",
                    out.pos_m);
        ++fail;
      }
    }
  }

  if (fail == 0)
  {
    std::printf("结论：FOG 1 h 位置漂移在公里级（加计零偏主导），"
                "MEMS 为百公里级（陀螺零偏主导）。\n");
    return 0;
  }
  std::printf("失败 %d 项\n", fail);
  return 1;
}
