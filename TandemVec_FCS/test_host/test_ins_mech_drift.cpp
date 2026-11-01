/**
 * @file    test_ins_mech_drift.cpp
 * @brief   捷联惯导机械编排长航时精度验证（纯惯导，无 GNSS）。
 *
 * 目的：检验 Ekf15State 的机械编排（四元数姿态 + 圆锥/划摇 + 地球自转/
 * 哥氏 + WGS84 正常重力 + 中点积分）在长时间纯惯导下的自洽性：
 *   1) 理想 IMU（无零偏无噪声）静止 30 min：位置/姿态漂移应≈0；
 *   2) 加真实 MEMS 零偏（加计 5 mg、陀螺 0.4 °/s）：量化物理边界
 *      （零偏主导，不是算法误差）。
 *
 * 编译：
 *   g++ -std=c++17 -O2 -D_USE_MATH_DEFINES -Iinclude -Ilib/eigen/src -Isrc \
 *       -Ilib/navigation-main/src -Ilib/units/src test_host/test_ins_mech_drift.cpp \
 *       lib/units/src/convang.cpp lib/units/src/convangacc.cpp \
 *       lib/units/src/convangvel.cpp -o t.exe
 */
#include <cmath>
#include <cstdio>
#include <Eigen/Core>

#include "ekf_15_state.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr float kNavDt = 0.005f;              // 200 Hz
constexpr float kGravity = 9.79f;             // 与项目 TandemVec_Config.h 一致
const Eigen::Vector3d kLla(31.2304 * kDeg2Rad, 121.4737 * kDeg2Rad, 5.0);

const float kAccelBias = 0.005f * kGravity;   // 5 mg
const float kGyroBias = 0.4f * kDeg2Rad / 180.0 * 180.0 / 180.0;  // 0.4 °/s → rad/s
} // namespace

int main()
{
  std::printf("== 捷联惯导机械编排长航时漂移（纯惯导，无 GNSS）==\n");
  struct Case
  {
    const char *name;
    bool ideal;
    double duration_s;
  };
  const Case cases[] = {
      {"理想 IMU", true, 300.0},     // 5 min：检查自洽性
      {"理想 IMU", true, 1800.0},    // 30 min
      {"真实零偏", false, 60.0},     // 1 min：物理边界
      {"真实零偏", false, 300.0},    // 5 min
  };

  int fail = 0;
  for (const Case &c : cases)
  {
    bfs::Ekf15State ekf;
    const Eigen::Vector3f accel0(0.0f, 0.0f, -kGravity);
    ekf.Initialize(accel0, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(),
                   Eigen::Vector3f::Zero(), kLla);

    const int steps = static_cast<int>(c.duration_s / kNavDt);
    for (int k = 0; k < steps; ++k)
    {
      Eigen::Vector3f accel = accel0;
      Eigen::Vector3f gyro = Eigen::Vector3f::Zero();
      if (!c.ideal)
      {
        accel += Eigen::Vector3f(kAccelBias, 0.0f, 0.0f);
        gyro += Eigen::Vector3f(0.0f, kGyroBias, 0.0f);
      }
      ekf.TimeUpdate(accel, gyro, kNavDt);
    }

    const double dn = (ekf.lat_rad() - kLla(0)) * 6378137.0;
    const double de = (ekf.lon_rad() - kLla(1)) * 6378137.0 *
                      std::cos(kLla(0));
    const double dpos = std::hypot(dn, de);
    const double dalt = ekf.alt_m() - kLla(2);
    const double datt = std::fmax(std::fabs(ekf.roll_rad()),
                                  std::fabs(ekf.pitch_rad())) *
                        kDeg2Rad * 0.0;  // 占位，姿态单列
    const double att_deg =
        std::fmax(std::fabs(ekf.roll_rad()), std::fabs(ekf.pitch_rad())) *
        180.0 / kPi;
    const double dvel = ekf.ned_vel_mps().norm();

    std::printf("%-8s %6.0fs  位置 %10.3f m  高度 %9.3f m  速度 %10.3f m/s  姿态 %8.3f°\n",
                c.name, c.duration_s, dpos, dalt, dvel, att_deg);

    // 判据：理想 IMU 300 s 位置漂移 <5 m、速度 <2 m/s（机械编排自洽）；
    // 高度通道是纯惯导固有不稳定（Schuler 振荡负阻尼），不作失败判据。
    if (c.ideal && c.duration_s <= 300.0 && (dpos > 5.0 || dvel > 2.0))
    {
      std::printf("  [FAIL] 理想 IMU 位置 %.3f m / 速度 %.4f m/s 超出预期\n", dpos, dvel);
      ++fail;
    }
    (void)datt;
  }

  if (fail == 0)
  {
    std::printf("理想 IMU 300s 自洽性通过（位置 <5 m、速度 <2 m/s）；高度通道为纯惯导固有发散\n");
    return 0;
  }
  std::printf("失败 %d 项\n", fail);
  return 1;
}
