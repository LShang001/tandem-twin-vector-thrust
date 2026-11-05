/**
 * @file    test_tilt_triad.cpp
 * @module   测试/宿主机
 * @brief   TRIAD 双矢量初始对准 vs TiltCompass 一致性 + 奇点规避验证。
 * @details 输入：网格姿态（yaw 0–330°、pitch −80…80°、roll −60…60°）下由 C_n^b
 *          反投影得到的比力与磁测，磁参考取 bfs::kDefaultMagReference（NED µT）；
 *          处理：(1) 非奇异 5 组姿态比对 TiltCompass 与 TiltCompassTRIAD；
 *          (2) pitch=89.9° 奇点；(3) 全姿态扫描 TRIAD 与真值差；输出：三个角
 *          差值与最大误差。判据：一致性差≈磁偏角（<1e-3 rad）、TRIAD 奇点有限
 *          且 yaw/pitch 误差 <1e-3 rad、扫描最大误差 <1e-3 rad。
 * @frame   NED（重力 up_n=[0,0,−G]、地磁矢量 mag_ned）→ 机体系：accel=C_nb^T·up_n、
 *          mag=C_nb^T·mag_ned（C_nb 为 rz·ry·rx 组合，测试按其转置把 NED 矢量
 *          投到机体系）；TRIAD 反解回 NED 姿态角，输出 yaw/pitch/roll 为 rad。
 * @units   比力 m/s²、磁场 µT、角度 rad（内部与判据）/deg（打印）、磁偏角 rad。
 *
 * 判据：
 *   1) 非奇异 5 组姿态下 |TiltCompass−TRIAD| 角差与磁偏角之差 < 1e-3 rad
 *      （TRIAD 用真北参考、TiltCompass 用磁北，两者天然差一个 declination）；
 *   2) pitch≈89.9° 时 TiltCompass 可出现 NaN/大误差，TRIAD 仍须有限且
 *      pitch/yaw 误差 < 1e-3 rad；
 *   3) 全姿态扫描（roll/pitch/yaw 网格）TRIAD 输出与真值差 < 1e-3 rad。
 */
#include <cmath>
#include "ins_test_gravity.h"
#include <cstdio>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "mag_reference.h"
#include "tilt_compass.h"
#include "tilt_compass_triad.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
const float kG = ins_test::GravityWgs84();

float wrapPi(float a)
{
  while (a > 3.14159265f) a -= 6.2831853f;
  while (a < -3.14159265f) a += 6.2831853f;
  return a;
}

Eigen::Matrix3f CbnFromYpr(float yaw, float pitch, float roll)
{
  const Eigen::AngleAxisf rz(yaw, Eigen::Vector3f::UnitZ());
  const Eigen::AngleAxisf ry(pitch, Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf rx(roll, Eigen::Vector3f::UnitX());
  // C_b^n（body→NED）= Rz(yaw)·Ry(pitch)·Rx(roll)；调用处取转置把 NED 矢量投到机体系
  return (rz * ry * rx).matrix();
}

Eigen::Vector3f MagNed()
{
  return Eigen::Vector3f(bfs::kDefaultMagReference.field_ut[0],
                         bfs::kDefaultMagReference.field_ut[1],
                         bfs::kDefaultMagReference.field_ut[2]);
}
} // namespace

int main()
{
  std::printf("== TRIAD 双矢量初始对准 vs TiltCompass ==\n");
  int fail = 0;
  const Eigen::Vector3f mag_ned = MagNed();
  const Eigen::Vector3f up_n(0.0f, 0.0f, -kG);

  // 1) 非奇异姿态一致性
  for (int i = 0; i < 5; ++i)
  {
    const float yaw = (10.0f + 20.0f * i) * static_cast<float>(kDeg2Rad);
    const float pitch = (5.0f + 10.0f * i) * static_cast<float>(kDeg2Rad);
    const float roll = (5.0f + 10.0f * i) * static_cast<float>(kDeg2Rad);
    const Eigen::Matrix3f C_nb = CbnFromYpr(yaw, pitch, roll);
    const Eigen::Vector3f accel = C_nb.transpose() * up_n;
    const Eigen::Vector3f mag = C_nb.transpose() * mag_ned;

    const Eigen::Vector3f ypr_tc = bfs::TiltCompass(accel, mag);
    const Eigen::Vector3f ypr_triad = TiltCompassTRIAD(accel, mag, mag_ned);
    const float d = std::fmax(
        std::fabs(wrapPi(ypr_tc(0) - ypr_triad(0))),
        std::fmax(std::fabs(ypr_tc(1) - ypr_triad(1)),
                  std::fabs(ypr_tc(2) - ypr_triad(2))));
    std::printf("姿态(%5.1f,%5.1f,%5.1f)°  max|TiltCompass-TRIAD| = %.5f rad\n",
                yaw / static_cast<float>(kDeg2Rad),
                pitch / static_cast<float>(kDeg2Rad),
                roll / static_cast<float>(kDeg2Rad), d);
    // TRIAD 用 mag_ned 参考（真北），TiltCompass 直接用 mag（磁北），
    // 两者应恰好相差磁偏角 declination。
    const float decl = std::fabs(bfs::kDefaultMagReference.declination_rad);
    if (!(std::fabs(d - decl) < 1.0e-3f))
    {
      std::printf("  [FAIL] 差值 %.5f 与磁偏角 %.5f 不符\n", d, decl);
      ++fail;
    }
  }

  // 2) 奇点：pitch=89.9°
  {
    const float yaw = 30.0f * static_cast<float>(kDeg2Rad);
    const float pitch = 89.9f * static_cast<float>(kDeg2Rad);
    const float roll = 0.0f;
    const Eigen::Matrix3f C_nb = CbnFromYpr(yaw, pitch, roll);
    const Eigen::Vector3f accel = C_nb.transpose() * up_n;
    const Eigen::Vector3f mag = C_nb.transpose() * mag_ned;
    const Eigen::Vector3f ypr_tc = bfs::TiltCompass(accel, mag);
    const Eigen::Vector3f ypr_triad = TiltCompassTRIAD(accel, mag, mag_ned);
    const bool tc_ok = ypr_tc.allFinite() &&
                       std::fabs(ypr_tc(1) - pitch) < 0.01f;
    const bool triad_ok = ypr_triad.allFinite() &&
                          std::fabs(ypr_triad(1) - pitch) < 1.0e-3f &&
                          std::fabs(ypr_triad(0) - yaw) < 1.0e-3f;
    std::printf("奇点 pitch=89.9°: TiltCompass ok=%d, TRIAD ok=%d\n",
                tc_ok ? 1 : 0, triad_ok ? 1 : 0);
    if (!triad_ok)
    {
      std::printf("  [FAIL] TRIAD 在奇点附近失败\n");
      ++fail;
    }
  }

  // 3) 全姿态扫描
  {
    double max_err = 0.0;
    for (int y = 0; y < 360; y += 30)
    {
      for (int p = -80; p <= 80; p += 20)
      {
        for (int r = -60; r <= 60; r += 30)
        {
          const float yaw = y * static_cast<float>(kDeg2Rad);
          const float pitch = p * static_cast<float>(kDeg2Rad);
          const float roll = r * static_cast<float>(kDeg2Rad);
          const Eigen::Matrix3f C_nb = CbnFromYpr(yaw, pitch, roll);
          const Eigen::Vector3f accel = C_nb.transpose() * up_n;
          const Eigen::Vector3f mag = C_nb.transpose() * mag_ned;
          const Eigen::Vector3f ypr = TiltCompassTRIAD(accel, mag, mag_ned);
          const double d = std::fmax(
              std::fabs(wrapPi(ypr(0) - yaw)),
              std::fmax(std::fabs(ypr(1) - pitch), std::fabs(ypr(2) - roll)));
          if (d > max_err)
          {
            max_err = d;
          }
        }
      }
    }
    std::printf("全姿态扫描 max 误差 = %.6f rad (%.4f°)\n", max_err,
                max_err / kDeg2Rad);
    if (max_err > 1.0e-3)
    {
      std::printf("  [FAIL] 扫描误差超限\n");
      ++fail;
    }
  }

  if (fail == 0)
  {
    std::printf("TRIAD 对准通过（一致性 <1e-3、奇点有限、全姿态扫描 <1e-3）\n");
    return 0;
  }
  std::printf("失败 %d 项\n", fail);
  return 1;
}
