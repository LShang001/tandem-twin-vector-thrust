/**
 * @file    test_mag_reference.cpp
 * @module  固件/宿主回归测试（fw-tests：磁参考场与坐标符号）
 * @brief   验证站点参考场的 NED 符号、总场/倾角/磁偏角一致性，以及
 *          TiltCompass 的磁北→真北偏置符号和站点距离门
 * @details 被测：`include/mag_reference.h`、`lib/navigation-main/src/tilt_compass.h`。
 *          输入：IGRF-14 生成的站点常量与合成的机头朝北/朝东磁场；
 *          处理：检查 D>0、H/F/I/dec 关系、TiltCompass 输出加磁偏角后的航向；
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译：g++ -std=c++17 -Iinclude -Ilib/navigation-main/src
 *                -Ilib/eigen/src test_host/test_mag_reference.cpp -o bin/mr
 * @theory  北半球地磁场指向北+下，NED 的 D 分量为正；TiltCompass 输出磁北航向，
 *          真北航向 = 磁北航向 + 磁偏角（东偏为正）。
 *          @ref REF-120 §9.9.1 p.323–324；notes/04-航向与磁辅助
 * @frame   NED 参考场 / FRD 机体量测；水平静止时 C^n_b=I
 * @units   µT、rad、deg、km
 * @note    这是符号/量纲回归，不是 WMM 精度验证；真实磁环境仍需上板标定。
 * @test    S1 站点参考场自洽；S2 D 向下为正；S3 TiltCompass 北/东航向与
 *          磁偏角偏移符号；S4 站点距离门
 * @see     TandemVec_FCS/include/mag_reference.h、tools/gen_mag_reference.py
 */
#include <cmath>
#include <cstdio>
#include <Eigen/Dense>

#include "mag_reference.h"
#include "tilt_compass.h"

static int g_fail = 0;
#define CHECK(cond, msg)                                                        \
  do                                                                            \
  {                                                                             \
    if (!(cond))                                                                \
    {                                                                           \
      std::printf("FAIL: %s (line %d)\n", msg, __LINE__);                       \
      g_fail++;                                                                 \
    }                                                                           \
  } while (0)

int main()
{
  constexpr float kPi = 3.14159265358979323846f;
  const bfs::MagReferenceNed &ref = bfs::kDefaultMagReference;

  // S1 参考场自洽：H/F/I/dec 由 N/E/D 定义，不能各自手填不一致。
  {
    const float h = std::hypot(ref.field_ut[0], ref.field_ut[1]);
    const float f = std::sqrt(h * h + ref.field_ut[2] * ref.field_ut[2]);
    const float dip = std::atan2(ref.field_ut[2], h);
    const float dec = std::atan2(ref.field_ut[1], ref.field_ut[0]);
    CHECK(std::fabs(h - ref.horizontal_ut) < 0.01f, "水平分量 H 与 N/E 一致");
    CHECK(std::fabs(f - ref.total_ut) < 0.01f, "总场 F 与 N/E/D 一致");
    CHECK(std::fabs(dip - ref.dip_rad) < 1e-4f, "倾角 I=atan2(D,H)");
    CHECK(std::fabs(dec - ref.declination_rad) < 1e-4f, "磁偏角 dec=atan2(E,N)");
    CHECK(ref.valid, "默认参考场 valid");
  }

  // S2 北半球 D 必须向下为正（与 MagMath 的 FRD z>0 判据一致）。
  CHECK(ref.field_ut[2] > 0.0f, "北半球 NED D>0");
  CHECK(ref.field_ut[0] > 0.0f, "北半球 N>0");
  CHECK(ref.field_ut[1] < 0.0f, "长沙磁偏角为西偏（E<0）");

  // S3 水平静止：TiltCompass 输出磁北，加 dec 后得到真北。
  {
    const Eigen::Vector3f accel(0.0f, 0.0f, 9.79f);
    const Eigen::Vector3f mag(ref.field_ut[0], ref.field_ut[1],
                              ref.field_ut[2]);
    const Eigen::Vector3f ypr = bfs::TiltCompass(accel, mag);
    const float magnetic_heading = ypr(0);
    const float true_heading = magnetic_heading + ref.declination_rad;
    CHECK(std::fabs(bfs::magref::radToDeg(magnetic_heading) +
                    bfs::magref::radToDeg(ref.declination_rad)) < 1e-3f,
          "机头真北时 TiltCompass 输出磁北航向=-dec");
    CHECK(std::fabs(true_heading) < 1e-4f, "磁北航向+dec=真北0°");

    // 机头朝东（真北 +90°）：水平磁场在机体系绕 z 旋转 -90°。
    const Eigen::Matrix3f rz =
        Eigen::AngleAxisf(-kPi / 2.0f, Eigen::Vector3f::UnitZ())
            .toRotationMatrix();
    const Eigen::Vector3f mag_east = rz * mag;
    const Eigen::Vector3f ypr_east = bfs::TiltCompass(accel, mag_east);
    const float true_east = ypr_east(0) + ref.declination_rad;
    CHECK(std::fabs(true_east - kPi / 2.0f) < 2e-3f,
          "机头朝东时真北航向=+90°");
  }

  // S4 站点距离门：同点 0，1° 纬度约 111 km。
  {
    const float same = bfs::magref::siteDistanceKm(
        ref.site_lat_deg, ref.site_lon_deg, ref.site_lat_deg,
        ref.site_lon_deg);
    const float one_deg = bfs::magref::siteDistanceKm(
        ref.site_lat_deg, ref.site_lon_deg, ref.site_lat_deg + 1.0f,
        ref.site_lon_deg);
    CHECK(same < 1e-3f, "同站点距离≈0");
    CHECK(one_deg > 100.0f && one_deg < 120.0f, "1°纬度≈111km");
  }

  if (g_fail == 0)
  {
    std::printf("=== test_mag_reference: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== test_mag_reference: %d 项失败 ===\n", g_fail);
  return 1;
}
