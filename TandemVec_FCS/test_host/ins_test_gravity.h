/**
 * @file    ins_test_gravity.h
 * @module   测试/宿主机
 * @brief   宿主惯导测试统一重力口径（WGS84 正常重力，与 EKF 机械编排一致）。
 * @details 输入：站点纬度 [deg] 与椭球高 [m]；处理：先换 rad，再调
 *          bfs::normalgravity_mps2()（Somigliana 级数式 + 反平方高度修正）；
 *          输出：float 正常重力 [m/s²]，供各测试构造静止比力/期望重力。
 *          不变量：与 Ekf15State 机械编排同口径（31.2304°N、5 m 约 9.794 m/s²）；
 *          用 kG=9.79 构造比力会注入 ~0.004 m/s²，1 h 纯惯导可伪造出 24 km 级
 *          "算法误差"；故测试真值统一走本函数，9.79（固件推力口径 G_TO_MS2）
 *          与 9.81 都不作导航真值。
 * @frame   NED：重力为标量幅值，沿 +D（向下）为正；无姿态与机体系转换。
 * @units   纬度 deg（入参 lat_deg）、椭球高 m（alt_m）、输出 m/s²；内部转 rad。
 *
 * 背景：测试曾用 kG=9.79 构造比力，而 EKF 用 WGS84 正常重力（≈9.7938），
 * 差 0.0038 m/s² 在 1 h 纯惯导中会伪造出 ~24 km 的"算法误差"（2026-11-01
 * FOG 端到端定位）。所有测试真值改用本函数，与滤波器同口径。
 *
 * @ref REF-120 §7.2.4 (7.2.17)(7.2.18) p.178
 */
#pragma once

#include "earth_model.h"

namespace ins_test
{
/// WGS84 正常重力 [m/s²]（默认：长沙/上海附近 31.2304°N、5 m 高度）。
inline float GravityWgs84(const float lat_deg = 31.2304f,
                          const float alt_m = 5.0f)
{
  const double lat_rad = static_cast<double>(lat_deg) * 3.14159265358979323846 /
                         180.0;
  return static_cast<float>(
      bfs::normalgravity_mps2(lat_rad, static_cast<double>(alt_m),
                              bfs::AngPosUnit::RAD));
}
}  // namespace ins_test
