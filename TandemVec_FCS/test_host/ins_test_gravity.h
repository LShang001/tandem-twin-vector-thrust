/**
 * @file    ins_test_gravity.h
 * @module   测试/宿主机
 * @brief   宿主惯导测试统一重力口径（WGS84 正常重力，与 EKF 机械编排一致）。
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
