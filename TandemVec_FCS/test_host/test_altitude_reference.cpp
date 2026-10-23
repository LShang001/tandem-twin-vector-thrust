/**
 * @file    test_altitude_reference.cpp
 * @module  固件/宿主回归测试（fw-tests：高度参考系换算纯函数）
 * @brief   验证气压/控制/GNSS 重锚三条高度参考换算与非法输入拒绝
 * @details 被测：`include/ins_altitude_reference.h` 的
 *          `InsComputeAbsoluteBaroAltitude()`、`InsRelativeHeightUpFromNedDown()`、
 *          `InsVerticalVelocityUpFromNedDown()`、`InsTakeoffOriginAltitudeFromCurrent()`。
 *          输入：手工构造的起飞点绝对高度与相对气压高度、NED 向下位置/速度；
 *          处理：逐条 require 断言返回值与数值；
 *          输出：任一断言失败即 exit(1) 并打印 FAILED 行。
 *          运行（仓库根目录）：
 *          g++ -std=c++17 -Iinclude test_host/test_altitude_reference.cpp -o bin/ar
 * @theory  高度基准换算：EKF 内部保持 WGS84 绝对高度，DPS310 对外发布相对起飞点高度，
 *          入 EKF 前必须叠加 `origin_alt_m`；控制/遥测高度用“向上为正”，
 *          与 NED 的“向下为正”差一次符号翻转。@ref AGENTS.md（模块化架构：不得把相对值直传 EKF）
 * @frame   NED（向下为正）→ 控制/遥测（向上为正）的显式换算式
 * @units   高度 m，垂直速度 m/s，容差 1e-6（double 双精度有效位内）
 * @note    “基准错位”是历史事故点：把相对高度当绝对高度会让高度环整体偏移一个
 *          起飞点海拔，本文件是该类改动的回归闸门。
 * @test    T1 相对气压高度叠加起飞点；T2 NED 向下位置→向上高度；
 *          T3 NED 向下速度→爬升率；T4 GNSS 重锚保持起飞点相对高度；
 *          T5 NaN/Inf/空指针三类非法输入拒绝
 * @see     TandemVec_FCS/include/ins_altitude_reference.h
 */
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <limits>

#include "ins_altitude_reference.h"

namespace
{
void require(bool condition, const char *message)
{
  if (!condition)
  {
    std::fprintf(stderr, "FAILED: %s\n", message);
    std::exit(1);
  }
}

bool nearlyEqual(double lhs, double rhs, double tolerance = 1.0e-6)
{
  return std::fabs(lhs - rhs) <= tolerance;
}
} // namespace

int main()
{
  double absolute_baro_altitude_m = 0.0;
  require(InsComputeAbsoluteBaroAltitude(123.4, 2.5f,
                                         &absolute_baro_altitude_m),
          "finite barometer observation should be accepted");
  require(nearlyEqual(absolute_baro_altitude_m, 125.9),
          "relative barometer height should be anchored to takeoff altitude");

  require(nearlyEqual(InsRelativeHeightUpFromNedDown(-4.25f), 4.25f),
          "negative NED down should become positive control height");
  require(nearlyEqual(InsVerticalVelocityUpFromNedDown(0.8f), -0.8f),
          "positive NED down velocity should become negative climb rate");

  require(nearlyEqual(InsTakeoffOriginAltitudeFromCurrent(152.0, -12.0),
                      140.0),
          "GNSS reanchor should preserve the existing takeoff-relative height");

  require(!InsComputeAbsoluteBaroAltitude(
              std::numeric_limits<double>::quiet_NaN(), 1.0f,
              &absolute_baro_altitude_m),
          "invalid origin altitude should be rejected");
  require(!InsComputeAbsoluteBaroAltitude(
              123.4, std::numeric_limits<float>::infinity(),
              &absolute_baro_altitude_m),
          "invalid relative barometer height should be rejected");
  require(!InsComputeAbsoluteBaroAltitude(123.4, 1.0f, nullptr),
          "null output pointer should be rejected");

  std::puts("[PASS] altitude reference tests");
  return 0;
}
