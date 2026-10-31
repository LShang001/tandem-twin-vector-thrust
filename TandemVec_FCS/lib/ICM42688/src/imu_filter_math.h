/**
 * @file    imu_filter_math.h
 * @module  固件/传感器（IMU 滤波参数计算，纯平台无关）
 * @brief   ICM-42688-P 的 AAF 带宽映射与 notch 系数计算，固件与宿主测试共用。
 * @details 手册 DS-000347 Rev1.2：
 *          §5.3 AAF：3dB 带宽（42..3979Hz）→ DELT/DELTSQR/BITSHIFT（63 档表）；
 *          §5.2 notch：COSWZ = cos(2π·f/32)，f 单位 kHz（1..3），
 *               |COSWZ|≤0.875 时 SEL=0，否则 SEL=1 并做 8×(1∓COSWZ) 缩放。
 *          本头文件不依赖 Arduino，test_host/test_imu_filter_math.cpp 直接回归。
 * @frame   N/A
 * @units   带宽 Hz、频率 kHz、COSWZ 为 9-bit 无符号编码。
 * @note    AAF 表来自手册表 5.3 逐行抄录，禁止手改单点；改表要整表重抄并回归。
 */
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace imu_filter
{

struct AafEntry
{
  uint16_t bw_hz;
  uint8_t delt;
  uint16_t deltsqr;
  uint8_t bitshift;
};

/// 手册 §5.3 的 AAF 表（63 档，按带宽升序）。
inline const AafEntry *aafTable(std::size_t &count)
{
  static const AafEntry kTable[] = {
    {  42,  1,    1, 15},
    {  84,  2,    4, 13},
    { 126,  3,    9, 12},
    { 170,  4,   16, 11},
    { 213,  5,   25, 10},
    { 258,  6,   36, 10},
    { 303,  7,   49,  9},
    { 348,  8,   64,  9},
    { 394,  9,   81,  9},
    { 441, 10,  100,  8},
    { 488, 11,  122,  8},
    { 536, 12,  144,  8},
    { 585, 13,  170,  8},
    { 634, 14,  196,  7},
    { 684, 15,  224,  7},
    { 734, 16,  256,  7},
    { 785, 17,  288,  7},
    { 837, 18,  324,  7},
    { 890, 19,  360,  6},
    { 943, 20,  400,  6},
    { 997, 21,  440,  6},
    {1051, 22,  488,  6},
    {1107, 23,  528,  6},
    {1163, 24,  576,  6},
    {1220, 25,  624,  6},
    {1277, 26,  680,  6},
    {1336, 27,  736,  5},
    {1395, 28,  784,  5},
    {1454, 29,  848,  5},
    {1515, 30,  896,  5},
    {1577, 31,  960,  5},
    {1639, 32, 1024,  5},
    {1702, 33, 1088,  5},
    {1766, 34, 1152,  5},
    {1830, 35, 1232,  5},
    {1896, 36, 1296,  5},
    {1962, 37, 1376,  4},
    {2029, 38, 1440,  4},
    {2097, 39, 1536,  4},
    {2166, 40, 1600,  4},
    {2235, 41, 1696,  4},
    {2306, 42, 1760,  4},
    {2377, 43, 1856,  4},
    {2449, 44, 1952,  4},
    {2522, 45, 2016,  4},
    {2596, 46, 2112,  4},
    {2671, 47, 2208,  4},
    {2746, 48, 2304,  4},
    {2823, 49, 2400,  4},
    {2900, 50, 2496,  4},
    {2978, 51, 2592,  4},
    {3057, 52, 2720,  4},
    {3137, 53, 2816,  3},
    {3217, 54, 2944,  3},
    {3299, 55, 3008,  3},
    {3381, 56, 3136,  3},
    {3464, 57, 3264,  3},
    {3548, 58, 3392,  3},
    {3633, 59, 3456,  3},
    {3718, 60, 3584,  3},
    {3805, 61, 3712,  3},
    {3892, 62, 3840,  3},
    {3979, 63, 3968,  3},
  };
  count = sizeof(kTable) / sizeof(kTable[0]);
  return kTable;
}

/// 取最接近目标带宽的 AAF 档；目标 ≤0 时返回最低档。
inline const AafEntry &aafForBandwidth(float bw_hz)
{
  std::size_t n = 0;
  const AafEntry *t = aafTable(n);
  const AafEntry *best = &t[0];
  float best_err = 1.0e30f;
  for (std::size_t i = 0; i < n; ++i)
  {
    const float err = std::fabs(static_cast<float>(t[i].bw_hz) - bw_hz);
    if (err < best_err)
    {
      best_err = err;
      best = &t[i];
    }
  }
  return *best;
}

/// 手册 §5.2 notch 系数；f_khz 必须在 [1,3]，否则返回 false。
inline bool notchCoefficients(float f_khz, uint16_t &coswz, uint8_t &sel)
{
  if (f_khz < 1.0f || f_khz > 3.0f)
  {
    return false;
  }
  const float c = std::cos(2.0f * 3.14159265358979f * f_khz / 32.0f);
  if (std::fabs(c) <= 0.875f)
  {
    coswz = static_cast<uint16_t>(std::lround(c * 256.0f));
    sel = 0U;
  }
  else if (c > 0.875f)
  {
    coswz = static_cast<uint16_t>(std::lround(8.0f * (1.0f - c) * 256.0f));
    sel = 1U;
  }
  else
  {
    coswz = static_cast<uint16_t>(std::lround(-8.0f * (1.0f + c) * 256.0f));
    sel = 1U;
  }
  return true;
}

} // namespace imu_filter
