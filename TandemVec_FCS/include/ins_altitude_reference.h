/**
 * @file    ins_altitude_reference.h
 * @module  固件/导航（高度基准换算）
 * @brief   在“相对起飞点气压高度”与“绝对 WGS84 高度”之间做确定性换算
 * @details 输入：起飞点绝对高度 origin_alt_m、DPS310 相对高度（向上为正）、
 *          EKF 的 NED 垂直分量；处理：只做一次加减与符号翻转，不做滤波/限幅；
 *          输出：送入 EKF 的绝对高度观测、供控制/遥测使用的相对高度与爬升率。
 *          不变量：所有函数都不修改输入，非有限值一律判失败而非钳位。
 * @theory  绝对高度 = 起飞点绝对高度 + 相对高度（同一 WGS84 基准，
 *          见 REF-120 §7.2 大地坐标与高度基准的定义）；
 *          NED“下正”与“向上为正”的符号关系 @ref MOD-002 §2
 * @frame   NED（z 向下为正）→ 向上为正的高度/垂速，仅翻转符号
 * @units   m、m/s；相对高度以起飞点为 0
 * @note    DPS310 对外发布的是相对起飞点高度，进入 EKF 前必须恢复为绝对高度，
 *          否则会把相对值当绝对观测（项目约定见 TandemVec_FCS/AGENTS.md 模块化架构节）。
 * @see     `handleNavigationSystem()` 气压高度更新段、`BARO_NOISE_STD`
 */

#pragma once

#include <cmath>

/**
 * @brief 把相对起飞点的气压高度恢复为绝对高度观测
 * @param[in]  takeoff_origin_altitude_m 起飞点绝对高度 [m]
 * @param[in]  relative_altitude_up_m    相对起飞点高度（向上为正）[m]
 * @param[out] absolute_altitude_m       绝对高度 [m]，失败时不写
 * @return true 输入有限且结果有限；false 含空指针/NaN/Inf
 * @pre  takeoff_origin_altitude_m 与相对高度属于同一气压基准（同一 Home）
 * @note  失败返回 false 而非钳位，调用方据此跳过本帧量测，避免把 0 当观测。
 */
inline bool InsComputeAbsoluteBaroAltitude(
    const double takeoff_origin_altitude_m,
    const float relative_altitude_up_m,
    double *absolute_altitude_m)
{
  if (absolute_altitude_m == nullptr ||
      !std::isfinite(takeoff_origin_altitude_m) ||
      !std::isfinite(relative_altitude_up_m))
  {
    return false;
  }

  *absolute_altitude_m =
      takeoff_origin_altitude_m + static_cast<double>(relative_altitude_up_m);
  return std::isfinite(*absolute_altitude_m);
}

inline float InsRelativeHeightUpFromNedDown(const float relative_down_m)
{
  return -relative_down_m;
}

inline float InsVerticalVelocityUpFromNedDown(const float velocity_down_mps)
{
  return -velocity_down_mps;
}

inline double InsTakeoffOriginAltitudeFromCurrent(
    const double current_absolute_altitude_m,
    const double takeoff_to_current_down_m)
{
  return current_absolute_altitude_m + takeoff_to_current_down_m;
}
