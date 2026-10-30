/**
 * @file PowerAllocation.h
 * @brief 制导/位置环加速度 → 推力方向分量（动力分配）的纯函数层
 *
 * 职责边界（2026-10-31 控制-机型解耦，用户要求"制导层输出三轴目标加速度，
 * 给动力分配层转换到目标姿态和推力大小"）：
 *   输入：机头/北东地系水平加速度需求、天向比力需求、当地重力、权限。
 *   输出：水平推力分量（sin 倾角，N/E）与合成比力模长。
 * 本层不碰控制器状态、不碰执行机构、不知道机型；姿态四元数构造仍由
 * generate_attitude_target() 调用 constructTiltTargetQuaternion() 完成。
 *
 * 注意：AUTO_POSITION 的油门/倾角滤波仍留在调用方（filter 位于分量计算与
 * 限幅之间，抽进来会改变滤波初值与顺序），本层只统一"圆形加速度限幅 +
 * 比力归一化 + 倾角分量限幅"这三步数学，保证与拆分前逐位一致。
 */
#ifndef POWER_ALLOCATION_H
#define POWER_ALLOCATION_H

#include <cmath>
#include <cstdint>

namespace bfs
{
namespace power
{

/** 饱和位（与 ControlContracts.h 的 ABI 位段兼容，从 bit10 起用）。 */
enum : uint32_t
{
  kSatNone = 0u,
  kSatAccelLimit = 1u << 10,  ///< 水平加速度圆形限幅生效
  kSatTiltLimit = 1u << 11,   ///< 推力分量倾角限幅生效
  kSatDegenerate = 1u << 12,  ///< 合成比力过小/非有限
};

struct PowerAllocationOutput
{
  float accel_n_mps2 = 0.0f;       ///< 圆形限幅后的北向加速度
  float accel_e_mps2 = 0.0f;       ///< 圆形限幅后的东向加速度
  float specific_force_mps2 = 0.0f;///< sqrt(an²+ae²+(g+az)²)（已做下限保护）
  float thrust_comp_n = 0.0f;      ///< 北向推力分量（sin 倾角）
  float thrust_comp_e = 0.0f;      ///< 东向推力分量（sin 倾角）
  uint32_t sat_mask = kSatNone;
  bool valid = false;
};

/**
 * @brief 水平加速度矢量圆形限幅
 * @param[in,out] an 北向加速度 (m/s²)
 * @param[in,out] ae 东向加速度 (m/s²)
 * @param[in] max_accel 允许的最大水平加速度模长 (m/s²)
 * @param[out] saturated 是否触发限幅
 * @note 与拆分前完全相同的运算顺序：mag_sq > max² 时按 max/sqrt(mag_sq) 缩放。
 */
inline void clampAccelCircle(float &an, float &ae, const float max_accel,
                             bool &saturated)
{
  const float mag_sq = an * an + ae * ae;
  const float max_sq = max_accel * max_accel;
  saturated = false;
  if (mag_sq > max_sq)
  {
    const float scale = max_accel / std::sqrt(mag_sq);
    an *= scale;
    ae *= scale;
    saturated = true;
  }
}

/**
 * @brief 三轴比力归一化，输出水平推力分量
 * @param[in] an 北向加速度 (m/s²)
 * @param[in] ae 东向加速度 (m/s²)
 * @param[in] az_up 天向加速度增量 (m/s²，向上为正)
 * @param[in] gravity 当地重力 (m/s²)
 * @param[in] min_force_norm 合成比力模长下限（位置环 0.5，制导 0）
 * @param[out] force_norm 合成比力模长
 * @return {comp_n, comp_e}
 */
inline void thrustComponentsFromAccel(const float an, const float ae,
                                      const float az_up, const float gravity,
                                      const float min_force_norm,
                                      float &comp_n, float &comp_e,
                                      float &force_norm, bool &degenerate)
{
  const float total_z = gravity + az_up;
  force_norm = std::sqrt(an * an + ae * ae + total_z * total_z);
  degenerate = !std::isfinite(force_norm) || force_norm < min_force_norm;
  if (force_norm < min_force_norm)
  {
    force_norm = min_force_norm;
  }
  if (!std::isfinite(force_norm) || force_norm <= 1.0e-6f)
  {
    comp_n = 0.0f;
    comp_e = 0.0f;
    force_norm = min_force_norm;
    degenerate = true;
    return;
  }
  comp_n = an / force_norm;
  comp_e = ae / force_norm;
}

/**
 * @brief 推力分量倾角限幅（sin 倾角上限）
 * @return 是否触发限幅
 */
inline bool clampTiltComponents(float &comp_n, float &comp_e,
                                const float max_comp)
{
  const float mag_sq = comp_n * comp_n + comp_e * comp_e;
  if (mag_sq > max_comp * max_comp)
  {
    const float scale = max_comp / std::sqrt(mag_sq);
    comp_n *= scale;
    comp_e *= scale;
    return true;
  }
  return false;
}

/**
 * @brief 组合入口：加速度 → 推力分量（无限幅滤波，GUIDED 用）
 *
 * AUTO_POSITION 需要"分量滤波后再倾角限幅"，请分别调用上面三个函数，
 * 以保持滤波状态与拆分前一致。
 */
inline PowerAllocationOutput allocatePowerFromAccel(
    const float accel_n, const float accel_e, const float accel_up,
    const float gravity, const float max_accel_mps2,
    const float max_tilt_comp, const float min_force_norm)
{
  PowerAllocationOutput out;
  out.accel_n_mps2 = accel_n;
  out.accel_e_mps2 = accel_e;

  bool accel_sat = false;
  clampAccelCircle(out.accel_n_mps2, out.accel_e_mps2, max_accel_mps2,
                   accel_sat);
  if (accel_sat)
  {
    out.sat_mask |= kSatAccelLimit;
  }

  bool degenerate = false;
  thrustComponentsFromAccel(out.accel_n_mps2, out.accel_e_mps2, accel_up,
                            gravity, min_force_norm, out.thrust_comp_n,
                            out.thrust_comp_e, out.specific_force_mps2,
                            degenerate);
  out.valid = !degenerate;
  if (degenerate)
  {
    out.sat_mask |= kSatDegenerate;
    return out;
  }

  if (clampTiltComponents(out.thrust_comp_n, out.thrust_comp_e, max_tilt_comp))
  {
    out.sat_mask |= kSatTiltLimit;
  }
  return out;
}

}  // namespace power
}  // namespace bfs

#endif  // POWER_ALLOCATION_H
