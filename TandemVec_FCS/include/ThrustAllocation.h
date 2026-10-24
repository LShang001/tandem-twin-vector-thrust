/**
 * @file    ThrustAllocation.h
 * @module  固件/控制（垂直推力分配，通用层，2026-10-24）
 * @brief   把机型无关的期望天向比力 + 推力轴投影，映射为总推力需求与油门百分比
 * @details 输入：期望总比力（含重力补偿）、推力轴在 NED 天向的投影 cos、质量、
 *          最大总推力；处理：cos 下限保护、F_total = m·a_up/cos、0..F_max 限幅、
 *          线性油门映射；输出：总推力 N、油门 %、高/低饱和标记、valid。
 *          纯函数、零 Arduino/Eigen 依赖，可 g++ 宿主机直接回归。
 * @theory  F_up = m·a_up；F_total = F_up / cos_tilt；u = F_total / F_max
 *          @ref PROP-001 §2、CTL-001 §8.4（串级定高与倾角补偿）、CTL-003 §2
 * @frame   NED 天向；cos_tilt 取 |R33|（存档系 z_b=推力轴在天向的投影）
 * @units   比力 m/s²、质量 kg、推力 N、油门 %
 * @note    ★ 当前 `allocateLinearThrustThrottle()` 是 TandemVec 既有固件口径：
 *          油门线性映射到总推力。它不是 ESC 的 kT·ω² 物理逆，换物理曲线会改变
 *          实机行为；共轴/火箭/变质量推力曲线应作为独立机型曲线接入，并先用
 *          SIL 对照，不得在本函数里静默改公式。
 * @redline 垂直推力映射口径（m/MAX_THRUST/线性油门）与 cos_tilt 下限 0.5
 * @test    test_host/test_thrust_allocation.cpp
 * @see     include/TandemVecAllocator.h、include/ActuatorOutputMap.h、src/flight_control.cpp
 */

#ifndef TANDEMVEC_THRUST_ALLOCATION_H
#define TANDEMVEC_THRUST_ALLOCATION_H

#include <cmath>

namespace bfs
{

struct ThrustAllocationInput
{
    float specific_force_up_mps2 = 0.0f; // 期望总比力（含重力），天向为正
    float thrust_axis_up_cos = 1.0f;     // 推力轴在天向的投影 |R33|
    float mass_kg = 0.0f;                // 当前质量
    float max_total_thrust_n = 0.0f;     // 最大总推力（机型能力）
    float min_cos_tilt = 0.5f;           // 倾角补偿下限（cos60°）
};

struct ThrustAllocationOutput
{
    bool valid = false;
    float total_thrust_n = 0.0f;
    float throttle_pct = 0.0f;
    bool saturated_high = false;
    bool saturated_low = false;
};

inline float thrustAllocationClamp(const float v, const float lo,
                                   const float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/**
 * @brief 期望天向比力 → 总推力/线性油门（TandemVec 当前机型口径）
 * @param[in] in 比力、cos_tilt、质量、最大推力、cos 下限
 * @return 总推力/油门/饱和标记；非有限输入或参数非法时 valid=false、输出为零
 * @formula F_total = clamp(m·a_up / max(|cos_tilt|, min_cos), 0, F_max)；
 *          throttle_pct = F_total·100 / F_max
 * @ref CTL-001 §8.4；CTL-003 §2；PROP-001 §2
 * @note ★ 运算顺序锁定为旧 `mapFloat(F, 0, MAX_THRUST, 0, 100)` 的
 *       `F * 100 / F_max`（先乘后除），改顺序会造成末位差异、破坏 SIL 逐值基线。
 *       `|cos_tilt|` 的绝对值是必须的：机头朝上/朝下时 R33 符号相反，
 *       推力方向与竖直投影的几何关系不变。
 * @test test_host/test_thrust_allocation.cpp
 */
inline ThrustAllocationOutput allocateLinearThrustThrottle(
    const ThrustAllocationInput &in)
{
    ThrustAllocationOutput out;
    if (!std::isfinite(in.specific_force_up_mps2) ||
        !std::isfinite(in.thrust_axis_up_cos) ||
        !std::isfinite(in.mass_kg) ||
        !std::isfinite(in.max_total_thrust_n) ||
        !std::isfinite(in.min_cos_tilt) || !(in.mass_kg >= 0.0f) ||
        !(in.max_total_thrust_n > 0.0f) || !(in.min_cos_tilt > 0.0f) ||
        !(in.min_cos_tilt <= 1.0f))
    {
        return out;
    }

    float cos_tilt = std::fabs(in.thrust_axis_up_cos);
    cos_tilt = std::fmax(cos_tilt, in.min_cos_tilt);

    float total_thrust = in.mass_kg * in.specific_force_up_mps2 / cos_tilt;
    if (!std::isfinite(total_thrust))
    {
        return out;
    }

    out.saturated_high = (total_thrust > in.max_total_thrust_n);
    out.saturated_low = (total_thrust < 0.0f);
    total_thrust =
        thrustAllocationClamp(total_thrust, 0.0f, in.max_total_thrust_n);

    out.total_thrust_n = total_thrust;
    out.throttle_pct = total_thrust * 100.0f / in.max_total_thrust_n;
    out.valid = std::isfinite(out.throttle_pct);
    return out;
}

} // namespace bfs

#endif // TANDEMVEC_THRUST_ALLOCATION_H
