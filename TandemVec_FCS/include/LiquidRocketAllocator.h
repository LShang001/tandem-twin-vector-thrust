/**
 * @file    LiquidRocketAllocator.h
 * @module  固件/控制（液体火箭 VTVL 机型分配器试点）
 * @brief   α_cmd（FRD，rad/s²）+ 油门 → 双轴喷管摆角 + 差动 RCS + 主阀；
 *          与 CoaxialAllocator 共用 ControlContracts 的 α/饱和/残差语义
 * @details 本文件是"控制律与机型解耦"的第三个机型试点（四旋翼尚未接）：
 *          姿态/速率层只给 α；本分配器按火箭的 L1 假设做逆解——
 *            My =  L·T·sin δy   （喷管 y 轴，pitch）
 *            Mz =  L·T·sin δx   （喷管 x 轴，yaw）
 *            Mx =  k_rcs·u      （差动 RCS，roll；u∈[-1,1] 由左右通道合成）
 *          推力/阀门只做直通限幅，点火与推进动态仍由 LiquidRocketModel 负责。
 *          **不写硬件、不读全局**；不改变任何现有机型路径。
 * @frame   机体系 FRD：x=滚转、y=俯仰、z=偏航；推力沿 -z。
 *          符号约定按 r×F 的 pilot 假设：喷管 +δy 产生 +My、+δx 产生 +Mz；
 *          实机需按喷管安装/舵机极性核对（不在本次试点内改硬件）。
 * @units   α rad/s²，τ N·m，T N，δ rad，RCS 百分比 0–100
 * @theory  τ=I·α；δ=asin(τ/(L·T))；RCS 差动产生滚转力矩
 *          @ref THY-003 §2/§3；CTL-003 §3.3；VER-002 §C5
 * @note    ★ 本文件是 pilot：默认参数取 LiquidRocketModel 的量级
 *          （干重 13.5 kg、Tmax 200 N、Isp 200 s），不是实机辨识值；
 *          RCS 力矩/摆座力臂均为 MODEL-DEFAULT。未接入固件主循环，
 *          也未接 GenericControlAllocation（冗余通道分配留给后续）。
 * @test    test_host/test_liquid_rocket_allocator.cpp
 * @see     ControlContracts.h、LiquidRocketModel.h、CoaxialAllocator.h、CTL-006
 */
#ifndef BFS_LIQUID_ROCKET_ALLOCATOR_H
#define BFS_LIQUID_ROCKET_ALLOCATOR_H

#include <cmath>
#include <cstdint>

#include "ControlContracts.h"

namespace bfs
{

/** @brief 液体火箭分配参数（L1 量级；MODEL-DEFAULT） */
struct LiquidRocketAllocatorParams
{
    float lever_m = 0.50f;            /**< 喷管摆心到推力作用点力臂 */
    float max_total_thrust_n = 200.0f;/**< 发动机最大推力 */
    float min_thrust_n = 80.0f;       /**< 稳定工作最小推力（信息用） */
    float max_gimbal_deg = 8.0f;      /**< 单轴机械摆角上限 */
    float rcs_moment_nm = 2.0f;       /**< 单侧 RCS 满行程滚转力矩 */
    float inertia_kgm2[3] = {0.60f, 0.60f, 0.25f}; /**< Ixx/Iyy/Izz */
    float min_thrust_for_gimbal_n = 40.0f; /**< 低于此推力摆座权限按 0 */
    bool compensate_rate_coupling = false;
};

/** @brief 分配输入（控制层 α + 油门；点火状态由发动机层维护） */
struct LiquidRocketAllocatorInput
{
    float alpha_body_radps2[3] = {0.0f, 0.0f, 0.0f};
    float gyro_body_radps[3] = {0.0f, 0.0f, 0.0f};
    float throttle_pct = 0.0f;      /**< 0–100 */
    float thrust_estimate_n = 0.0f; /**< <=0 时按 throttle×Tmax 估计 */
    float dt_s = 0.005f;
};

/** @brief 分配输出（执行器指令 + 力矩诊断） */
struct LiquidRocketAllocatorOutput
{
    bool valid = false;
    float nozzle_x_deg = 0.0f;  /**< 喷管 x 轴（yaw） */
    float nozzle_y_deg = 0.0f;  /**< 喷管 y 轴（pitch） */
    float rcs_left_pct = 0.0f;  /**< 左 RCS 0–100（u≥0 时工作） */
    float rcs_right_pct = 0.0f; /**< 右 RCS 0–100（u<0 时工作） */
    float valve_pct = 0.0f;     /**< 主阀/油门直通 */
    float thrust_used_n = 0.0f;
    float alpha_applied[3] = {0.0f, 0.0f, 0.0f};
    float torque_requested_nm[3] = {0.0f, 0.0f, 0.0f};
    float torque_cmd_nm[3] = {0.0f, 0.0f, 0.0f};
    float torque_achieved_nm[3] = {0.0f, 0.0f, 0.0f};
    float torque_residual_nm = 0.0f;
    uint16_t sat_mask = control::kSatNone;
};

inline bool liquidRocketAllocParamsValid(const LiquidRocketAllocatorParams &p)
{
    return std::isfinite(p.lever_m) && p.lever_m > 0.0f &&
           std::isfinite(p.max_total_thrust_n) && p.max_total_thrust_n > 0.0f &&
           std::isfinite(p.max_gimbal_deg) && p.max_gimbal_deg > 0.0f &&
           std::isfinite(p.rcs_moment_nm) && p.rcs_moment_nm > 0.0f &&
           std::isfinite(p.inertia_kgm2[0]) && p.inertia_kgm2[0] > 0.0f &&
           std::isfinite(p.inertia_kgm2[1]) && p.inertia_kgm2[1] > 0.0f &&
           std::isfinite(p.inertia_kgm2[2]) && p.inertia_kgm2[2] > 0.0f &&
           std::isfinite(p.min_thrust_for_gimbal_n) &&
           p.min_thrust_for_gimbal_n >= 0.0f;
}

/**
 * @brief 液体火箭分配：α + 油门 → 摆角 / RCS / 主阀
 * @return valid=false 时输出回零、电机/RCS 为 0、sat_mask 含 kSatInvalid
 */
inline LiquidRocketAllocatorOutput allocateLiquidRocket(
    const LiquidRocketAllocatorInput &in, const LiquidRocketAllocatorParams &p)
{
    using namespace bfs::control;
    LiquidRocketAllocatorOutput out;

    const bool in_ok = controlFinite3(in.alpha_body_radps2) &&
                       controlFinite3(in.gyro_body_radps) &&
                       controlFinite(in.throttle_pct) &&
                       controlFinite(in.thrust_estimate_n) &&
                       controlFinite(in.dt_s) && in.dt_s > 0.0f;
    if (!in_ok || !liquidRocketAllocParamsValid(p))
    {
        out.valid = false;
        out.sat_mask = kSatInvalid;
        return out;
    }

    const float deg2rad = 0.017453292519943295f;
    const float throttle = controlClamp(in.throttle_pct, 0.0f, 100.0f);
    const float thrust =
        controlClamp((in.thrust_estimate_n > 0.0f)
                         ? in.thrust_estimate_n
                         : throttle * 0.01f * p.max_total_thrust_n,
                     0.0f, p.max_total_thrust_n);
    out.thrust_used_n = thrust;
    out.valve_pct = throttle;

    const float gimbal_limit_rad = p.max_gimbal_deg * deg2rad;
    const float sin_limit = std::sin(gimbal_limit_rad);
    const float gimbal_authority = p.lever_m * thrust;
    const bool gimbal_available =
        (gimbal_authority > p.min_thrust_for_gimbal_n * p.lever_m);

    float alpha_max[3] = {0.0f, 0.0f, 0.0f};
    alpha_max[0] = p.rcs_moment_nm / p.inertia_kgm2[0];
    alpha_max[1] = gimbal_available
                       ? gimbal_authority * sin_limit / p.inertia_kgm2[1]
                       : 0.0f;
    alpha_max[2] = gimbal_available
                       ? gimbal_authority * sin_limit / p.inertia_kgm2[2]
                       : 0.0f;

    float alpha[3];
    bool clamped[3] = {false, false, false};
    for (int i = 0; i < 3; ++i)
    {
        alpha[i] = in.alpha_body_radps2[i];
        if (std::fabs(alpha[i]) > alpha_max[i])
        {
            alpha[i] = (alpha[i] > 0.0f) ? alpha_max[i] : -alpha_max[i];
            clamped[i] = true;
            out.sat_mask |= (i == 0) ? kSatAlphaRoll
                                     : ((i == 1) ? kSatAlphaPitch : kSatAlphaYaw);
        }
        out.alpha_applied[i] = alpha[i];
    }
    // α 权限本身就是执行器行程权限：α 被钳位时同步置执行器饱和位，
    // 避免 kSatGimbalLimit / kSatRcsLimit 因"钳位后恰好等于边界"而不可达。
    if (clamped[0])
    {
        out.sat_mask |= kSatRcsLimit;
    }
    if ((clamped[1] || clamped[2]) && gimbal_available)
    {
        out.sat_mask |= kSatGimbalLimit;
    }

    auto torqueOf = [&](const float a[3], float t[3]) {
        t[0] = p.inertia_kgm2[0] * a[0];
        t[1] = p.inertia_kgm2[1] * a[1];
        t[2] = p.inertia_kgm2[2] * a[2];
        if (p.compensate_rate_coupling)
        {
            const float wx = in.gyro_body_radps[0];
            const float wy = in.gyro_body_radps[1];
            const float wz = in.gyro_body_radps[2];
            const float ix = p.inertia_kgm2[0];
            const float iy = p.inertia_kgm2[1];
            const float iz = p.inertia_kgm2[2];
            t[0] += wy * iz * wz - wz * iy * wy;
            t[1] += wz * ix * wx - wx * iz * wz;
            t[2] += wx * iy * wy - wy * ix * wx;
        }
    };
    float tau_req[3];
    float tau[3];
    torqueOf(in.alpha_body_radps2, tau_req);
    torqueOf(alpha, tau);
    for (int i = 0; i < 3; ++i)
    {
        out.torque_requested_nm[i] = tau_req[i];
        out.torque_cmd_nm[i] = tau[i];
    }

    // 1) 摆座：pitch/yaw 力矩 → 两轴摆角；推力不足时归中
    float delta_x = 0.0f;
    float delta_y = 0.0f;
    if (gimbal_available)
    {
        const float sy = controlClamp(tau[1] / gimbal_authority, -sin_limit, sin_limit);
        const float sx = controlClamp(tau[2] / gimbal_authority, -sin_limit, sin_limit);
        delta_y = std::asin(sy);
        delta_x = std::asin(sx);
        if (std::fabs(tau[1] / gimbal_authority) > sin_limit)
        {
            out.sat_mask |= kSatGimbalLimit;
        }
        if (std::fabs(tau[2] / gimbal_authority) > sin_limit)
        {
            out.sat_mask |= kSatGimbalLimit;
        }
    }
    else
    {
        out.sat_mask |= kSatLowThrust;
    }
    out.nozzle_x_deg = controlClamp(delta_x / deg2rad, -p.max_gimbal_deg,
                                    p.max_gimbal_deg);
    out.nozzle_y_deg = controlClamp(delta_y / deg2rad, -p.max_gimbal_deg,
                                    p.max_gimbal_deg);

    // 2) 差动 RCS：roll 力矩 → 左右通道（u>0 左喷，u<0 右喷）
    const float u_raw = tau[0] / p.rcs_moment_nm;
    const float u = controlClamp(u_raw, -1.0f, 1.0f);
    if (std::fabs(u_raw) > 1.0f)
    {
        out.sat_mask |= kSatRcsLimit;
    }
    if (u >= 0.0f)
    {
        out.rcs_left_pct = u * 100.0f;
        out.rcs_right_pct = 0.0f;
    }
    else
    {
        out.rcs_left_pct = 0.0f;
        out.rcs_right_pct = -u * 100.0f;
    }

    // 3) 实际力矩与残差
    out.torque_achieved_nm[0] = p.rcs_moment_nm * u;
    out.torque_achieved_nm[1] =
        gimbal_available ? gimbal_authority * std::sin(out.nozzle_y_deg * deg2rad) : 0.0f;
    out.torque_achieved_nm[2] =
        gimbal_available ? gimbal_authority * std::sin(out.nozzle_x_deg * deg2rad) : 0.0f;
    float residual = 0.0f;
    for (int i = 0; i < 3; ++i)
    {
        residual = std::fmax(residual,
                             std::fabs(tau_req[i] - out.torque_achieved_nm[i]));
    }
    out.torque_residual_nm = residual;
    out.valid = true;
    return out;
}

} // namespace bfs

#endif // BFS_LIQUID_ROCKET_ALLOCATOR_H
