/**
 * @file    CoaxialAllocator.h
 * @module  固件/控制（共轴双桨 VTVL 机型分配器）
 * @brief   α_cmd（FRD，rad/s²）+ 油门 → 两轴 TVC 修正角 + 差速 µs + 电机 µs
 * @details 这是共轴机型的机型逆解边界，替换旧的"控制器直接输出修正角"路径。
 *          输入：机体系三轴角加速度、陀螺（已减零偏）、油门、推力估计、dt；
 *          处理：惯量逆解 τ=I·α（可选 ω×Iω 前馈）→ TVC 摆角逆解
 *                δ=asin(τ/(l·T)) → 差速逆解 diff=τ_z/k_yaw → 电机基准 µs；
 *          输出：修正角、差速、电机 µs、力矩实际/残差、饱和位。
 *          **不写硬件、不读全局**；只使用参数结构体。
 * @frame   机体系 FRD：x=滚转、y=俯仰、z=偏航/推力轴（悬停 z 指地）。
 *          与 SIL plant（fw_sil_env_main.cpp computeCoaxialWrench）一致：
 *          Mx=l·T·sin(δ_roll)、My=l·T·sin(δ_pitch)、Mz=k_yaw·diff_us。
 * @units   α rad/s²，ω rad/s，τ N·m，T N，δ rad，diff µs，电机 µs
 * @theory  M = I·α + ω×(I·ω)；δ = asin(M/(l·T))；Mz = k_yaw·diff
 *          @ref CTL-003 §2；THY-003 §2（惯量逆解）；VER-004 §10（共轴模型）
 * @note    k_yaw 默认由 SIL 归一化模型换算：plant 的 yaw_nm_per_norm=0.0005
 *          N·m 每归一化转速差、满行程 500 µs ⇒ 1.0e-6 N·m/µs。该值是
 *          MODEL-DEFAULT，不是台架辨识值；改为实测值需重新跑 A/B 与验收。
 *          低推力时 TVC 权限按 l·T 自然衰减；T 低于阈值时置 kSatLowThrust
 *          并把摆角归零（差速仍允许，偏航权限不依赖推力）。
 * @test    test_host/test_allocator_contracts.cpp（逆解/轴序/残差/边界）
 * @see     ControlContracts.h、CoaxialVtvControl.h、ActuatorOutputMap.h
 */
#ifndef BFS_COAXIAL_ALLOCATOR_H
#define BFS_COAXIAL_ALLOCATOR_H

#include <cmath>
#include <cstdint>

#include "ControlContracts.h"

namespace bfs
{

/** @brief 共轴机型分配参数；默认值 = SIL plant 的 L1 集总参数 */
struct CoaxialAllocatorParams
{
    float lever_m = 0.15f;                 /**< 舵机轴到推力线力臂 */
    float yaw_moment_nm_per_us = 1.0e-6f;  /**< 差速 1 µs 产生的偏航力矩 */
    float inertia_kgm2[3] = {0.05f, 0.05f, 0.08f}; /**< Ixx/Iyy/Izz */
    float servo_half_angle_deg = 15.0f;    /**< 舵机机械半行程 */
    float max_correction_deg = 10.0f;      /**< 控制/安全允许的 TVC 上限 */
    float max_yaw_diff_us = 180.0f;        /**< 差速上限 */
    float motor_min_us = 1000.0f;
    float motor_max_us = 2000.0f;
    float mass_kg = 0.84f;
    float max_total_thrust_n = 14.0f;
    float min_thrust_for_tvc_n = 0.35f;    /**< 低于此推力不给 TVC 权限 */
    bool compensate_rate_coupling = false; /**< 默认关：保持旧手感；开启=真 α 跟踪 */
};

/** @brief 分配器输入（控制层输出） */
struct CoaxialAllocatorInput
{
    float alpha_body_radps2[3] = {0.0f, 0.0f, 0.0f};
    float gyro_body_radps[3] = {0.0f, 0.0f, 0.0f};
    float throttle_pct = 0.0f;
    float thrust_estimate_n = 0.0f; /**< <=0 时按油门×max_thrust 估计 */
    float dt_s = 0.005f;
};

/** @brief 分配器输出（执行器层输入 + 遥测） */
struct CoaxialAllocatorOutput
{
    bool valid = false;
    float servo_roll_deg = 0.0f;  /**< 修正角（与旧控制器同口径） */
    float servo_pitch_deg = 0.0f; /**< 修正角（与旧控制器同口径） */
    float yaw_diff_us = 0.0f;
    float motor1_us = 1000.0f;
    float motor2_us = 1000.0f;
    float thrust_used_n = 0.0f;
    float alpha_applied[3] = {0.0f, 0.0f, 0.0f};
    float torque_requested_nm[3] = {0.0f, 0.0f, 0.0f};
    float torque_cmd_nm[3] = {0.0f, 0.0f, 0.0f};
    float torque_achieved_nm[3] = {0.0f, 0.0f, 0.0f};
    float torque_residual_nm = 0.0f;
    uint16_t sat_mask = control::kSatNone;
};

inline bool coaxialAllocParamsValid(const CoaxialAllocatorParams &p)
{
    return std::isfinite(p.lever_m) && p.lever_m > 0.0f &&
           std::isfinite(p.yaw_moment_nm_per_us) &&
           p.yaw_moment_nm_per_us > 0.0f &&
           std::isfinite(p.inertia_kgm2[0]) && p.inertia_kgm2[0] > 0.0f &&
           std::isfinite(p.inertia_kgm2[1]) && p.inertia_kgm2[1] > 0.0f &&
           std::isfinite(p.inertia_kgm2[2]) && p.inertia_kgm2[2] > 0.0f &&
           std::isfinite(p.servo_half_angle_deg) &&
           p.servo_half_angle_deg > 0.0f &&
           std::isfinite(p.max_correction_deg) && p.max_correction_deg > 0.0f &&
           std::isfinite(p.max_yaw_diff_us) && p.max_yaw_diff_us > 0.0f &&
           std::isfinite(p.motor_min_us) && std::isfinite(p.motor_max_us) &&
           p.motor_max_us > p.motor_min_us &&
           std::isfinite(p.max_total_thrust_n) && p.max_total_thrust_n > 0.0f &&
           std::isfinite(p.min_thrust_for_tvc_n) &&
           p.min_thrust_for_tvc_n >= 0.0f;
}

/**
 * @brief 共轴机型分配：α + 油门 → 摆角/差速/电机
 * @return valid=false 时执行器输出已回零（电机 min、差值 0），sat_mask 含
 *         kSatInvalid
 */
inline CoaxialAllocatorOutput allocateCoaxial(
    const CoaxialAllocatorInput &in, const CoaxialAllocatorParams &p)
{
    using namespace bfs::control;
    CoaxialAllocatorOutput out;

    const bool in_ok = controlFinite3(in.alpha_body_radps2) &&
                       controlFinite3(in.gyro_body_radps) &&
                       controlFinite(in.throttle_pct) &&
                       controlFinite(in.thrust_estimate_n) &&
                       controlFinite(in.dt_s) && in.dt_s > 0.0f;
    if (!in_ok || !coaxialAllocParamsValid(p))
    {
        out.valid = false;
        out.sat_mask = kSatInvalid;
        out.motor1_us = p.motor_min_us;
        out.motor2_us = p.motor_min_us;
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

    const float servo_limit_deg =
        controlClamp(p.max_correction_deg, 0.0f, p.servo_half_angle_deg);
    const float servo_limit_rad = servo_limit_deg * deg2rad;
    const float tvc_authority = p.lever_m * thrust;

    // 1) 物理角加速度权限：由 l·T 与机械行程反推
    float alpha_max[3] = {0.0f, 0.0f, 0.0f};
    alpha_max[0] = tvc_authority * std::sin(servo_limit_rad) / p.inertia_kgm2[0];
    alpha_max[1] = tvc_authority * std::sin(servo_limit_rad) / p.inertia_kgm2[1];
    alpha_max[2] = p.yaw_moment_nm_per_us * p.max_yaw_diff_us /
                   p.inertia_kgm2[2];

    float alpha[3];
    for (int i = 0; i < 3; ++i)
    {
        alpha[i] = in.alpha_body_radps2[i];
        if (std::fabs(alpha[i]) > alpha_max[i])
        {
            alpha[i] = (alpha[i] > 0.0f) ? alpha_max[i] : -alpha_max[i];
            out.sat_mask |= (i == 0) ? kSatAlphaRoll
                                     : ((i == 1) ? kSatAlphaPitch : kSatAlphaYaw);
            if (i == 2)
            {
                // 偏航 α 权限就是差速行程权限：α 钳位即差速饱和，
                // 同步置位，避免 kSatYawDiff 结构性不可达。
                out.sat_mask |= kSatYawDiff;
            }
        }
        out.alpha_applied[i] = alpha[i];
    }

    // 2) 力矩指令（FRD）；默认不加 ω×Iω，保证与旧路径手感一致。
    //    tau_req 保留"未钳位的请求"，用于饱和时的权限缺口/残差诊断；
    //    tau 是钳位后的可实现指令，驱动 TVC/差速逆解。
    const float ix = p.inertia_kgm2[0];
    const float iy = p.inertia_kgm2[1];
    const float iz = p.inertia_kgm2[2];
    auto torqueOf = [&](const float a[3], float t[3]) {
        t[0] = ix * a[0];
        t[1] = iy * a[1];
        t[2] = iz * a[2];
        if (p.compensate_rate_coupling)
        {
            const float wx = in.gyro_body_radps[0];
            const float wy = in.gyro_body_radps[1];
            const float wz = in.gyro_body_radps[2];
            t[0] += wy * iz * wz - wz * iy * wy;
            t[1] += wz * ix * wx - wx * iz * wz;
            t[2] += wx * iy * wy - wy * ix * wx;
        }
    };
    float tau_req[3];
    float tau[3];
    torqueOf(in.alpha_body_radps2, tau_req);
    torqueOf(alpha, tau);
    out.torque_requested_nm[0] = tau_req[0];
    out.torque_requested_nm[1] = tau_req[1];
    out.torque_requested_nm[2] = tau_req[2];
    out.torque_cmd_nm[0] = tau[0];
    out.torque_cmd_nm[1] = tau[1];
    out.torque_cmd_nm[2] = tau[2];

    // 3) TVC 摆角逆解；低推力或零力臂时归中
    const bool tvc_available = (tvc_authority > p.min_thrust_for_tvc_n * p.lever_m);
    float delta_roll = 0.0f;
    float delta_pitch = 0.0f;
    if (tvc_available)
    {
        const float sin_limit = std::sin(servo_limit_rad);
        const float sx = controlClamp(tau[0] / tvc_authority, -sin_limit, sin_limit);
        const float sy = controlClamp(tau[1] / tvc_authority, -sin_limit, sin_limit);
        delta_roll = std::asin(sx);
        delta_pitch = std::asin(sy);
        if (std::fabs(tau[0] / tvc_authority) > sin_limit)
        {
            out.sat_mask |= kSatTvcRoll;
        }
        if (std::fabs(tau[1] / tvc_authority) > sin_limit)
        {
            out.sat_mask |= kSatTvcPitch;
        }
    }
    else
    {
        out.sat_mask |= kSatLowThrust;
    }

    // 4) 差速逆解；电机行程钳位后回报残差
    const float yaw_gain = p.yaw_moment_nm_per_us;
    float yaw_diff = controlClamp(tau[2] / yaw_gain,
                                  -p.max_yaw_diff_us, p.max_yaw_diff_us);
    if (std::fabs(tau[2] / yaw_gain) > p.max_yaw_diff_us)
    {
        out.sat_mask |= kSatYawDiff;
    }

    const float span = p.motor_max_us - p.motor_min_us;
    const float base_us = p.motor_min_us + throttle * 0.01f * span;
    const float m1_raw = base_us - yaw_diff;
    const float m2_raw = base_us + yaw_diff;
    out.motor1_us = controlClamp(m1_raw, p.motor_min_us, p.motor_max_us);
    out.motor2_us = controlClamp(m2_raw, p.motor_min_us, p.motor_max_us);
    if (out.motor1_us != m1_raw || out.motor2_us != m2_raw)
    {
        out.sat_mask |= kSatMotorClamp;
    }
    const float yaw_diff_achieved_us = 0.5f * (out.motor2_us - out.motor1_us);
    out.yaw_diff_us = yaw_diff_achieved_us;

    out.servo_roll_deg = delta_roll / deg2rad;
    out.servo_pitch_deg = delta_pitch / deg2rad;
    // 数值上 asin 已保证 ±servo_limit；再做一次符号/边界保护
    out.servo_roll_deg = controlClamp(out.servo_roll_deg,
                                      -servo_limit_deg, servo_limit_deg);
    out.servo_pitch_deg = controlClamp(out.servo_pitch_deg,
                                       -servo_limit_deg, servo_limit_deg);

    // 5) 实际力矩与残差：残差 = 未钳位请求 − 实际可实现（饱和时 >0）
    out.torque_achieved_nm[0] =
        p.lever_m * thrust * std::sin(out.servo_roll_deg * deg2rad);
    out.torque_achieved_nm[1] =
        p.lever_m * thrust * std::sin(out.servo_pitch_deg * deg2rad);
    out.torque_achieved_nm[2] = yaw_gain * yaw_diff_achieved_us;
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

#endif // BFS_COAXIAL_ALLOCATOR_H
