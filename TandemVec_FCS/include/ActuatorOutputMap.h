/**
 * @file    ActuatorOutputMap.h
 * @module  固件/控制（执行器归一化指令 → 硬件输出百分比，通用层）
 * @brief   按 VehicleProfile 输出表把归一化控制量映射为舵机/电调/阀门/点火/RCS
 *          输出；提供当前纵列双发的兼容适配器
 * @details 输入：机型输出表、归一化指令数组（舵机 [-1,1]，其余 [0,1]）；
 *          处理：按通道类型钳位/零点/量程映射，非法输入回落到零点；
 *          输出：固定长度输出命令表（逻辑硬件槽位 + 类型 + 值 + valid）。
 *          不依赖 Arduino/Eigen，不直接写 GPIO/PWM。
 * @theory  执行器输出 = 零点 + 归一化指令 × 量程；舵机/电调/阀门共用同一映射，
 *          点火为阈值开关。控制分配只产生归一化指令，硬件标定集中在本层。
 *          @ref THY-003 §2–§3；CTL-001 §8.6（输出限幅/执行器口径）
 * @frame   N/A（不解释姿态/位置；输入已是输出通道量纲）
 * @units   输出百分比 %（点火 0/1）；归一化指令无量纲
 * @note    当前 `mapTandemVecOutputs()` 与 `mapCoaxialVtvOutputs()` 分别是两套
 *          旧 mix 公式的等价适配器，用于把固定两舵机/两电机路径收敛到通用输出表；
 *          新增机型只需填 profile。共轴适配器不包含其 DirectTvcRate 控制律。
 * @test    test_host/test_vehicle_profile.cpp
 * @see     include/VehicleProfile.h、src/flight_control.cpp
 */

#ifndef TANDEMVEC_ACTUATOR_OUTPUT_MAP_H
#define TANDEMVEC_ACTUATOR_OUTPUT_MAP_H

#include <cmath>
#include <cstdint>

#include "VehicleProfile.h"

namespace bfs
{

struct OutputCommand
{
    uint8_t hardware_slot = 0;
    OutputKind kind = OutputKind::EscPwmPct;
    float value = 0.0f;
    bool valid = false;
};

struct OutputCommandSet
{
    uint8_t count = 0;
    OutputCommand commands[kMaxVehicleOutputs] = {};
};

inline float clampOutput(const float value, const float lo, const float hi)
{
    return value < lo ? lo : (value > hi ? hi : value);
}

/**
 * @brief 归一化指令 → 输出命令表
 * @param[in] profile 机型描述（必须通过 validateVehicleProfile）
 * @param[in] normalized 归一化指令；索引与 profile.outputs 一一对应
 * @param[in] normalized_count 输入数组长度
 * @return 输出命令表；非法 profile/输入为空时 count=0
 * @note 非有限输入不丢弃通道，而是回落到 zero_value 并置 valid=false，
 *       便于上层 failsafe/遥测识别，而不是静默少写一路执行器。
 */
inline OutputCommandSet mapNormalizedOutputs(const VehicleProfile &profile,
                                             const float *normalized,
                                             const uint8_t normalized_count)
{
    OutputCommandSet out;
    if (normalized == nullptr ||
        validateVehicleProfile(profile) != VehicleProfileError::Ok)
    {
        return out;
    }
    const uint8_t n = (normalized_count < profile.output_count)
                          ? normalized_count
                          : profile.output_count;
    for (uint8_t i = 0; i < n; ++i)
    {
        const OutputChannelSpec &channel = profile.outputs[i];
        OutputCommand command;
        command.hardware_slot = channel.hardware_slot;
        command.kind = channel.kind;
        const float raw = normalized[i];
        if (!std::isfinite(raw))
        {
            command.value = channel.zero_value;
            command.valid = false;
        }
        else if (channel.kind == OutputKind::IgnitionBool)
        {
            command.value = (raw >= 0.5f) ? channel.max_value : channel.min_value;
            command.valid = true;
        }
        else
        {
            const float x = (channel.kind == OutputKind::ServoPwmPct)
                                ? clampOutput(raw, -1.0f, 1.0f)
                                : clampOutput(raw, 0.0f, 1.0f);
            command.value = clampOutput(channel.zero_value + x * channel.span,
                                        channel.min_value, channel.max_value);
            command.valid = true;
        }
        out.commands[out.count++] = command;
    }
    return out;
}

struct TandemVecOutputCalibration
{
    float gear_ratio = 40.0f / 30.0f;
    float half_travel_deg = 45.0f;
    float dir_roll = 1.0f;
    float dir_pitch = 1.0f;
    float zero_roll_pct = 0.0f;
    float zero_pitch_pct = 0.0f;
};

/**
 * @brief 旧纵列双发 mix 公式的归一化适配器
 * @param[in] upper_gimbal_deg 上摆/滚转有向角 deg
 * @param[in] lower_gimbal_deg 下摆/俯仰有向角 deg
 * @param[in] motor1_pct 前电机输出百分比
 * @param[in] motor2_pct 尾电机输出百分比
 * @param[in] cal 舵机齿轮/行程/方向/中位标定
 * @param[out] out 4 个归一化量：roll servo, pitch servo, motor1, motor2
 * @formula servo_norm = gimbal_deg·gear·dir/half_travel + zero_pct/50；
 *          motor_norm = motor_pct/100
 * @ref CTL-001 §8.6；旧公式见 src/flight_control.cpp 层3
 * @note 只从旧公式抽取，不改变数值；方向/中位仍由 ServoConfig 提供。
 */
inline void normalizeTandemVecOutputs(const float upper_gimbal_deg,
                                      const float lower_gimbal_deg,
                                      const float motor1_pct,
                                      const float motor2_pct,
                                      const TandemVecOutputCalibration &cal,
                                      float out[4])
{
    const float safe_half = (std::fabs(cal.half_travel_deg) > 1.0e-3f)
                                ? cal.half_travel_deg
                                : 1.0f;
    out[0] = upper_gimbal_deg * cal.gear_ratio * cal.dir_roll / safe_half +
             cal.zero_roll_pct / 50.0f;
    out[1] = lower_gimbal_deg * cal.gear_ratio * cal.dir_pitch / safe_half +
             cal.zero_pitch_pct / 50.0f;
    out[2] = motor1_pct / 100.0f;
    out[3] = motor2_pct / 100.0f;
}

/**
 * @brief 旧纵列双发 mix 等价输出映射
 * @return 4 路输出命令（槽位 0/1 舵机，2/3 电调）
 * @note 与旧公式的唯一差异是先做归一化再按 profile 量程映射；数值等价由
 *       test_vehicle_profile 与 SIL 基线锁定。
 */
inline OutputCommandSet mapTandemVecOutputs(
    const float upper_gimbal_deg, const float lower_gimbal_deg,
    const float motor1_pct, const float motor2_pct,
    const TandemVecOutputCalibration &cal)
{
    float normalized[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    normalizeTandemVecOutputs(upper_gimbal_deg, lower_gimbal_deg, motor1_pct,
                              motor2_pct, cal, normalized);
    return mapNormalizedOutputs(kTandemVecProfile, normalized, 4U);
}

struct CoaxialVtvOutputCalibration
{
    float servo_half_angle_deg = 15.0f;
    float motor_min_us = 1000.0f;
    float motor_max_us = 2000.0f;
    float attitude_limit_deg = 45.0f;
};

struct CoaxialVtvOutputInput
{
    float roll_correction_deg = 0.0f;
    float pitch_correction_deg = 0.0f;
    float throttle_pct = 0.0f;
    float yaw_diff_us = 0.0f;
    float attitude_roll_deg = 0.0f;
    float attitude_pitch_deg = 0.0f;
    bool unlocked = false;
    bool attitude_safe = true;
};

/**
 * @brief 共轴双桨 VTVL 旧 mix 公式的兼容适配器
 * @details 外部工程 `VTVL_ElectricDualRotor_FCS` 的 mix：
 *          roll 舵机 = map(-roll,-15,15,66.67,33.33) ⇒ 50 + roll/15×16.67；
 *          pitch 舵机 = map(pitch,-15,15,66.67,33.33) ⇒ 50 − pitch/15×16.67；
 *          电机 = 1000–2000 µs 基准油门 ± yaw_diff，联合限幅；
 *          未解锁/大倾角保护时基准回 1000 µs。
 * @param[in] in 角度/油门/偏航差速/安全门控
 * @param[in] cal 舵机半行程、电机 PWM 量程、姿态保护角
 * @return 4 路输出命令（槽位 0/1 舵机，2/3 电调）
 * @note 该适配器只统一“输出/量纲”，不统一共轴 VTVL 的 DirectTvcRate 控制律；
 *       控制律接入仍是后续阶段。
 * @ref CTL-003 §2.2（共轴双桨 VTVL 对照）；
 *      外部实现参考 `VTVL_ElectricDualRotor_FCS/src/flight_control.cpp` mix 函数
 */
inline OutputCommandSet mapCoaxialVtvOutputs(
    const CoaxialVtvOutputInput &in,
    const CoaxialVtvOutputCalibration &cal)
{
    const float half = (std::fabs(cal.servo_half_angle_deg) > 1.0e-3f)
                           ? cal.servo_half_angle_deg
                           : 15.0f;
    const float span = cal.motor_max_us - cal.motor_min_us;
    const float safe_span = (span > 1.0e-3f) ? span : 1000.0f;
    const float throttle = clampOutput(in.throttle_pct, 0.0f, 100.0f);

    float base_us = cal.motor_min_us + throttle * 0.01f * safe_span;
    const bool attitude_ok =
        in.attitude_safe &&
        std::fabs(in.attitude_roll_deg) < cal.attitude_limit_deg &&
        std::fabs(in.attitude_pitch_deg) < cal.attitude_limit_deg;
    if (!in.unlocked || !attitude_ok)
    {
        base_us = cal.motor_min_us;
    }
    const float yaw_diff = clampOutput(in.yaw_diff_us,
                                       cal.motor_min_us - base_us,
                                       cal.motor_max_us - base_us);
    const float motor1_us =
        clampOutput(base_us - yaw_diff, cal.motor_min_us, cal.motor_max_us);
    const float motor2_us =
        clampOutput(base_us + yaw_diff, cal.motor_min_us, cal.motor_max_us);
    const float motor1_pct = (motor1_us - cal.motor_min_us) / safe_span * 100.0f;
    const float motor2_pct = (motor2_us - cal.motor_min_us) / safe_span * 100.0f;

    float normalized[4] = {
        in.roll_correction_deg / half,   // 上摆方向：roll 正 → 舵机百分比增大
        -in.pitch_correction_deg / half, // 外部工程 pitch 映射方向相反
        motor1_pct / 100.0f,
        motor2_pct / 100.0f,
    };
    return mapNormalizedOutputs(kCoaxialVtvProfile, normalized, 4U);
}

} // namespace bfs

#endif // TANDEMVEC_ACTUATOR_OUTPUT_MAP_H
