/**
 * @file    coax_reference.cpp
 * @module  固件/宿主参考（实飞共轴 DirectTvcRate 输出级参考）
 * @brief   逐行复刻外部实飞共轴工程的控制链，用于逐拍输出对比
 * @details 输入/输出见 coax_reference.h；内部使用外部 PositionPID 与
 *          ComplementaryFilter 原头文件，增益/限幅/滤波与实飞工程一致。
 *          仅测试编译，不写 GPIO、不参与固件。
 * @theory  ω_ref=PID_att(e,−ω)；u=PID_rate(ω_ref,ω)；yaw=PID_yaw(ω_ref,ω_z)
 *          @ref CTL-003 §2.2
 * @frame   存档系 x=滚转、y=俯仰、z=推力轴
 * @units   deg、deg/s、deg、us、s
 * @note    ★ 参考实现的 PID 是外部实飞工程的 PositionPID 版本；统一固件
 *          CoaxialVtvControl 使用同语义 CoaxialLegacyPid。本文件不接入固件。
 * @test    test_host/test_coaxial_reference_equivalence.cpp
 */
#include "coax_reference.h"

#include <Arduino.h> // 宿主机桩提供 constrain；实飞工程由 Arduino 核提供

#include "PositionPID.h"
#include "ComplementaryFilter.h"

namespace
{
PositionPID roll_angle_pid(4.25f, 0.0f, 0.0f, -80.0f, 80.0f, true, 40.0f,
                          0.0f, 0.0f);
PositionPID pitch_angle_pid(4.25f, 0.0f, 0.0f, -80.0f, 80.0f, true, 40.0f,
                           0.0f, 0.0f);
PositionPID roll_rate_pid(0.25f, 0.08f, 0.0f, -10.0f, 10.0f, true, 5.0f,
                          0.0f, 0.2f);
PositionPID pitch_rate_pid(0.25f, 0.08f, 0.0f, -10.0f, 10.0f, true, 5.0f,
                           0.0f, 0.2f);
PositionPID yaw_rate_pid(3.8f, 30.0f, 0.0f, -180.0f, 180.0f, true, 50.0f,
                         40.0f, 0.2f);
ComplementaryFilter roll_angle_filter(0.85f);
ComplementaryFilter pitch_angle_filter(0.85f);
ComplementaryFilter roll_output_filter(0.25f);
ComplementaryFilter pitch_output_filter(0.25f);

float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void reset_roll_pitch()
{
    roll_angle_pid.reset();
    pitch_angle_pid.reset();
    roll_rate_pid.reset();
    pitch_rate_pid.reset();
    roll_angle_filter.reset();
    pitch_angle_filter.reset();
    roll_output_filter.reset();
    pitch_output_filter.reset();
}
} // namespace

void coax_ref_reset()
{
    reset_roll_pitch();
    yaw_rate_pid.reset();
    roll_angle_pid.setIntegralEnable(true);
    pitch_angle_pid.setIntegralEnable(true);
    roll_rate_pid.setIntegralEnable(true);
    pitch_rate_pid.setIntegralEnable(true);
    yaw_rate_pid.setIntegralEnable(true);
}

void coax_ref_update(const CoaxRefInput *in, CoaxRefOutput *out)
{
    if (in == nullptr || out == nullptr)
    {
        return;
    }
    out->roll_correction_deg = 0.0f;
    out->pitch_correction_deg = 0.0f;
    out->yaw_diff_us = 0.0f;

    roll_rate_pid.setIntegralEnable(in->attitude_active && !in->rate_mode);
    pitch_rate_pid.setIntegralEnable(in->attitude_active && !in->rate_mode);
    roll_angle_pid.setIntegralEnable(in->attitude_active && !in->rate_mode);
    pitch_angle_pid.setIntegralEnable(in->attitude_active && !in->rate_mode);
    yaw_rate_pid.setIntegralEnable(in->yaw_active != 0);

    if (!in->attitude_active || in->manual_tvc)
    {
        reset_roll_pitch();
        out->roll_correction_deg =
            clampf(in->manual_roll_deg, -10.0f, 10.0f);
        out->pitch_correction_deg =
            clampf(in->manual_pitch_deg, -10.0f, 10.0f);
    }
    else
    {
        float roll_rate_ref = 0.0f;
        float pitch_rate_ref = 0.0f;
        if (in->rate_mode)
        {
            reset_roll_pitch();
            roll_rate_ref = in->roll_rate_target_dps;
            pitch_rate_ref = in->pitch_rate_target_dps;
        }
        else
        {
            roll_rate_ref = roll_angle_pid.computeWithExternalDerivative(
                in->error_roll_deg, 0.0f, in->rate_roll_dps, in->dt_s);
            pitch_rate_ref = pitch_angle_pid.computeWithExternalDerivative(
                in->error_pitch_deg, 0.0f, in->rate_pitch_dps, in->dt_s);
        }
        roll_rate_ref = roll_angle_filter.filter(roll_rate_ref);
        pitch_rate_ref = pitch_angle_filter.filter(pitch_rate_ref);
        out->roll_correction_deg = roll_rate_pid.computeDerivativeOnMeasurement(
            roll_rate_ref, in->rate_roll_dps, in->dt_s);
        out->pitch_correction_deg =
            pitch_rate_pid.computeDerivativeOnMeasurement(
                pitch_rate_ref, in->rate_pitch_dps, in->dt_s);
        out->roll_correction_deg =
            roll_output_filter.filter(out->roll_correction_deg);
        out->pitch_correction_deg =
            pitch_output_filter.filter(out->pitch_correction_deg);
    }

    if (in->yaw_active)
    {
        out->yaw_diff_us = yaw_rate_pid.computeDerivativeOnMeasurement(
            in->yaw_rate_target_dps, in->rate_yaw_dps, in->dt_s);
    }
    else
    {
        yaw_rate_pid.reset();
    }
}
