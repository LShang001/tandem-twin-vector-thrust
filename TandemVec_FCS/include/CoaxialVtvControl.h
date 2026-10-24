/**
 * @file    CoaxialVtvControl.h
 * @module  固件/控制（共轴双桨 VTVL DirectTvcRate，机型层，2026-10-24）
 * @brief   对齐实飞共轴固件控制结构的 DirectTvcRate：姿态外环 PID → 目标角速率
 *          滤波 → 速率内环 PID → 修正角滤波；偏航速率 PID → PWM 差速。
 * @details 输入：滚转/俯仰姿态误差、机体角速率、偏航速率目标、手动 TVC 修正、
 *          姿态/偏航使能、dt；处理：复用 `PositionPID`（与实飞工程同一套
 *          位置式 PID 语义）+ 一阶互补滤波（姿态目标 α=0.85、修正角 α=0.25）
 *          + 输出限幅 ±80°/s、±10°、±180 µs；输出：两轴修正角与偏航差速。
 *          纯类、零 Arduino/Eigen 依赖，可宿主直接回归。
 * @theory  ω_ref = PID_att(e_att, −ω)（限幅 ±80°/s）；u = PID_rate(ω_ref, ω)
 *          （±10°）；yaw_diff = PID_yaw(ω_ref_yaw, ω_z)（±180 µs）
 *          @ref CTL-003 §2.2；CTL-001 §4–§5（SAS 串级）
 * @frame   存档系：滚转绕 x_b、俯仰绕 y_b、偏航绕 z_b（推力轴）
 * @units   角度 deg、角速率 deg/s、修正角 deg、偏航差速 us、时间 s
 * @note    ★ 增益/限幅/滤波口径对齐外部实飞工程 `VTVL_ElectricDualRotor_FCS`：
 *          att 4.25/0（±80，积分限 40）；rate 0.25/0.08（±10，积分限 5，
 *          微分滤波 0.2，输出滤波 0.25）；yaw 3.8/30（±180，积分限 50，
 *          阈值 40）。位置式 PID 的积分限幅/抗饱和语义由 `PositionPID` 保证。
 *          RATE_MODE 下实飞工程每拍重置 rate PID，本实现同样处理（速率环
 *          退化为纯 P），不要“顺手”改成积分保持。
 *          参数仍是外部工程的 MODEL-DEFAULT，未做本机台架/实机再标定。
 * @redline 修正角 ±10°、偏航差速 ±180 µs、RATE_MODE 每拍复位速率 PID
 * @test    test_host/test_coaxial_vtv_control.cpp
 * @see     include/ActuatorOutputMap.h、include/PositionPID.h、CTL-003 §2.2
 */

#ifndef TANDEMVEC_COAXIAL_VTV_CONTROL_H
#define TANDEMVEC_COAXIAL_VTV_CONTROL_H

#include <cmath>

#include "PositionPID.h"

namespace bfs
{

struct CoaxialVtvParams
{
    float att_kp_roll = 4.25f;
    float att_kp_pitch = 4.25f;
    float rate_kp_roll = 0.25f;
    float rate_ki_roll = 0.08f;
    float rate_kp_pitch = 0.25f;
    float rate_ki_pitch = 0.08f;
    float rate_kp_yaw = 3.8f;
    float rate_ki_yaw = 30.0f;
    float max_target_rate_dps = 80.0f;
    float max_correction_deg = 10.0f;
    float max_yaw_diff_us = 180.0f;
    float att_integral_limit = 40.0f;   // 外环 PID 积分项输出限幅 (deg/s)
    float rate_integral_limit = 5.0f;   // 内环 PID 积分项输出限幅 (deg)
    float yaw_integral_limit = 50.0f;   // 偏航 PID 积分项输出限幅 (us)
    float yaw_integral_threshold = 40.0f;
    float att_rate_filter_alpha = 0.85f;
    float correction_filter_alpha = 0.25f;
    float pid_derivative_filter_alpha = 0.2f;
};

struct CoaxialVtvInput
{
    float error_roll_deg = 0.0f;
    float error_pitch_deg = 0.0f;
    float rate_roll_dps = 0.0f;
    float rate_pitch_dps = 0.0f;
    float rate_yaw_dps = 0.0f;
    float yaw_rate_target_dps = 0.0f;
    bool rate_mode = false;
    float roll_rate_target_dps = 0.0f;
    float pitch_rate_target_dps = 0.0f;
    bool attitude_active = false; // 解锁 && !手动TVC && 油门有效
    bool yaw_active = false;      // 解锁 && 油门有效（手动TVC 也保留偏航）
    bool manual_tvc = false;
    float manual_roll_deg = 0.0f;
    float manual_pitch_deg = 0.0f;
    float dt_s = 0.005f;
};

struct CoaxialVtvOutput
{
    bool valid = false;
    float roll_correction_deg = 0.0f;
    float pitch_correction_deg = 0.0f;
    float yaw_diff_us = 0.0f;
    bool saturated = false;
};

inline float coaxialVtvClamp(const float v, const float lo, const float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

// 与实飞工程 ComplementaryFilter 相同的一阶滤波公式。
struct CoaxialFirstOrderFilter
{
    float state = 0.0f;
    void reset() { state = 0.0f; }
    float update(const float x, const float alpha)
    {
        const float a = coaxialVtvClamp(alpha, 0.0f, 1.0f);
        state = a * x + (1.0f - a) * state;
        return state;
    }
};

class CoaxialVtvController
{
public:
    CoaxialVtvController() { applyParams(); }

    void setParams(const CoaxialVtvParams &params)
    {
        params_ = params;
        applyParams();
        reset();
    }
    const CoaxialVtvParams &params() const { return params_; }

    void reset()
    {
        roll_angle_pid_.reset();
        pitch_angle_pid_.reset();
        roll_rate_pid_.reset();
        pitch_rate_pid_.reset();
        yaw_rate_pid_.reset();
        roll_rate_filter_.reset();
        pitch_rate_filter_.reset();
        roll_corr_filter_.reset();
        pitch_corr_filter_.reset();
    }

    CoaxialVtvOutput update(const CoaxialVtvInput &in)
    {
        CoaxialVtvOutput out;
        if (!std::isfinite(in.error_roll_deg) ||
            !std::isfinite(in.error_pitch_deg) ||
            !std::isfinite(in.rate_roll_dps) ||
            !std::isfinite(in.rate_pitch_dps) ||
            !std::isfinite(in.rate_yaw_dps) ||
            !std::isfinite(in.yaw_rate_target_dps) ||
            !std::isfinite(in.roll_rate_target_dps) ||
            !std::isfinite(in.pitch_rate_target_dps) ||
            !std::isfinite(in.manual_roll_deg) ||
            !std::isfinite(in.manual_pitch_deg) ||
            !std::isfinite(in.dt_s) || !(in.dt_s > 0.0f))
        {
            reset();
            return out;
        }

        // 内环/偏航 PID 的积分使能：与实飞工程 manage_pid_integrals 一致。
        roll_rate_pid_.setIntegralEnable(in.attitude_active && !in.rate_mode);
        pitch_rate_pid_.setIntegralEnable(in.attitude_active && !in.rate_mode);
        yaw_rate_pid_.setIntegralEnable(in.yaw_active);
        roll_angle_pid_.setIntegralEnable(in.attitude_active && !in.rate_mode);
        pitch_angle_pid_.setIntegralEnable(in.attitude_active && !in.rate_mode);

        // ---- 滚转/俯仰 ----
        if (!in.attitude_active || in.manual_tvc)
        {
            resetRollPitch();
            out.roll_correction_deg = coaxialVtvClamp(
                in.manual_roll_deg, -params_.max_correction_deg,
                params_.max_correction_deg);
            out.pitch_correction_deg = coaxialVtvClamp(
                in.manual_pitch_deg, -params_.max_correction_deg,
                params_.max_correction_deg);
            out.saturated =
                (out.roll_correction_deg != in.manual_roll_deg) ||
                (out.pitch_correction_deg != in.manual_pitch_deg);
        }
        else
        {
            float roll_rate_ref = 0.0f;
            float pitch_rate_ref = 0.0f;
            if (in.rate_mode)
            {
                // 实飞工程 RATE_MODE 每拍 reset 外环/内环，再直接给目标速率；
                // 速率环因此退化为纯 P（不累计积分/微分历史）。
                resetRollPitch();
                roll_rate_ref = in.roll_rate_target_dps;
                pitch_rate_ref = in.pitch_rate_target_dps;
            }
            else
            {
                roll_rate_ref = roll_angle_pid_.computeWithExternalDerivative(
                    in.error_roll_deg, 0.0f, in.rate_roll_dps, in.dt_s);
                pitch_rate_ref = pitch_angle_pid_.computeWithExternalDerivative(
                    in.error_pitch_deg, 0.0f, in.rate_pitch_dps, in.dt_s);
            }
            roll_rate_ref = roll_rate_filter_.update(
                roll_rate_ref, params_.att_rate_filter_alpha);
            pitch_rate_ref = pitch_rate_filter_.update(
                pitch_rate_ref, params_.att_rate_filter_alpha);

            float roll_corr = roll_rate_pid_.computeDerivativeOnMeasurement(
                roll_rate_ref, in.rate_roll_dps, in.dt_s);
            float pitch_corr = pitch_rate_pid_.computeDerivativeOnMeasurement(
                pitch_rate_ref, in.rate_pitch_dps, in.dt_s);
            roll_corr = roll_corr_filter_.update(
                roll_corr, params_.correction_filter_alpha);
            pitch_corr = pitch_corr_filter_.update(
                pitch_corr, params_.correction_filter_alpha);
            out.roll_correction_deg = roll_corr;
            out.pitch_correction_deg = pitch_corr;
            out.saturated = (std::fabs(roll_rate_ref) >=
                             params_.max_target_rate_dps - 1e-4f) ||
                            (std::fabs(pitch_rate_ref) >=
                             params_.max_target_rate_dps - 1e-4f) ||
                            (std::fabs(roll_corr) >=
                             params_.max_correction_deg - 1e-4f) ||
                            (std::fabs(pitch_corr) >=
                             params_.max_correction_deg - 1e-4f);
        }

        // ---- 偏航 ----
        if (in.yaw_active)
        {
            out.yaw_diff_us = yaw_rate_pid_.computeDerivativeOnMeasurement(
                in.yaw_rate_target_dps, in.rate_yaw_dps, in.dt_s);
            if (std::fabs(out.yaw_diff_us) >= params_.max_yaw_diff_us - 1e-4f)
            {
                out.saturated = true;
            }
        }
        else
        {
            yaw_rate_pid_.reset();
            out.yaw_diff_us = 0.0f;
        }

        out.valid = std::isfinite(out.roll_correction_deg) &&
                    std::isfinite(out.pitch_correction_deg) &&
                    std::isfinite(out.yaw_diff_us);
        if (!out.valid)
        {
            reset();
            out.roll_correction_deg = out.pitch_correction_deg = 0.0f;
            out.yaw_diff_us = 0.0f;
        }
        return out;
    }

private:
    void applyParams()
    {
        roll_angle_pid_ = PositionPID(
            params_.att_kp_roll, 0.0f, 0.0f, -params_.max_target_rate_dps,
            params_.max_target_rate_dps, true, params_.att_integral_limit, 0.0f,
            0.0f);
        pitch_angle_pid_ = PositionPID(
            params_.att_kp_pitch, 0.0f, 0.0f, -params_.max_target_rate_dps,
            params_.max_target_rate_dps, true, params_.att_integral_limit, 0.0f,
            0.0f);
        roll_rate_pid_ = PositionPID(
            params_.rate_kp_roll, params_.rate_ki_roll, 0.0f,
            -params_.max_correction_deg, params_.max_correction_deg, true,
            params_.rate_integral_limit, 0.0f,
            params_.pid_derivative_filter_alpha);
        pitch_rate_pid_ = PositionPID(
            params_.rate_kp_pitch, params_.rate_ki_pitch, 0.0f,
            -params_.max_correction_deg, params_.max_correction_deg, true,
            params_.rate_integral_limit, 0.0f,
            params_.pid_derivative_filter_alpha);
        yaw_rate_pid_ = PositionPID(
            params_.rate_kp_yaw, params_.rate_ki_yaw, 0.0f,
            -params_.max_yaw_diff_us, params_.max_yaw_diff_us, true,
            params_.yaw_integral_limit, params_.yaw_integral_threshold,
            params_.pid_derivative_filter_alpha);
    }

    void resetRollPitch()
    {
        roll_angle_pid_.reset();
        pitch_angle_pid_.reset();
        roll_rate_pid_.reset();
        pitch_rate_pid_.reset();
        roll_rate_filter_.reset();
        pitch_rate_filter_.reset();
        roll_corr_filter_.reset();
        pitch_corr_filter_.reset();
    }

    CoaxialVtvParams params_{};
    PositionPID roll_angle_pid_;
    PositionPID pitch_angle_pid_;
    PositionPID roll_rate_pid_;
    PositionPID pitch_rate_pid_;
    PositionPID yaw_rate_pid_;
    CoaxialFirstOrderFilter roll_rate_filter_;
    CoaxialFirstOrderFilter pitch_rate_filter_;
    CoaxialFirstOrderFilter roll_corr_filter_;
    CoaxialFirstOrderFilter pitch_corr_filter_;
};

} // namespace bfs

#endif // TANDEMVEC_COAXIAL_VTV_CONTROL_H
