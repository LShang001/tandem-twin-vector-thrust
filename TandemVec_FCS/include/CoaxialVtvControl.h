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
 *          att 4.25/0（±80，积分限 40）；rate 0.25/0.08*0.005（±10，积分限 5，
 *          微分滤波 0.2，输出滤波 0.25）；yaw 3.8/0.15（±180，积分限 50，
 *          阈值 40）。位置式 PID 的积分限幅/抗饱和语义由本文件 `CoaxialLegacyPid` 保证。
 *          ATTITUDE_MODE 走串级：姿态外环 → 目标角速率 → 速率内环；RATE_MODE
 *          跳过外环、杆量直接给目标角速率，速率内环照常跟踪并保留积分/微分/
 *          滤波状态（每拍只复位两个角度 PID，避免切回姿态模式携带旧积分）。
 *          速率环 D 项当前 kd=0（等效 PI），参数保留，不改变串级结构。
 *          参数仍是外部工程的 MODEL-DEFAULT，未做本机台架/实机再标定。
 * @redline 修正角 ±10°、偏航差速 ±180 µs、RATE_MODE 只复位角度外环
 * @test    test_host/test_coaxial_vtv_control.cpp
 * @see     include/ActuatorOutputMap.h、CTL-003 §2.2
 */

#ifndef TANDEMVEC_COAXIAL_VTV_CONTROL_H
#define TANDEMVEC_COAXIAL_VTV_CONTROL_H

#include <cmath>

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
    bool attitude_active = false; // 闭环激活：解锁 && !手动TVC（不设油门门控）
    bool attitude_integral_active = false; // 姿态外环积分：ATTITUDE && 油门>1100
    bool rate_integral_active = false;     // 速率内环积分：两模式 && 油门>1100
    bool yaw_active = false;      // 偏航闭环：解锁 && 油门>1050
    bool yaw_integral_active = false;      // 偏航积分：解锁 && 油门>1100
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

// 原始实飞工程杆量→目标角速率：mapFloat(raw, 988, 2012, ∓max, ±max)，
// 即 (us-1500)/512*max_dps；俯仰轴反向。RATE_MODE 的滚转/俯仰与偏航共用。
// 注意：实飞工程 RATE 杆量是 ±80 dps 线性，不是 600 dps 曲线。
inline float coaxialVtvStickToRateDps(const float us, const float max_dps,
                                      const bool invert)
{
    const float n = coaxialVtvClamp((us - 1500.0f) / 512.0f, -1.0f, 1.0f);
    const float v = n * max_dps;
    return invert ? -v : v;
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

// 实飞共轴工程 PositionPID 的逐行等价实现（无 v3 无扰切换差异）。
// 仅供本机型控制律使用；TandemVec 仍使用项目统一的 PositionPID v3。
class CoaxialLegacyPid
{
public:
    CoaxialLegacyPid(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f,
                     float minOutput = -100.0f, float maxOutput = 100.0f,
                     bool enableIntegral = true, float integralLimit = 250.0f,
                     float integralThreshold = 0.0f,
                     float filterCoefficient = 0.0f)
        : kp_(kp), ki_(ki), kd_(kd), integral_(0.0f), previousError_(0.0f),
          previousInput_(0.0f), previousDerivative_(0.0f), lastOutput_(0.0f),
          outputMin_(minOutput), outputMax_(maxOutput),
          integralLimit_(integralLimit), integralThreshold_(integralThreshold),
          filterCoefficient_(filterCoefficient),
          enableIntegral_(enableIntegral)
    {
        setFilterCoefficient(filterCoefficient_);
    }

    void setOutputLimits(float lo, float hi)
    {
        outputMin_ = lo;
        outputMax_ = hi;
    }
    void setIntegralEnable(bool enable)
    {
        enableIntegral_ = enable;
        if (!enableIntegral_)
        {
            integral_ = 0.0f;
        }
    }
    void setIntegralLimit(float limit)
    {
        integralLimit_ = std::fabs(limit);
    }
    void setIntegralThreshold(float threshold)
    {
        integralThreshold_ = std::fabs(threshold);
    }
    void setFilterCoefficient(float coefficient)
    {
        filterCoefficient_ = (coefficient <= 0.0f || coefficient > 1.0f)
                                 ? 1.0f
                                 : coefficient;
    }
    void reset()
    {
        integral_ = 0.0f;
        previousError_ = 0.0f;
        previousInput_ = 0.0f;
        previousDerivative_ = 0.0f;
        lastOutput_ = 0.0f;
    }

    float computeWithExternalDerivative(float setpoint, float input,
                                        float derivative, float dt)
    {
        if (!isFiniteF(setpoint) || !isFiniteF(input) ||
            !isFiniteF(derivative))
        {
            return lastOutput_;
        }
        const float error = setpoint - input;
        const float filteredDerivative = filterDerivative(-derivative);
        const float output = computePID(error, filteredDerivative, dt);
        previousError_ = error;
        previousInput_ = input;
        lastOutput_ = output;
        return output;
    }

    float computeDerivativeOnMeasurement(float setpoint, float input, float dt)
    {
        if (!isFiniteF(setpoint) || !isFiniteF(input))
        {
            return lastOutput_;
        }
        const float error = setpoint - input;
        const float derivative = previousInput_ - input;
        const float filteredDerivative = filterDerivative(derivative);
        const float output = computePID(error, filteredDerivative, dt);
        previousError_ = error;
        previousInput_ = input;
        lastOutput_ = output;
        return output;
    }

private:
    static bool isFiniteF(float v)
    {
        return (v == v) && (v < 3.0e38f) && (v > -3.0e38f);
    }
    float filterDerivative(float currentDerivative)
    {
        if (filterCoefficient_ >= 1.0f)
        {
            previousDerivative_ = currentDerivative;
            return currentDerivative;
        }
        const float filtered = filterCoefficient_ * currentDerivative +
                               (1.0f - filterCoefficient_) * previousDerivative_;
        previousDerivative_ = filtered;
        return filtered;
    }
    void clampIntegralState()
    {
        if (ki_ == 0.0f)
        {
            integral_ = 0.0f;
            return;
        }
        const float bound = std::fabs(integralLimit_ / ki_);
        integral_ = coaxialVtvClamp(integral_, -bound, bound);
    }
    float computePID(float error, float derivative, float dt)
    {
        const float pOut = kp_ * error;
        float iOut = 0.0f;
        if (enableIntegral_ && ki_ != 0.0f)
        {
            if (integralThreshold_ <= 0.0f ||
                std::fabs(error) < integralThreshold_)
            {
                integral_ += error * dt;
            }
            clampIntegralState();
            iOut = coaxialVtvClamp(ki_ * integral_, -integralLimit_,
                                   integralLimit_);
        }
        else
        {
            integral_ = 0.0f;
        }
        const float dOut = kd_ * derivative;
        return coaxialVtvClamp(pOut + iOut + dOut, outputMin_, outputMax_);
    }

    float kp_, ki_, kd_;
    float integral_, previousError_, previousInput_, previousDerivative_,
        lastOutput_;
    float outputMin_, outputMax_, integralLimit_, integralThreshold_,
        filterCoefficient_;
    bool enableIntegral_;
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

        // 积分使能对齐实飞工程 manage_pid_integrals：外环积分只在 ATTITUDE
        // 且油门>1100；内环积分在 ATTITUDE/RATE 两种模式都使能（油门>1100）；
        // 偏航积分按油门>1100。RATE 下禁用内环积分是本次修正掉的错误语义。
        roll_rate_pid_.setIntegralEnable(in.rate_integral_active);
        pitch_rate_pid_.setIntegralEnable(in.rate_integral_active);
        yaw_rate_pid_.setIntegralEnable(in.yaw_integral_active);
        roll_angle_pid_.setIntegralEnable(in.attitude_integral_active &&
                                          !in.rate_mode);
        pitch_angle_pid_.setIntegralEnable(in.attitude_integral_active &&
                                           !in.rate_mode);

        // ---- 滚转/俯仰 ----
        if (!in.attitude_active || in.manual_tvc)
        {
            // 原始 execute_attitude_controller 的 else 分支：复位角度/速率 PID，
            // 摇杆目标角映射到修正角后仍过 α0.25 输出滤波（滤波状态不清），
            // 因此手动 TVC 切换没有阶跃。
            resetAngleRatePids();
            const float manual_roll = coaxialVtvClamp(
                in.manual_roll_deg, -params_.max_correction_deg,
                params_.max_correction_deg);
            const float manual_pitch = coaxialVtvClamp(
                in.manual_pitch_deg, -params_.max_correction_deg,
                params_.max_correction_deg);
            out.roll_correction_deg = roll_corr_filter_.update(
                manual_roll, params_.correction_filter_alpha);
            out.pitch_correction_deg = pitch_corr_filter_.update(
                manual_pitch, params_.correction_filter_alpha);
            out.saturated = (manual_roll != in.manual_roll_deg) ||
                            (manual_pitch != in.manual_pitch_deg);
        }
        else
        {
            float roll_rate_ref = 0.0f;
            float pitch_rate_ref = 0.0f;
            if (in.rate_mode)
            {
                // RATE_MODE 跳过姿态外环，只清外环历史；速率内环 PID 的积分、
                // 微分历史与两级滤波（α0.85 目标、α0.25 输出）全部跨拍保持，
                // 与原始 execute_attitude_controller 的 RATE 分支一致。
                roll_angle_pid_.reset();
                pitch_angle_pid_.reset();
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
        roll_angle_pid_ = CoaxialLegacyPid(
            params_.att_kp_roll, 0.0f, 0.0f, -params_.max_target_rate_dps,
            params_.max_target_rate_dps, true, params_.att_integral_limit, 0.0f,
            0.0f);
        pitch_angle_pid_ = CoaxialLegacyPid(
            params_.att_kp_pitch, 0.0f, 0.0f, -params_.max_target_rate_dps,
            params_.max_target_rate_dps, true, params_.att_integral_limit, 0.0f,
            0.0f);
        roll_rate_pid_ = CoaxialLegacyPid(
            params_.rate_kp_roll, params_.rate_ki_roll, 0.0f,
            -params_.max_correction_deg, params_.max_correction_deg, true,
            params_.rate_integral_limit, 0.0f,
            params_.pid_derivative_filter_alpha);
        pitch_rate_pid_ = CoaxialLegacyPid(
            params_.rate_kp_pitch, params_.rate_ki_pitch, 0.0f,
            -params_.max_correction_deg, params_.max_correction_deg, true,
            params_.rate_integral_limit, 0.0f,
            params_.pid_derivative_filter_alpha);
        yaw_rate_pid_ = CoaxialLegacyPid(
            params_.rate_kp_yaw, params_.rate_ki_yaw, 0.0f,
            -params_.max_yaw_diff_us, params_.max_yaw_diff_us, true,
            params_.yaw_integral_limit, params_.yaw_integral_threshold,
            params_.pid_derivative_filter_alpha);
    }

    // 全量复位：仅用于输入非法/控制器重建（会丢掉滤波状态）。
    void resetRollPitch()
    {
        resetAngleRatePids();
        roll_rate_filter_.reset();
        pitch_rate_filter_.reset();
        roll_corr_filter_.reset();
        pitch_corr_filter_.reset();
    }

    // 原始 else 分支的复位范围：角度+速率 PID；保留 α0.85/α0.25 滤波状态。
    void resetAngleRatePids()
    {
        roll_angle_pid_.reset();
        pitch_angle_pid_.reset();
        roll_rate_pid_.reset();
        pitch_rate_pid_.reset();
    }

    CoaxialVtvParams params_{};
    CoaxialLegacyPid roll_angle_pid_;
    CoaxialLegacyPid pitch_angle_pid_;
    CoaxialLegacyPid roll_rate_pid_;
    CoaxialLegacyPid pitch_rate_pid_;
    CoaxialLegacyPid yaw_rate_pid_;
    CoaxialFirstOrderFilter roll_rate_filter_;
    CoaxialFirstOrderFilter pitch_rate_filter_;
    CoaxialFirstOrderFilter roll_corr_filter_;
    CoaxialFirstOrderFilter pitch_corr_filter_;
};

} // namespace bfs

#endif // TANDEMVEC_COAXIAL_VTV_CONTROL_H
