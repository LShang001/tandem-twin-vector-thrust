/**
 * @file    CoaxialVtvControl.h
 * @module  固件/控制（共轴双桨 VTVL DirectTvcRate 控制律，机型层，2026-10-24）
 * @brief   姿态/角速率环 → 双轴 TVC 修正角 + 偏航 PWM 差速；对应外部工程
 *          `VTVL_ElectricDualRotor_FCS` 的 DirectTvcRate 架构
 * @details 输入：滚转/俯仰姿态误差、机体角速率、偏航速率目标、手动 TVC 修正、
 *          姿态环使能、dt；处理：外环 P（误差→目标角速率，±80°/s）、内环
 *          PI（速率误差→修正角，±10°）、一阶输出滤波、条件积分抗饱和；
 *          偏航：PI（速率误差→us 差速，±180 us）；输出：两轴修正角与偏航差速。
 *          纯函数/类、零 Arduino/Eigen 依赖，可宿主直接回归。
 * @theory  ω_ref = Kp_att·e_att（限幅）；u = Kp_rate·(ω_ref−ω)+Ki_rate∫；
 *          u = clamp(u, ±MAX_CORRECTION)；yaw_diff = PI(ω_ref_yaw−ω_z)
 *          @ref CTL-003 §2.2；CTL-001 §4–§5（SAS 串级）
 * @frame   存档系：滚转绕 x_b、俯仰绕 y_b、偏航绕 z_b（推力轴）
 * @units   角度 deg、角速率 deg/s、修正角 deg、偏航差速 us、时间 s
 * @note    ★ 本模块只产生 DirectTvcRate 控制量，不写 GPIO；最终 33.33–66.67%
 *          舵机映射/1000–2000 µs 电机差速/45° 锁电机由
 *          `ActuatorOutputMap::mapCoaxialVtvOutputs()` 完成。
 *          增益默认值来自外部共轴工程（att 4.25、rate 0.25/0.08、yaw 3.8/30），
 *          参数为 MODEL-DEFAULT，未经本机台架/实机标定。
 * @redline 修正角 ±MAX_CORRECTION、偏航差速 ±180 us、非有限输入回零
 * @test    test_host/test_coaxial_vtv_control.cpp
 * @see     include/ActuatorOutputMap.h、include/VehicleProfile.h、CTL-003 §2.2
 */

#ifndef TANDEMVEC_COAXIAL_VTV_CONTROL_H
#define TANDEMVEC_COAXIAL_VTV_CONTROL_H

#include <cmath>

namespace bfs
{

struct CoaxialVtvParams
{
    float att_kp_roll = 4.25f;          // (deg/s)/deg
    float att_kp_pitch = 4.25f;
    float rate_kp_roll = 0.25f;         // deg / (deg/s)
    float rate_ki_roll = 0.08f;         // deg / ((deg/s)·s)
    float rate_kp_pitch = 0.25f;
    float rate_ki_pitch = 0.08f;
    float rate_kp_yaw = 3.8f;           // us / (deg/s)
    float rate_ki_yaw = 30.0f;          // us / ((deg/s)·s)
    float max_target_rate_dps = 80.0f;
    float max_correction_deg = 10.0f;
    float max_yaw_diff_us = 180.0f;
    float rate_integral_limit_deg = 5.0f;
    float yaw_integral_limit_us = 50.0f;
    float output_filter_alpha = 0.2f;   // 每拍一阶滤波
};

struct CoaxialVtvInput
{
    float error_roll_deg = 0.0f;
    float error_pitch_deg = 0.0f;
    float rate_roll_dps = 0.0f;
    float rate_pitch_dps = 0.0f;
    float rate_yaw_dps = 0.0f;
    float yaw_rate_target_dps = 0.0f;
    bool attitude_active = false;       // 解锁 && !手动TVC && 油门有效
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

class CoaxialVtvController
{
public:
    void setParams(const CoaxialVtvParams &params) { params_ = params; }
    const CoaxialVtvParams &params() const { return params_; }

    void reset()
    {
        int_roll_ = 0.0f;
        int_pitch_ = 0.0f;
        int_yaw_ = 0.0f;
        filt_roll_ = 0.0f;
        filt_pitch_ = 0.0f;
        filt_yaw_ = 0.0f;
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
            !std::isfinite(in.manual_roll_deg) ||
            !std::isfinite(in.manual_pitch_deg) ||
            !std::isfinite(in.dt_s) || !(in.dt_s > 0.0f))
        {
            reset();
            return out;
        }

        if (in.manual_tvc || !in.attitude_active)
        {
            reset();
            out.roll_correction_deg = coaxialVtvClamp(
                in.manual_roll_deg, -params_.max_correction_deg,
                params_.max_correction_deg);
            out.pitch_correction_deg = coaxialVtvClamp(
                in.manual_pitch_deg, -params_.max_correction_deg,
                params_.max_correction_deg);
            out.yaw_diff_us = 0.0f;
            out.valid = true;
            out.saturated =
                (out.roll_correction_deg != in.manual_roll_deg) ||
                (out.pitch_correction_deg != in.manual_pitch_deg);
            return out;
        }

        const float alpha =
            coaxialVtvClamp(params_.output_filter_alpha, 0.0f, 1.0f);
        out.roll_correction_deg = updateAxis(
            in.error_roll_deg, in.rate_roll_dps, params_.att_kp_roll,
            params_.rate_kp_roll, params_.rate_ki_roll,
            params_.rate_integral_limit_deg, params_.max_target_rate_dps,
            params_.max_correction_deg, in.dt_s, alpha, int_roll_, filt_roll_,
            out.saturated);
        out.pitch_correction_deg = updateAxis(
            in.error_pitch_deg, in.rate_pitch_dps, params_.att_kp_pitch,
            params_.rate_kp_pitch, params_.rate_ki_pitch,
            params_.rate_integral_limit_deg, params_.max_target_rate_dps,
            params_.max_correction_deg, in.dt_s, alpha, int_pitch_, filt_pitch_,
            out.saturated);

        const float yaw_err = in.yaw_rate_target_dps - in.rate_yaw_dps;
        const float yaw_unsat = params_.rate_kp_yaw * yaw_err +
                                params_.rate_ki_yaw * int_yaw_;
        if ((yaw_unsat < params_.max_yaw_diff_us && yaw_err > 0.0f) ||
            (yaw_unsat > -params_.max_yaw_diff_us && yaw_err < 0.0f))
        {
            int_yaw_ += yaw_err * in.dt_s;
            int_yaw_ = coaxialVtvClamp(int_yaw_, -params_.yaw_integral_limit_us,
                                       params_.yaw_integral_limit_us);
        }
        float yaw_raw = params_.rate_kp_yaw * yaw_err +
                        params_.rate_ki_yaw * int_yaw_;
        if (yaw_raw > params_.max_yaw_diff_us ||
            yaw_raw < -params_.max_yaw_diff_us)
        {
            out.saturated = true;
        }
        yaw_raw = coaxialVtvClamp(yaw_raw, -params_.max_yaw_diff_us,
                                  params_.max_yaw_diff_us);
        filt_yaw_ += alpha * (yaw_raw - filt_yaw_);
        out.yaw_diff_us = filt_yaw_;

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
    static float updateAxis(const float error_deg, const float rate_dps,
                            const float att_kp, const float rate_kp,
                            const float rate_ki, const float int_limit,
                            const float max_rate, const float max_correction,
                            const float dt_s, const float alpha,
                            float &integral, float &filtered,
                            bool &saturated)
    {
        float rate_ref = att_kp * error_deg;
        if (rate_ref > max_rate || rate_ref < -max_rate)
        {
            saturated = true;
        }
        rate_ref = coaxialVtvClamp(rate_ref, -max_rate, max_rate);
        const float rate_err = rate_ref - rate_dps;
        const float unsat = rate_kp * rate_err + rate_ki * integral;
        if ((unsat < max_correction && rate_err > 0.0f) ||
            (unsat > -max_correction && rate_err < 0.0f))
        {
            integral += rate_err * dt_s;
            integral = coaxialVtvClamp(integral, -int_limit, int_limit);
        }
        float raw = rate_kp * rate_err + rate_ki * integral;
        if (raw > max_correction || raw < -max_correction)
        {
            saturated = true;
        }
        raw = coaxialVtvClamp(raw, -max_correction, max_correction);
        filtered += alpha * (raw - filtered);
        return filtered;
    }

    CoaxialVtvParams params_{};
    float int_roll_ = 0.0f;
    float int_pitch_ = 0.0f;
    float int_yaw_ = 0.0f;
    float filt_roll_ = 0.0f;
    float filt_pitch_ = 0.0f;
    float filt_yaw_ = 0.0f;
};

} // namespace bfs

#endif // TANDEMVEC_COAXIAL_VTV_CONTROL_H
