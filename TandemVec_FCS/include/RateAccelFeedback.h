/**
 * @file    RateAccelFeedback.h
 * @module  固件/控制（实测角加速度反馈 AAF，延迟鲁棒候选）
 * @brief   用滤波后的陀螺角加速度提供附加阻尼：α_corr = -K·dω/dt
 * @details 输入：滤波后角速率（deg/s）与 dt；处理：一阶差分 + 低通滤波；
 *          输出：角加速度修正（deg/s²），由调用方加到速率环输出。
 *          纯类、零 Arduino 依赖；**未接入默认飞控路径**。
 * @theory  对速率环附加实测角加速度负反馈等价于增加阻尼/惯量，
 *          不经过指令通道，因此不会像反馈 alpha_ref 那样被积分器放大。
 *          相位上近似 PD 的 D 项；差分噪声由一阶低通抑制。
 *          @ref VER-003 §3.11；CTL-001 §8.6
 * @frame   N/A（标量速率轴）
 * @units   输入 deg/s；dt s；输出 deg/s²；K 无量纲
 * @note    ★ 2026-10-24：该模块是对 rate-lead SIL 失败的修正路线——
 *          lead 用上一拍指令 alpha_ref，构成近似代数正反馈；AAF 只用实测值。
 *          默认不接入飞行路径，需 SIL/台架 A/B。
 * @test    test_host/test_rate_accel_feedback.cpp
 * @see     VER-003、simulations/three_vehicle_mc/fast_mc.py
 */
#ifndef TANDEMVEC_RATE_ACCEL_FEEDBACK_H
#define TANDEMVEC_RATE_ACCEL_FEEDBACK_H

#include <cmath>

#ifndef BFS_RATE_ACCEL_FB_GAIN
#define BFS_RATE_ACCEL_FB_GAIN 0.8f
#endif

namespace bfs
{

struct RateAccelFeedbackParams
{
    float gain = BFS_RATE_ACCEL_FB_GAIN;
    float tau_s = 0.02f;
    float max_correction_dps2 = 500.0f;
};

class RateAccelFeedback
{
public:
    void setParams(const RateAccelFeedbackParams &p) { params_ = p; }
    const RateAccelFeedbackParams &params() const { return params_; }

    void reset()
    {
        prev_rate_dps_ = 0.0f;
        alpha_filt_dps2_ = 0.0f;
        correction_dps2_ = 0.0f;
        primed_ = false;
    }

    float update(const float rate_dps, const float dt_s)
    {
        if (!std::isfinite(rate_dps) || !std::isfinite(dt_s) ||
            !(dt_s > 0.0f))
        {
            return correction_dps2_;
        }
        if (!primed_)
        {
            prev_rate_dps_ = rate_dps;
            primed_ = true;
            return correction_dps2_;
        }
        const float raw_dps2 = (rate_dps - prev_rate_dps_) / dt_s;
        const float tau = (params_.tau_s > dt_s) ? params_.tau_s : dt_s;
        const float a = 1.0f - std::exp(-dt_s / tau);
        alpha_filt_dps2_ += (raw_dps2 - alpha_filt_dps2_) * a;
        prev_rate_dps_ = rate_dps;

        float corr = -params_.gain * alpha_filt_dps2_;
        const float lim = std::fabs(params_.max_correction_dps2);
        if (corr > lim) corr = lim;
        if (corr < -lim) corr = -lim;
        correction_dps2_ = corr;
        return correction_dps2_;
    }

    float alphaFilteredDps2() const { return alpha_filt_dps2_; }
    float correctionDps2() const { return correction_dps2_; }

private:
    RateAccelFeedbackParams params_{};
    float prev_rate_dps_ = 0.0f;
    float alpha_filt_dps2_ = 0.0f;
    float correction_dps2_ = 0.0f;
    bool primed_ = false;
};

} // namespace bfs

#endif // TANDEMVEC_RATE_ACCEL_FEEDBACK_H
