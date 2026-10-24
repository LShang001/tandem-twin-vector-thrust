/**
 * @file    AdaptiveGainDamper.h
 * @module  固件/控制（振荡检测降增益，可选鲁棒层，2026-10-24）
 * @brief   监测速率误差换向频率；高频换向时逐步降低增益，安静时缓慢恢复
 * @details 输入：速率误差（rad/s 或 deg/s，量纲无关）、dt；处理：统计窗口内
 *          符号换向次数，超过阈值则 gain *= decay（下限 floor），否则
 *          gain *= recovery（上限 1）；输出：增益乘子 [floor,1]。
 *          纯类、零 Arduino 依赖；**未接入默认飞控路径**，只作为候选鲁棒层。
 * @theory  速率环高频换向 = 相位裕度不足/延迟过大；降低环路增益可把极点
 *          拉回稳定区。恢复慢于衰减，避免在振荡边界来回切换。
 *          @ref CTL-001 §8.6；CTL-003 §2；VER-003 §3.4
 * @frame   N/A（标量速率误差）
 * @units   时间 s；增益无量纲
 * @note    ★ 默认增益保持 1.0；要上机必须 SIL/台架 A/B，并保证可回退。
 *          该模块不能修复推力/质量物理包线或舵机卡死。
 * @test    test_host/test_adaptive_gain_damper.cpp
 * @see     VER-003、simulations/three_vehicle_mc/fast_mc.py
 */
#ifndef TANDEMVEC_ADAPTIVE_GAIN_DAMPER_H
#define TANDEMVEC_ADAPTIVE_GAIN_DAMPER_H

#include <cmath>

namespace bfs
{

struct AdaptiveGainDamperParams
{
    float window_s = 0.25f;
    int reversal_threshold = 6;
    float decay = 0.90f;
    float recovery = 1.01f;
    float floor = 0.55f;
};

class AdaptiveGainDamper
{
public:
    void setParams(const AdaptiveGainDamperParams &p) { params_ = p; }
    const AdaptiveGainDamperParams &params() const { return params_; }

    void reset()
    {
        gain_ = 1.0f;
        reversals_ = 0;
        timer_s_ = 0.0f;
        prev_sign_ = 0.0f;
    }

    float update(const float rate_error, const float dt_s)
    {
        if (!std::isfinite(rate_error) || !std::isfinite(dt_s) ||
            !(dt_s > 0.0f))
        {
            return gain_;
        }
        const float sign = (rate_error > 0.0f) ? 1.0f
                           : (rate_error < 0.0f ? -1.0f : 0.0f);
        if (sign != 0.0f && prev_sign_ != 0.0f && sign * prev_sign_ < 0.0f)
        {
            ++reversals_;
        }
        if (sign != 0.0f)
        {
            prev_sign_ = sign;
        }
        timer_s_ += dt_s;
        if (timer_s_ >= params_.window_s)
        {
            if (reversals_ > params_.reversal_threshold)
            {
                gain_ *= params_.decay;
                if (gain_ < params_.floor) gain_ = params_.floor;
            }
            else
            {
                gain_ *= params_.recovery;
                if (gain_ > 1.0f) gain_ = 1.0f;
            }
            reversals_ = 0;
            timer_s_ = 0.0f;
        }
        return gain_;
    }

    float gain() const { return gain_; }

private:
    AdaptiveGainDamperParams params_{};
    float gain_ = 1.0f;
    int reversals_ = 0;
    float timer_s_ = 0.0f;
    float prev_sign_ = 0.0f;
};

} // namespace bfs

#endif // TANDEMVEC_ADAPTIVE_GAIN_DAMPER_H
