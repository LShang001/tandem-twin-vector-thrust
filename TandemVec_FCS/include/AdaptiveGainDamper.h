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
 *          @ref CTL-001 §8.6；CTL-003 §2；VER-003 §3.8
 * @frame   N/A（标量速率误差）
 * @units   时间 s；增益无量纲；min_abs_error 与输入同单位
 * @note    ★ 2026-10-24 晚消融：100k L1 MC 中固定 gain_scale 0.85/0.70 的失败率
 *          1.01/0.83%，优于本模块选择性阻尼 1.51%；本模块暂不推荐上机，
 *          先验证固定增益/延迟裕度。若要研究，默认参数为
 *          1.0 s/2/0.90/1.01/0.55，filter 0.05 s，min 0.035 rad/s；
 *          输入 deg/s 时 min_abs_error 设 2.0；驾驶杆活动时 allow_update=false。
 *          多轴使用时每轴独立实例。要上机必须 SIL/台架 A/B。
 *          该模块不能修复推力/质量物理包线或舵机卡死。
 * @test    test_host/test_adaptive_gain_damper.cpp
 * @see     VER-003 §3.9、simulations/three_vehicle_mc/fast_mc.py
 */
#ifndef TANDEMVEC_ADAPTIVE_GAIN_DAMPER_H
#define TANDEMVEC_ADAPTIVE_GAIN_DAMPER_H

#include <cmath>

namespace bfs
{

struct AdaptiveGainDamperParams
{
    float window_s = 1.0f;
    int reversal_threshold = 2;
    float decay = 0.90f;
    float recovery = 1.01f;
    float floor = 0.55f;
    float filter_tau_s = 0.05f;
    // 与输入同单位；默认按 rad/s 设计（0.035 rad/s ≈ 2°/s）。
    // 若调用方传 deg/s，必须设成 2.0。
    float min_abs_error = 0.035f;
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
        err_filt_ = 0.0f;
    }

    float update(const float rate_error, const float dt_s,
                 const bool allow_update = true)
    {
        if (!std::isfinite(rate_error) || !std::isfinite(dt_s) ||
            !(dt_s > 0.0f))
        {
            return gain_;
        }
        if (!allow_update)
        {
            reversals_ = 0;
            timer_s_ = 0.0f;
            prev_sign_ = 0.0f;
            err_filt_ = 0.0f;
            return gain_;
        }
        const float a = 1.0f - std::exp(-dt_s /
                         (params_.filter_tau_s > 0.0f ? params_.filter_tau_s : dt_s));
        err_filt_ += (rate_error - err_filt_) * a;
        const float sign = (std::fabs(err_filt_) < params_.min_abs_error)
                           ? 0.0f
                           : (err_filt_ > 0.0f ? 1.0f : -1.0f);
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
    float err_filt_ = 0.0f;
};

} // namespace bfs

#endif // TANDEMVEC_ADAPTIVE_GAIN_DAMPER_H
