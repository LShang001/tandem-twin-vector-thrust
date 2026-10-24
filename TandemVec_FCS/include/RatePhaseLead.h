/**
 * @file    RatePhaseLead.h
 * @module  固件/控制（速率相位超前滤波器，延迟鲁棒候选）
 * @brief   ω_eff = ω + K·(ω − LPF(ω))，提供中频相位超前而不放大差分噪声
 * @details 输入：滤波后角速率（deg/s）与 dt；处理：一阶低通提取慢分量，
 *          再用高通分量抬升；输出：超前后的速率（deg/s），送原速率 PID。
 *          DC 增益为 1，高频增益为 1+K；纯类、零 Arduino 依赖。
 *          **未接入默认飞控路径**。
 * @theory  超前网络/高通抬升在穿越频率附近提供相位超前，可补偿传感器/
 *          执行器延迟；与差分预测不同，不直接对噪声做微分。
 *          @ref VER-003 §3.11；CTL-001 §8.6
 * @frame   N/A（标量速率轴）
 * @units   deg/s、s；K 无量纲
 * @note    ★ 2026-10-24：lead（指令反馈）与 AAF（实测差分）SIL 均失败后
 *          的低噪声替代；默认不接入飞行路径，需 SIL/台架 A/B。
 * @test    test_host/test_rate_phase_lead.cpp
 * @see     VER-003、simulations/three_vehicle_mc/fast_mc.py
 */
#ifndef TANDEMVEC_RATE_PHASE_LEAD_H
#define TANDEMVEC_RATE_PHASE_LEAD_H

#include <cmath>

namespace bfs
{

struct RatePhaseLeadParams
{
    float gain = 0.5f;
    float tau_s = 0.02f;
};

class RatePhaseLead
{
public:
    void setParams(const RatePhaseLeadParams &p) { params_ = p; }
    const RatePhaseLeadParams &params() const { return params_; }

    void reset()
    {
        lp_dps_ = 0.0f;
        primed_ = false;
    }

    float update(const float rate_dps, const float dt_s)
    {
        if (!std::isfinite(rate_dps) || !std::isfinite(dt_s) ||
            !(dt_s > 0.0f))
        {
            return rate_dps;
        }
        if (!primed_)
        {
            lp_dps_ = rate_dps;
            primed_ = true;
            return rate_dps;
        }
        const float tau = (params_.tau_s > dt_s) ? params_.tau_s : dt_s;
        const float a = 1.0f - std::exp(-dt_s / tau);
        lp_dps_ += (rate_dps - lp_dps_) * a;
        return rate_dps + params_.gain * (rate_dps - lp_dps_);
    }

private:
    RatePhaseLeadParams params_{};
    float lp_dps_ = 0.0f;
    bool primed_ = false;
};

} // namespace bfs

#endif // TANDEMVEC_RATE_PHASE_LEAD_H
