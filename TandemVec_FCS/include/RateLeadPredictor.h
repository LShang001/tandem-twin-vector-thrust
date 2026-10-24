/**
 * @file    RateLeadPredictor.h
 * @module  固件/控制（速率预测/超前补偿，延迟鲁棒候选）
 * @brief   用上一拍角加速度预测当前角速率：ω_pred = ω_meas + L·α_prev
 * @details 纯类；用于速率内环测量前馈，补偿传感器/执行器延迟造成的相位滞后。
 *          默认 L=0.05 s；调用方必须保存上一拍角加速度并按轴独立实例。
 *          **未接入默认飞控路径**；需 SIL/台架 A/B 后才能上机。
 * @theory  对近似积分型速率环，短时预测 ω(t+L)≈ω(t)+α·L 提供相位超前；
 *          是 Smith/predictor 类延迟补偿的简化形式。L 过大会放大噪声/超调。
 *          @ref VER-003 §3.11；CTL-001 §8.6
 * @frame   N/A（标量速率轴）
 * @units   ω 与 α 必须同轴同单位；L 为 s
 * @note    ★ 预测使用上一拍实际/指令角加速度；若用指令值代替实际值，
 *          需确认执行器滞后不会让预测失真。默认不接入飞行路径。
 * @test    test_host/test_rate_lead_predictor.cpp
 * @see     VER-003、simulations/three_vehicle_mc/fast_mc.py
 */
#ifndef TANDEMVEC_RATE_LEAD_PREDICTOR_H
#define TANDEMVEC_RATE_LEAD_PREDICTOR_H

#include <cmath>

namespace bfs
{

struct RateLeadParams
{
    float lead_s = 0.05f;
};

class RateLeadPredictor
{
public:
    void setParams(const RateLeadParams &p) { params_ = p; }
    const RateLeadParams &params() const { return params_; }

    float predict(const float rate, const float alpha_prev) const
    {
        if (!std::isfinite(rate) || !std::isfinite(alpha_prev) ||
            !(params_.lead_s >= 0.0f))
        {
            return std::isfinite(rate) ? rate : 0.0f;
        }
        return rate + params_.lead_s * alpha_prev;
    }

private:
    RateLeadParams params_{};
};

} // namespace bfs

#endif // TANDEMVEC_RATE_LEAD_PREDICTOR_H
