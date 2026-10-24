/**
 * @file    AdrcController.h
 * @module  固件/控制（一阶 ADRC + ESO，现代抗扰控制候选）
 * @brief   一阶自抗扰控制器：扩张状态观测器估计总扰动，前馈补偿后做带宽整定
 * @details 被控对象近似 `dy/dt = b0·u + d`；ESO 状态 z1≈y、z2≈d：
 *          e=z1−y, z1̇=z2+b0·u−wo·e, z2̇=−wo²·e；
 *          控制 u=(wc·(r−z1)−z2)/b0，输出限幅。纯类、零 Arduino 依赖。
 *          **未接入默认飞控路径**，先作为候选/仿真验证模块。
 * @theory  一阶线性 ADRC/ESO（韩京清；Gao 带宽参数化）：
 *          wo 为观测器带宽、wc 为控制器带宽，通常 wo≈3–5·wc；
 *          z2 补偿常值/慢变扰动，b0 为控制增益（速率环取 1 rad/s² per unit）。
 *          @ref CTL-001 §8.6；CTL-003 §2；VER-003 §3.10
 * @frame   N/A（标量速率/角度轴）
 * @units   y、r 同量纲；u 为 b0 量纲；时间 s
 * @note    ★ 默认不启用；b0 必须与实际执行器/惯量口径匹配，否则 ESO 会估错扰动。
 *          多轴使用时每轴独立实例；输出限幅必须与分配器/舵机权限一致。
 * @test    test_host/test_adrc_controller.cpp
 * @see     VER-003、simulations/three_vehicle_mc/fast_mc.py
 */
#ifndef TANDEMVEC_ADRC_CONTROLLER_H
#define TANDEMVEC_ADRC_CONTROLLER_H

#include <cmath>

namespace bfs
{

struct AdrcParams
{
    float wc = 16.0f;        // 控制器带宽 [rad/s]
    float wo = 50.0f;        // ESO 带宽 [rad/s]，建议 3–5×wc
    float b0 = 1.0f;         // 控制增益
    float dt = 0.005f;       // 控制周期 [s]
    float out_min = -100.0f;
    float out_max = 100.0f;
    float z2_limit = 1.0e4f; // 扰动估计限幅，防数值爆炸
};

class FirstOrderAdrc
{
public:
    void setParams(const AdrcParams &p) { params_ = p; }
    const AdrcParams &params() const { return params_; }

    void reset()
    {
        z1_ = 0.0f;
        z2_ = 0.0f;
        u_prev_ = 0.0f;
    }

    float update(const float setpoint, const float measurement)
    {
        const AdrcParams &p = params_;
        if (!std::isfinite(setpoint) || !std::isfinite(measurement) ||
            !(p.dt > 0.0f) || !(p.b0 > 1.0e-9f) ||
            !std::isfinite(p.wc) || !std::isfinite(p.wo))
        {
            return 0.0f;
        }
        const float e = z1_ - measurement;
        z1_ += (z2_ + p.b0 * u_prev_ - p.wo * e) * p.dt;
        z2_ += (-p.wo * p.wo * e) * p.dt;
        if (z2_ > p.z2_limit) z2_ = p.z2_limit;
        if (z2_ < -p.z2_limit) z2_ = -p.z2_limit;
        float u = (p.wc * (setpoint - z1_) - z2_) / p.b0;
        if (u > p.out_max) u = p.out_max;
        if (u < p.out_min) u = p.out_min;
        u_prev_ = u;
        return u;
    }

    float z1() const { return z1_; }
    float z2() const { return z2_; }

private:
    AdrcParams params_{};
    float z1_ = 0.0f;
    float z2_ = 0.0f;
    float u_prev_ = 0.0f;
};

} // namespace bfs

#endif // TANDEMVEC_ADRC_CONTROLLER_H
