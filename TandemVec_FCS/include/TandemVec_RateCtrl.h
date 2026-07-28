// ============================================================
//  TandemVec_RateCtrl.h — 内环：目标角速率 → 目标角加速度
//
//  三轴独立位置式 PID，输出物理量 α_ref（rad/s²）：
//
//    e(t)    = ω_ref − ω_meas
//    α_raw   = kp·e + ki·∫e dt + kd·(−ω̇_meas)    （微分先行）
//    α_ref   = clamp(α_raw, ±alpha_max)
//    ∫e      = clamp(∫e + e·dt, ±int_max)         （条件积分抗饱和）
//
//  微分先行（derivative-on-measurement）：
//    微分项作用于测量角速率变化量而非误差导数，避免因 ω_ref 阶跃引起
//    的"微分冲击"（derivative kick）。实现为 −kd·(ω_meas − ω_prev)/dt。
//
//  抗积分饱和（conditional integration）：
//    仅当输出未饱和，或误差与积分同号（会减小积分）时才更新积分器。
//    该策略保证饱和期间积分不继续增长，切出饱和后立即响应。
//
//  使用方法：
//    RateCtrl ctrl;
//    ctrl.reset();
//    // 在控制循环中：
//    float alpha[3];
//    ctrl.step(omega_ref, omega_meas, params.rate, dt, alpha);
// ============================================================
#pragma once
#include <cmath>
#include "TandemVec_CtrlParams.h"

// static 版 clamp（避免与 TandemVec_AttitudeCtrl.h 重复定义冲突）
// 若同时包含两个头文件，编译器会合并 inline 符号，保留一份。
static inline float _rc_clamp(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

// ============================================================
//  状态结构体（可通过遥测读取，便于地面站调参监视）
// ============================================================
struct RateCtrlState
{
    float integral[3];    // 积分累积量（角速率等效，rad/s）
    float omega_prev[3];  // 上一拍测量角速率（微分先行用）

    // 诊断：当前各轴是否积分冻结
    bool  int_frozen[3];
};

// ============================================================
//  RateCtrl — 三轴角速率 PID 控制器
// ============================================================
struct RateCtrl
{
    RateCtrlState state{};

    // 重置所有内部状态（模式切换、上电时调用）
    void reset()
    {
        for (int i = 0; i < 3; ++i) {
            state.integral[i]   = 0.0f;
            state.omega_prev[i] = 0.0f;
            state.int_frozen[i] = false;
        }
    }

    // ---- 主计算函数 ----
    // @param omega_ref  目标角速率 [p,q,r] rad/s（来自外环或 RC 直接映射）
    // @param omega_meas 测量角速率 [p,q,r] rad/s（IMU 输出）
    // @param g          增益参数（kp/ki/kd/alpha_max/int_max）
    // @param dt         控制周期 s（典型 0.005 s @ 200 Hz）
    // @param[out] alpha 目标角加速度 [p̈, q̈, r̈] rad/s²（已限幅）
    void step(const float omega_ref[3],
              const float omega_meas[3],
              const RateCtrlGains& g,
              float dt,
              float alpha[3])
    {
        const float dt_safe = (dt > 1e-5f) ? dt : 1e-5f; // 防止 dt=0

        for (int i = 0; i < 3; ++i)
        {
            // ---- P 项 ----
            float e = omega_ref[i] - omega_meas[i];

            // ---- I 项：条件积分（抗饱和） ----
            // 先暂时叠加积分，后面根据饱和状态决定是否保留
            float int_tentative = state.integral[i] + e * dt_safe;
            int_tentative = _rc_clamp(int_tentative, -g.int_max[i], g.int_max[i]);

            // ---- D 项：微分先行，作用于测量值变化量 ----
            float d_meas = -(omega_meas[i] - state.omega_prev[i]) / dt_safe;
            // 对微分项加软饱和，避免陀螺噪声尖峰
            const float d_limit = g.alpha_max[i] * 0.5f;
            d_meas = _rc_clamp(d_meas, -d_limit, d_limit);

            // ---- 合并输出（未限幅） ----
            float alpha_raw = g.kp[i] * e
                            + g.ki[i] * int_tentative
                            + g.kd[i] * d_meas;

            // ---- 输出限幅 ----
            float alpha_sat = _rc_clamp(alpha_raw, -g.alpha_max[i], g.alpha_max[i]);

            // ---- 抗积分饱和：判断是否需要冻结积分 ----
            // 标准判据（clamping / conditional integration）：
            // 输出已饱和，且误差与未限幅输出同号（即误差会把输出推得更深入饱和）
            // → 冻结积分。反向误差则允许积分更新，使系统能立即退出饱和。
            bool output_sat  = (alpha_raw != alpha_sat);
            bool err_pushes  = output_sat && (e * alpha_raw > 0.0f);
            if (!err_pushes) {
                state.integral[i] = int_tentative;  // 正常更新
                state.int_frozen[i] = false;
            } else {
                state.int_frozen[i] = true;          // 冻结，不更新积分
            }

            // ---- 输出 ----
            alpha[i] = alpha_sat;

            // 更新上一拍测量（仅在非首帧有效，首帧使用 0 导数是可接受的）
            state.omega_prev[i] = omega_meas[i];
        }
    }
};
