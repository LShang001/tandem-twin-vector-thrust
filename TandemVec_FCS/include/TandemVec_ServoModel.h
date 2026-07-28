// ============================================================
//  TandemVec_ServoModel.h — 舵机动态模型（速率限制+一阶滞后+死区）
//
//  典型9g舵机参数（实测标定前默认值）：
//    max_rate:  600 °/s  (0.10s/60°)
//    bandwidth: 20 Hz    (一阶滞后 τ≈0.008s)
//    deadband:  0.15°    (≈1μs PWM死区)
//
//  舵机动态是闭环回路中的额外相位滞后源。
//  在200Hz控制率下，τ=0.008s → 在20Hz处产生约45°相位滞后。
//  总回路相位裕度必须扣除这部分。
//
//  工业标准：舵机带宽应 ≥ 5× 控制回路穿越频率。
//  当前内环带宽≈2.7Hz → 要求舵机带宽≥13.5Hz。20Hz舵机满足。
// ============================================================
#pragma once
#include <cmath>
#include <algorithm>

struct ServoState
{
    float angle;       // 当前实际角度 (rad)
    float rate;        // 当前角速率 (rad/s)
};

// 舵机模型参数
struct ServoParams
{
    float max_rate;    // 最大角速率 (rad/s) — 默认600°/s = 10.47 rad/s
    float tau;         // 一阶滞后的时间常数 (s) — 默认0.008s (20Hz带宽)
    float deadband;    // 死区 (rad) — 默认0.15° = 0.0026 rad

    ServoParams()
        : max_rate(10.47f), tau(0.008f), deadband(0.0026f) {}
};

// 单步舵机动态仿真
// @param target    目标角度 (rad)
// @param state     当前状态 (in/out)
// @param p         舵机参数
// @param dt        仿真步长 (s)，典型0.005
// @return          本步实际达到的角度 (rad)
inline float stepServo(float target, ServoState &state,
                       const ServoParams &p, float dt)
{
    // 死区：小指令不响应
    float error = target - state.angle;
    if (fabsf(error) < p.deadband)
        return state.angle;  // 死区内不动作

    // 速率限制：本步最大转角
    float max_step = p.max_rate * dt;
    float step = std::clamp(error, -max_step, max_step);

    // 一阶滞后：指数趋近
    float alpha = dt / (p.tau + dt);          // 离散化一阶滤波器系数
    state.angle += step * alpha;

    // 更新角速率估计
    state.rate = step / dt;

    return state.angle;
}
