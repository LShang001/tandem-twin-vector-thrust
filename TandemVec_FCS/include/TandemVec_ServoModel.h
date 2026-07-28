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
    float max_rate;    // 最大角速率 (rad/s) — 默认600°/s = 10.47 rad/s（舵机端）
    float tau;         // 一阶滞后的时间常数 (s) — 默认0.008s (20Hz带宽)
    float deadband;    // 死区 (rad) — 默认0.15° = 0.0026 rad（摆座端）
    float gear_ratio;  // 齿轮传动比 = 从动齿数/主动齿数 = 40/30 = 1.333
                       //   servo_angle = gimbal_angle × gear_ratio
                       //   有效舵机速率↓×(1/gear_ratio), 死区↑×gear_ratio

    ServoParams()
        : max_rate(10.47f), tau(0.008f), deadband(0.0026f), gear_ratio(1.333f) {}
};

// 单步舵机动态仿真（工作在摆座端角度空间）
// @param target_gimbal  摆座端目标角度 (rad)
// @param state          舵机当前状态 (in/out, 存储摆座端角度)
// @param p              舵机参数 (含齿轮比)
// @param dt             仿真步长 (s)，典型0.005
// @return               摆座端实际达到的角度 (rad)
//
// 处理流程: gimbal_target → ×gear_ratio → servo_target
//           → 伺服动态(速率/死区/滞后) → servo_actual
//           → ÷gear_ratio → gimbal_actual
inline float stepServo(float target_gimbal, ServoState &state,
                       const ServoParams &p, float dt)
{
    // 1. 转换到舵机端
    float servo_target = target_gimbal * p.gear_ratio;
    float servo_angle  = state.angle * p.gear_ratio;

    // 2. 死区（摆座端判断，齿轮放大后更明显）
    float error_gimbal = target_gimbal - state.angle;
    if (fabsf(error_gimbal) < p.deadband)
        return state.angle;

    // 3. 速率限制（舵机端物理限制）
    float max_step_servo = p.max_rate * dt;               // 舵机端本拍最大转角
    float error_servo    = servo_target - servo_angle;
    float step_servo     = std::clamp(error_servo, -max_step_servo, max_step_servo);

    // 4. 一阶滞后（舵机端）
    float alpha = dt / (p.tau + dt);
    servo_angle += step_servo * alpha;

    // 5. 转换回摆座端
    state.angle = servo_angle / p.gear_ratio;
    state.rate  = (step_servo / dt) / p.gear_ratio;       // 摆座端等效角速率

    return state.angle;
}

// 将摆座端角度(rad)映射为舵机PWM百分比
// @param gimbal_rad   摆座目标角度 (rad)
// @param gimbal_max   摆座最大角度 (rad) = dMax = 25°
// @param gear_ratio   齿轮比 = 40/30 = 1.333
// @param servo_range  舵机机械行程 (rad) — 默认±45° = ±0.785rad
// @return             PWM百分比 (0~100，中位50)
inline float gimbalToServoPWM(float gimbal_rad, float gimbal_max,
                              float gear_ratio, float servo_range = 0.7854f)
{
    // 摆座角度 → 舵机角度
    float servo_rad = gimbal_rad * gear_ratio;
    // 舵机角度 → PWM (线性映射: -servo_range→0%, 0→50%, +servo_range→100%)
    float pwm = 50.f + (servo_rad / servo_range) * 50.f;
    if (pwm < 0.f) pwm = 0.f;
    if (pwm > 100.f) pwm = 100.f;
    return pwm;
}
