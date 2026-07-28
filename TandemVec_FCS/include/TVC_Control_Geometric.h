#ifndef TVC_CONTROL_GEOMETRIC_H
#define TVC_CONTROL_GEOMETRIC_H

#define _USE_MATH_DEFINES
#include <cmath>

/**
 * 基于几何建模的TVC控制量计算函数（使用原始公式）
 * 输入两个喷管目标姿态角（单位：度），计算对应的推杆长度 r1、r2（单位：mm），
 * 再根据标定的比例关系将推杆长度映射为控制量 s1_control、s2_control（范围：-50 ~ 150）。
 *
 * @param s1_angle S1通道喷管目标摆角（单位：度），对应姿态角 δ₁
 * @param s2_angle S2通道喷管目标摆角（单位：度），对应姿态角 δ₂
 * @param s1_control 输出参数：S1控制量 
 * @param s2_control 输出参数：S2控制量
 */
inline void calculateTVCControlGeometric(float s1_angle, float s2_angle, float &s1_control, float &s2_control)
{
    // === 系统几何参数定义（单位：mm），参考图15 ===

    const float l1 = 164.4f;       // L₁S₂ = L₂S₂ = l₁：固定推杆上端至S₂的长度
    const float l2 = 113.25f;      // S₂A₁ = S₂A₂ = l₂：推力框下端至S₂的长度
    const float s1_param = 130.0f; // S₀L₃ = S₀L₄ = s₁：壳体中心至推杆基座的横向距离
    const float s2_param = 43.0f;  // L₃L₁ = L₄L₂ = s₂：上端连接点之间的水平间距
    const float s3 = 40.0f;        // QA₁ = QA₂ = s₃：喷管尾端至球铰中心Q的距离（喷管臂长）
    const float d1 = 143.0f;       // S₀S₂ = d₁：壳体中心到S₂平面的轴向高度
    const float d2 = 106.0f;       // S₂Q = d₂：喷管基底到球铰Q的垂向距离

    // === 控制量映射参数 ===
    const float zero_length = 224.8f;                  // r₁, r₂ = 224.8mm 对应 δ₁ = δ₂ = 0，推杆零位长度
    const float control_zero = 50.0f;                  // 控制量为 50 时推杆处于零位
    const float length_per_control = 28.274f / 100.0f; // 每100控制量对应推杆变化 28.274 mm

    // === 步骤1：将目标角度从度转换为弧度（供三角函数使用）===
    float delta1 = s1_angle * M_PI / 180.0f; // δ₁
    float delta2 = s2_angle * M_PI / 180.0f; // δ₂

    // === 步骤2：根据原始几何模型公式计算 r1 推杆长度（对应通道 S1）===
    // r₁² = l₁² + l₂² - 2 × [s₁(s₃cosδ₂ - d₂sinδ₂) + (s₂ - d₁)(d₂cosδ₁cosδ₂ + s₃cosδ₁sinδ₂)]
    float r1_squared =
        l1 * l1 + l2 * l2 - 2.0f * (s1_param * (s3 * cos(delta2) - d2 * sin(delta2)) + (s2_param - d1) * (d2 * cos(delta1) * cos(delta2) + s3 * cos(delta1) * sin(delta2)));

    // 若结果非法（负数），退回零位
    float r1 = r1_squared > 0.0f ? sqrtf(r1_squared) : zero_length;

    // === 步骤3：根据原始几何模型公式计算 r2 推杆长度（对应通道 S2）===
    // r₂² = l₁² + l₂² - 2 × [s₁(s₃cosδ₁ + d₂cosδ₂sinδ₁) + (s₂ - d₁)(d₂cosδ₁cosδ₂ - s₃sinδ₁)]
    float r2_squared =
        l1 * l1 + l2 * l2 - 2.0f * (s1_param * (s3 * cos(delta1) + d2 * cos(delta2) * sin(delta1)) + (s2_param - d1) * (d2 * cos(delta1) * cos(delta2) - s3 * sin(delta1)));

    float r2 = r2_squared > 0.0f ? sqrtf(r2_squared) : zero_length;

    // === 步骤4：将推杆长度映射为控制量（线性关系）===
    // control = 50 + (r - 224.8) / (28.274 / 100)
    s1_control = control_zero + (r1 - zero_length) / length_per_control;
    s2_control = control_zero + (r2 - zero_length) / length_per_control;
}

#endif // TVC_CONTROL_GEOMETRIC_H
