/**
 * @file    TVC_Control_Geometric.h
 * @module  固件/控制（历史 TVC 推杆几何模型，已弃用）
 * @brief   由两个喷管目标摆角反解推杆长度 r1/r2，再线性映射为舵机控制量
 * @details 输入：S1/S2 目标摆角 [deg]；处理：用 8 个几何常数（mm）按余弦定理
 *          展开 r² 表达式求 r1/r2，再按 28.274 mm/100 控制量线性映射；
 *          输出：两路控制量（0–100，中位 50）。
 *          ⚠️ 本模型已弃用：现行链路由 TandemVec_ControlAllocation.h 分配摆角、
 *          flight_control.cpp 层3 换算 PWM；保留仅为几何标定对照。
 * @theory  平面连杆余弦定理（几何推导，非动力学）
 * @ref     CTL-001 §2（本构型不用该模型；分配链见 THY-003 §3）
 * @frame   N/A（喷管/推杆平面几何，坐标系已在常数中固化）
 * @units   角度 deg（内部转 rad）；长度 mm；控制量 0–100；与 PWM 的对应见 ServoConfig
 * @note    ★ r² 表达式为负时退回零位长度（几何包络外），这是**静默兜底**：
 *          标定时若发现某角度恒返回 50，说明该角度已超出连杆可达范围。
 */
#ifndef TVC_CONTROL_GEOMETRIC_H
#define TVC_CONTROL_GEOMETRIC_H

#define _USE_MATH_DEFINES
#include <cmath>

/**
 * @brief 由喷管目标摆角反解推杆长度并映射为两路控制量（历史几何模型）
 * @param[in]  s1_angle   S1 通道喷管目标摆角 [deg]，对应 δ₁
 * @param[in]  s2_angle   S2 通道喷管目标摆角 [deg]，对应 δ₂
 * @param[out] s1_control S1 控制量（0–100，中位 50）
 * @param[out] s2_control S2 控制量（0–100，中位 50）
 * @formula r₁² = l₁² + l₂² − 2·[s₁(s₃cosδ₂ − d₂sinδ₂)
 *            + (s₂ − d₁)(d₂cosδ₁cosδ₂ + s₃cosδ₁sinδ₂)]（r₂ 同理换 δ₁）
 *          控制量 = 50 + (r − 224.8) / (28.274/100)
 * @ref  CTL-001 §2（本项目不用该模型；几何式为余弦定理展开，无文献式号）
 * @pre  摆角在连杆可达包络内；r² < 0 时退回零位长度 224.8 mm
 * @note 原名与签名保留，勿改动（历史标定脚本仍按此签名复算）
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
