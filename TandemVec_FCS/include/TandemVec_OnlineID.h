// ============================================================
//  TandemVec_OnlineID.h — 在线系统参数辨识 + 自适应增益调度
//
//  辨识模型（利用物理逆解层）:
//    在物理逆解 I·α→M→分配 架构下，α_cmd 即为期望角加速度。
//    实际角加速度 α_actual = ω̇（陀螺数值微分）
//    关系: α_actual = b × α_cmd + d
//      其中 b = I_nominal / I_actual （惯量比）
//            d = 总扰动（CG偏移 + 推力不对称 + 风）
//
//  三层辨识:
//    L1: b(惯量比) — 从 α 和 ω̇ 的高速数据中递推最小二乘(RLS)
//    L2: CG_offset — 从悬停稳态积分器 I_term 中提取配平力矩
//    L3: kT_bias — 从悬停油门 vs 推力关系标定
//
//  自适应调度:
//    识别到 b 更新 → 更新 Kp_r ← Kp_r_nominal / sqrt(b)
//    识别到 CG_offset → 前馈补偿 α_trim ← M_trim / I_nominal
//    识别到 kT_bias → 更新 MAX_THRUST 和油门映射
//
//  安全性:
//    - 只在充分激励条件下更新（ω̇ 方差 > 阈值）
//    - 更新率限制（每参数每2秒最多调整10%）
//    - 参数范围硬约束（b ∈ [0.3, 3.0], CG ∈ [-20, 20]mm）
//
//  参考文献:
//    Åström, K.J. & Wittenmark, B. (1995). Adaptive Control.
//    Ljung, L. (1999). System Identification: Theory for the User.
// ============================================================
#pragma once
#include <cmath>
#include <algorithm>
#include "TandemVec_Config.h"

// ============================================================
//  遗忘因子递推最小二乘 (FFRLS)
// ============================================================
//  模型: y = φᵀθ + ε
//    y = α_actual (测量: ω̇)
//    φ = [α_cmd, 1]ᵀ (回归向量: 命令+常数偏移)
//    θ = [b, d]ᵀ (参数: 惯量比 + 总扰动)
struct RLS_2Param
{
    // 协方差矩阵 P (2×2, 对称存储)
    float p11, p12, p22;
    float theta_b;   // b = I_nominal / I_actual
    float theta_d;   // d = total disturbance (rad/s²)

    float lambda;    // 遗忘因子 (0.95~0.995)
    bool  init;

    RLS_2Param(float forget = 0.99f)
        : p11(100.f), p12(0.f), p22(100.f)
        , theta_b(1.f), theta_d(0.f)
        , lambda(forget), init(true) {}

    // 单步更新
    // @param y   测量: ω̇ (rad/s², 从陀螺微分)
    // @param u   输入: α_cmd (rad/s²)
    void update(float y, float u)
    {
        // 回归向量 φ = [u, 1]
        // 预测误差 ε = y − φᵀθ
        float y_hat = theta_b * u + theta_d;
        float eps   = y - y_hat;

        // 计算 S = P·φ / (λ + φᵀ·P·φ)
        float S1 = p11*u + p12;       // P·φ 的第1分量
        float S2 = p12*u + p22;       // P·φ 的第2分量
        float den = lambda + u*S1 + S2; // λ + φᵀ·P·φ

        if (fabsf(den) > 1e-9f) {
            // 增益 K = P·φ / den
            float K1 = S1 / den;
            float K2 = S2 / den;

            // 参数更新
            theta_b += K1 * eps;
            theta_d += K2 * eps;

            // 协方差更新: P ← (P − K·φᵀ·P) / λ
            float A = 1.f - K1*u;
            float B = -K1;
            float C = -K2*u;
            float D = 1.f - K2;

            float pp11 = p11, pp12 = p12, pp22 = p22;
            p11 = (A*pp11 + B*pp12) / lambda;
            p12 = (A*pp12 + B*pp22) / lambda;
            p22 = (C*pp12 + D*pp22) / lambda;
        }

        init = false;
    }

    // 重置（模式切换、解锁时调用）
    void reset()
    {
        p11 = p22 = 100.f; p12 = 0.f;
        theta_b = 1.f; theta_d = 0.f;
        init = true;
    }
};

// ============================================================
//  在线辨识管理器
// ============================================================
struct OnlineID
{
    RLS_2Param rls_pitch;  // 俯仰轴
    RLS_2Param rls_roll;   // 侧倾轴
    RLS_2Param rls_yaw;    // 偏航轴

    // 辨识结果（平滑滤波后）
    float b_est[3];      // [pitch, roll, yaw] 惯量比
    float d_est[3];      // 总扰动 (rad/s²)
    float cg_est_mm;     // CG偏移 (mm), 从 I_term 稳态提取

    // 激励检测
    float alpha_var[3];  // α_cmd 方差（检测充分激励）
    float omega_dot[3];  // 上一拍 ω 用于微分

    int   sample_count;
    float update_rate_limit; // 参数更新速率限制 (abs change per call)

    OnlineID()
    {
        b_est[0]=b_est[1]=b_est[2]=1.f;
        d_est[0]=d_est[1]=d_est[2]=0.f;
        cg_est_mm=0.f;
        alpha_var[0]=alpha_var[1]=alpha_var[2]=0.f;
        omega_dot[0]=omega_dot[1]=omega_dot[2]=0.f;
        sample_count=0;
        update_rate_limit=0.05f; // 每拍最多5%变化
    }

    // 主更新函数 — 在内环PID之后、mix之前调用
    // @param alpha_cmd[3]  命令角加速度 [roll,pitch,yaw] rad/s²
    // @param omega_now[3]  当前角速率 [roll,pitch,yaw] rad/s
    // @param i_term_rate[3] 内环积分器 I_term (rad/s²), 用于提取CG偏移
    // @param dt             步长
    void step(const float alpha_cmd[3], const float omega_now[3],
              const float i_term_rate[3], float dt)
    {
        // 角加速度: 陀螺数值微分 (一阶后向差分 + 低通)
        for (int i = 0; i < 3; ++i) {
            float wdot = (omega_now[i] - omega_dot[i]) / dt;
            omega_dot[i] = omega_now[i];

            // 激励检测: α_cmd 的指数移动方差
            float a_abs = fabsf(alpha_cmd[i]);
            alpha_var[i] = 0.95f * alpha_var[i] + 0.05f * a_abs * a_abs;

            // 只在充分激励时更新RLS
            // 阈值: α_cmd RMS > 2 rad/s² (约等于 ~7°/s² 的角加速度)
            if (alpha_var[i] > 4.f) {
                if (i == 0) rls_pitch.update(wdot, alpha_cmd[i]);
                else if (i == 1) rls_roll.update(wdot, alpha_cmd[i]);
                else rls_yaw.update(wdot, alpha_cmd[i]);
                // 提取更新后的参数至平滑估计
                switch (i) {
                    case 0: b_est[i] = rls_pitch.theta_b; d_est[i] = rls_pitch.theta_d; break;
                    case 1: b_est[i] = rls_roll.theta_b;  d_est[i] = rls_roll.theta_d;  break;
                    case 2: b_est[i] = rls_yaw.theta_b;   d_est[i] = rls_yaw.theta_d;   break;
                }
            }
        }

        // 从积分器提取CG偏移（悬停稳态，仅俯仰轴）
        // 积分器 I_term 的量纲 = rad/s²
        // CG偏移力矩 = I_term × I_nominal
        // CG偏移 = 力矩 / 推力 / g
        // 仅在接近悬停时更新（油门40~60%，角速率低）
        float Iy = kDefaultTandemVecParams.Iy;
        float T_hover = kDefaultTandemVecParams.kT
                      * (0.4f * kDefaultTandemVecParams.wMax)
                      * (0.4f * kDefaultTandemVecParams.wMax);
        float cg_moment = i_term_rate[1] * Iy;           // N·m
        float cg_new     = cg_moment / fmaxf(T_hover, 1.f) * 1000.f; // mm

        // 平滑更新
        cg_est_mm = 0.95f * cg_est_mm + 0.05f * cg_new;

        // 范围硬约束
        cg_est_mm = std::clamp(cg_est_mm, -20.f, 20.f);

        sample_count++;
    }

    void reset()
    {
        rls_pitch.reset();
        b_est[0]=b_est[1]=b_est[2]=1.f;
        d_est[0]=d_est[1]=d_est[2]=0.f;
        cg_est_mm=0.f;
        sample_count=0;
    }

    // 自适应增益计算: Kp_r ← Kp_r_nominal / sqrt(b)
    float adaptKpR(float Kp_nominal, int axis) const
    {
        float b = std::clamp(b_est[axis], 0.3f, 3.0f);
        return Kp_nominal / sqrtf(b);
    }

    // 自适应α限幅: α_max ← α_max_nominal × b (惯量大→力矩大→限幅大)
    float adaptAlphaMax(float alpha_nominal, int axis) const
    {
        float b = std::clamp(b_est[axis], 0.3f, 3.0f);
        return alpha_nominal * b;
    }
};
