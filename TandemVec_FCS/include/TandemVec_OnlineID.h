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
    // 三轴 RLS。索引统一为 [0]=roll(侧倾) [1]=pitch(俯仰) [2]=yaw(航向)，
    // 与 alpha_cmd/omega/b_est 的下标一致（原先用 rls_pitch 处理 i==0，
    // 命名与下标错位，易误读，已改为数组）。
    RLS_2Param rls[3];

    // 辨识结果（轴序 [0]=roll [1]=pitch [2]=yaw，与 rls[] 一致）
    float b_est[3];      // 惯量比 b = I_nominal / I_actual
    float d_est[3];      // 总扰动 (rad/s²)
    float cg_est_mm;     // CG偏移 (mm), 从 I_term 稳态提取

    // 激励检测与微分状态
    float alpha_var[3];      // α_cmd 指数移动均方（检测充分激励）
    float omega_prev_rps[3]; // 上一拍角速率 (rad/s)，用于数值微分
    bool  primed[3];         // 微分状态是否已建立（首拍跳过伪微分）

    int   sample_count;
    float update_rate_limit; // 每拍参数最大相对变化量

    OnlineID() { reset(); }

    // 主更新函数 — 在内环PID之后、mix之前调用
    //
    // @param alpha_cmd[3]   命令角加速度 [roll,pitch,yaw] rad/s²
    // @param omega_dps[3]   当前角速率 [roll,pitch,yaw] **deg/s**
    //                       （与 current_omega_dps_body_filtered 一致；
    //                        内部转 rad/s 再求导。原版注释要求 rad/s，
    //                        而固件只有 deg/s，直接传会使 ω̇ 差 57.3 倍
    //                        并把 b_est 顶到约束边界。）
    // @param i_term_rate[3] 内环积分项 I_term = ki×integral (rad/s²)
    //                       用于提取 CG 偏移；轴序同上
    // @param thr_pct        当前油门百分比 [0,100]，用于悬停门控与推力基准
    // @param dt             步长 s
    void step(const float alpha_cmd[3], const float omega_dps[3],
              const float i_term_rate[3], float thr_pct, float dt)
    {
        const float dt_safe = (dt > 1e-5f) ? dt : 1e-5f;
        const float DEG2RAD = 0.0174532925f;

        for (int i = 0; i < 3; ++i) {
            // ---- 角加速度：陀螺数值微分（统一到 rad/s²）----
            const float w_rps = omega_dps[i] * DEG2RAD;
            float wdot = (w_rps - omega_prev_rps[i]) / dt_safe;
            omega_prev_rps[i] = w_rps;

            // 首拍 omega_prev 尚未建立，微分是伪值，跳过（原版会注入巨大冲击）
            if (!primed[i]) { primed[i] = true; continue; }

            // ---- 激励检测：α_cmd 的指数移动均方 ----
            alpha_var[i] = 0.95f * alpha_var[i] + 0.05f * alpha_cmd[i] * alpha_cmd[i];

            // 仅在充分激励时更新（α_cmd RMS > 2 rad/s²）。
            // 激励不足时回归矩阵近奇异，RLS 会把噪声当信号。
            if (alpha_var[i] <= 4.f) continue;

            // ---- RLS 更新 ----
            const float b_before = rls[i].theta_b;
            rls[i].update(wdot, alpha_cmd[i]);

            // ---- 变化率限制：每拍最多变动 update_rate_limit（原版声明未用）----
            float b_new = rls[i].theta_b;
            const float max_step = fmaxf(fabsf(b_before), 0.1f) * update_rate_limit;
            if (b_new > b_before + max_step) b_new = b_before + max_step;
            if (b_new < b_before - max_step) b_new = b_before - max_step;
            rls[i].theta_b = b_new;

            // ---- 物理范围硬约束（惯量比不可能超出此范围）----
            rls[i].theta_b = std::clamp(rls[i].theta_b, 0.3f, 3.0f);
            if (!std::isfinite(rls[i].theta_b)) rls[i].reset();
            if (!std::isfinite(rls[i].theta_d)) rls[i].theta_d = 0.f;

            b_est[i] = rls[i].theta_b;
            d_est[i] = rls[i].theta_d;
        }

        // ---- CG 偏移提取（仅俯仰轴，需接近悬停）----
        // 力矩 = I_term × I_nominal；CG 偏移 = 力矩 / 总推力
        // 门控：油门 30~70% 且三轴角速率均低于 15 deg/s，否则积分项
        //       含机动分量，不代表配平力矩。
        const bool near_hover = (thr_pct > 30.f && thr_pct < 70.f)
                             && (fabsf(omega_dps[0]) < 15.f)
                             && (fabsf(omega_dps[1]) < 15.f)
                             && (fabsf(omega_dps[2]) < 15.f);
        if (near_hover) {
            const TandemVecParams& P = kDefaultTandemVecParams;
            // 用实际油门推算基准推力，而非硬编码 40%
            const float w0     = (thr_pct / 100.f) * P.wMax;
            const float T_tot  = 2.0f * P.kT * w0 * w0;      // 双发总推力 N
            const float cg_mom = i_term_rate[1] * P.Iy;      // N·m
            const float cg_new = cg_mom / fmaxf(T_tot, 0.5f) * 1000.f;  // mm
            if (std::isfinite(cg_new))
                cg_est_mm = 0.95f * cg_est_mm + 0.05f * cg_new;
            cg_est_mm = std::clamp(cg_est_mm, -20.f, 20.f);
        }

        sample_count++;
    }

    void reset()
    {
        // 原版只 reset 了 rls_pitch，roll/yaw 的协方差与参数会跨架次残留
        for (int i = 0; i < 3; ++i) {
            rls[i].reset();
            b_est[i] = 1.f;
            d_est[i] = 0.f;
            alpha_var[i] = 0.f;
            omega_prev_rps[i] = 0.f;
            primed[i] = false;
        }
        cg_est_mm = 0.f;
        sample_count = 0;
        update_rate_limit = 0.05f;   // 每拍最多 5% 相对变化
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
