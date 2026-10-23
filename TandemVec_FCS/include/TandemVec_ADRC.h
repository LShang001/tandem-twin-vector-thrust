/**
 * @file    TandemVec_ADRC.h
 * @module  固件/控制（自抗扰控制候选模块，尚未接入生产控制链）
 * @brief   二阶 ESO（扩张状态观测器）+ PD 的 ADRC 速率环实现（角度域）
 * @details 输入：ω [deg/s]、ω_ref [deg/s]、ω̇_ref [deg/s²]、dt；
 *          处理：ESO 估计 z1→ω、z2→总扰动，控制律
 *          u = (ω̇_ref + Kp(ω_ref−z1) − z2)/b0；输出：α_ref [deg/s²]。
 *          ⚠️ 现行代码未接线：flight_control.cpp 不含本文件，速率环仍用 PI。
 *
 * 理论:
 *   被控对象: ω̇ = b0·u + f（f=总扰动：模型误差+外扰+耦合）
 *   扩张状态: x1=ω、x2=f
 *   ESO 估计: z1→ω、z2→f
 *   控制律: u = (ω̇_ref + Kp·(ω_ref−z1) − z2) / b0
 *
 * 参数物理含义（带宽参数化）:
 *   ωo = ESO 带宽 [rad/s]，越大观测越快但噪声敏感
 *   ωc = 控制器带宽 [rad/s]，决定闭环响应速度
 *   b0 = 名义控制增益，对 I×α 逆解后的系统 b0 ≈ 1
 *   ESO 增益 β1=2ωo、β2=ωo²；PD 增益 Kp=ωc（Kd 暂未使用）
 *
 * @theory  二阶 ESO + 扰动对消的 PD 控制（Gao 带宽参数化）
 * @ref     THY-003 §1.2（转速环被控对象）；见 TandemVec-Paper/ch/16-adrc-onlineid.tex
 *          (eq:adrc_plant、eq:eso2_continuous、eq:eso2_discrete、eq:adrc_law)
 * @frame   N/A（单轴标量；轴由调用方决定）
 * @units   输入 ω/ω_ref [deg/s]、ref_dot [deg/s²]、输出 α [deg/s²]；
 *          ωo/ωc [rad/s]、dt [s]
 * @note    ① 控制量 u 必须回传给 ESO 用**上一拍实际值**，否则 ESO 会把控制
 *          效果当扰动抵消（闭环增益被吃掉）。
 *        ② ωo 受执行器带宽约束：τm=0.28 s → ω_act≈3.6 rad/s，默认 ωo=8；
 *          仿真扫描 ωo≤12 稳定、ωo=50 全发散（仅该模型的口径，不是普适定律）。
 *        ③ 本模块没有实机验证，不得作为现有速率的替代证据。
 * @test    test_host/test_advanced_theory.cpp（L2b 偏差工况扫描）
 */
#pragma once
#include <cmath>

// ============================================================
//  扩张状态观测器 (ESO)
// ============================================================
struct ESO2
{
    float z1;   // 状态估计: ω (角速率, deg/s)
    float z2;   // 扩张状态估计: 总扰动 f (deg/s²)

    ESO2() : z1(0), z2(0) {}

    // 单步观测器更新
    // @param y    测量值 (ω, deg/s)
    // @param u_dps2 控制量 (α_ref, deg/s²)
    // @param b0   名义控制增益
    // @param wo   观测器带宽 (rad/s)
    // @param dt   步长 (s)
    void update(float y, float u_dps2, float b0, float wo, float dt)
    {
        float e = z1 - y;                         // 观测误差

        // ESO dynamics (Euler discretization)
        z1 += (z2 - 2.f*wo*e + b0*u_dps2) * dt;  // ω̇ = f + b0*u − β1*e
        z2 += (-wo*wo*e) * dt;                    // ḟ = −β2*e
    }

    void reset() { z1 = 0; z2 = 0; }
};

// ============================================================
//  ADRC 控制器 (ESO + PD)
// ============================================================
struct ADRC
{
    ESO2 eso;
    float Kp;    // PD比例增益 (deg/s² per deg/s error = 1/s)
    float Kd;    // PD微分增益（暂未使用）
    float b0;    // 名义控制增益
    float wo;    // ESO带宽 (rad/s)
    float u_prev_dps2; // 上一拍实际施加的控制量 (deg/s²)，供 ESO 正确分离控制与扰动

    // @param bandwidth 控制器带宽 ωc (rad/s)
    // @param eso_bw   ESO带宽 ωo (rad/s)
    //
    //   ⚠️ ωo 不可按教科书的 "3~10× ωc" 选取，必须受执行器带宽约束：
    //      本机电机 τm=0.28s → 执行器带宽 ≈ 1/τm ≈ 3.6 rad/s。
    //      ωo 显著高于执行器带宽时，ESO 会把执行器滞后误判为扰动
    //      并过度补偿，闭环发散。
    //      仿真扫描（test_advanced_theory.cpp L2b，偏差工况）结论：
    //        ωo ≤ 12 rad/s（≈3× 执行器带宽）稳定；ωo=20 仅在 ωc=4 时稳定；
    //        ωo=50 在所有 ωc 下发散。
    //      → 默认取 ωo=8（≈2× 执行器带宽），对 ωc 失调最鲁棒。
    //
    // @param b0_nom   名义控制增益 (物理逆解后 b0≈1)
    ADRC(float bandwidth=6.f, float eso_bw=8.f, float b0_nom=1.f)
        : Kp(bandwidth), Kd(0), b0(b0_nom), wo(eso_bw), u_prev_dps2(0.f) {}

    void reset() { eso.reset(); u_prev_dps2 = 0.f; }

    // 单步控制计算
    // @param y         测量值: ω (deg/s)
    // @param ref       参考值: ω_ref (deg/s)
    // @param ref_dot   参考微分: ω̇_ref (deg/s²) — 外部提供或设为0
    // @param dt        步长 (s)
    // @return          控制量 α_ref (deg/s²)；物理层边界再转 rad/s²
    float step(float y, float ref, float ref_dot, float dt)
    {
        // 1. ESO估计状态和总扰动
        //    必须传入上一拍实际施加的控制量：否则 ESO 无法区分
        //    "自己施加的控制" 与 "外部扰动"，会把正常控制响应全部
        //    归入 z2，等效于抵消掉控制作用（闭环有效增益被吃掉）。
        eso.update(y, u_prev_dps2, b0, wo, dt);

        // 2. PD误差控制
        float err = ref - eso.z1;

        // 3. 扰动补偿 + PD
        float u_deg = (ref_dot + Kp*err - eso.z2) / b0;  // deg/s²

        // 4. 控制量限幅 (防止ESO暂态输出极端值)
        if (u_deg >  5000.f) u_deg =  5000.f;
        if (u_deg < -5000.f) u_deg = -5000.f;

        // 5. 缓存本拍控制量，供下一拍 ESO 使用
        u_prev_dps2 = u_deg;

        return u_deg;
    }
};
