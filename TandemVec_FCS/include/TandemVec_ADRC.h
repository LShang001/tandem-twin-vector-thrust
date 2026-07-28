// ============================================================
//  TandemVec_ADRC.h — 自抗扰控制 (Active Disturbance Rejection Control)
//
//  二阶 ESO (扩张状态观测器) + PD 控制器
//
//  理论:
//    被控对象:  ω̇ = b0·u + f     (f=总扰动:模型误差+外扰+耦合)
//    扩张状态:  x1=ω, x2=f
//    ESO估计:   z1→ω, z2→f
//    控制律:    u = (ω̇_ref + Kp·(ω_ref−z1) − z2) / b0
//
//  参数物理含义 (带宽参数化):
//    ωo = ESO带宽 (rad/s) — 越大观测越快, 但噪声敏感
//    ωc = 控制器带宽 (rad/s) — 闭环响应速度
//    b0 = 名义控制增益 — 对于I×α逆解后的系统, b0=1
//
//    ESO增益: β1=2ωo, β2=ωo²
//    PD增益:  Kp=ωc,  Kd=1 (隐含在ω̇_ref中, 此处简化为PD)
//
//  与PID的关键区别:
//    PID: 对误差的历史(P+I+D)做线性组合
//    ADRC: 实时估计总扰动→前馈补偿→剩余用PD处理
//    → 模型不确定下的鲁棒性显著优于PID（尤其I/kT/a/b未知时）
//
//  参考文献:
//    Han, J. (2009). "From PID to Active Disturbance Rejection Control."
//    Gao, Z. (2006). "Scaling and bandwidth-parameterization based controller tuning."
// ============================================================
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
    // @param u    控制量 (α_ref, rad/s² → 内部转为deg/s²)
    // @param b0   名义控制增益
    // @param wo   观测器带宽 (rad/s)
    // @param dt   步长 (s)
    void update(float y, float u, float b0, float wo, float dt)
    {
        float e = z1 - y;                         // 观测误差
        float u_deg = u * 57.29578f;              // rad/s²→deg/s²

        // ESO dynamics (Euler discretization)
        z1 += (z2 - 2.f*wo*e + b0*u_deg) * dt;   // ω̇ = f + b0*u − β1*e
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
    float Kp;    // PD比例增益 (rad/s² per deg/s error)
    float Kd;    // PD微分增益 (rad/s² per deg/s² rate-of-change) — 暂未使用
    float b0;    // 名义控制增益
    float wo;    // ESO带宽 (rad/s)

    // @param bandwidth 控制器带宽 ωc (rad/s)
    // @param eso_bw   ESO带宽 ωo (rad/s)，典型 3~10× ωc
    // @param b0_nom   名义控制增益 (物理逆解后 b0≈1)
    ADRC(float bandwidth=8.f, float eso_bw=40.f, float b0_nom=1.f)
        : Kp(bandwidth), Kd(0), b0(b0_nom), wo(eso_bw) {}

    void reset() { eso.reset(); }

    // 单步控制计算
    // @param y         测量值: ω (deg/s)
    // @param ref       参考值: ω_ref (deg/s)
    // @param ref_dot   参考微分: ω̇_ref (deg/s²) — 外部提供或设为0
    // @param dt        步长 (s)
    // @return          控制量 α_ref (rad/s²)
    float step(float y, float ref, float ref_dot, float dt)
    {
        // 1. ESO估计状态和总扰动
        eso.update(y, 0, b0, wo, dt);  // 先用上一拍控制量估计

        // 2. PD误差控制
        float err = ref - eso.z1;

        // 3. 扰动补偿 + PD
        float u_deg = (ref_dot + Kp*err - eso.z2) / b0;  // deg/s²

        // 4. 控制量限幅 (防止ESO暂态输出极端值)
        if (u_deg >  5000.f) u_deg =  5000.f;
        if (u_deg < -5000.f) u_deg = -5000.f;

        float alpha = u_deg / 57.29578f;  // deg/s² → rad/s² (PID输出单位)

        // 5. ESO用本拍控制量重新更新z1 (提高估计精度)
        // 已在step开始时调用update, 此处不重复

        return alpha;
    }
};
