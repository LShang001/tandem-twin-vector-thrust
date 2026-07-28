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
    float u_prev; // 上一拍实际施加的控制量 (rad/s²)，供 ESO 正确分离控制与扰动

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
        : Kp(bandwidth), Kd(0), b0(b0_nom), wo(eso_bw), u_prev(0.f) {}

    void reset() { eso.reset(); u_prev = 0.f; }

    // 单步控制计算
    // @param y         测量值: ω (deg/s)
    // @param ref       参考值: ω_ref (deg/s)
    // @param ref_dot   参考微分: ω̇_ref (deg/s²) — 外部提供或设为0
    // @param dt        步长 (s)
    // @return          控制量 α_ref (rad/s²)
    float step(float y, float ref, float ref_dot, float dt)
    {
        // 1. ESO估计状态和总扰动
        //    必须传入上一拍实际施加的控制量：否则 ESO 无法区分
        //    "自己施加的控制" 与 "外部扰动"，会把正常控制响应全部
        //    归入 z2，等效于抵消掉控制作用（闭环有效增益被吃掉）。
        eso.update(y, u_prev, b0, wo, dt);

        // 2. PD误差控制
        float err = ref - eso.z1;

        // 3. 扰动补偿 + PD
        float u_deg = (ref_dot + Kp*err - eso.z2) / b0;  // deg/s²

        // 4. 控制量限幅 (防止ESO暂态输出极端值)
        if (u_deg >  5000.f) u_deg =  5000.f;
        if (u_deg < -5000.f) u_deg = -5000.f;

        float alpha = u_deg / 57.29578f;  // deg/s² → rad/s² (PID输出单位)

        // 5. 缓存本拍控制量，供下一拍 ESO 使用
        u_prev = alpha;

        return alpha;
    }
};
