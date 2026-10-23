/**
 * @file    TandemVec_Propulsion.h
 * @module  固件/控制（推进正向映射、差速分配、控制效能矩阵）
 * @brief   推进状态 → 六维力/力矩；差速指令 → 双电机转速；B_true 解析/数值
 * @details 输入：PropulsionState（wf/wt [rad/s]、δf/δt [rad]）、机型参数、w0；
 *          处理：T=kT·ω²、Q=kQ·ω²，按摆角旋转后叠加 r×F + Q；
 *          输出：模型系六维力/力矩、双电机目标转速、3×3 效能矩阵。
 *
 * 论文引用约定：下文 "TandemVec-Paper ch/NN-*.tex" 指论文工程对应章，
 * 2026-08-11 由 docs/03-理论推导/THY-004 迁出（原 §3→ch03、§5→ch05）。
 *
 * 本文件实现三件事：
 *
 * 1. computeWrench()
 *    推进状态 (wf, wt, δ_f, δ_t) → 机体系六维力/力矩
 *    直接对应 propulsion.mjs 的 dyn.Fx/My/Mz… 计算逻辑，
 *    符号约定与 TandemVec-Paper ch/03-mapping.tex 六维映射式（eq:six_component）完全一致。
 *
 * 2. allocateDifferential()
 *    差速指令 Δω → (ωf_target, ωt_target)
 *    保持 ωf² + ωt² = 2ω₀²（平方和不变），含 wMax 钳位与饱和标记。
 *
 * 3. computeEffectMatrix()
 *    在当前工作点计算 ∂M/∂[Δω, δ_t, δ_f] 的真实 Jacobian B_true。
 *    与 TandemVec-Paper ch/05-allocation.tex 线性化效能矩阵（eq:effectiveness_matrix）的差异：
 *      - B_true 依赖实际 wf/wt/δ_f/δ_t，在大摆角/非对称转速时更准；
 *      - B_full 仅在 δ_f=δ_t=0、wf=wt=w₀ 处有效；
 *      - B_true 退化到 B_full 的条件：δ_f=δ_t=0 且 wf=wt=w₀。
 *
 * @theory  T = kT·ω²、Q = kQ·ω²（+Jp·ω̇ 瞬态项）；差速开方分配；一阶 Jacobian
 * @ref     THY-002 §2（kT/kQ）、§3（反扭角动量守恒）、§6（集总模型）；
 *          THY-003 §2（多矢量推进器映射）、§3（分配）；
 *          见 TandemVec-Paper/ch/03-mapping.tex (eq:six_component)、
 *          ch/05-allocation.tex (eq:diff_def、eq:differential_roll、eq:Btrue_allocation)
 * @frame   模型系（与仿真/LaTeX 一致）：x'=推力轴朝机头、y'=右、z'=上摆轴。
 *          本文件沿用的"x_b/y_b/z_b"即该模型系（x_b 为推力轴）。
 *          ⚠️ 固件存档系把推力轴记作 z_b，两套轴名不同，跨文档不得混读。
 * @units   ω [rad/s]、δ [rad]、T [N]、Q [N·m]、M [N·m]、kT [N·s²]、kQ [N·m·s²]、
 *          a/b [m]；Δω 无量纲（|Δω| ≤ dwMax = 0.7）
 * @redline 差速公式 ωf=ω0√(1+Δω)（行为红线，公式本身不得改）
 * @test    test_host/test_tandemvec_allocation.cpp（T11 解析↔数值等价）
 */
#pragma once
#include <cmath>
#include "TandemVec_Config.h"
#include "AirframeModel.h"   // ★2026-08-10 机型数据驱动执行器模型（通用层）

// ============================================================
//  数据结构
// ============================================================

// 推进器当前状态（执行器实际值，非指令值）
struct PropulsionState
{
    float wf;       // rad/s   前电机当前转速
    float wt;       // rad/s   尾电机当前转速
    float delta_f;  // rad     上摆角（★控制语义=滚转/侧倾，2026-08-07 轴置换；物理公式为巡航读法 Mz=a·Tf·sδf）
    float delta_t;  // rad     下摆角（绕 y_b，俯仰）
};

// 机体系六维力/力矩
struct SixDOFWrench
{
    float Fx, Fy, Fz; // N    机体系力（NED：x前/y右/z下）
    float Mx, My, Mz; // N·m  机体系力矩（对应 roll/pitch/yaw）
};

// 差速分配结果
struct DiffAllocResult
{
    float wf_target;    // rad/s   前电机目标转速
    float wt_target;    // rad/s   尾电机目标转速
    bool  wf_clamped;   //         前电机触达 wMax
    bool  wt_clamped;   //         尾电机触达 wMax
};

// 控制效能矩阵 ∂M/∂[Δω, δ_t, δ_f]，行主序
// B.M[行][列]，行→{Mx,My,Mz}，列→{Δω,δ_t,δ_f}
struct EffectMatrix
{
    float M[3][3];
};

// ============================================================
//  1. 正向映射：推进状态 → 六维力/力矩
// ============================================================
// 稳态简化（ω̇ = 0 → 无 Jp·ω̇ 项）。
// 需要瞬态精度时，调用方须在外部将 Qf/Qt 加上 Jp·dω/dt 后传入；
// 当前版本与 propulsion.mjs 稳态行为一致。
//
// 公式来源（TandemVec-Paper ch/03-mapping.tex eq:six_component，模型系）：
//   公式 Fx = Tf·cδf + Tt·cδt（前向推力）
//   公式 Fy = Tf·sδf（上摆侧向分量）
//   公式 Fz = −Tt·sδt（下摆法向分量）
//   公式 Mx = −Qf·cδf + Qt·cδt（反扭矩差；前 CW 负、后 CCW 正）
//   公式 My = −b·Tt·sδt − Qf·sδf（下摆主控 + 上摆反扭耦合）
//   公式 Mz = a·Tf·sδf − Qt·sδt（上摆主控 + 下摆反扭耦合）
// 符号：cδ=cos δ、sδ=sin δ；轴名换算见文件头 @frame。
inline SixDOFWrench computeWrench(const PropulsionState& s, const TandemVecParams& p)
{
    const float Tf = p.kT * s.wf * s.wf;
    const float Tt = p.kT * s.wt * s.wt;
    const float Qf = p.kQ * s.wf * s.wf;
    const float Qt = p.kQ * s.wt * s.wt;
    const float cf = cosf(s.delta_f), sf = sinf(s.delta_f);
    const float ct = cosf(s.delta_t), st = sinf(s.delta_t);

    return SixDOFWrench{
        /* Fx 前向 */ Tf * cf + Tt * ct,
        /* Fy 侧向 */ Tf * sf,
        /* Fz 法向 */ -Tt * st,
        /* Mx 反扭差 */ -Qf * cf + Qt * ct,
        /* My 俯仰 */ -p.b * Tt * st - Qf * sf,
        /* Mz 偏航 */  p.a * Tf * sf - Qt * st,
    };
}

/**
 * @brief 差速开方分配：差速指令 Δω → 前/尾电机目标转速
 * @param[in] w0 基准转速 [rad/s]（= 油门百分比/100 × wMax）
 * @param[in] dw 差速指令 Δω（无量纲，调用方已限幅到 ±dwMax）
 * @param[in] p  机型参数（用 wMax 做单侧钳位）
 * @return 目标转速 [rad/s] + 两侧是否触达 wMax 的饱和标记
 * @formula ωf_target = w0·√(max(0, 1+Δω))；ωt_target = w0·√(max(0, 1−Δω))
 * @ref THY-002 §2；见 TandemVec-Paper/ch/05-allocation.tex (eq:diff_def)、
 *      CTL-001 §7（平方和不变与钳位后的偏差）
 * @pre  w0 ≥ 0；|Δω| ≤ dwMax（否则一侧根号内可能为负，max(0,·) 兜底）
 * @post 未钳位时 ωf²+ωt² = 2w0²（总推力一阶不变）；钳位后该恒等式被破坏，
 *       总推力低于预期，调用方应通过遥测提示操纵手。
 * @note  ★ 行为红线公式：不得改为线性差速或改变根号形式（会改变整机偏航响应）。
 *        钳位只作用在目标值上，实际转速仍由电机 τm 一阶滞后跟随。
 */
inline DiffAllocResult allocateDifferential(float w0, float dw, const TandemVecParams& p)
{
    DiffAllocResult r;
    r.wf_target = w0 * sqrtf(fmaxf(0.0f, 1.0f + dw));
    r.wt_target = w0 * sqrtf(fmaxf(0.0f, 1.0f - dw));
    r.wf_clamped = (r.wf_target > p.wMax);
    r.wt_clamped = (r.wt_target > p.wMax);
    if (r.wf_clamped) r.wf_target = p.wMax;
    if (r.wt_clamped) r.wt_target = p.wMax;
    return r;
}

// ============================================================
//  3. 真实控制效能矩阵 B_true(u_k) = ∂M/∂[Δω, δ_t, δ_f]
// ============================================================
// 在当前工作点 (s, w0) 处求偏导，适用于需精确效能的控制律（固件 BTRUE 分配）。
//
// 推导（稳态 ω̇=0，链式法则）：
//   公式 ∂wf²/∂Δω = w0²，∂wt²/∂Δω = −w0²
//
//   row 0  ∂Mx/∂[Δω,δ_t,δ_f]:
//     ∂Mx/∂Δω  = -kQ·w0²·(cf + ct)      // 前/尾反扭均贡献
//     ∂Mx/∂δ_t  = -Qt·st                 // 尾反扭矩角度分量
//     ∂Mx/∂δ_f  =  Qf·sf                 // 前反扭矩角度分量
//
//   row 1  ∂My/∂[Δω,δ_t,δ_f]:
//     ∂My/∂Δω  =  b·kT·w0²·st - kQ·w0²·sf  // 尾推力摆角+前反扭摆角
//     ∂My/∂δ_t  = -b·Tt·ct                  // 俯仰主控
//     ∂My/∂δ_f  = -Qf·cf                    // 上摆反扭耦合到俯仰
//
//   row 2  ∂Mz/∂[Δω,δ_t,δ_f]:
//     ∂Mz/∂Δω  =  kT·w0²·a·sf + kQ·w0²·st  // 前推力摆角+尾反扭摆角
//     ∂Mz/∂δ_t  = -Qt·ct                    // 下摆反扭耦合到偏航
//     ∂Mz/∂δ_f  =  a·Tf·cf                  // 偏航主控
//
// 验证：在 δ_f=δ_t=0（sf=st=0，cf=ct=1）、wf=wt=w0 时退化为：
//   B_true|₀ = [[-2kQ·w0², 0,     0    ],
//               [0,         -b·Tt, -Qf  ],
//               [0,         -Qt,   a·Tf ]]
// 与 TandemVec-Paper ch/05-allocation.tex B_full 一致（T0=Tt=Tf，τ0=Qt=Qf）。
inline EffectMatrix computeEffectMatrix(const PropulsionState& s,
                                        const TandemVecParams& p,
                                        float w0)
{
    const float Tf   = p.kT * s.wf * s.wf;
    const float Tt   = p.kT * s.wt * s.wt;
    const float Qf   = p.kQ * s.wf * s.wf;
    const float Qt   = p.kQ * s.wt * s.wt;
    const float w0sq = w0 * w0;
    const float cf   = cosf(s.delta_f), sf = sinf(s.delta_f);
    const float ct   = cosf(s.delta_t), st = sinf(s.delta_t);

    EffectMatrix E;

    // row 0: ∂Mx / ∂[Δω, δ_t, δ_f]
    E.M[0][0] = -p.kQ * w0sq * (cf + ct);
    E.M[0][1] = -Qt * st;
    E.M[0][2] =  Qf * sf;

    // row 1: ∂My / ∂[Δω, δ_t, δ_f]
    E.M[1][0] =  p.b * p.kT * w0sq * st - p.kQ * w0sq * sf;
    E.M[1][1] = -p.b * Tt * ct;
    E.M[1][2] = -Qf * cf;

    // row 2: ∂Mz / ∂[Δω, δ_t, δ_f]
    E.M[2][0] =  p.kT * w0sq * p.a * sf + p.kQ * w0sq * st;
    E.M[2][1] = -Qt * ct;
    E.M[2][2] =  p.a * Tf * cf;

    return E;
}

// ============================================================
//  4. 机型数据驱动控制效能矩阵（★2026-08-10 通用层）
// ============================================================
// 纵列双发机型 = 2 电机（前摆+尾摆）+ 3 控制输入（Δω/δ_t/δ_f），
// 通用合成器 computeJacobianNumeric（数值中心差分）自动生成 B——与上方
// 解析 computeEffectMatrix 逐元素 1e-3 内等价（ta 测试 T11 回归锁定）。
// 换机型 = 填新电机表 + 输入映射，合成器/分配器零改动。
//
// 模型系几何（与解析式同构）：
//   电机0 前摆（上摆）：r=(a,0,0)、推力轴 x'、摆轴 z'、spin=−1（CW），
//       转速按 w0√(1+Δω)（u[0]，spd_sign=+1）、摆角取 u[2]（δ_f）
//   电机1 尾摆（下摆）：r=(−b,0,0)、推力轴 x'、摆轴 y'、spin=+1（CCW），
//       转速按 w0√(1−Δω)（u[0]，spd_sign=−1）、摆角取 u[1]（δ_t）
inline EffectMatrix computeEffectMatrixDataDriven(const PropulsionState &s,
                                                  const TandemVecParams &p,
                                                  float w0)
{
    // 纵列双发电机表（static constexpr 几何/映射；力臂 a/b 为运行时参数）
    static constexpr ActuatorDef kTandemActs[2] = {
        // 前摆（上摆）：u_spd=0（Δω 联动 +）、u_angle=2（δ_f）
        {ActuatorKind::kGimbal, {1.f, 0.f, 0.f}, {1.f, 0.f, 0.f}, {0.f, 0.f, 1.f}, -1.f, 0, 1.f, 2},
        // 尾摆（下摆）：u_spd=0（Δω 联动 −）、u_angle=1（δ_t）
        {ActuatorKind::kGimbal, {-1.f, 0.f, 0.f}, {1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}, 1.f, 0, -1.f, 1},
    };
    ActuatorDef acts[2] = {kTandemActs[0], kTandemActs[1]};
    acts[0].r[0] = p.a;   // 前摆 +a
    acts[1].r[0] = -p.b;  // 尾摆 -b
    AirframeModel model = {acts, 2};

    // 控制输入 [Δω, δ_t, δ_f]——工作点：Δω 从当前 wf/wt 反推
    // （差速参数化 wf²=w0²(1+Δω)，中心差分导数恒 = w0² 与解析一致，
    // 但当前转速的幅值项（如 Qt=kQ·wt²）必须用工作点 Δω 表达）
    const float dw_work = (s.wf * s.wf - s.wt * s.wt) / (2.f * w0 * w0);
    const float u[3] = {dw_work, s.delta_t, s.delta_f};

    EffectMatrix E;
    computeJacobianNumeric<3>(model, u, w0, p.kT, p.kQ, E.M);
    return E;
}
