// ============================================================
//  TandemVec_Propulsion.h — 推进正向映射、差速分配与控制效能矩阵
//
//  本文件实现三件事：
//
//  1. computeWrench()
//     推进状态 (wf, wt, δ_f, δ_t) → 机体系六维力/力矩
//     直接对应 propulsion.mjs 的 dyn.Fx/My/Mz… 计算逻辑，
//     符号约定与 THY-004 §3 六维映射式（eq:six_component）完全一致。
//
//  2. allocateDifferential()
//     差速指令 Δω → (ωf_target, ωt_target)
//     保持 ωf² + ωt² = 2ω₀²（平方和不变），含 wMax 钳位与饱和标记。
//
//  3. computeEffectMatrix()
//     在当前工作点计算 ∂M/∂[Δω, δ_t, δ_f] 的真实 Jacobian B_true。
//     与 THY-004 §5 线性化效能矩阵（eq:effectiveness_matrix）的差异：
//       - B_true 依赖实际 wf/wt/δ_f/δ_t，在大摆角/非对称转速时更准；
//       - B_full 仅在 δ_f=δ_t=0、wf=wt=w₀ 处有效；
//       - B_true 退化到 B_full 的条件：δ_f=δ_t=0 且 wf=wt=w₀。
//
//  坐标系约定（与仿真和 LaTeX 一致）：
//    机体系 NED：x_b 前 / y_b 右 / z_b 下
//    前电机：拉力式 CW，绕 z_b 摆动 δ_f（>0 推力向 +y_b 偏，偏航右）
//    尾电机：推进式 CCW，绕 y_b 摆动 δ_t（>0 机体受力 -z_b，即向上；产生低头力矩）
// ============================================================
#pragma once
#include <cmath>
#include "TandemVec_Config.h"

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
// 符号来源（THY-004 §3 eq:six_component）：
//   Fx =  Tf·cδf + Tt·cδt
//   Fy =  Tf·sδf
//   Fz = -Tt·sδt
//   Mx = -Qf·cδf + Qt·cδt        // 反扭矩差（前负后正，因前 CW 后 CCW）
//   My = -b·Tt·sδt - Qf·sδf      // 下摆主控 + 上摆反扭耦合
//   Mz =  a·Tf·sδf - Qt·sδt      // 上摆主控 + 下摆反扭耦合
inline SixDOFWrench computeWrench(const PropulsionState& s, const TandemVecParams& p)
{
    const float Tf = p.kT * s.wf * s.wf;
    const float Tt = p.kT * s.wt * s.wt;
    const float Qf = p.kQ * s.wf * s.wf;
    const float Qt = p.kQ * s.wt * s.wt;
    const float cf = cosf(s.delta_f), sf = sinf(s.delta_f);
    const float ct = cosf(s.delta_t), st = sinf(s.delta_t);

    return SixDOFWrench{
        /* Fx */ Tf * cf + Tt * ct,
        /* Fy */ Tf * sf,
        /* Fz */ -Tt * st,
        /* Mx */ -Qf * cf + Qt * ct,
        /* My */ -p.b * Tt * st - Qf * sf,
        /* Mz */  p.a * Tf * sf - Qt * st,
    };
}

// ============================================================
//  2. 差速开方分配：Δω → (ωf_target, ωt_target)
// ============================================================
// 保持性质 ωf² + ωt² = 2ω₀²（总推力一阶不变）。
// 高油门+大差速时一侧可能超过 wMax，此时施加钳位并置饱和标记。
// 钳位后 ωf² + ωt² < 2ω₀²，总推力低于预期——调用方应通过遥测通知操纵手。
//
// 来源：THY-004 §5 eq:diff_def，与 propulsion.mjs:13-14 逻辑完全一致。
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
// 在当前工作点 (s, w0) 处求偏导，适用于 INDI 或需精确效能的控制律。
//
// 推导（稳态 ω̇=0，链式法则）：
//   ∂wf²/∂Δω = w0²,  ∂wt²/∂Δω = -w0²
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
// 与 THY-004 §5 B_full 一致（T0=Tt=Tf，τ0=Qt=Qf）。
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
