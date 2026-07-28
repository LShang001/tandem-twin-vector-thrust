// ============================================================
//  TandemVec_ControlAllocation.h — 力矩指令 → 执行器指令的控制分配
//
//  四种策略（AllocationStrategy）：
//
//  DIRECT         对角近似 B_main，O(1)，忽略反扭交叉项。
//  FULL_B         B_full 在 δ=0, wf=wt=w0 处的线性化 3×3 逆（2×2 子块解析逆，
//                 Mx 行仍解耦）。覆盖范围：小到中等摆角（±5°—±15°）。
//  DIRECT_WITH_FF 主通道 + 一阶前馈耦合补偿（FULL_B 的低阶近似）。
//  BTRUE          在当前真实工作点 (δ_f, δ_t, wf, wt) 计算完整 Jacobian
//                 B_true(u_k)，再做 3×3 完整 Cramer 逆。
//                 适用场景：大摆角（>±15°）、高精度轨迹跟踪、INDI 内层。
//                 需要在 AllocationInput 中填入 current_state。
//                 近奇异保护：|det(B)| < DET_MIN 时自动退降到 FULL_B。
// ============================================================
#pragma once
#include <cmath>
#include <cstdint>
#include <algorithm>
#include "TandemVec_Config.h"
#include "TandemVec_Propulsion.h"   // PropulsionState, EffectMatrix, computeEffectMatrix

// ============================================================
//  控制分配策略枚举
// ============================================================
enum class AllocationStrategy : uint8_t
{
    DIRECT          = 0,   // 对角主通道近似（最快，忽略交叉项）
    FULL_B          = 1,   // B_full 3×3 解析逆（线性化，δ=0处）
    DIRECT_WITH_FF  = 2,   // 主通道 + 一阶前馈
    BTRUE           = 3,   // B_true 完整 3×3 Cramer 逆（当前工作点精确 Jacobian）
};

// ============================================================
//  输入：期望力矩 + 当前工作点
// ============================================================
struct AllocationInput
{
    float Mx_cmd;   // N·m 滚转力矩
    float My_cmd;   // N·m 俯仰力矩
    float Mz_cmd;   // N·m 偏航力矩
    float w0;       // rad/s 基准转速 = thr * wMax

    // BTRUE 策略专用：当前真实工作点（上一拍执行器实际状态）
    // DIRECT/FULL_B/DIRECT_WITH_FF 忽略此字段。
    PropulsionState current_state;  // wf, wt, delta_f, delta_t
};

// ============================================================
//  输出：执行器指令 + 诊断信息
// ============================================================
struct AllocationOutput
{
    // 执行器指令（已限幅）
    float delta_f;  // rad   前摆角（偏航）
    float delta_t;  // rad   尾摆角（俯仰）
    float dw;       //  -    差速指令 Δω ∈ [-dwMax, +dwMax]

    // 饱和标记（任意为 true 表示有通道已触达物理极限）
    bool  sat_delta_f;  // |δ_f| 已触达 dMax
    bool  sat_delta_t;  // |δ_t| 已触达 dMax
    bool  sat_dw;       // |Δω| 已触达 dwMax

    // 滚转通道效能（归一化，0=完全失效，1=额定全效）
    // roll_authority = (2·kQ·w0²) / (2·kQ·wMax²)
    // 低于 ~0.1 时滚转控制显著退化（见 THY-004 §5.4）
    float roll_authority;

    // 所使用的策略（原样回传，便于日志/遥测）
    AllocationStrategy strategy_used;
};

// ============================================================
//  内部辅助：饱和限幅（通用）
// ============================================================
static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

// ============================================================
//  主函数：控制分配
// ============================================================
// @param in       期望力矩 + 工作点
// @param p        机型参数（kT/kQ/a/b/dMax/dwMax/wMax）
// @param strategy 分配策略
// @return         限幅后的执行器指令及诊断信息
//
// 调用频率：200 Hz（与 GNC 主循环同频）。
// 计算复杂度：DIRECT/FF = O(1)；FULL_B = O(1)（2×2 解析逆，无迭代）。
inline AllocationOutput allocateMoments(const AllocationInput& in,
                                        const TandemVecParams& p,
                                        AllocationStrategy strategy)
{
    AllocationOutput out{};
    out.strategy_used = strategy;

    // ---- 工作点：基准推力/反扭矩 ----
    const float w0sq = in.w0 * in.w0;
    const float T0   = p.kT * w0sq;   // 基准推力（单台，N）
    const float tau0 = p.kQ * w0sq;   // 基准反扭矩（稳态，N·m）

    // 防止零油门时的除零：设置最小可信推力
    // 当 T0 < T0_MIN，分配结果无意义（执行器无法产生足够力矩），
    // 直接返回零指令，roll_authority 量化了该退化程度。
    const float T0_MIN = 1e-4f;

    // ---- 滚转通道效能（独立于策略） ----
    const float T0_max = p.kT * p.wMax * p.wMax;
    out.roll_authority = (T0_max > 1e-9f) ? (T0 / T0_max) : 0.0f;

    // ---- 各策略实现 ----
    float df = 0.0f, dt = 0.0f, dw = 0.0f;

    if (T0 < T0_MIN)
    {
        // 零油门保护：执行器无推力，任何分配指令均无效
        df = 0.0f; dt = 0.0f; dw = 0.0f;
    }
    else if (strategy == AllocationStrategy::DIRECT)
    {
        // ===== DIRECT：对角近似 B_main = diag(-2τ₀, -b·T₀, a·T₀) =====
        //
        //   Δω = -Mx_cmd / (2·τ₀)        （滚转，效率 -2τ₀）
        //   δ_t = -My_cmd / (b·T₀)        （俯仰，效率 -b·T₀）
        //   δ_f =  Mz_cmd / (a·T₀)        （偏航，效率 +a·T₀）
        //
        // THY-004 §5 eq:direct_mapping
        dw = (tau0 > T0_MIN) ? (-in.Mx_cmd / (2.0f * tau0)) : 0.0f;
        dt = -in.My_cmd / (p.b * T0);
        df =  in.Mz_cmd / (p.a * T0);
    }
    else if (strategy == AllocationStrategy::FULL_B)
    {
        // ===== FULL_B：B_full 3×3 解析逆 =====
        //
        // Mx 行与 My/Mz 解耦（B_full[0][1]=B_full[0][2]=0），直接分离：
        dw = (tau0 > T0_MIN) ? (-in.Mx_cmd / (2.0f * tau0)) : 0.0f;

        // 2×2 子块：B_22 = [[-b·T₀, -τ₀], [-τ₀, a·T₀]]
        // D22 = a·b·T₀² + τ₀²（正定）
        const float D22 = p.a * p.b * T0 * T0 + tau0 * tau0;

        // δ_t = (-1/D22) · (a·T₀·My + τ₀·Mz)
        // δ_f = (1/D22)  · (b·T₀·Mz - τ₀·My)
        dt = (-1.0f / D22) * (p.a * T0 * in.My_cmd + tau0 * in.Mz_cmd);
        df = ( 1.0f / D22) * (p.b * T0 * in.Mz_cmd - tau0 * in.My_cmd);
    }
    else if (strategy == AllocationStrategy::DIRECT_WITH_FF)
    {
        const float dw0 = (tau0 > T0_MIN) ? (-in.Mx_cmd / (2.0f * tau0)) : 0.0f;
        const float dt0 = -in.My_cmd / (p.b * T0);
        const float df0 =  in.Mz_cmd / (p.a * T0);
        const float dMy_coupling = -tau0 * df0;
        const float dMz_coupling = -tau0 * dt0;
        dt = -(in.My_cmd - dMy_coupling) / (p.b * T0);
        df =  (in.Mz_cmd - dMz_coupling) / (p.a * T0);
        dw = dw0;
        (void)dw0;
    }
    else // AllocationStrategy::BTRUE
    {
        // ===== BTRUE：在当前工作点计算完整 Jacobian B_true，3×3 Cramer 逆 =====
        //
        // B_true = computeEffectMatrix(current_state, p, w0)
        //   行→[Mx, My, Mz]，列→[Δω, δ_t, δ_f]
        //
        // u = B_true⁻¹ × M_cmd（Cramer 法则，完全通用，无轴假设）
        //   [Δω ]   C^T   [Mx_cmd]
        //   [δ_t] = ─── × [My_cmd]
        //   [δ_f]   det   [Mz_cmd]
        //
        // 近奇异保护：|det| < DET_MIN 时退降到 FULL_B，保证安全。
        const float DET_MIN = 1.0e-8f;

        EffectMatrix E = computeEffectMatrix(in.current_state, p, in.w0);
        const float* B = &E.M[0][0];   // 行主序访问宏：B[r*3+c]
#define _B(r,c) E.M[r][c]

        // 伴随矩阵（代数余子式矩阵的转置）
        float C00 =  _B(1,1)*_B(2,2) - _B(1,2)*_B(2,1);
        float C01 = -(_B(1,0)*_B(2,2) - _B(1,2)*_B(2,0));
        float C02 =  _B(1,0)*_B(2,1) - _B(1,1)*_B(2,0);
        float C10 = -(_B(0,1)*_B(2,2) - _B(0,2)*_B(2,1));
        float C11 =  _B(0,0)*_B(2,2) - _B(0,2)*_B(2,0);
        float C12 = -(_B(0,0)*_B(2,1) - _B(0,1)*_B(2,0));
        float C20 =  _B(0,1)*_B(1,2) - _B(0,2)*_B(1,1);
        float C21 = -(_B(0,0)*_B(1,2) - _B(0,2)*_B(1,0));
        float C22 =  _B(0,0)*_B(1,1) - _B(0,1)*_B(1,0);
#undef _B
        (void)B;

        float det = E.M[0][0]*C00 + E.M[0][1]*C01 + E.M[0][2]*C02;

        if (fabsf(det) < DET_MIN)
        {
            // 退降到 FULL_B（近奇异：零油门或极端摆角）
            dw = (tau0 > T0_MIN) ? (-in.Mx_cmd / (2.0f * tau0)) : 0.0f;
            const float D22 = p.a * p.b * T0 * T0 + tau0 * tau0;
            dt = (-1.0f / D22) * (p.a * T0 * in.My_cmd + tau0 * in.Mz_cmd);
            df = ( 1.0f / D22) * (p.b * T0 * in.Mz_cmd - tau0 * in.My_cmd);
        }
        else
        {
            float inv = 1.0f / det;
            // u = (1/det) × C^T × M_cmd
            dw = (C00*in.Mx_cmd + C10*in.My_cmd + C20*in.Mz_cmd) * inv;  // Δω
            dt = (C01*in.Mx_cmd + C11*in.My_cmd + C21*in.Mz_cmd) * inv;  // δ_t
            df = (C02*in.Mx_cmd + C12*in.My_cmd + C22*in.Mz_cmd) * inv;  // δ_f
        }
    }

    // ---- 执行器限幅并记录饱和标记 ----
    const float df_raw = df, dt_raw = dt, dw_raw = dw;

    out.delta_f = clampf(df, -p.dMax, p.dMax);
    out.delta_t = clampf(dt, -p.dMax, p.dMax);
    out.dw      = clampf(dw, -p.dwMax, p.dwMax);

    out.sat_delta_f = (df_raw != out.delta_f);
    out.sat_delta_t = (dt_raw != out.delta_t);
    out.sat_dw      = (dw_raw != out.dw);

    return out;
}

// ============================================================
//  便捷重载：使用默认参数
// ============================================================
inline AllocationOutput allocateMoments(const AllocationInput& in,
                                        AllocationStrategy strategy =
                                            AllocationStrategy::FULL_B)
{
    return allocateMoments(in, kDefaultTandemVecParams, strategy);
}
