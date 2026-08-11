#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robust_lmi_analysis.py — 级联架构 LMI 鲁棒稳定性分析
======================================================
对应论文 TandemVec-Paper/ch/15-cascade-control.tex §15.7（新增）

三件事（"还论文的债"）：
  1. 标称 Lyapunov LMI 可行性（P≻0, A0ᵀP + PA0 ≺ 0）——框架自检
  2. Petersen 引理鲁棒 LMI：二分搜索最大不确定界 γ̄
  3. 增益裕度扫描：固定 γ 求 vtolAttKp 可行区间（对照实机调参记录）

数学（Petersen 1988, IJC；Scherer 原始形式）：
  范数有界系统 ẋ = (A0 + γ·BΔ·δ'·CΔ)x，‖δ'‖ ≤ 1 二次稳定
  ⟺ 存在 P ≻ 0, ε > 0：
      [[A0ᵀP + PA0 + ε·CΔᵀCΔ,   γ·P·BΔ],
       [γ·BΔᵀP,                 -ε·I  ]] ≺ 0

工具：cvxpy + CLARABEL（开源 Rust 后端，适合小规模精确 LMI）
依赖：cascade_closedform（A0 / 不确定分解）
"""
import numpy as np
import cvxpy as cp

from cascade_closedform import (
    nominal_A, uncertainty_decomposition, reconstruct,
    VTOL_ATT_KP, BTRUE_K, K_P_PAPER, K_OMEGA_OVER_I,
    time_scale_separation,
)

SOLVER = cp.CLARABEL
SLACK = 1e-7        # 严格不等式数值松弛
PSD_SLACK = 1e-7


# ============================================================
#  求解器封装
# ============================================================
def _solve_feasibility(constraints):
    prob = cp.Problem(cp.Minimize(0), constraints)
    try:
        prob.solve(solver=SOLVER, verbose=False)
    except cp.error.SolverError:
        return "solver_error", prob
    return prob.status, prob


def _is_feasible(status):
    return status in ("optimal", "optimal_inaccurate")


def _val(var):
    """稳健取 cvxpy 变量值（求解器状态依赖兜底）。"""
    try:
        v = var.value
        if v is not None and np.all(np.isfinite(v)):
            return np.array(v)
    except Exception:
        pass
    return None


# ============================================================
#  1. 标称 Lyapunov LMI
# ============================================================
def max_robust_gamma_hermes(kp_r, kp_w, lo=0.0, hi=5.0, tol=1e-4, iters=60):
    """Hermes 重构符号（旋转向量 e_R）下的 γ̄。

    A0 = hermes_A(kp_r, kp_w)，不确定分解：
      BΔ = [0, 1]ᵀ, CΔ = [k_pw·k_pR, -k_pw]（第二行整行缩放）
    返回 (gamma_bar, P_at_bar)。
    """
    from cascade_closedform import hermes_A
    A0 = hermes_A(kp_r, kp_w)
    B_D = np.array([[0.0], [1.0]])
    C_D = np.array([[kp_w * kp_r, -kp_w]])
    feas0, _ = petersen_lmi_feasible(A0, B_D, C_D, 0.0)
    if not feas0:
        return 0.0, None
    best = 0.0
    best_P = None
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        feas, P_val = petersen_lmi_feasible(A0, B_D, C_D, mid)
        if feas:
            best = mid
            best_P = P_val
            lo = mid
        else:
            hi = mid
        if hi - lo < tol:
            break
    return best, best_P


def nominal_lyapunov(A0, verbose=False):
    """求 P ≻ 0 使 A0ᵀP + PA0 ≺ 0（等价 A0 Hurwitz）。

    返回 (feasible, P_value)。
    """
    P = cp.Variable((2, 2), symmetric=True)
    constraints = [
        P >> PSD_SLACK * np.eye(2),
        A0.T @ P + P @ A0 << -SLACK * np.eye(2),
    ]
    status, prob = _solve_feasibility(constraints)
    feasible = _is_feasible(status)
    P_val = _val(P) if feasible else None
    if verbose:
        print(f"    status = {status}")
        if P_val is not None:
            M = A0.T @ P_val + P_val @ A0
            ev = np.linalg.eigvalsh(M).real
            print(f"    P = \n{np.array2string(P_val, precision=4, floatmode='fixed')}")
            print(f"    A0ᵀP + PA0 特征值 = {ev}  (应全 < 0)")
            print(f"    P 特征值           = {np.linalg.eigvalsh(P_val).real}  (应全 > 0)")
    return feasible, P_val


# ============================================================
#  2. Petersen 鲁棒 LMI（固定 γ）
# ============================================================
def petersen_lmi_feasible(A0, B_D, C_D, gamma):
    """Petersen 鲁棒 LMI 可行性（固定 γ）。

    系统 ẋ = (A0 + γ·BΔ·δ'·CΔ)x 对所有 ‖δ'‖ ≤ 1 二次稳定
    ⟺ 存在 P ≻ 0, ε > 0 满足块 LMI ≺ 0。
    返回 (feasible, P_value)。
    """
    P = cp.Variable((2, 2), symmetric=True)
    eps = cp.Variable(nonneg=True)
    top_left = A0.T @ P + P @ A0 + eps * (C_D.T @ C_D)
    top_right = gamma * (P @ B_D)
    M = cp.bmat([
        [top_left,    top_right],
        [top_right.T, -eps * np.eye(1)],
    ])
    constraints = [
        P >> PSD_SLACK * np.eye(2),
        eps >= 1e-9,
        M << -SLACK * np.eye(3),
    ]
    status, _ = _solve_feasibility(constraints)
    feasible = _is_feasible(status)
    P_val = _val(P) if feasible else None
    return feasible, P_val


def max_robust_gamma(vtol_att_kp=VTOL_ATT_KP, btrue_k=BTRUE_K,
                     lo=0.0, hi=5.0, tol=1e-4, iters=60):
    """二分搜索最大不确定界 γ̄（固定增益）。

    γ̄ 物理意义：内环有效增益允许的最大相对扰动 |δ| ≤ γ̄ 仍保证二次稳定。
    返回 (gamma_bar, P_at_bar)。
    """
    A0, B_D, C_D = uncertainty_decomposition(vtol_att_kp, btrue_k)
    feas0, _ = petersen_lmi_feasible(A0, B_D, C_D, 0.0)
    if not feas0:
        return 0.0, None
    best = 0.0
    best_P = None
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        feas, P_val = petersen_lmi_feasible(A0, B_D, C_D, mid)
        if feas:
            best = mid
            best_P = P_val
            lo = mid
        else:
            hi = mid
        if hi - lo < tol:
            break
    return best, best_P


# ============================================================
#  3. 增益裕度扫描
# ============================================================
def gamma_bar_vs_gain(btrue_k=BTRUE_K, kp_range=(0.5, 20.0), n=80):
    """扫描 vtolAttKp，对每个值求 γ̄。返回 (kp_array, gamma_bar_array)。"""
    kps = np.linspace(kp_range[0], kp_range[1], n)
    gbars = np.empty_like(kps)
    for i, kp in enumerate(kps):
        gb, _ = max_robust_gamma(kp, btrue_k, hi=20.0, iters=40)
        gbars[i] = gb
    return kps, gbars


def feasible_kp_band(gamma_target, btrue_k=BTRUE_K,
                     kp_range=(0.5, 20.0), n=200):
    """对给定 γ_target，求 vtolAttKp 可行区间 [kp_min, kp_max] 使 γ̄ ≥ γ_target。"""
    kps, gbars = gamma_bar_vs_gain(btrue_k, kp_range, n)
    mask = gbars >= gamma_target
    if not mask.any():
        return None
    idx = np.where(mask)[0]
    return kps[idx.min()], kps[idx.max()], kps, gbars


# ============================================================
#  main：完整分析报告
# ============================================================
def _main():
    print("=" * 72)
    print(" 级联架构 LMI 鲁棒稳定性分析")
    print(" 论文 ch/15 §15.7（严格 Lyapunov + 可计算鲁棒界）")
    print("=" * 72)
    print(f"求解器: {SOLVER}   松弛: SLACK={SLACK}, PSD_SLACK={PSD_SLACK}\n")

    # ---- 1. 标称 Lyapunov ----
    print("─" * 72)
    print("[1] 标称 Lyapunov LMI（P≻0, A0ᵀP+PA0 ≺ 0）")
    print("─" * 72)
    A0 = nominal_A()
    feas, P_val = nominal_lyapunov(A0, verbose=True)
    print(f"    ⟹ 标称二次稳定: {'是 ✓' if feas else '否 ✗'}")

    # ---- 2. γ̄（仿真增益）----
    print("\n" + "─" * 72)
    print(f"[2] 最大不确定界 γ̄（vtolAttKp={VTOL_ATT_KP}, btrueK={BTRUE_K}）")
    print("─" * 72)
    gbar, P_bar = max_robust_gamma()
    print(f"    γ̄ = {gbar:.4f}")
    print(f"    物理含义：内环有效增益 btrueK·(1+δ) 在 δ ∈ [{-gbar:+.3f}, {gbar:+.3f}]")
    print(f"              即有效增益 ∈ [{BTRUE_K*(1-gbar):.2f}, {BTRUE_K*(1+gbar):.2f}] rad/s")
    print(f"              区间内保证二次稳定（充分条件）")
    # 与论文 ch/15:485 定性 ±10% 对照
    if gbar >= 0.1:
        print(f"    ✓ 覆盖论文 ch/15:485 假设的 ±10% 模型误差（γ_B≈0.1）")
    if gbar >= 0.3:
        print(f"    ✓ 进一步覆盖 ±30% 惯量不确定估计（γ_I≈0.2）")

    # ---- 3. 增益裕度扫描 ----
    print("\n" + "─" * 72)
    print("[3] 增益裕度：vtolAttKp 扫描下 γ̄ 曲线")
    print("─" * 72)
    kps, gbars = gamma_bar_vs_gain(n=60)
    # 打印曲线采样
    print(f"    {'vtolAttKp':>10} {'K_p(论文)':>10} {'γ̄':>8}  {'(覆盖±10%?)':>12}")
    for kp, gb in list(zip(kps, gbars))[::6]:
        print(f"    {kp:>10.3f} {2*kp:>10.3f} {gb:>8.3f}  {'✓' if gb>=0.1 else '✗':>12}")

    # ---- 4. 固定 γ 可行区间 ----
    for g_target in (0.1, 0.2, 0.3):
        res = feasible_kp_band(g_target, n=200)
        if res is None:
            print(f"\n    γ_target={g_target}: 无可行 vtolAttKp")
            continue
        kp_min, kp_max, _, _ = res
        print(f"\n    γ_target = {g_target}: vtolAttKp ∈ [{kp_min:.3f}, {kp_max:.3f}]"
              f"  (K_p(论文) ∈ [{2*kp_min:.3f}, {2*kp_max:.3f}])")

    # ---- 5. 当前方程下的三组量纲口径 ----
    print("\n" + "─" * 72)
    print("[4] 当前旋转向量方程下的量纲口径对照")
    print("─" * 72)
    cases = [
        ("仿真 parameters.mjs", 2.5, 60.0, "仿真比例模型"),
        ("现行固件比例项", 2.8, 16.042818, "角度域 rate.kp 直接为 s^-1"),
        ("迁移失败反例", 2.8, 0.28, "旧数值未缩放，物理增益降低57.3倍"),
    ]
    print(f"    {'来源':>22} {'k_p,R':>9} {'k_p,ω':>10} {'γ̄':>8}  说明")
    for name, kp_r, kp_w, note in cases:
        gb, _ = max_robust_gamma_hermes(kp_r, kp_w)
        print(f"    {name:>22} {kp_r:>9.2f} {kp_w:>10.2f} {gb:>8.3f}  {note}")

    print("\n    注：三组结果只适用于无积分、无滤波、无饱和、无执行器动态的")
    print("        单通道比例模型，不是固件闭环稳定裕度。")

    print("\n" + "=" * 72)
    print(" 分析完成。结果将写入论文 ch/15 §15.7，并由 robust_montecarlo.py 验证。")
    print("=" * 72)
    return feas and gbar > 0


if __name__ == "__main__":
    import sys
    ok = _main()
    sys.exit(0 if ok else 1)
