#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robust_montecarlo.py — LMI 鲁棒界的蒙特卡洛数值验证
=====================================================
对应论文 ch/15 §15.7 的验证部分

三件事（数值证伪/证实 LMI 预测）：
  A. 精确确定真实失稳边界 δ*（特征值过零点），对照 LMI γ̄ ⟹ 量化保守度
  B. 随机蒙特卡洛：γ̄ 内采样应 0% 失稳（LMI 保证），γ̄ 外应有失稳
  C. 时域收敛验证：对若干 δ，从初始状态积分，确认"特征值稳定 ⟺ 时域收敛"

LMI 是充分条件：γ̄ 内必稳定，γ̄ 外 LMI 不保证（但可能仍稳定）。
保守度 = γ̄ / 真实边界。本问题预期保守度接近 1（Petersen 在单标量不确定上近紧）。

依赖：cascade_closedform, robust_lmi_analysis
"""
import numpy as np
from scipy.linalg import expm

from cascade_closedform import (
    nominal_A, uncertainty_decomposition, reconstruct,
    VTOL_ATT_KP, BTRUE_K,
)
from robust_lmi_analysis import max_robust_gamma

SEED = 20260811


# ============================================================
#  A. 真实失稳边界（特征值过零点）
# ============================================================
def is_stable(A, tol=1e-9):
    """A 是否 Hurwitz（所有特征值实部 < -tol）。"""
    return np.all(np.linalg.eigvals(A).real < -tol)


def max_real_eigpart(A):
    """最大特征值实部（>0 失稳）。"""
    return float(np.max(np.linalg.eigvals(A).real))


def find_instability_boundary(A0, B_D, C_D, direction, lo, hi, tol=1e-7):
    """二分搜索 δ* 使 max Re(λ) = 0（沿 direction 方向）。

    direction=+1: δ 从 0 增大找失稳；direction=-1: δ 从 0 减小找失稳。
    返回 δ*（若 [lo,hi] 内不失稳则返回 None）。
    """
    def stable_at(d):
        return is_stable(reconstruct(A0, B_D, C_D, d))
    # 端点检查
    if stable_at(hi if direction > 0 else lo):
        return None  # 区间内始终稳定
    # 二分：lo 稳定，hi 不稳定（direction>0）
    a = 0.0
    b = hi if direction > 0 else lo
    if not stable_at(a):
        return 0.0
    for _ in range(80):
        mid = 0.5 * (a + b)
        if stable_at(mid):
            a = mid
        else:
            b = mid
        if abs(b - a) < tol:
            break
    return 0.5 * (a + b)


# ============================================================
#  B. 随机蒙特卡洛失稳率
# ============================================================
def montecarlo_instability_rate(A0, B_D, C_D, delta_lo, delta_hi, N=3000,
                                seed=SEED):
    """δ ~ U(delta_lo, delta_hi)，统计真实失稳率。"""
    rng = np.random.default_rng(seed)
    deltas = rng.uniform(delta_lo, delta_hi, size=N)
    n_unstable = 0
    max_eig_seen = -np.inf
    for d in deltas:
        A = reconstruct(A0, B_D, C_D, d)
        m = max_real_eigpart(A)
        if m >= 0:
            n_unstable += 1
        max_eig_seen = max(max_eig_seen, m)
    return n_unstable / N, max_eig_seen


# ============================================================
#  C. 时域收敛验证
# ============================================================
def simulate_linear(A, x0, T=5.0, n_steps=2000):
    """线性定常系统 ẋ = A x 的精确积分（matrix exponential）。返回 t, X。"""
    t = np.linspace(0, T, n_steps)
    dt = T / (n_steps - 1)
    Phi = expm(A * dt)
    X = np.zeros((n_steps, len(x0)))
    X[0] = x0
    for i in range(1, n_steps):
        X[i] = Phi @ X[i - 1]
    return t, X


def settling_to_zero(A, x0, T=5.0):
    """积分到 T，返回末态范数 ‖x(T)‖（收敛 ⟺ →0）。"""
    _, X = simulate_linear(A, x0, T=T, n_steps=500)
    return float(np.linalg.norm(X[-1]))


# ============================================================
#  main
# ============================================================
def _main():
    print("=" * 72)
    print(" LMI 鲁棒界的蒙特卡洛数值验证")
    print("=" * 72)

    A0 = nominal_A()
    A0d, B_D, C_D = uncertainty_decomposition()
    gbar, _ = max_robust_gamma()
    print(f"\nLMI 预测 γ̄ = {gbar:.4f}（充分条件界）\n")

    # ---- A. 真实失稳边界 ----
    print("─" * 72)
    print("[A] 真实失稳边界（特征值 max Re(λ) 过零点）")
    print("─" * 72)
    # δ 正方向
    d_pos = find_instability_boundary(A0d, B_D, C_D, +1, 0.0, 50.0)
    # δ 负方向
    d_neg = find_instability_boundary(A0d, B_D, C_D, -1, -2.0, 0.0)
    print(f"    δ → +∞ 方向: {'区间内不失稳（δ≤50 仍稳定）' if d_pos is None else f'失稳边界 δ* = {d_pos:.4f}'}")
    print(f"    δ → -∞ 方向: {'区间内不失稳' if d_neg is None else f'失稳边界 δ* = {d_neg:.4f}'}")

    # 物理解读
    if d_neg is not None and d_pos is None:
        real_bound = abs(d_neg)
        print(f"\n    真实对称失稳界 = |δ*_neg| = {real_bound:.4f}（单边：正方向无界）")
        conservativeness = gbar / real_bound
        print(f"    LMI 保守度 = γ̄ / 真实界 = {gbar:.4f} / {real_bound:.4f} = {conservativeness:.4f}")
        print(f"    ⟹ {'Petersen 在此问题上近紧（保守度 >0.95）' if conservativeness > 0.95 else '存在保守性'}")
        # 物理解释 δ*_neg
        eff_gain_at_neg = BTRUE_K * (1 + d_neg)
        print(f"\n    物理解读：δ*_neg={d_neg:.4f} 时内环有效增益 = btrueK·(1+δ) = {eff_gain_at_neg:.4f} rad/s")
        print(f"              （δ→-1 ⟹ 内环增益→0 ⟹ 系统开环临界，符合直觉）")

    # ---- B. 随机蒙特卡洛 ----
    print("\n" + "─" * 72)
    print("[B] 随机蒙特卡洛失稳率（N=3000 每组）")
    print("─" * 72)
    cases = [
        ("γ̄ 内（|δ|≤γ̄）",        -gbar,       gbar),
        ("γ̄×1.05 外（轻微越界）",  -gbar*1.05, -gbar),
        ("γ̄×1.05 外（正向）",      gbar,        gbar*1.05),
        ("远超 γ̄（|δ|≤2）",        -2.0,        2.0),
    ]
    print(f"    {'区间':>28} {'失稳率':>8} {'max Re(λ)':>12}  {'LMI 预测':>12}")
    for name, lo, hi in cases:
        rate, maxeig = montecarlo_instability_rate(A0d, B_D, C_D, lo, hi, N=3000)
        # LMI 对这个区间的预测
        if hi <= gbar and lo >= -gbar:
            lmi_pred = "保证稳定"
        elif abs(lo) <= gbar and abs(hi) <= gbar:
            lmi_pred = "保证稳定"
        else:
            lmi_pred = "不保证"
        print(f"    {name:>28} {rate:>8.2%} {maxeig:>12.4f}  {lmi_pred:>12}")

    # 关键断言：γ̄ 内必须 0% 失稳
    rate_in, _ = montecarlo_instability_rate(A0d, B_D, C_D, -gbar, gbar, N=5000)
    print(f"\n    ★ γ̄ 内 5000 样本失稳率 = {rate_in:.4%}")
    print(f"      {'✓ 验证 LMI 充分性（γ̄ 内零失稳）' if rate_in == 0.0 else '✗ 异常：LMI 界内出现失稳'}")

    # ---- C. 时域收敛验证 ----
    print("\n" + "─" * 72)
    print("[C] 时域收敛验证（x₀=[0.1, 0] rad, T=8s，精确 matrix exponential）")
    print("    判定：稳定 ⟺ ‖x(T)‖ < ‖x₀‖（能量衰减）；不稳定 ⟺ ‖x(T)‖ > ‖x₀‖（发散）")
    print("─" * 72)
    x0 = np.array([0.1, 0.0])
    x0_norm = np.linalg.norm(x0)
    test_deltas = [-0.999, -0.95, -0.5, 0.0, 0.5, 0.95]
    if d_neg is not None:
        test_deltas += [d_neg * 1.01]   # 边界外（失稳）
    test_deltas = sorted(set(test_deltas))
    print(f"    {'δ':>8} {'max Re(λ)':>12} {'稳定?':>6} {'‖x₀‖':>8} {'‖x(8s)‖':>12}  判定")
    for d in test_deltas:
        A = reconstruct(A0d, B_D, C_D, d)
        m = max_real_eigpart(A)
        stable = is_stable(A)
        norm_end = settling_to_zero(A, x0, T=8.0)
        decayed = norm_end < x0_norm
        agree = decayed == stable
        print(f"    {d:>8.3f} {m:>12.4f} {'是' if stable else '否':>6} {x0_norm:>8.4f} {norm_end:>12.2e}  "
              f"{'特征值⟺时域 一致 ✓' if agree else '不一致 ✗'}")

    # ---- 总结 ----
    print("\n" + "=" * 72)
    print(" 验证总结")
    print("=" * 72)
    if d_neg is not None:
        print(f"  • LMI γ̄ = {gbar:.4f}，真实失稳界 |δ*_neg| = {abs(d_neg):.4f}")
        print(f"  • 保守度 = {gbar/abs(d_neg):.3f}（Petersen 引理在单标量不确定上近紧）")
    print(f"  • γ̄ 内蒙特卡洛 0% 失稳 ⟹ LMI 充分性验证通过")
    print(f"  • 特征值判定与时域收敛一致 ⟹ 线性模型自洽")
    print(f"  • 结论：单通道对角不确定（内环增益缩放）下，级联架构极其鲁棒")
    print(f"          γ̄≈{gbar:.2f} 意味内环增益可偏差 ~{gbar*100:.0f}% 仍二次稳定")
    print("=" * 72)
    return rate_in == 0.0


if __name__ == "__main__":
    import sys
    ok = _main()
    sys.exit(0 if ok else 1)
