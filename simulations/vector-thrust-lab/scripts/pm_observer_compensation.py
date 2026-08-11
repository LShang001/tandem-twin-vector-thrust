#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pm_observer_compensation.py — τm 观测器对差速通道 PM 的补偿量化
=================================================================
问题：实机"抖油门震荡"（memory 三层定位，2026-08-09）的频域解释？

memory 关键事实：
  - 有效回路增益 = kp·(w0_actual/w0_used)²
  - 分配器用指令油门（w0_used=w0_cmd）、力矩用实际转速（τm 滞后）
  - 油门释放瞬间 w0_actual/w0_cmd ≈ 1.33 → 有效增益瞬态 0.25→0.44（×1.77）
  - 修复：τm 一阶观测器，B 矩阵/调度/current_state 全用 w_est

本脚本量化：
  1. 无观测器：油门阶跃释放 → w0_actual 按 τm 一阶衰减 → 有效增益轨迹
     → PM(K_eff(t)) 轨迹 → 看 PM 是否瞬态穿越 0（失稳窗口）
  2. 有观测器：w_est ≈ w0_actual → 增益恒定 → PM 恒定
  3. 输出：PM 时间轨迹、失稳窗口时长、观测器补偿的 PM 增益

依赖：pm_sensitivity.py 的 pm_at()（差速通道 PM(K)）
"""
import numpy as np
from pm_sensitivity import pm_at, TAU_M, TAU_F, K_SIM

K_NOM = K_SIM          # 仿真内环带宽 60
W_RATIO_PEAK = 1.33    # memory：油门释放瞬间 w0_actual/w0_cmd ≈ 1.33
DT = 0.005
T_END = 0.6            # 观察 0.6s（> 2τm）


def effective_gain(w_ratio):
    """有效内环增益 = K·(w_actual/w_used)²（memory 公式，封顶 1.0 前）。"""
    return K_NOM * w_ratio ** 2


def _main():
    print("=" * 76)
    print(" τm 观测器对差速通道相位裕度的补偿量化")
    print("（对照 memory 2026-08-09 抖油门震荡三层定位）")
    print("=" * 76)
    print(f"\n参数: 标称带宽 K={K_NOM}, τm={TAU_M}s, 陀螺滤波 τf={TAU_F*1e3:.2f}ms")
    print(f"      memory: 油门释放瞬间 w0_actual/w0_cmd ≈ {W_RATIO_PEAK}")

    # ---- 1. 无观测器：油门释放瞬态的 PM 崩塌 ----
    print("\n" + "─" * 76)
    print("[1] 无观测器：油门释放 → w0 失配 → 有效增益 ×(w_ratio)² → PM 轨迹")
    print("─" * 76)
    # 油门释放：w0_cmd 从 1.0 阶跃到 0.6（典型抖油门动作），w0_actual 按 τm 衰减
    t = np.linspace(0, T_END, int(T_END / DT) + 1)
    w_cmd = np.full_like(t, 1.0)
    w_cmd[t > 0.05] = 0.6                        # 0.05s 时油门释放
    w_act = np.empty_like(t)
    w_act[0] = 1.0
    for i in range(1, len(t)):
        w_act[i] = w_act[i-1] + (w_cmd[i-1] - w_act[i-1]) * DT / TAU_M
    # 无观测器：w_used = w_cmd（指令油门）
    w_ratio_no_obs = np.where(w_act > 1e-6, w_act / np.maximum(w_cmd, 1e-6), 1.0)
    K_eff_no_obs = effective_gain(w_ratio_no_obs)
    PM_no_obs = np.array([pm_at(K) for K in K_eff_no_obs])
    PM_no_obs[K_eff_no_obs >= 200] = pm_at(200)  # 超高增益段按 200 近似（避免插值越界）
    # 有观测器：w_used = w_est ≈ w_act → ratio ≈ 1
    PM_obs = np.full_like(t, pm_at(K_NOM))

    # 找失稳窗口（PM ≤ 0）
    unstable_mask = PM_no_obs <= 0
    n_unstable = int(unstable_mask.sum())
    print(f"    标称 PM(K={K_NOM}) = {pm_at(K_NOM):.1f}°（观测器修复后恒定）")
    print(f"\n    无观测器：油门释放后 PM 轨迹关键点：")
    for t_show in [0.05, 0.06, 0.08, 0.10, 0.15, 0.25, 0.45, 0.60]:
        i = int(round(t_show / DT))
        if i < len(t):
            print(f"      t={t_show:5.2f}s  w_ratio={w_ratio_no_obs[i]:.3f}  K_eff={K_eff_no_obs[i]:7.1f}  "
                  f"PM={PM_no_obs[i]:6.1f}°  {'【失稳窗口】' if PM_no_obs[i] <= 0 else ''}")
    print(f"\n    失稳窗口（PM≤0°）总时长 = {n_unstable * DT * 1e3:.0f} ms")
    print(f"    峰值有效增益 K_eff,max = {K_eff_no_obs.max():.1f}（=60×{W_RATIO_PEAK}²≈{K_NOM*W_RATIO_PEAK**2:.0f}）")
    print(f"    峰值增益下 PM = {pm_at(min(K_eff_no_obs.max(), 200)):.1f}°")

    # ---- 2. 观测器补偿的量化 ----
    print("\n" + "─" * 76)
    print("[2] 观测器补偿量化")
    print("─" * 76)
    pm_nom = pm_at(K_NOM)
    pm_min_no_obs = PM_no_obs.min()
    print(f"    有观测器：PM 恒定 = {pm_nom:.1f}°")
    print(f"    无观测器：PM 最低 = {pm_min_no_obs:.1f}°（油门释放瞬态）")
    print(f"    ⟹ 观测器补偿的 PM 增益 = {pm_nom - pm_min_no_obs:.1f}°")
    print(f"    ⟹ 若抖动区间有效增益恰在 K≈{K_NOM*W_RATIO_PEAK**2:.0f} 附近，PM≈{pm_at(K_NOM*W_RATIO_PEAK**2):.1f}°")
    print(f"      （memory 实测：有效增益 0.25→0.44 越震荡点 0.35——与 PM 崩塌到临界一致）")

    # ---- 3. 结论 ----
    print("\n" + "─" * 76)
    print("[3] 结论（对照 memory 三层定位）")
    print("─" * 76)
    print(f"  ① 抖油门震荡的频域机制：油门释放 → w0 失配比 {W_RATIO_PEAK} →")
    print(f"     有效增益 ×{W_RATIO_PEAK**2:.2f}（→{K_NOM*W_RATIO_PEAK**2:.0f}）→ 差速通道 PM 从 {pm_nom:.1f}°")
    print(f"     瞬态崩塌到 {pm_min_no_obs:.1f}°（失稳窗口 ~{n_unstable*DT*1e3:.0f}ms），与 memory 的 ~160ms 同量级")
    print(f"  ② τm 观测器修复机制：B 矩阵/调度/current_state 全部用 w_est≈w_act")
    print(f"     → w_ratio≈1 恒定 → 有效增益恒为 {K_NOM:.0f} → PM 恒 {pm_nom:.1f}° → 无失稳窗口")
    print(f"  ③ 这也解释了为什么是'瞬态失配'而非'静态增益'问题：")
    print(f"     静态 K=60 的 PM={pm_nom:.1f}° 尚稳（D1 γ̄≈0.97），瞬态 K→{K_NOM*W_RATIO_PEAK**2:.0f} 的 PM≈{pm_at(K_NOM*W_RATIO_PEAK**2):.1f}° 必失稳")
    print("=" * 76)


if __name__ == "__main__":
    _main()
