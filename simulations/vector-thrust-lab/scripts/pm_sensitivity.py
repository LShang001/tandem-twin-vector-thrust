#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pm_sensitivity.py — 差速通道相位裕度对内环带宽的敏感度分析
============================================================
背景：固件/仿真/旧论文三方的内环带宽数字不一致，需厘清量纲后给出
      PM 对带宽的完整依赖曲线，供论文重构（Hermes）裁决使用。

三方带宽（已从代码厘清）：
  - 固件实机：rate.kp=16.042818 s⁻¹，PID 输入 deg/s、输出 deg/s²，
      进入惯量逆解前统一乘 pi/180；因此比例项带宽直接是 16.04 rad/s。
  - 仿真 parameters.mjs：btrueK=60（control.mjs:142-147，纯 rad 域）
  - 旧论文假设：K_ω=8.0, I=0.022 ⟹ K_ω/I=364（Hermes 已声明废止）

差速通道开环（含电机 τm + 陀螺滤波 τf，见 phase_margin_analysis.py）：
  L(s) = K / [s·(τf·s+1)·(τm·s+1)]
  PM(K) = 180° - 90° - atan(ωgc·τm) - atan(ωgc·τf)，ωgc 由 |L|=1 解出
"""
import numpy as np
from scipy.optimize import brentq

# ---- 参数 ----
TAU_M = 0.28          # 电机时间常数（差速执行器）
TAU_F = 0.009788      # 陀螺滤波连续等效（α=0.4, dt=0.005）
TAU_F_NONE = 1e-6     # 无滤波近似

# 三方带宽
K_FW = 16.042818      # 现行固件 rate_roll/pitch.kp（s^-1）
K_SIM = 60.0          # 仿真 btrueK
K_OLD_PAPER = 364.0   # 旧论文 K_ω/I

# 实机经验边界（memory 滤波大审计）
LIVE_STABLE = 4.2     # 滤波滞后 4.2° 稳定
LIVE_OSCILLATE = 6.9  # 6.9° 必抖


def omega_gc(K, tau_m=TAU_M, tau_f=TAU_F):
    """求解增益穿越频率：K² = ω²(1+ω²τm²)(1+ω²τf²)。"""
    def f(x):  # x = ω²
        return x * (1 + x * tau_m**2) * (1 + x * tau_f**2) - K**2
    x0 = K**2
    lo, hi = 0.0, max(10.0 * K**2, x0 + 1.0)
    return np.sqrt(brentq(f, lo, hi))


def pm_at(K, tau_m=TAU_M, tau_f=TAU_F):
    """差速通道相位裕度（度）。"""
    w = omega_gc(K, tau_m, tau_f)
    return 180.0 - 90.0 - np.degrees(np.arctan(w * tau_m)) - np.degrees(np.arctan(w * tau_f))


def _main():
    print("=" * 76)
    print(" 差速通道相位裕度对内环带宽的敏感度分析")
    print(" 目的：厘清固件/仿真/旧论文三方带宽量纲后，给出 PM(K) 依赖曲线")
    print("=" * 76)
    print(f"\n参数: τm={TAU_M}s（电机）, τf={TAU_F*1e3:.2f}ms（陀螺滤波 α=0.4）")

    print("\n" + "─" * 76)
    print("[1] 三方带宽的量纲统一（从代码厘清）")
    print("─" * 76)
    print(f"  固件 rate.kp={K_FW:.6f} s^-1（输入 deg/s、输出 deg/s²）→ 带宽 = {K_FW:.1f} rad/s")
    print("    （flight_control.cpp 在 ControlOutputs 物理边界显式乘 pi/180）")
    print(f"  仿真 btrueK = {K_SIM:.0f} rad/s（control.mjs:142-147，纯 rad 域）")
    print(f"  旧论文 K_ω/I = {K_OLD_PAPER:.0f} rad/s（Hermes 已声明废止）")

    print("\n" + "─" * 76)
    print("[2] 三方带宽下的差速通道 PM（含 τm 执行器）")
    print("─" * 76)
    print(f"    {'来源':>12} {'K(rad/s)':>10} {'ωgc(rad/s)':>10} {'τm滞后':>8} {'τf滞后':>8} {'PM(°)':>8}  判定")
    for name, K in [("固件实机", K_FW), ("仿真", K_SIM), ("旧论文", K_OLD_PAPER)]:
        w = omega_gc(K)
        lag_m = np.degrees(np.arctan(w * TAU_M))
        lag_f = np.degrees(np.arctan(w * TAU_F))
        pm = 180 - 90 - lag_m - lag_f
        verdict = "临界🛑" if pm < 10 else ("充足" if pm > 30 else "偏紧")
        print(f"    {name:>12} {K:>10.2f} {w:>10.2f} {lag_m:>8.1f} {lag_f:>8.1f} {pm:>8.1f}  {verdict}")

    print(f"\n    实机经验边界：滤波滞后 {LIVE_STABLE}° 稳 / {LIVE_OSCILLATE}° 必抖")
    print(f"    ⟹ 真实 PM 应在 [{LIVE_STABLE}°, {LIVE_OSCILLATE}°] 区间内")
    print(f"    固件等效模型 PM={pm_at(K_FW):.1f}°、仿真模型 PM={pm_at(K_SIM):.1f}° 均落在同一量级")

    print("\n" + "─" * 76)
    print("[3] PM(K) 敏感度扫描（含迁移失败反例 0.28 与现行值）")
    print("─" * 76)
    print(f"    {'K(rad/s)':>10} {'ωgc':>8} {'PM(°)':>8}  说明")
    for K in [0.28, 1.0, 3.0, K_FW, 10.0, 20.0, 30.0, K_SIM, 100.0, 200.0, K_OLD_PAPER]:
        pm = pm_at(K)
        note = ""
        if abs(K - K_FW) < 0.01: note = "← 固件等效"
        elif abs(K - K_SIM) < 0.01: note = "← 仿真 btrueK"
        elif abs(K - K_OLD_PAPER) < 0.01: note = "← 旧论文（失稳）"
        elif pm <= 0: note = "失稳"
        print(f"    {K:>10.2f} {omega_gc(K):>8.2f} {pm:>8.1f}  {note}")

    print("\n" + "─" * 76)
    print("[4] 关键结论")
    print("─" * 76)
    pm_fw = pm_at(K_FW)
    pm_sim = pm_at(K_SIM)
    pm_old = pm_at(K_OLD_PAPER)
    print(f"  ① 差速通道 PM 随内环带宽单调下降（τm 滞后 ∝ atan(K·τm)）：")
    print(f"     K=16.0(固件) → PM={pm_fw:.1f}°；K=60(仿真) → PM={pm_sim:.1f}°；K=364(旧论文) → PM={pm_old:.1f}°")
    print(f"  ② 旧论文 K_ω/I=364 的假设下差速通道【必然失稳】——")
    print(f"     现行固件直接使用 kp={K_FW:.3f} s^-1；迁移前 0.28 仅是历史混合域数值。")
    print(f"     论文旧仿真参数与实机不符，不能把 364 当成现役固件带宽。")
    print(f"  ③ 仿真参数 PM=6.0° 恰好落在实机经验区间 [4.2°, 6.9°] 内——")
    print(f"     实机'4.2°稳/6.9°抖'现象与【仿真模型】(btrueK=60) 吻合最佳；")
    print(f"     固件等效模型 PM=22.5° 偏乐观，说明实机差速通道的有效滞后")
    print(f"     大于理想 τm=0.28 单一模型（可能含输出滤波/ESC/PWM 额外滞后），")
    print(f"     或实机 kp 在线调参高于默认 {K_FW:.3f} s^-1。'差速通道瓶颈、τm 主导'机制稳健。")
    print(f"  ④ 若按固件等效 K=16 重算 D1 γ̄：γ̄ 依赖分离比 K/vtolAttKp（16/2.8=5.7），")
    print(f"     仍大于过阻尼条件 4，γ̄ 应仍接近 1（可用 robust_lmi_analysis 复核）")
    print("=" * 76)


if __name__ == "__main__":
    _main()
