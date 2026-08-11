#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
phase_margin_analysis.py — 级联架构相位裕度与滤波滞后分析
============================================================
D1（增益裕度 γ̄=0.968）的自然延伸：补全"稳定裕度"拼图的相位侧。

核心问题：实机"滤波滞后 4.2° 稳 / 6.9° 必抖"（memory 滤波大审计）无理论解释。
本脚本建立含陀螺一阶滤波的增广模型，分析滤波对闭环极点/相位裕度的影响，
并对照实机现象。

模型（单通道，状态 [e_R, ω, ω_f]）：
  ė_R = -ω                         （旋转向量运动学）
  ω̇ = btrueK·(ω_des - ω_f)        （内环，控制器用滤波后 ω_f）
  ω̇_f = (ω - ω_f) / τ_f           （一阶低通，τ_f 为连续等效时间常数）

  ω_des = K_p·e_R，K_p = vtolAttKp（外环）

参数源：parameters.mjs + FlightCtrlParams.h（speed_filter_alpha=0.4, 200Hz）
  τ_f 连续等效（z 极点映射）= -dt / ln(1-α) ≈ 9.8ms（α=0.4）
"""
import numpy as np
from scipy import signal

# ---- 参数 ----
VTOL_ATT_KP = 2.5
BTRUE_K = 60.0
K_P = VTOL_ATT_KP                # 论文符号外环增益 = 2.5
DT = 0.005                        # 控制环周期（200Hz）
ALPHA_FILTER = 0.4                # speed_filter_alpha（一级）
TAU_F_NOM = -DT / np.log(1.0 - ALPHA_FILTER)   # ≈ 9.81ms（z极点映射连续等效）
TAU_M = 0.28                      # 电机一阶滞后时间常数（差速通道执行器，parameters.mjs）


# ============================================================
#  增广模型
# ============================================================
def augmented_A(tau_f: float,
                vtol_att_kp: float = VTOL_ATT_KP,
                btrue_k: float = BTRUE_K) -> np.ndarray:
    """含陀螺一阶滤波的增广闭环 A_aug（状态 [e_R, ω, ω_f]）。

    ė_R = -ω
    ω̇ = btrue_k·(K_p·e_R - ω_f)       （K_p = vtol_att_kp）
    ω̇_f = (ω - ω_f) / τ_f
    """
    Kp = vtol_att_kp
    return np.array([
        [0.0,         -1.0,       0.0],
        [btrue_k*Kp,   0.0,      -btrue_k],
        [0.0,          1.0/tau_f, -1.0/tau_f],
    ])


def nominal_A_no_filter() -> np.ndarray:
    """无滤波基线（D1 的 A0，状态 [e_R, ω]）。"""
    return np.array([
        [0.0,            -1.0],
        [BTRUE_K*VTOL_ATT_KP,  -BTRUE_K],
    ])


# ============================================================
#  极点 / 阻尼比分析
# ============================================================
def eig_info(A: np.ndarray):
    """返回特征值与（共轭复根的）阻尼比、自然频率。"""
    eigs = np.linalg.eigvals(A)
    return eigs


def damping_of_complex_pair(eigs: np.ndarray):
    """从特征值里挑出共轭复根，返回 (ζ, ω_n) 列表。实根返回 None。"""
    results = []
    eigs = list(eigs)
    used = [False] * len(eigs)
    for i, lam in enumerate(eigs):
        if used[i]:
            continue
        if abs(lam.imag) > 1e-9:
            # 找共轭
            for j in range(i + 1, len(eigs)):
                if not used[j] and abs(eigs[j] - np.conj(lam)) < 1e-6:
                    used[i] = used[j] = True
                    wn = abs(lam)
                    zeta = -lam.real / wn if wn > 0 else 0.0
                    results.append((zeta, wn))
                    break
        else:
            used[i] = True
    return results


def max_real_part(A: np.ndarray) -> float:
    return float(np.max(np.linalg.eigvals(A).real))


# ============================================================
#  内环开环传函与 margin
# ============================================================
def inner_loop_tf(tau_f: float, btrue_k: float = BTRUE_K,
                  actuator_tau: float = 0.0):
    """内环开环传函（降幂系数）。

    无执行器（摆角通道 δf/δt，舵机快）：
        L(s) = btrueK / [s·(τ_f s+1)]   （二阶）
    含执行器（差速通道 Δω，电机 τm）：
        L(s) = btrueK / [s·(τ_f s+1)·(τm s+1)]   （三阶）

    回路：ω_des → ×btrueK → [执行器 1/(τm s+1)] → ω̇ → ∫ → ω → 滤波 1/(τ_f s+1) → ω_f → 反馈。
    差速通道化简后 B_yaw/Ix 约掉，回路增益仅依赖 btrueK（见论文推导）。
    """
    num = [btrue_k]
    den = np.polymul([tau_f, 1.0, 0.0], [actuator_tau, 1.0]) if actuator_tau > 1e-12 \
          else [tau_f, 1.0, 0.0]
    return num, den


def continuous_margins_general(num, den, w_min=1e-2, w_max=1e4):
    """通用连续传函 margin（数值扫频 s=jω，支持任意阶）。返回 dict。"""
    ws = np.logspace(np.log10(w_min), np.log10(w_max), 300000)
    s = 1j * ws
    H = np.polyval(num, s) / np.polyval(den, s)
    mag = np.abs(H)
    phase = np.unwrap(np.angle(H))      # rad
    logmag = np.log(mag)
    pm, wcp, gm = np.nan, np.nan, np.inf
    for i in np.where(np.diff(np.sign(logmag)) != 0)[0]:
        frac = (0.0 - logmag[i]) / (logmag[i + 1] - logmag[i])
        wc = ws[i] * (ws[i + 1] / ws[i]) ** frac
        pc = phase[i] + frac * (phase[i + 1] - phase[i])
        if np.isnan(wcp) or wc < wcp:
            wcp, pm = wc, 180.0 + np.degrees(pc)
    for i in np.where(np.diff(np.sign(phase + np.pi)) != 0)[0]:
        frac = (-np.pi - phase[i]) / (phase[i + 1] - phase[i])
        wc = ws[i] * (ws[i + 1] / ws[i]) ** frac
        mc = mag[i] * (mag[i + 1] / mag[i]) ** frac
        if mc > 1e-12:
            gm = min(gm, 1.0 / mc)
    return dict(gain_margin=gm, phase_margin_deg=pm, wcp=wcp, wcg=np.nan)


def continuous_margins(tau_f: float, btrue_k: float = BTRUE_K):
    """连续域内环开环 L_i(s)=btrueK/(s(τ_f s+1)) 的 margin（解析，二阶精确）。

    GM=∞（相位 ∈ (-90°,-180°) 不过 -180°）；
    PM = 90° - arctan(ω_gc·τ_f)，ω_gc 由 |L|=1 解出。
    """
    if tau_f < 1e-12:
        return dict(gain_margin=np.inf, phase_margin_deg=90.0, wcg=np.nan, wcp=btrue_k)
    # |L|=1: btrueK² = ω²(1+ω²τ_f²) → τ_f²ω⁴ + ω² - btrueK² = 0，令 x=ω²
    x = (-1.0 + np.sqrt(1.0 + 4.0 * tau_f**2 * btrue_k**2)) / (2.0 * tau_f**2)
    wgc = np.sqrt(max(x, 0.0))
    pm = 90.0 - np.degrees(np.arctan(wgc * tau_f))
    return dict(gain_margin=np.inf, phase_margin_deg=pm, wcg=np.nan, wcp=wgc)


def filter_phase_lag(tau_f: float, omega: float) -> float:
    """一阶低通在频率 omega 的相位滞后（度，正值=滞后量）。"""
    return np.degrees(np.arctan(omega * tau_f))


def _numerical_margin(num_d, den_d, dt):
    """离散传函（降幂系数）的数值 margin：扫频找 |H|=1 与 phase=-180° 穿越点。"""
    ws = np.linspace(1e-4, np.pi / dt, 60000)
    z = np.exp(1j * ws * dt)
    H = np.polyval(num_d, z) / np.polyval(den_d, z)
    mag = np.abs(H)
    phase = np.unwrap(np.angle(H))   # rad，连续
    # 增益穿越 |H|=1 → PM
    pm, wcp = np.nan, np.nan
    s = np.sign(mag - 1.0)
    for i in np.where(np.diff(s) != 0)[0]:
        frac = (1.0 - mag[i]) / (mag[i + 1] - mag[i])
        wc = ws[i] + frac * (ws[i + 1] - ws[i])
        pc = phase[i] + frac * (phase[i + 1] - phase[i])
        if np.isnan(wcp) or wc < wcp:
            wcp, pm = wc, 180.0 + np.degrees(pc)
    # 相位穿越 phase=-180° → GM=1/|H|
    gm = np.inf
    sp = np.sign(phase + np.pi)
    for i in np.where(np.diff(sp) != 0)[0]:
        frac = (-np.pi - phase[i]) / (phase[i + 1] - phase[i])
        wc = ws[i] + frac * (ws[i + 1] - ws[i])
        mc = mag[i] + frac * (mag[i + 1] - mag[i])
        if mc > 1e-9:
            gm = min(gm, 1.0 / mc)
    return dict(gain_margin=gm, phase_margin_deg=pm, wcp=wcp, wcg=np.nan)


# ============================================================
#  离散化分析（200Hz ZOH）
# ============================================================
def discrete_poles(tau_f: float, dt: float = DT,
                   vtol_att_kp: float = VTOL_ATT_KP,
                   btrue_k: float = BTRUE_K) -> np.ndarray:
    """连续增广模型 ZOH 离散化后的极点（应在单位圆内 = 稳定）。"""
    A = augmented_A(tau_f, vtol_att_kp, btrue_k)
    B = np.zeros((3, 1))
    C = np.zeros((1, 3))
    D = np.zeros((1, 1))
    sys_c = signal.StateSpace(A, B, C, D)
    sys_d = sys_c.to_discrete(dt, method='zoh')
    return np.linalg.eigvals(sys_d.A)


def discrete_margin(tau_f: float, dt: float = DT, btrue_k: float = BTRUE_K):
    """离散域（ZOH）内环开环 margin——数值扫频，含采样/ZOH 延迟效应。"""
    num, den = inner_loop_tf(tau_f, btrue_k)
    sys_d = signal.TransferFunction(num, den).to_discrete(dt, method='zoh')
    num_d = np.trim_zeros(np.atleast_1d(sys_d.num), 'f')
    den_d = np.trim_zeros(np.atleast_1d(sys_d.den), 'f')
    if len(num_d) == 0:
        num_d = np.array([0.0])
    return _numerical_margin(num_d, den_d, dt)


# ============================================================
#  main
# ============================================================
def _main():
    print("=" * 74)
    print(" 级联架构相位裕度与滤波滞后分析")
    print(" D1 增益裕度（γ̄=0.968）的相位侧补全")
    print("=" * 74)
    print(f"\n参数: vtolAttKp={VTOL_ATT_KP}, btrueK={BTRUE_K}, K_p(论文)={K_P}")
    print(f"      控制环 200Hz (dt={DT}s), 陀螺滤波 α={ALPHA_FILTER}")
    print(f"      τ_f 连续等效 = -dt/ln(1-α) = {TAU_F_NOM*1e3:.3f} ms\n")

    # ---- 0. 无滤波基线（D1 的 A0）----
    print("─" * 74)
    print("[0] 无滤波基线（D1 的 A0，对照）")
    print("─" * 74)
    A0 = nominal_A_no_filter()
    eigs0 = np.sort_complex(np.linalg.eigvals(A0))
    print(f"    A0 特征值 = {eigs0}")
    print(f"    （D1 已证：过阻尼，两负实根 -2.61/-57.39，二次稳定）")

    # ---- 1. 含滤波增广模型（τ_f 标称值）----
    print("\n" + "─" * 74)
    print(f"[1] 含滤波增广模型（τ_f = {TAU_F_NOM*1e3:.2f} ms 标称值）")
    print("─" * 74)
    A_aug = augmented_A(TAU_F_NOM)
    eigs_aug = np.linalg.eigvals(A_aug)
    print(f"    A_aug 特征值 = {np.sort_complex(eigs_aug)}")
    pairs = damping_of_complex_pair(eigs_aug)
    if pairs:
        for zeta, wn in pairs:
            print(f"    共轭复根: ζ={zeta:.4f}, ω_n={wn:.2f} rad/s "
                  f"({'阻尼良好' if zeta>0.3 else '欠阻尼⚠' if zeta>0.1 else '近失稳🛑'})")
    else:
        print(f"    全实根（无振荡模态）")
    maxre = max_real_part(A_aug)
    print(f"    max Re(λ) = {maxre:.4f}  ⟹  {'稳定 ✓' if maxre<0 else '失稳 ✗'}")

    # ---- 2. τ_f 扫描：根轨迹（连续域）----
    print("\n" + "─" * 74)
    print("[2] 连续域：τ_f 扫描下闭环极点轨迹")
    print("─" * 74)
    taus = np.array([1e-6, 1e-4, 1e-3, 5e-3, TAU_F_NOM, 2e-2, 5e-2, 0.1, 0.2, 0.5, 1.0])
    print(f"    {'τ_f':>10} {'max Re(λ)':>12} {'极点':>36} {'稳定?':>6}")
    for tau in taus:
        A = augmented_A(tau)
        ev = np.sort_complex(np.linalg.eigvals(A))
        mr = max_real_part(A)
        print(f"    {tau:>10.2e} {mr:>12.4f} {str(np.round(ev,3)):>36} {'是' if mr<0 else '否':>6}")

    # ---- 3. 内环开环 margin（连续域，标称 τ_f）----
    print("\n" + "─" * 74)
    print(f"[3] 内环开环 L_i(s)=btrueK/(s(τ_f s+1)) 连续域 margin")
    print("─" * 74)
    m_cont = continuous_margins(TAU_F_NOM)
    print(f"    τ_f = {TAU_F_NOM*1e3:.2f} ms")
    print(f"    增益穿越频率 ω_gc = {m_cont['wcp']:.2f} rad/s ({m_cont['wcp']/2/np.pi:.2f} Hz)")
    print(f"    相位裕度 PM = {m_cont['phase_margin_deg']:.2f}°")
    print(f"    增益裕度 GM = {m_cont['gain_margin']:.3f} (∞，因相位不过 -180°)")
    # 滤波在 ω_gc 的相位滞后
    lag = filter_phase_lag(TAU_F_NOM, m_cont['wcp'])
    print(f"    滤波在 ω_gc 的相位滞后 = {lag:.2f}°")
    print(f"    无滤波 PM（τ_f→0）= 90°（纯积分器）")
    print(f"    ⟹ 滤波吃掉 {lag:.1f}°，连续域 PM 仍 {m_cont['phase_margin_deg']:.1f}°（充足）")

    # ---- 4. 差速通道（含电机 τm）—— 核心发现 ----
    print("\n" + "─" * 74)
    print(f"[4] 差速(yaw)通道：含电机 τm={TAU_M}s 执行器滞后 ★ 核心发现")
    print("─" * 74)
    print("    摆角通道(δf/δt, 舵机快): L=btrueK/[s(τ_f s+1)]       → PM 充足 (见 [3])")
    print("    差速通道(Δω, 电机τm):    L=btrueK/[s(τ_f s+1)(τm s+1)] → 三阶，PM 受限")
    num_y, den_y = inner_loop_tf(TAU_F_NOM, actuator_tau=TAU_M)
    m_yaw = continuous_margins_general(num_y, den_y)
    wcg_y = m_yaw['wcp']
    print(f"\n    增益穿越频率 ω_gc = {wcg_y:.2f} rad/s ({wcg_y/2/np.pi:.2f} Hz)")
    print(f"    ★ 差速通道连续 PM = {m_yaw['phase_margin_deg']:.2f}°")
    print(f"    增益裕度 GM = {m_yaw['gain_margin']:.3f}")
    # 相位分解
    ph_int = -90.0
    ph_act = -np.degrees(np.arctan(wcg_y * TAU_M))
    ph_fil = -np.degrees(np.arctan(wcg_y * TAU_F_NOM))
    print(f"\n    相位分解 @ω_gc={wcg_y:.1f} rad/s:")
    print(f"      积分器 s           : {ph_int:+.1f}°")
    print(f"      执行器 τm={TAU_M}s   : {ph_act:+.1f}°  ← 主导滞后源")
    print(f"      滤波 τ_f={TAU_F_NOM*1e3:.1f}ms : {ph_fil:+.1f}°")
    print(f"      合计               : {ph_int+ph_act+ph_fil:+.1f}°")
    print(f"      PM = 180° + 合计 = {m_yaw['phase_margin_deg']:.1f}°")
    pm_yaw = m_yaw['phase_margin_deg']
    print(f"\n    差速通道简化模型 PM ≈ {pm_yaw:.1f}°（仿真侧参数）")
    if 2 < pm_yaw < 12:
        print(f"    ⟹ 该简化模型裕度偏小；与实机阈值的接近只能作为待检验假设")
        print(f"       不能据此证明 τm 观测器已经提高真实闭环 PM")
        print(f"       实际差速通道仍需按固件PI、采样保持、BTRUE和原始日志复核")

    # 差速内环闭环极点（频域 PM ↔ 时域阻尼比一致性验证）
    print(f"\n    差速内环闭环极点（1+L=0，τ_f·τm·s³+(τ_f+τm)s²+s+btrueK=0）:")
    cl_poles = np.roots([TAU_F_NOM * TAU_M, TAU_F_NOM + TAU_M, 1.0, BTRUE_K])
    for p in np.sort_complex(cl_poles):
        if abs(p.imag) > 1e-9:
            wn = abs(p)
            zeta = -p.real / wn if wn > 0 else 0.0
            print(f"      {p:.3f}  ζ={zeta:.4f}（经验 PM≈100ζ={zeta*100:.1f}° ↔ 频域 PM={m_yaw['phase_margin_deg']:.1f}°）")
        else:
            print(f"      {p.real:.3f}（实根）")

    # ---- 5. α 扫描：找连续域失稳边界 ----
    print("\n" + "─" * 74)
    print("[5] 滤波 α 扫描：连续域何时失稳？（摆角 vs 差速双通道）")
    print("─" * 74)
    print(f"    {'α':>6} {'τ_f(ms)':>9} {'摆角PM':>8} {'差速PM':>8} {'差速余量':>10}")
    for alpha in [0.0, 0.1, 0.2, 0.4, 0.6, 0.8, 0.9, 0.95, 0.99]:
        if alpha < 1e-6:
            tau = 1e-6
        else:
            tau = -DT / np.log(1 - alpha)
        mc_servo = continuous_margins(tau)                       # 摆角(无τm)
        num_y, den_y = inner_loop_tf(tau, actuator_tau=TAU_M)
        mc_yaw = continuous_margins_general(num_y, den_y)        # 差速(含τm)
        headroom = mc_yaw['phase_margin_deg']
        flag = '[越界]' if headroom <= 0 else ('[临界]' if headroom < 10 else '')
        print(f"    {alpha:>6.2f} {tau*1e3:>9.2f} {mc_servo['phase_margin_deg']:>8.2f} "
              f"{headroom:>8.2f} {headroom:>8.2f}° {flag}")

    # ---- 6. 离散域分析（200Hz，含采样延迟效应）----
    print("\n" + "─" * 74)
    print("[6] 离散域分析（200Hz ZOH）—— 仿真采样域（摆角通道）")
    print("─" * 74)
    print(f"    {'τ_f(ms)':>10} {'连续maxRe':>10} {'离散max|λ|':>12} {'离散PM(°)':>10} {'判':>6}")
    for tau_ms in [0.01, 1.0, 5.0, TAU_F_NOM*1e3, 20.0, 50.0]:
        tau = tau_ms * 1e-3
        mr_c = max_real_part(augmented_A(tau))
        dp = discrete_poles(tau)
        mr_d = float(np.max(np.abs(dp)))
        try:
            md = discrete_margin(tau)
            pm_d = md['phase_margin_deg']
        except Exception:
            pm_d = float('nan')
        judge = '稳' if mr_d < 1.0 else '失稳!'
        print(f"    {tau_ms:>10.2f} {mr_c:>10.4f} {mr_d:>12.6f} {pm_d:>10.2f} {judge:>8}")

    # ---- 7. 关键结论 ----
    print("\n" + "=" * 74)
    print(" 关键结论")
    print("=" * 74)
    pm_c = continuous_margins(TAU_F_NOM)['phase_margin_deg']
    print(f"  • 摆角通道(δf/δt, 舵机快)：连续 PM={pm_c:.1f}°，离散 PM≈54.9°（充足）")
    print(f"    ⟹ 该简化采样模型的摆角通道裕度较大；不能替代实机闭环裕度")
    print(f"  • ★ 差速通道(Δω, 含电机τm={TAU_M}s)：连续 PM≈{m_yaw['phase_margin_deg']:.1f}°")
    print(f"    执行器 τm 在 ω_gc≈{wcg_y:.0f}rad/s 贡献 {ph_act:.0f}° 相位滞后（主导）")
    print(f"  • 差速简化模型 PM≈{m_yaw['phase_margin_deg']:.0f}°，提示 τm 与滤波是需要优先验证的动态环节")
    print(f"    该数值与项目记录接近只能作为待检验假设，不能解释实机振荡或观测器收益")
    print("=" * 74)
    return m_yaw['phase_margin_deg'] > 0


if __name__ == "__main__":
    import sys
    ok = _main()
    sys.exit(0 if ok else 1)
