# -*- coding: utf-8 -*-
"""生成仿真可视化图表 → PDF（v2: 中文专业图件，统一 plot_style）"""
import sys, os
sys.path.insert(0, 'simulations/high-fidelity-analysis')
from core import load_params, quat_norm, quat_multiply
from trim_analysis import trim_longitudinal, linearize_at_trim
from simulate import SASController, simulate
from plot_style import (apply_style, finish, ref_hline, log_sci_ticks,
                        C_BLUE, C_VERM, C_GREEN, C_GRAY)
import numpy as np
import matplotlib.pyplot as plt

OUT = "docs/03-理论推导/THY-004/fig-sim"
os.makedirs(OUT, exist_ok=True)
apply_style()

P = load_params()
trim = trim_longitudinal(24, P)
sas = SASController(P, trim)

# ========== 图1: SAS 俯仰阶跃响应 ==========
print("图1: SAS 俯仰阶跃响应...")
data = simulate(sas, P, trim, T_total=8, disturbance=(2, 2.1, "pitch", 5.0))
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(5.6, 5.2), sharex=True)

ax1.plot(data["t"], np.degrees(data["theta"]), color=C_BLUE, label=r'俯仰角 $\theta$')
ref_hline(ax1, np.degrees(-trim["alpha"]), label='配平值')
ax1.set_ylabel(r'$\theta$ [°]')
ax1.legend(loc='lower right')

ax2.plot(data["t"], np.degrees(data["q"]), color=C_VERM)
ax2.set_ylabel(r'俯仰角速率 $q$ [°/s]')

ax3.plot(data["t"], np.degrees(data["delta_t"]), color=C_GREEN)
ref_hline(ax3, np.degrees(trim["delta_t"]))
ax3.set_ylabel(r'尾摆角 $\delta_t$ [°]')
ax3.set_xlabel('时间 $t$ [s]')

ax1.set_title('SAS 俯仰阶跃响应（$t$ = 2 s 施加 5°/s 角速率脉冲）')
finish(fig, f'{OUT}/sas_pitch_step.pdf')

# ========== 图2: RK4 vs Euler 积分精度 ==========
print("图2: RK4 vs Euler 积分精度...")
# 纯偏航旋转 ω=[0,0,1] rad/s, 解析解 q_theo(t)=[cos(t/2),0,0,sin(t/2)]
# 两种积分器均每步保范投影（与动力学实现一致），比较姿态角误差累积
q_e = np.array([1.0, 0.0, 0.0, 0.0]); q_r = np.array([1.0, 0.0, 0.0, 0.0])
omega = np.array([0.0, 0.0, 1.0]); dt = 0.004; N = 15000
err_e = np.zeros(N); err_r = np.zeros(N)
def f_q(qq): return 0.5*quat_multiply(qq, np.array([0, omega[0], omega[1], omega[2]]))
for i in range(N):
    # 欧拉（一阶）
    q_e = quat_norm(q_e + dt*f_q(q_e))
    # RK4（四阶）
    d1 = f_q(q_r); q2 = quat_norm(q_r + 0.5*dt*d1); d2 = f_q(q2)
    q3 = quat_norm(q_r + 0.5*dt*d2); d3 = f_q(q3)
    q4 = quat_norm(q_r + dt*d3); d4 = f_q(q4)
    q_r = quat_norm(q_r + dt/6*(d1 + 2*d2 + 2*d3 + d4))
    # 解析解（与本步末时刻对齐）
    t1 = (i+1)*dt
    q_theo = np.array([np.cos(t1/2), 0, 0, np.sin(t1/2)])
    err_e[i] = np.linalg.norm(q_e - q_theo)
    err_r[i] = np.linalg.norm(q_r - q_theo)

fig, ax = plt.subplots(figsize=(5.6, 2.6))
t_ax = np.arange(N)*dt
ax.semilogy(t_ax, err_e, color=C_BLUE, lw=0.6, alpha=0.8, label='欧拉法（一阶）')
ax.semilogy(t_ax, err_r, color=C_VERM, lw=0.6, alpha=0.8, label='RK4（四阶）')
log_sci_ticks(ax)
ax.set_ylabel(r'四元数误差 $\Vert q - q_{\mathrm{theo}}\Vert$')
ax.set_xlabel('时间 $t$ [s]')
ax.legend(loc='center right')
ax.set_title('四元数积分精度对比：欧拉法 vs RK4（纯偏航旋转）')
finish(fig, f'{OUT}/rk4_vs_euler.pdf')

# ========== 图3: 配平点特征值分布 ==========
print("图3: 特征值分布...")
A, Al, Ala = linearize_at_trim(trim, P)
e_long = np.linalg.eigvals(Al); e_lat = np.linalg.eigvals(Ala)

fig, ax = plt.subplots(figsize=(5.6, 3.6))
ax.scatter(np.real(e_long), np.imag(e_long), c=C_BLUE, marker='o', s=42,
           zorder=3, label='纵向')
ax.scatter(np.real(e_lat), np.imag(e_lat), c=C_VERM, marker='s', s=42,
           zorder=3, label='横航向')
ax.axhline(0, color='k', lw=0.4); ax.axvline(0, color='k', lw=0.4)
for e, c, n in [(e_long, C_BLUE, 'SP'), (e_lat, C_VERM, 'DR')]:
    for ev in [complex(ei.real, ei.imag) for ei in e if abs(ei.imag) > 1]:
        ax.annotate(n, (ev.real, abs(ev.imag)), fontsize=7.5, color=c,
                    textcoords='offset points', xytext=(7, 3))
ax.set_xlabel('实部')
ax.set_ylabel('虚部 [rad/s]')
ax.set_title('配平点特征值分布（$V$ = 24 m/s）')
ax.legend(loc='lower left')
finish(fig, f'{OUT}/eigenvalues.pdf')

# ========== 图4: 配平保持 (20s) ==========
print("图4: 配平保持...")
# 注意: 必须用全新控制器实例, 避免图1运行残留的积分器状态污染
sas_fresh = SASController(P, trim)
data_t = simulate(sas_fresh, P, trim, T_total=20, disturbance=None)
th_ref = np.degrees(-trim["alpha"])
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(5.6, 3.4), sharex=True)
ax1.plot(data_t["t"], (data_t["u"] - data_t["u"][0])*1000, color=C_BLUE)
ax1.set_ylabel(r'速度偏差 $\Delta u$ [mm/s]')
ax1.set_title('配平保持：SAS 20 s 配平直飞（$V$ = 24 m/s）')
ax2.plot(data_t["t"], (np.degrees(data_t["theta"]) - th_ref)*1000, color=C_VERM)
ref_hline(ax2, 0.0)
ax2.set_ylabel(r'俯仰角偏差 $\Delta\theta$ [$10^{-3}$°]')
ax2.set_xlabel('时间 $t$ [s]')
finish(fig, f'{OUT}/trim_stability.pdf')

print(f"\n全部图件已保存 → {OUT}/")
