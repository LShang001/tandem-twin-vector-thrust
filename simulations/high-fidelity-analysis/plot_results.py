# -*- coding: utf-8 -*-
"""生成仿真可视化图表 → PDF/PNG"""
import sys; sys.path.insert(0,'simulations/high-fidelity-analysis')
from core import load_params, aero_forces
from trim_analysis import trim_longitudinal, linearize_at_trim
from simulate import SASController, simulate
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = "docs/03-理论推导/THY-004/fig-sim"
import os; os.makedirs(OUT, exist_ok=True)

P = load_params()
trim = trim_longitudinal(24, P)
sas = SASController(P, trim)

# ========== 图1: SAS 俯仰阶跃响应 ==========
data = simulate(sas, P, trim, T_total=8, disturbance=(2, 2.1, "pitch", 5.0))
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(5.5, 5), sharex=True)

ax1.plot(data["t"], np.degrees(data["theta"]), 'b-', lw=1, label=r'$\theta$ (pitch)')
ax1.axhline(np.degrees(-trim["alpha"]), color='gray', ls='--', lw=0.5, label='trim')
ax1.set_ylabel(r'$\theta$ [deg]'); ax1.legend(fontsize=7); ax1.grid(True, alpha=0.3)

ax2.plot(data["t"], np.degrees(data["q"]), 'r-', lw=1)
ax2.set_ylabel(r'$q$ [deg/s]'); ax2.grid(True, alpha=0.3)

ax3.plot(data["t"], np.degrees(data["delta_t"]), 'g-', lw=1)
ax3.axhline(np.degrees(trim["delta_t"]), color='gray', ls='--', lw=0.5)
ax3.set_ylabel(r'$\delta_t$ [deg]'); ax3.set_xlabel('Time [s]')
ax3.grid(True, alpha=0.3)

fig.suptitle('SAS Pitch Step Response (5 deg/s impulse at t=2s)', fontsize=9)
plt.tight_layout(); fig.savefig(f'{OUT}/sas_pitch_step.pdf', dpi=150, bbox_inches='tight')
plt.close(); print("fig1: sas_pitch_step.pdf")

# ========== 图2: RK4 vs Euler 精度 ==========
from core import quat_norm, quat_multiply
q_e = np.array([1.0, 0.0, 0.0, 0.0]); q_r = np.array([1.0, 0.0, 0.0, 0.0])
omega = np.array([0.0, 0.0, 1.0]); dt = 0.004; N = 15000
err_e = np.zeros(N); err_r = np.zeros(N)
for i in range(N):
    dq = 0.5*dt*quat_multiply(q_e, np.array([0,omega[0],omega[1],omega[2]]))
    q_e = quat_norm(q_e + dq)
    def f_q(qq): return 0.5*quat_multiply(qq, np.array([0,omega[0],omega[1],omega[2]]))
    d1=f_q(q_r); q2=quat_norm(q_r+0.5*dt*d1); d2=f_q(q2)
    q3=quat_norm(q_r+0.5*dt*d2); d3=f_q(q3)
    q4=quat_norm(q_r+dt*d3); d4=f_q(q4)
    q_r = quat_norm(q_r + dt/6*(d1+2*d2+2*d3+d4))
    q_theo = np.array([np.cos(i*dt/2), 0, 0, np.sin(i*dt/2)])
    err_e[i] = np.linalg.norm(q_e - q_theo)
    err_r[i] = np.linalg.norm(q_r - q_theo)

fig, ax = plt.subplots(figsize=(5.5, 2.5))
ax.semilogy(np.arange(N)*dt, err_e, 'b-', lw=0.5, alpha=0.7, label='Euler (1st order)')
ax.semilogy(np.arange(N)*dt, err_r, 'r-', lw=0.5, alpha=0.7, label='RK4 (4th order)')
ax.set_ylabel('Quaternion error ||q - q_theo||')
ax.set_xlabel('Time [s]'); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
ax.set_title('Integration Accuracy: Euler vs RK4 (pure yaw rotation)', fontsize=9)
plt.tight_layout(); fig.savefig(f'{OUT}/rk4_vs_euler.pdf', dpi=150, bbox_inches='tight')
plt.close(); print("fig2: rk4_vs_euler.pdf")

# ========== 图3: 特征值分布 ==========
A, Al, Ala = linearize_at_trim(trim, P)
e_long = np.linalg.eigvals(Al); e_lat = np.linalg.eigvals(Ala)

fig, ax = plt.subplots(figsize=(5.5, 3.5))
ax.scatter(np.real(e_long), np.imag(e_long), c='b', marker='o', s=40, label='Longitudinal')
ax.scatter(np.real(e_lat), np.imag(e_lat), c='r', marker='s', s=40, label='Lateral-directional')
ax.axhline(0, color='k', lw=0.3); ax.axvline(0, color='k', lw=0.3)
for e,c,n in [(e_long,'b','SP'),(e_lat,'r','DR')]:
    for ev in [complex(ei.real, ei.imag) for ei in e if abs(ei.imag)>1]:
        ax.annotate(n, (ev.real, abs(ev.imag)), fontsize=7, color=c)
ax.set_xlabel('Real'); ax.set_ylabel('Imag [rad/s]')
ax.set_title('Eigenvalue Distribution at Trim (V=24 m/s)', fontsize=9)
ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
plt.tight_layout(); fig.savefig(f'{OUT}/eigenvalues.pdf', dpi=150, bbox_inches='tight')
plt.close(); print("fig3: eigenvalues.pdf")

# ========== 图4: 配平稳定性 (20s) ==========
data_t = simulate(sas, P, trim, T_total=20, disturbance=None)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(5.5, 3.5), sharex=True)
ax1.plot(data_t["t"], data_t["u"], 'b-', lw=0.8)
ax1.set_ylabel('u [m/s]'); ax1.grid(True, alpha=0.3)
ax1.set_title('Trim Stability: 20s SAS Hold at V=24 m/s', fontsize=9)
ax2.plot(data_t["t"], np.degrees(data_t["theta"]), 'r-', lw=0.8)
ax2.axhline(np.degrees(-trim["alpha"]), color='gray', ls='--', lw=0.5)
ax2.set_ylabel(r'$\theta$ [deg]'); ax2.set_xlabel('Time [s]')
ax2.grid(True, alpha=0.3)
plt.tight_layout(); fig.savefig(f'{OUT}/trim_stability.pdf', dpi=150, bbox_inches='tight')
plt.close(); print("fig4: trim_stability.pdf")

print(f"\nAll figures saved → {OUT}/")
