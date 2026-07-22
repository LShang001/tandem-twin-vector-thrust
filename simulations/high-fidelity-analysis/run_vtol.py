# -*- coding: utf-8 -*-
"""VTOL Tail-sitter 仿真 + 可视化 (v4: 修复 dt 遮蔽导致的空图)"""
import sys, os
sys.path.insert(0,'simulations/high-fidelity-analysis')
from core import load_params, quat_multiply
from controllers import QuatSASController, QuatINDIController, simulate_vtol
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = "docs/03-理论推导/THY-004/fig-sim"
os.makedirs(OUT, exist_ok=True)

P = load_params(); dt = 0.004
qh = np.array([np.cos(-np.pi/4), 0, np.sin(-np.pi/4), 0])  # hover quat
omega0_trim = np.sqrt(P["m"]*9.81/(2*P["kT"]))       # 精确悬停配平转速
omega0_h = omega0_trim * 1.05  # 105% hover thrust for positive climb (起飞用)
print(f"Hover ω0 = {omega0_trim:.1f} rad/s ({omega0_trim/P['wMax']*100:.1f}%), 起飞用 105% = {omega0_h:.1f}")

# ====== 图0: 垂直起飞 (3s slerp, 水平姿态 -> 悬停姿态) ======
print("Fig0: VTOL takeoff (3s slerp)...")
q_level = np.array([1.0, 0.0, 0.0, 0.0])  # 初始水平姿态
sas_t = QuatSASController(P, omega0=omega0_h)
data_t = simulate_vtol(sas_t, P, qh, omega0_h, T_total=10,
                       slerp_duration=3.0, q_des_initial=q_level)

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(5.5, 5.0), sharex=True)
ax1.plot(data_t["t"], data_t["h"], 'b-', lw=0.8)
ax1.set_ylabel('Height [m]'); ax1.grid(True, alpha=0.3)
ax1.set_title('VTOL Tail-sitter Takeoff (3s slerp)', fontsize=9)

ax2.plot(data_t["t"], np.degrees(data_t["theta"]), 'r-', lw=0.8)
ax2.axhline(-90, color='gray', ls='--', lw=0.5, label='hover')
ax2.set_ylabel(r'$\theta$ [deg]'); ax2.legend(fontsize=7); ax2.grid(True, alpha=0.3)

ax3.semilogy(data_t["t"], np.maximum(data_t["eps_norm"], 1e-12), 'g-', lw=0.8)
ax3.set_ylabel(r'$\|\varepsilon_{err}\|$'); ax3.set_xlabel('Time [s]')
ax3.grid(True, alpha=0.3)
plt.tight_layout(); fig.savefig(f'{OUT}/vtol_takeoff.pdf', dpi=150, bbox_inches='tight')
plt.close(); print(f"  -> vtol_takeoff.pdf  h(10s)={data_t['h'][-1]:.1f} m")

# ====== 图1: 悬停保持 (四元数 SAS) ======
print("Fig1: Hover hold (quaternion SAS)...")
sas = QuatSASController(P, omega0=omega0_trim)
data = simulate_vtol(sas, P, qh, omega0_trim, T_total=8, q_des_initial=qh)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(5.5, 3.5), sharex=True)
ax1.plot(data["t"], np.degrees(data["theta"]), 'r-', lw=0.8)
ax1.axhline(-90, color='gray', ls='--', lw=0.5)
ax1.set_ylabel(r'$\theta$ [deg]'); ax1.grid(True, alpha=0.3)
th_std = np.std(np.degrees(data["theta"][50:]))
ax1.set_title(f'Hover Hold (quaternion SAS, σ={th_std:.2f}°)', fontsize=9)

ax2.plot(data["t"], data["h"], 'b-', lw=0.8)
ax2.set_ylabel('Height [m]'); ax2.set_xlabel('Time [s]'); ax2.grid(True, alpha=0.3)
plt.tight_layout(); fig.savefig(f'{OUT}/vtol_hover_hold.pdf', dpi=150, bbox_inches='tight')
plt.close(); print(f"  -> vtol_hover_hold.pdf  θ_std={th_std:.3f}°")

# ====== 图2: SAS vs INDI hover 对比 ======
print("Fig2: SAS vs INDI hover...")
# 3° 初始俯仰偏差，展示两种控制器的误差收敛过程
dq = np.array([np.cos(np.radians(1.5)), 0, np.sin(np.radians(1.5)), 0])
q_pert = quat_multiply(qh, dq)
sas2 = QuatSASController(P, omega0=omega0_trim)
indi = QuatINDIController(P)
data_s = simulate_vtol(sas2, P, qh, omega0_trim, T_total=5, q_des_initial=q_pert)
data_i = simulate_vtol(indi, P, qh, omega0_trim, T_total=5, q_des_initial=q_pert)

fig, ax = plt.subplots(figsize=(5.5, 2.5))
eps_s = np.abs(data_s["eps_norm"]); eps_i = np.abs(data_i["eps_norm"])
ax.semilogy(data_s["t"], np.maximum(eps_s, 1e-12), 'b-', lw=0.8, label='Quat SAS')
ax.semilogy(data_i["t"], np.maximum(eps_i, 1e-12), 'r-', lw=0.8, label='Quat INDI')
ax.set_ylabel(r'$\|\varepsilon_{err}\|$'); ax.set_xlabel('Time [s]')
ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
ax.set_title('Quaternion Error Norm: SAS vs INDI (Hover)', fontsize=9)
plt.tight_layout(); fig.savefig(f'{OUT}/vtol_sas_vs_indi.pdf', dpi=150, bbox_inches='tight')
plt.close()
print(f"  -> vtol_sas_vs_indi.pdf  SAS median={np.nanmedian(eps_s):.2e}  INDI median={np.nanmedian(eps_i):.2e}")

# ====== 图3: 悬停推力配平验证 ======
print("Fig3: Hover thrust trim verification...")
# 6秒仿真，记录推力
data_v = simulate_vtol(sas, P, qh, omega0_trim, T_total=6, q_des_initial=qh)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(5.5, 3.5), sharex=True)
# 从 vx, vz 推算总推力 (简化: a = dv/dt, F = m*a + m*g)
ax1.plot(data_v["t"], data_v["vx"], 'b-', lw=0.8, label='vx (body)')
ax1.plot(data_v["t"], data_v["vz"], 'r-', lw=0.8, label='vz (body)')
ax1.set_ylabel('Velocity [m/s]'); ax1.legend(fontsize=7); ax1.grid(True, alpha=0.3)
ax1.set_title('Body-Frame Velocity During Hover', fontsize=9)

ax2.plot(data_v["t"], np.degrees(np.abs(data_v["eps_norm"])), 'g-', lw=0.8)
ax2.set_ylabel(r'$\|\varepsilon_{err}\|$'); ax2.set_xlabel('Time [s]')
ax2.grid(True, alpha=0.3)
plt.tight_layout(); fig.savefig(f'{OUT}/vtol_trim_verify.pdf', dpi=150, bbox_inches='tight')
plt.close(); print("  -> vtol_trim_verify.pdf")

print("\nAll VTOL figures regenerated successfully.")
