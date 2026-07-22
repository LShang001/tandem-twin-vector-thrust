# -*- coding: utf-8 -*-
"""VTOL Tail-sitter 仿真 + 可视化 (v5: 三控制器对比, 中文专业图件)"""
import sys, os
sys.path.insert(0, 'simulations/high-fidelity-analysis')
from core import load_params, quat_multiply
from controllers import (QuatSASController, QuatINDIController,
                         QuatLQRController, simulate_vtol)
from plot_style import (apply_style, finish, ref_hline, log_sci_ticks,
                        C_BLUE, C_VERM, C_GREEN, C_GRAY)
import numpy as np
import matplotlib.pyplot as plt

OUT = "docs/03-理论推导/THY-004/fig-sim"
os.makedirs(OUT, exist_ok=True)
apply_style()

P = load_params()
qh = np.array([np.cos(np.pi/4), 0, np.sin(np.pi/4), 0])  # 悬停姿态四元数 (绕+y转+90°=机头朝天, JS读数 theta=-90°)
q_level = np.array([1.0, 0.0, 0.0, 0.0])                   # 水平姿态
omega0_trim = np.sqrt(P["m"]*9.81/(2*P["kT"]))   # 精确悬停配平转速
omega0_to = omega0_trim * 1.05                  # 起飞 105% 转速
print(f"悬停配平 ω0 = {omega0_trim:.1f} rad/s ({omega0_trim/P['wMax']*100:.1f}%)")

T_SLERP = 3.0   # slerp 过渡时长
T_TO = 10.0     # 起飞仿真时长

def make_controllers(omega0):
    return {
        'SAS':  (QuatSASController(P, omega0=omega0), C_BLUE,  '-'),
        'INDI': (QuatINDIController(P),               C_VERM,  '-'),
        'LQR':  (QuatLQRController(P, omega0=omega0), C_GREEN, '-'),
    }

def slerp_ref_theta(t):
    """slerp 参考俯仰角 [deg]"""
    return np.where(t < T_SLERP, -90.0*t/T_SLERP, -90.0)

# ====== 图1: LQR 垂直起飞（高度 / 俯仰角 / 误差范数）======
print("图1: LQR 垂直起飞...")
d = simulate_vtol(QuatLQRController(P, omega0=omega0_to), P, qh, omega0_to,
                  T_total=T_TO, slerp_duration=T_SLERP, q_des_initial=q_level)
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(5.6, 5.4), sharex=True)
ax1.plot(d["t"], d["h"], color=C_GREEN)
ax1.set_ylabel('高度 [m]')
ax1.set_title('Tail-sitter 垂直起飞（3 s 球面插值过渡，LQR 姿态控制）')

ax2.plot(d["t"], slerp_ref_theta(d["t"]), color=C_GRAY, ls='--', lw=0.8, label='参考轨迹')
ax2.plot(d["t"], np.degrees(d["theta"]), color=C_GREEN, label='实际姿态')
ref_hline(ax2, -90)
ax2.set_ylabel(r'俯仰角 $\theta$ [°]')
ax2.legend(loc='lower right')

ax3.semilogy(d["t"], np.maximum(d["eps_norm"], 1e-10), color=C_GREEN)
log_sci_ticks(ax3)
ax3.set_ylabel(r'姿态误差 $\Vert\varepsilon_{\mathrm{err}}\Vert$')
ax3.set_xlabel('时间 $t$ [s]')
finish(fig, f'{OUT}/vtol_takeoff.pdf')

# ====== 图2: 三控制器起飞跟踪对比 ======
print("图2: 三控制器起飞对比...")
runs = {}
for name, (ctrl, color, ls) in make_controllers(omega0_to).items():
    runs[name] = (simulate_vtol(ctrl, P, qh, omega0_to, T_total=T_TO,
                                slerp_duration=T_SLERP, q_des_initial=q_level), color, ls)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(5.6, 4.4), sharex=True)
ax1.plot(runs['LQR'][0]["t"], slerp_ref_theta(runs['LQR'][0]["t"]),
         color=C_GRAY, ls='--', lw=0.8, label='参考轨迹')
for name, (dd, color, ls) in runs.items():
    ax1.plot(dd["t"], np.degrees(dd["theta"]), color=color, ls=ls, label=name)
ref_hline(ax1, -90)
ax1.set_ylabel(r'俯仰角 $\theta$ [°]')
ax1.set_ylim(-100, 5)
ax1.legend(loc='upper right', ncol=4, columnspacing=0.9, handlelength=1.6)
ax1.set_title('垂直起飞姿态跟踪：SAS / INDI / LQR 对比')

for name, (dd, color, ls) in runs.items():
    ax2.semilogy(dd["t"], np.maximum(dd["eps_norm"], 1e-10), color=color, ls=ls, label=name)
log_sci_ticks(ax2)
ax2.set_ylabel(r'姿态误差 $\Vert\varepsilon_{\mathrm{err}}\Vert$')
ax2.set_xlabel('时间 $t$ [s]')
finish(fig, f'{OUT}/vtol_takeoff_compare.pdf')

# ====== 图3: 悬停保持 (四元数 SAS, 精确配平) ======
print("图3: 悬停保持...")
d = simulate_vtol(QuatSASController(P, omega0=omega0_trim), P, qh, omega0_trim,
                  T_total=8, q_des_initial=qh)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(5.6, 3.4), sharex=True)
th_std = np.std(np.degrees(d["theta"]))
ax1.plot(d["t"], np.degrees(d["theta"]), color=C_BLUE)
ref_hline(ax1, -90)
ax1.set_ylabel(r'俯仰角 $\theta$ [°]')
ax1.set_ylim(-92.5, -87.5)
ax1.set_title(f'悬停保持（四元数 SAS，$\\sigma_\\theta$ = {th_std:.2f}°）')

ax2.plot(d["t"], d["h"], color=C_BLUE)
ax2.set_ylabel('高度 [m]')
ax2.set_xlabel('时间 $t$ [s]')
ax2.set_ylim(-0.06, 0.06)
finish(fig, f'{OUT}/vtol_hover_hold.pdf')

# ====== 图4: 三控制器悬停抗扰对比 (3° 初始俯仰偏差) ======
print("图4: 三控制器悬停抗扰对比...")
dq = np.array([np.cos(np.radians(1.5)), 0, np.sin(np.radians(1.5)), 0])
q_pert = quat_multiply(qh, dq)
runs3 = {}
for name, (ctrl, color, ls) in make_controllers(omega0_trim).items():
    runs3[name] = (simulate_vtol(ctrl, P, qh, omega0_trim, T_total=5,
                                 q_des_initial=q_pert), color, ls)

fig, ax = plt.subplots(figsize=(5.6, 2.6))
for name, (dd, color, ls) in runs3.items():
    ax.semilogy(dd["t"], np.maximum(dd["eps_norm"], 1e-10), color=color, ls=ls, label=name)
log_sci_ticks(ax)
ax.set_ylabel(r'姿态误差 $\Vert\varepsilon_{\mathrm{err}}\Vert$')
ax.set_xlabel('时间 $t$ [s]')
ax.set_title(r'悬停抗扰对比（$3°$ 初始俯仰偏差）')
ax.legend(loc='upper right')
finish(fig, f'{OUT}/vtol_ctrl_compare.pdf')

# ====== 图5: 悬停配平验证 ======
print("图5: 悬停配平验证...")
d = simulate_vtol(QuatSASController(P, omega0=omega0_trim), P, qh, omega0_trim,
                  T_total=6, q_des_initial=qh)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(5.6, 3.4), sharex=True)
ax1.plot(d["t"], d["vx"], color=C_BLUE, label='$v_x$（机体系）')
ax1.plot(d["t"], d["vz"], color=C_VERM, label='$v_z$（机体系）')
ax1.set_ylabel('速度 [m/s]')
ax1.set_ylim(-0.06, 0.06)
ax1.legend(loc='center right')
ax1.set_title('悬停配平验证：机体系速度保持为零')

ax2.plot(d["t"], d["eps_norm"], color=C_GREEN)
ax2.set_ylabel(r'姿态误差 $\Vert\varepsilon_{\mathrm{err}}\Vert$')
ax2.set_xlabel('时间 $t$ [s]')
finish(fig, f'{OUT}/vtol_trim_verify.pdf')

# 旧文件名清理（v4 及之前）
stale = f'{OUT}/vtol_sas_vs_indi.pdf'
if os.path.exists(stale):
    os.remove(stale)
    print('  (已删除旧图 vtol_sas_vs_indi.pdf)')

print("\n全部 VTOL 图件生成完毕。")
