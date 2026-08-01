# -*- coding: utf-8 -*-
"""开环激励段辨识验证：悬停开环 PRBS 激励 → 批处理最小二乘 B_eff
（工程正解：专门的辨识机动，而非闭环内连续 RLS）"""
import sys
sys.path.insert(0, 'simulations/high-fidelity-analysis')
import numpy as np
from core import (load_params, control_effectiveness, rk4_step, Propulsion)

P = load_params()
P_true = dict(P)
P_true['kT'] *= 0.8; P_true['kQ'] *= 0.8
P_true['Iy'] *= 1.3; P_true['Ix'] *= 1.2
qh = np.array([np.cos(np.pi/4), 0, np.sin(np.pi/4), 0])
W0_H = np.sqrt(P['m']*P['g']/(2*P['kT']))
DT = 0.004

B0_true, _, _ = control_effectiveness(W0_H, 0, 0, 0, P_true)
Beff_true = np.diag([1/P_true['Ix'], 1/P_true['Iy'], 1/P_true['Iz']]) @ B0_true
print('真实 Beff:\n', np.round(Beff_true, 2))

# —— 开环 PRBS 激励段（2s，三通道交替 ±幅值）——
rng = np.random.default_rng(42)
v = np.zeros(3); w = np.zeros(3); q = qh.copy()
prop = Propulsion(P_true); prop.wf = prop.wt = W0_H
prop.prev_wf = prop.prev_wt = W0_H
N = int(2.0 / DT)
U = []          # 回归量 u_eff（滞后实现）
Y = []          # ω̇_corr（减陀螺，名义 I）
w_prev = w.copy()
for i in range(N):
    # PRBS：每 50 步（0.2s）换通道/方向
    seg = i // 50
    ch = seg % 3
    sign = 1.0 if (seg // 3) % 2 == 0 else -1.0
    u = np.zeros(3)
    u[ch] = sign * [0.10, 0.06, 0.06][ch]
    prop.update(W0_H, u[0], DT)
    Fx, Fy, Fz, Mx, My, Mz = prop.forces(u[2], u[1])
    v, w, q = rk4_step(v, w, q, prop, P_true, Fx, Fy, Fz, Mx, My, Mz, DT)
    if i > 20:  # 跳过初始瞬态
        wdot = (w - w_prev) / DT
        ww = w
        hx = P['Jp'] * (prop.wf - prop.wt)
        g = np.array([(P['Iz']-P['Iy'])*ww[1]*ww[2],
                      (P['Ix']-P['Iz'])*ww[2]*ww[0] - ww[2]*hx,
                      (P['Iy']-P['Ix'])*ww[0]*ww[1] + ww[1]*hx])
        wdot_c = wdot - g / np.array([P['Ix'], P['Iy'], P['Iz']])
        # Jp·ω̇ 反扭瞬态修正（prop 缓存的 dwf/dwt）：
        # Mx = −kQ·wf² + kQ·wt² − Jp·dwf + Jp·dwt（稳态线性模型不含瞬态项）
        wdot_c[0] -= P['Jp'] * (prop.dwt - prop.dwf) / P['Ix']
        dw_eff = (prop.wf / W0_H) ** 2 - 1.0
        U.append([dw_eff, u[1], u[2]])
        Y.append(wdot_c)
    w_prev = w.copy()

U = np.array(U); Y = np.array(Y)
print('样本数:', len(U))

# —— 批处理最小二乘（逐通道）——
Beff_hat = np.zeros((3, 3))
for i in range(3):
    Beff_hat[i] = np.linalg.lstsq(U, Y[:, i], rcond=None)[0]
print('辨识 Beff:\n', np.round(Beff_hat, 2))
err = np.linalg.norm(Beff_hat - Beff_true) / np.linalg.norm(Beff_true)
print('辨识相对误差: %.1f%%' % (err * 100))
