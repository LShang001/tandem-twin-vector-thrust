# -*- coding: utf-8 -*-
"""B_eff 单轴脉冲辨识 v5
定理：陀螺项 g = ω×(Iω) + ω×h 为三轴交叉乘积——单轴转动时恒为零。
每通道独立脉冲激励 → ω 仅沿该轴积累 → g=0 → ω̇ = I⁻¹·M 干净可辨。
差速段：长脉冲（wf/wt 稳定后采样，Jp 瞬态=0）；尾摆/前摆段：短脉冲（ω 积累小，无 Jp）。
"""
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

def pulse_segment(ch, amp):
    """单轴脉冲：预热 0.5s → 脉冲（差速 1.5s/其他 0.4s）→ 采样稳态窗口 → 恢复"""
    v = np.zeros(3); w = np.zeros(3); q = qh.copy()
    prop = Propulsion(P_true); prop.wf = prop.wt = W0_H
    prop.prev_wf = prop.prev_wt = W0_H
    w_prev = w.copy()
    samples = []
    pulse_u = np.zeros(3); pulse_u[ch] = amp   # 记录脉冲指令（循环结束后 u=0）
    if ch == 0:
        pulse_n, warm_n = int(1.5 / DT), int(1.0 / DT)   # 差速：长脉冲（转速稳定）
    else:
        pulse_n, warm_n = int(0.4 / DT), int(0.1 / DT)    # 摆角：短脉冲（ω 积累小）
    for i in range(int(0.5 / DT) + pulse_n + int(0.3 / DT)):
        u = np.zeros(3)
        if 0.5 / DT <= i < 0.5 / DT + pulse_n:
            u[ch] = amp
        prop.update(W0_H, u[0], DT)
        Fx, Fy, Fz, Mx, My, Mz = prop.forces(u[2], u[1])
        v, w, q = rk4_step(v, w, q, prop, P_true, Fx, Fy, Fz, Mx, My, Mz, DT)
        # 采样：脉冲后期（转速稳定/ω 单轴），减陀螺（理论为零，双保险）
        if pulse_n - warm_n <= i - int(0.5/DT) < pulse_n:
            wdot = (w - w_prev) / DT
            ww = w
            hx = P['Jp'] * (prop.wf - prop.wt)
            g = np.array([(P['Iz']-P['Iy'])*ww[1]*ww[2],
                          (P['Ix']-P['Iz'])*ww[2]*ww[0] - ww[2]*hx,
                          (P['Iy']-P['Ix'])*ww[0]*ww[1] + ww[1]*hx])
            wdot_c = wdot - g / np.array([P['Ix'], P['Iy'], P['Iz']])
            samples.append(wdot_c)
        w_prev = w.copy()
    dw_eff = (prop.wf / W0_H) ** 2 - 1.0
    return [dw_eff, pulse_u[1], pulse_u[2]], np.mean(samples, axis=0)

# 6 个脉冲（3 通道 × ±）
if __name__ == '__main__':
    U, Y = [], []
    for ch, amp in [(0, 0.10), (0, -0.10), (1, 0.06), (1, -0.06), (2, 0.06), (2, -0.06)]:
        u, y = pulse_segment(ch, amp)
        U.append(u); Y.append(y)
    U = np.array(U); Y = np.array(Y)

    Beff_hat = np.zeros((3, 3))
    for i in range(3):
        Beff_hat[i] = np.linalg.lstsq(U, Y[:, i], rcond=None)[0]
    print('辨识 Beff:\n', np.round(Beff_hat, 2))
    err = np.linalg.norm(Beff_hat - Beff_true) / np.linalg.norm(Beff_true)
    print('辨识相对误差: %.1f%%' % (err * 100))
