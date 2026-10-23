# -*- coding: utf-8 -*-
"""ident_phys.py — 物理参数非线性辨识（PRBS 激励 + 最小二乘）。

@file    ident_phys.py
@module  analysis/系统辨识（物理参数）
@brief   用开环 PRBS 激励 + 非线性最小二乘辨识 [kT, kQ, Ix, Iy, Iz]。
@details 用实测 wf/wt（免一阶滞后建模）→ 残差只含物理参数 → 模型不依赖工作点。
         回归式：ω̇ = I⁻¹[M(wf,wt,δf,δt) − ω×(Iω) − ω×h(wf,wt)]。
         运行：仓库根目录 `py -3.12 scripts/ident_phys.py`。
@theory  推力/反扭与转子角动量 @ref THY-002 §2、@ref MOD-001 §3；
         刚体转动方程 @ref MOD-001 §5.2；
         非线性最小二乘（leastsq/信赖域）@ref TandemVec-Paper/ch16。
@frame  FRD 机体系。
@units   SI：kT [N·s²]、kQ [N·m·s²]、I [kg·m²]、ω rad/s、ω̇ rad/s²。
@note    "用实测转速而非指令转速"是关键：否则 τm=0.28 s 的滞后会被错误吸收进 kT/kQ，
         导致辨识结果随工作点漂移（这是 v5 之前的失败模式）。
@see     ident_probe.py、@ref THY-002、@ref TandemVec-Paper/ch16
"""
import sys
sys.path.insert(0, 'simulations/high-fidelity-analysis')
import numpy as np
from scipy.optimize import least_squares
from core import load_params, rk4_step, Propulsion

P = load_params()
P_true = dict(P)
P_true['kT'] *= 0.8; P_true['kQ'] *= 0.8
P_true['Iy'] *= 1.3; P_true['Ix'] *= 1.2
qh = np.array([np.cos(np.pi/4), 0, np.sin(np.pi/4), 0])
W0_H = np.sqrt(P['m']*P['g']/(2*P['kT']))
DT = 0.004

# ============================================================
#  数据采集：开环 PRBS（姿态漂移不影响物理参数辨识）
# ============================================================
v = np.zeros(3); w = np.zeros(3); q = qh.copy()
prop = Propulsion(P_true); prop.wf = prop.wt = W0_H
prop.prev_wf = prop.prev_wt = W0_H
N = int(6.0 / DT)
rec = {'u': [], 'wf': [], 'wt': [], 'w': [], 'wdot': []}
w_prev = w.copy()
for i in range(N):
    seg = i // 125
    ch = seg % 3
    sign = 1.0 if (seg // 3) % 2 == 0 else -1.0
    u = np.zeros(3)
    u[ch] = sign * [0.10, 0.06, 0.06][ch]
    prop.update(W0_H, u[0], DT)
    Fx, Fy, Fz, Mx, My, Mz = prop.forces(u[2], u[1])
    v, w, q = rk4_step(v, w, q, prop, P_true, Fx, Fy, Fz, Mx, My, Mz, DT)
    if i > 20:
        rec['u'].append(u.copy())
        rec['wf'].append(prop.wf); rec['wt'].append(prop.wt)
        rec['w'].append(w.copy())
        rec['wdot'].append((w - w_prev) / DT)
    w_prev = w.copy()

u_a = np.array(rec['u']); wf_a = np.array(rec['wf']); wt_a = np.array(rec['wt'])
w_a = np.array(rec['w']); wdot_a = np.array(rec['wdot'])
print('样本数:', len(u_a))

# ============================================================
#  模型与残差
# ============================================================
def predict_wdot(theta):
    """θ = [kT, kQ, Ix, Iy, Iz] → 每步 ω̇ 预测"""
    kT, kQ, Ix, Iy, Iz = theta
    I = np.array([Ix, Iy, Iz])
    out = np.zeros_like(wdot_a)
    for i in range(len(u_a)):
        df, dt_, dw = u_a[i][2], u_a[i][1], u_a[i][0]
        wf, wt = wf_a[i], wt_a[i]
        Tf = kT * wf * wf; Tt = kT * wt * wt
        Qf = kQ * wf * wf; Qt = kQ * wt * wt
        cf = np.cos(df); sf = np.sin(df); ct = np.cos(dt_); st = np.sin(dt_)
        M = np.array([-Qf*cf + Qt*ct, -P['b']*Tt*st - Qf*sf, P['a']*Tf*sf - Qt*st])
        ww = w_a[i]
        hx = P['Jp'] * (wf - wt)
        g = np.array([(Iz-Iy)*ww[1]*ww[2],
                      (Ix-Iz)*ww[2]*ww[0] - ww[2]*hx,
                      (Iy-Ix)*ww[0]*ww[1] + ww[1]*hx])
        out[i] = (M - g) / I
    return out

def resid(theta):
    """非线性最小二乘残差向量：ω̇_pred(θ) − ω̇_meas（rad/s²，逐轴逐样本）。"""
    return (predict_wdot(theta) - wdot_a).ravel()

theta0 = np.array([P['kT'], P['kQ'], P['Ix'], P['Iy'], P['Iz']])
lo = np.array([0.3e-5, 0.1e-6, 0.5e-3, 0.005, 0.005])
hi = np.array([5e-5, 1e-5, 0.01, 0.1, 0.1])
res = least_squares(resid, theta0, bounds=(lo, hi), max_nfev=200)
kT_h, kQ_h, Ix_h, Iy_h, Iz_h = res.x
print('辨识参数: kT=%.2e (真 %.2e)  kQ=%.2e (真 %.2e)' % (kT_h, P_true['kT'], kQ_h, P_true['kQ']))
print('          Ix=%.4f (真 %.4f)  Iy=%.4f (真 %.4f)  Iz=%.4f (真 %.4f)' % (
    Ix_h, P_true['Ix'], Iy_h, P_true['Iy'], Iz_h, P_true['Iz']))
print('kT 误差 %.1f%%  kQ 误差 %.1f%%  Iy 误差 %.1f%%  Ix 误差 %.1f%%' % (
    abs(kT_h-P_true['kT'])/P_true['kT']*100, abs(kQ_h-P_true['kQ'])/P_true['kQ']*100,
    abs(Iy_h-P_true['Iy'])/P_true['Iy']*100, abs(Ix_h-P_true['Ix'])/P_true['Ix']*100))
