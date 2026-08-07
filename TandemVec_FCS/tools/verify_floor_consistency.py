# -*- coding: utf-8 -*-
"""核查 w0 floor 引入的两个自洽性问题：
1. ai.w0=w0_eff 但 ai.current_state=真实转速 → B 矩阵混用两个工作点，
   零/低油门时 wf=wt≈0 使 B 的第1、2列全零 → det=0 → BTRUE 退降 FULL_B
2. allocateMoments 的零油门保护 T0<T0_MIN 基于 in.w0 → 用 w0_eff 后永不触发
"""
import numpy as np
import sys
sys.path.insert(0, 'simulations/high-fidelity-analysis')
from core import load_params

P = load_params()
kT, kQ, wMax, a, b = P['kT'], P['kQ'], P['wMax'], P['a'], P['b']
m, g = P['m'], P['g']
w_hover = (0.5 * m * g / kT) ** 0.5
w0_floor = 0.6 * w_hover


def effect_matrix(wf, wt, df, dt, w0):
    """复刻固件 computeEffectMatrix"""
    Tf, Tt = kT * wf * wf, kT * wt * wt
    Qf, Qt = kQ * wf * wf, kQ * wt * wt
    w0sq = w0 * w0
    cf, sf = np.cos(df), np.sin(df)
    ct, st = np.cos(dt), np.sin(dt)
    return np.array([
        [-kQ * w0sq * (cf + ct), -Qt * st,        Qf * sf],
        [b * kT * w0sq * st - kQ * w0sq * sf, -b * Tt * ct, -Qf * cf],
        [kT * w0sq * a * sf + kQ * w0sq * st, -Qt * ct,  a * Tf * cf],
    ])


print(f'w_hover={w_hover:.0f}  w0_floor=0.6·w_hover={w0_floor:.0f} rad/s')
print('\n【问题1】B 矩阵混用工作点（w0=w0_eff, wf/wt=真实）')
print(f'  {"油门":>6} {"wf/wt(真实)":>12} {"det(B)":>12} {"退降?":>8}')
DET_MIN = 1e-8
for thr in (0.0, 0.05, 0.15, 0.30, 0.50):
    w0 = thr * wMax
    w0_eff = max(w0, w0_floor)
    B = effect_matrix(w0, w0, 0.0, 0.0, w0_eff)      # 混用：状态用真实、w0 用 eff
    det = np.linalg.det(B)
    print(f'  {thr*100:5.0f}% {w0:12.0f} {det:12.3e} {"YES→FULL_B" if abs(det) < DET_MIN else "no":>8}')

print('\n  修复后（wf/wt 也做 floor）：')
for thr in (0.0, 0.05, 0.15, 0.30, 0.50):
    w0 = thr * wMax
    w0_eff = max(w0, w0_floor)
    wf_eff = max(w0, w0_floor)
    B = effect_matrix(wf_eff, wf_eff, 0.0, 0.0, w0_eff)
    det = np.linalg.det(B)
    print(f'  {thr*100:5.0f}% {wf_eff:12.0f} {det:12.3e} {"YES" if abs(det) < DET_MIN else "no":>8}')

print('\n【问题2】零油门保护 T0 < T0_MIN(1e-4) 是否仍触发？')
for thr in (0.0, 0.02, 0.05):
    w0 = thr * wMax
    T0_real = kT * w0 * w0
    T0_eff = kT * max(w0, w0_floor) ** 2
    print(f'  {thr*100:4.1f}% 油门: T0(真实)={T0_real:.3e} {"<MIN 触发" if T0_real < 1e-4 else ">=MIN"}'
          f'   T0(w0_eff)={T0_eff:.3e} {"<MIN" if T0_eff < 1e-4 else ">=MIN 不触发!"}')
print('  → 用 w0_eff 后零油门保护失效；但 allocateDifferential 仍用真实 w0，')
print('     wf=w0·√(1±Δω)=0 → 电机输出 0%，故【电机安全】')
print('  → 残留影响：解锁+零油门+自动档时舵机会响应姿态误差（原先全零）')
print('     该行为本身合理（姿态环已激活），但属行为变化，需记录')
