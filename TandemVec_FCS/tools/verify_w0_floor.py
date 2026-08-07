# -*- coding: utf-8 -*-
"""w0 下限（floor）方案验证：低油门不再一路缩放到 0 附近

思路（用户提出）：正常悬停油门在一定范围内，没必要从 0 油门开始算
调度系数和 B 矩阵——低油门区是数值病态区（B∝w0² 整体趋零、det 趋零、
逆解增益∝1/w0² 爆炸），且那里本来也飞不起来。

方案：w0_eff = max(w0, floor·w_hover)，用 w0_eff 算调度系数与 B 矩阵。
"""
import numpy as np
import sys
sys.path.insert(0, 'simulations/high-fidelity-analysis')
from core import load_params, control_effectiveness

P = load_params()
kT, kQ, wMax = P['kT'], P['kQ'], P['wMax']
Ix, dwMax = P['Ix'], P['dwMax']
m, g = P['m'], P['g']
w_hover = (0.5 * m * g / kT) ** 0.5
KP_YAW, RATE = 0.20, 80.0

print(f'w_hover={w_hover:.0f} rad/s = {w_hover/wMax*100:.0f}% wMax')
print(f'（正常悬停油门区间约 40~60% wMax）\n')

print('=== B 矩阵条件数与 det（低油门病态程度）===')
print(f'  {"油门":>6} {"det(B)":>12} {"cond(B)":>10} {"1/(2kQw0²)":>12}')
for thr in (0.05, 0.10, 0.19, 0.30, 0.50, 0.70):
    w0 = thr * wMax
    B, T0, tau0 = control_effectiveness(w0, P['dfTrim'], P['dtTrim'], 0.0, P)
    det = np.linalg.det(B)
    cond = np.linalg.cond(B)
    inv = 1.0 / (2 * kQ * w0 * w0)
    print(f'  {thr*100:5.0f}% {det:12.3e} {cond:10.1f} {inv:12.1f}')
print('  → 5% 油门 det 比 50% 小 6 个数量级，逆解增益放大 100×')

print('\n=== 不同 floor 下的调度系数（下限）与差速 Δω ===')
denom_h = 2 * kQ * w_hover ** 2
for floor in (0.0, 0.4, 0.5, 0.6, 0.7):
    w0_floor = floor * w_hover
    print(f'\n  floor = {floor:.1f}×w_hover = {w0_floor:5.0f} rad/s'
          f' ({w0_floor/wMax*100:.0f}% wMax)   调度下限 = {max(floor**2, 0.02):.3f}')
    print(f'    {"油门":>6} {"w0_eff":>8} {"sched":>7} {"Δω":>7} {"实际Mx(N·m)":>12}')
    for thr in (0.10, 0.19, 0.30, 0.50):
        w0 = thr * wMax
        w0_eff = max(w0, w0_floor)
        sched = min(max((w0_eff / w_hover) ** 2, 0.02), 4.0)
        Mx_cmd = Ix * KP_YAW * RATE * sched
        # 分配器用 w0_eff 计算效能
        dw = Mx_cmd / (2 * kQ * w0_eff ** 2)
        dw = min(dw, dwMax)
        # 实际物理力矩由真实 w0 决定
        Mx_phys = 2 * kQ * w0 ** 2 * dw
        print(f'    {thr*100:5.0f}% {w0_eff:8.0f} {sched:7.3f} {dw:7.3f} {Mx_phys:12.5f}')

print('\n=== 结论 ===')
print('  floor=0（当前）：低油门 sched→0.02 仍在缩放，B 矩阵在病态区计算')
print('  floor=0.6：≤30% wMax 油门统一按 344 rad/s 工作点算')
print('    · 调度系数下限 0.36（不再趋零）')
print('    · B 矩阵在良态工作点求逆（det 大 2 个数量级、cond 更小）')
print('    · Δω 在低油门区恒定 = 悬停区同一手感')
print('    · 实际物理力矩仍 ∝真实w0²（物理限制，诚实变弱）')
print('  ⚠ 零油门保护须保留：用【真实 w0】判 T0<T0_MIN，勿用 w0_eff')
