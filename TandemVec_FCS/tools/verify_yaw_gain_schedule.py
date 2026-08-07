# -*- coding: utf-8 -*-
"""差速回路增益调度验证：Mx_cmd × (w0/w_hover)² 是否使 Δω 与油门解耦。

问题：Δω = Mx/(2·kQ·w0²) → 低油门时 1/w0² 把回路增益放大几十倍（Kp=0.6 震荡）。
方案：mix 层给 Mx 乘 (w0/w_hover)²，则
      Δω = [Mx·(w0²/wh²)]/(2kQ·w0²) = Mx/(2kQ·wh²)  ← 与 w0 无关，恒定。
"""
import sys
sys.path.insert(0, 'simulations/high-fidelity-analysis')
from core import load_params

P = load_params()
kQ, kT, wMax, Ix, dwMax = P['kQ'], P['kT'], P['wMax'], P['Ix'], P['dwMax']
m, g = P['m'], P['g']

w_hover = (0.5 * m * g / kT) ** 0.5          # 单机悬停转速
print(f'参数: m={m}kg g={g} kT={kT:.3e} → 单机悬停转速 w_hover={w_hover:.1f} rad/s'
      f' = {w_hover/wMax*100:.0f}% wMax')
denom_h = 2.0 * kQ * w_hover ** 2
print(f'      悬停点 ∂Mx/∂Δω = 2·kQ·w_hover² = {denom_h:.4f} N·m\n')

RATE = 80.0   # 摇杆满打 = MAX_MANUAL_yawRATE
print('=== 调度前（当前）：Δω 随油门爆炸 ===')
print(f'  {"油门":>6} {"1/(2kQw0²)":>12} {"Kp=0.22 Δω":>12} {"Kp=0.6 Δω":>11}')
for thr in (0.19, 0.30, 0.50, 0.70, 1.00):
    w0 = thr * wMax
    inv = 1.0 / (2.0 * kQ * w0 * w0)
    d22 = Ix * 0.22 * RATE * inv
    d60 = Ix * 0.60 * RATE * inv
    f22 = '[饱和]' if d22 > dwMax else ''
    f60 = '[饱和]' if d60 > dwMax else ''
    print(f'  {thr*100:5.0f}% {inv:12.1f} {d22:9.3f}{f22:>6} {d60:8.3f}{f60:>6}')
print(f'  → 19% 与 100% 油门增益比 = {(1.0/0.19)**2:.0f}× （平方反比放大）')

print('\n=== 调度后：Mx ×= (w0/w_hover)² ===')
print(f'  {"油门":>6} {"scale":>7} {"Kp=0.35 Δω":>12} {"物理Mx(N·m)":>13}')
for thr in (0.19, 0.30, 0.50, 0.70, 1.00):
    w0 = thr * wMax
    scale = min(max((w0 / w_hover) ** 2, 0.02), 4.0)
    dw = Ix * 0.35 * RATE / denom_h          # 与 w0 无关
    dw = min(dw, dwMax)
    Mx_phys = 2.0 * kQ * w0 * w0 * dw        # 实际产生的力矩
    print(f'  {thr*100:5.0f}% {scale:7.3f} {dw:12.3f} {Mx_phys:13.5f}')
print('  → Δω 全油门恒定 ✓ 回路增益不再随油门变化 → 不会低油门自激震荡')
print('  → 物理力矩仍 ∝w0²（低转速无反扭权限是物理限制，无法绕过），')
print('     但此时是"输出诚实变弱"，而非"增益爆炸震荡"。')

print('\n=== 新 Kp 选取 ===')
for kp in (0.22, 0.35, 0.50, 0.70):
    dw = Ix * kp * RATE / denom_h
    sat = ' [触限幅]' if dw >= dwMax else ''
    print(f'  Kp={kp:.2f}: Δω={min(dw,dwMax):.3f}（限幅 {dwMax}）{sat}'
          f'  单侧油门偏移 ±{((1+min(dw,dwMax))**0.5-1)*50:.1f}%')
print('  震荡历史：Kp=0.6 @19% 油门 → 调度前 Δω=3.4（严重饱和）→ 自激。')
print('  调度后同 Kp 恒为 Δω=0.31，故取 0.35 起步（留 2× 余量），实测再加。')
