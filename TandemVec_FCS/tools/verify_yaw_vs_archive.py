# -*- coding: utf-8 -*-
"""存档 VTVL vs 当前实现：差速控制量级对比（统一量纲 = 电机输出百分比差）

存档链路（实飞验证）：
  yawRateTarget = 摇杆 → ±MAX_MANUAL_yawRATE(80°/s)
  yaw_diff_us   = Kp(3.8) × error            ← 输出直接是 PWM 微秒
  motor1/2_us   = base ∓ yaw_diff_us         ← 与油门无关的绝对 us 差
  百分比        = us / 1000 × 100%           (PWM 1000~2000 → 0~100%)

当前链路：
  alpha_yaw = Kp(0.25) × error(35°/s)        ← rad/s²
  Mx'       = Ix × alpha
  Δω        = Mx' / (2·kQ·w0²)   [限幅 dwMax]
  ωf/ωt     = w0·√(1±Δω)                     ← 归一化，随油门缩放
  百分比    = ω / wMax × 100%
"""
import sys
sys.path.insert(0, 'simulations/high-fidelity-analysis')
from core import load_params

P = load_params()
kQ, wMax, Ix, dwMax = P['kQ'], P['wMax'], P['Ix'], P['dwMax']

# ---------- 存档 ----------
ARC_RATE, ARC_KP = 80.0, 3.8
arc_us = ARC_KP * ARC_RATE                    # 满打杆
arc_pct = arc_us / 1000.0 * 100.0             # 单侧油门偏移
print('=== 存档 VTVL（实飞验证基准）===')
print(f'  满打杆 error={ARC_RATE:.0f}°/s × Kp={ARC_KP} = {arc_us:.0f} us')
print(f'  → 单侧油门偏移 ±{arc_pct:.1f}%   双侧转速差 {2*arc_pct:.1f}%')
print(f'  ★ 与油门无关（绝对 us 差），低油门时权限同样充足')

# ---------- 当前 ----------
CUR_RATE, CUR_KP = 35.0, 0.25
print(f'\n=== 当前实现（满打杆 error={CUR_RATE:.0f}°/s × Kp={CUR_KP}）===')
alpha = CUR_KP * CUR_RATE
Mx = Ix * alpha
print(f'  alpha={alpha:.2f} rad/s²  Mx={Mx:.5f} N·m')
print(f'  {"油门":>6} {"Δω":>8} {"限幅":>5} {"ωf%":>7} {"ωt%":>7} {"单侧偏移":>9}')
rows = {}
for thr in (0.19, 0.30, 0.50, 0.70):
    w0 = thr * wMax
    dw_raw = Mx / (2.0 * kQ * w0 * w0)
    dw = max(-dwMax, min(dwMax, dw_raw))
    sat = 'YES' if abs(dw_raw) > dwMax else '-'
    wf = w0 * (1.0 + dw) ** 0.5
    wt = w0 * (1.0 - dw) ** 0.5
    pf, pt, pb = wf / wMax * 100, wt / wMax * 100, thr * 100
    dev = (pf - pb + pb - pt) / 2.0            # 平均单侧偏移
    rows[thr] = dev
    print(f'  {pb:5.0f}% {dw:8.3f} {sat:>5} {pf:7.1f} {pt:7.1f} {dev:8.1f}%')

# ---------- 结论 ----------
print('\n=== 对比结论 ===')
for thr in (0.19, 0.50):
    ratio = arc_pct / rows[thr]
    print(f'  {thr*100:.0f}% 油门：当前 ±{rows[thr]:.1f}%  vs  存档 ±{arc_pct:.1f}%'
          f'  → 偏小 {ratio:.1f}×')
print(f'\n  两项差异来源：')
print(f'    1. 摇杆满打速率 {CUR_RATE:.0f}°/s vs 存档 {ARC_RATE:.0f}°/s = {ARC_RATE/CUR_RATE:.1f}×')
print(f'    2. 通道增益（Kp + 归一化架构）')
print(f'\n  建议：MAX_MANUAL_yawRATE 35→80（对齐存档），Kp_r 0.25→0.6')
alpha2 = 0.6 * 80.0
Mx2 = Ix * alpha2
for thr in (0.19, 0.50):
    w0 = thr * wMax
    dw = max(-dwMax, min(dwMax, Mx2 / (2.0 * kQ * w0 * w0)))
    wf, wt = w0 * (1 + dw) ** 0.5, w0 * (1 - dw) ** 0.5
    dev = ((wf - w0) + (w0 - wt)) / 2 / wMax * 100
    print(f'    改后 {thr*100:.0f}% 油门：Δω={dw:.3f}{" [饱和]" if abs(Mx2/(2*kQ*w0*w0))>dwMax else ""}'
          f'  单侧 ±{dev:.1f}%（存档 ±{arc_pct:.1f}%）')
