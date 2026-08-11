# -*- coding: utf-8 -*-
"""2026-08-07 历史调参回放：量化 yaw(差速) 通道的"打杆无反应"。

链路：摇杆(°/s) → 内环 Kp_r → alpha_yaw(deg/s²) → 边界转 rad/s² → Mx'=Ix·alpha
      → 分配 Δω = Mx'/(2·kQ·w0²) → 转速差
对比 TVC 通道（Iy、δt 分配）看数量级差异。数值已换算为新角度域单位，
但未引入后续的 w0 floor、观测器和 FPV 曲线，不代表现役链路。
"""
import sys
import math
sys.path.insert(0, 'simulations/high-fidelity-analysis')
from core import load_params

P = load_params()
kQ, wMax = P['kQ'], P['wMax']
Ix, Iy = P['Ix'], P['Iy']
dwMax = P['dwMax']
KpR_yaw_now, KpR_tvc = 5.729578, 14.323945
ERR = 35.0   # 满打摇杆 = MAX_MANUAL_yawRATE

print(f'参数: Ix={Ix:.4f} Iy={Iy:.4f} kQ={kQ:.2e} wMax={wMax:.0f} dwMax={dwMax}')
print(f'惯量比 Iy/Ix = {Iy/Ix:.1f}×  ← 差速轴惯量极小\n')

print('=== 差速(yaw)通道：不同油门下满打摇杆的 Δω ===')
for thr in (0.16, 0.19, 0.30, 0.50, 1.00):
    w0 = thr * wMax
    denom = 2.0 * kQ * w0 * w0          # ∂Mx/∂Δω
    authority = (w0 / wMax) ** 2         # 归一化滚转效能
    for name, KpR in (('历史5.73', KpR_yaw_now), ('拟改14.32', 14.323945)):
        alpha_dps2 = KpR * ERR
        alpha = alpha_dps2 * math.pi / 180.0
        Mx = Ix * alpha
        dw = Mx / denom if denom > 1e-12 else 9e9
        sat = ' [饱和]' if abs(dw) >= dwMax else ''
        if name.startswith('历史'):
            print(f'  油门{thr*100:5.0f}%  w0={w0:6.1f}  效能={authority*100:5.1f}%  '
                  f'∂Mx/∂Δω={denom:.5f} N·m')
        print(f'      Kp_r={name}: alpha={alpha:5.2f} rad/s²  Mx={Mx:.5f} N·m  '
              f'Δω={dw:+.3f}{sat}')

print('\n=== 对比 TVC(pitch)通道 满打摇杆 ===')
alpha_t = KpR_tvc * ERR * math.pi / 180.0
My = Iy * alpha_t
print(f'  Kp_r=14.32 s^-1: alpha={alpha_t:.2f} rad/s²  My={My:.4f} N·m')
alpha_yaw_hist = KpR_yaw_now * ERR * math.pi / 180.0
print(f'  → 力矩指令是差速通道的 {My/(Ix*alpha_yaw_hist):.0f} 倍（Iy/Ix × Kp 比）')

print('\n=== 结论 ===')
w0_gnd = 0.19 * wMax
dw_now = Ix * alpha_yaw_hist / (2 * kQ * w0_gnd ** 2)
print(f'地面油门 19%：历史 Kp_r=5.73 s^-1 满打摇杆仅 Δω={dw_now:.3f}'
      f'（限幅 {dwMax} 的 {dw_now/dwMax*100:.0f}%）')
print(f'  转速差 ≈ {(1+dw_now)**0.5:.3f}/{(1-dw_now)**0.5:.3f} = '
      f'{((1+dw_now)**0.5/(1-dw_now)**0.5 - 1)*100:.0f}% → 肉眼/听觉难辨')
print('  瓶颈 = Ix 极小(0.0021) × Kp_r 偏小 → 力矩指令太小，不是 dwMax 限幅')
