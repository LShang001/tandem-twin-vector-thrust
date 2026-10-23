# -*- coding: utf-8 -*-
"""数值验证：摆角正方向 → 力矩符号（固件符号链的物理锚点）

@file    verify_torque_sign.py
@module  工具/固件核对（力矩符号）
@brief   数值验证摆角正方向产生的推力与力矩符号
@details 输入：高保真 Propulsion；处理：±1° 摆角比较推力与力矩；
         输出：stdout 符号（供 mix 层符号链核对）。
@theory  T=kT·ω²、δf→Mz'、δt→My' @ref THY-002 §2
@frame   模型系
@units   摆角 deg，力 N，力矩 N·m
@ref     THY-002 §2–3（推力与反扭模型）、CTL-001 §5
"""
import sys
sys.path.insert(0, 'simulations/high-fidelity-analysis')
from core import load_params, Propulsion

P = load_params()
prop = Propulsion(P)
w0 = P['thrTrim'] * P['wMax']
prop.wf = prop.prev_wf = w0
prop.wt = prop.prev_wt = w0

for dt_deg in (1.0, -1.0):
    Fx, Fy, Fz, Mx, My, Mz = prop.forces(P['dfTrim'], dt_deg * 0.01745)
    print(f'dt={dt_deg:+.1f}deg : Fz={Fz:+.4f} N  My={My:+.5f} N*m')

for df_deg in (1.0, -1.0):
    Fx, Fy, Fz, Mx, My, Mz = prop.forces(df_deg * 0.01745, P['dtTrim'])
    print(f'df={df_deg:+.1f}deg : Fy={Fy:+.4f} N  Mx={Mx:+.5f} N*m')
