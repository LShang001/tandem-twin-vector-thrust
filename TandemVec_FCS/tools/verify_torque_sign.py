# -*- coding: utf-8 -*-
"""数值验证：摆角正方向 → 力矩符号（固件符号链的物理锚点）"""
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
