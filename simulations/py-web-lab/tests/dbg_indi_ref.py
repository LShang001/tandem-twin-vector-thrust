"""对照：simulate.py 原版 INDI 全闭环（rk4），初始 q=0.3 rad/s 扰动"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'high-fidelity-analysis'))
import numpy as np
from core import (load_params, Propulsion, aero_forces, euler_from_quat,
                  rk4_step, quat_norm)
from simulate import INDIController

P = load_params()
trim = {'thr': P['thrTrim'], 'alpha': P['aTrim'], 'delta_t': P['dtTrim']}
ctl = INDIController(P, trim)
prop = Propulsion(P)
a0 = P['aTrim']
v = np.array([P['vTrim'] * np.cos(a0), 0, P['vTrim'] * np.sin(a0)])
q = np.array([np.cos(a0 / 2), 0, np.sin(a0 / 2), 0])
w = np.array([0.0, 0.3, 0.0])
w0 = P['thrTrim'] * P['wMax']
prop.wf = prop.prev_wf = w0
prop.wt = prop.prev_wt = w0
df = P['dfTrim']
dt_cmd = P['dtTrim']
dw = 0.0
DT = 0.004
for i in range(2000):
    prop.update(w0, dw, DT)
    Fx, Fy, Fz, Mx, My, Mz = prop.forces(df, dt_cmd)
    aero = aero_forces(v, w, P)
    phi, theta, psi = euler_from_quat(q)
    df, dt_cmd, dw = ctl.update(phi, theta, w[0], w[1], w[2], w0, DT,
                                aero[3:], np.array([Mx, My, Mz]))
    v, w, q = rk4_step(v, w, q, prop, P, Fx, Fy, Fz, Mx, My, Mz, DT, use_aero=True)
    q = quat_norm(q)
    if i % 250 == 0:
        print(f'{i:5d} w=({w[0]:+.4f},{w[1]:+.4f},{w[2]:+.4f}) dt={dt_cmd:+.4f} dw={dw:+.4f} theta={theta:+.4f}')
