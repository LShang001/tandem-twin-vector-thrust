# -*- coding: utf-8 -*-
"""交叉验证（Python 侧）：py-web-lab 引擎跑场景 → JSON 轨迹。
与 run_js.mjs 同场景，由 compare.mjs 对比（门槛 <1e-6）。"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]   # simulations/py-web-lab
sys.path.insert(0, str(ROOT / 'server'))

from sim import Simulation          # noqa: E402
import controllers                  # noqa: E402

DT = 0.004
OUT = Path(__file__).resolve().parent / 'cross'


def run(scene, secs):
    sim = Simulation()
    if scene == 'vtol':
        sim.reset_vtol()
    elif scene == 'vtol_pert':
        sim.reset_vtol()
        sim.S['omega'] = {'x': 0.0, 'y': 0.3, 'z': 0.0}
    elif scene == 'vtol_alt':
        sim.reset_vtol()
        sim.S['altHold'] = True
        sim.S['altRef'] = 5.0
    elif scene == 'vtol_btrue':
        sim.reset_vtol()
        sim.S['useBtrue'] = True
        sim.S['omega'] = {'x': 0.0, 'y': 0.3, 'z': 0.2}
    elif scene == 'cruise_sas':
        sim.reset_full()
        sim.S['dt'] = 3 * 0.017453292519943295
        sim.S['df'] = -2 * 0.017453292519943295
        sim.S['dw'] = 0.1
    elif scene == 'vtol_dw':
        sim.reset_vtol()
        sim.S['dw'] = 30 * 0.017453292519943295
    else:
        raise ValueError(scene)

    trace = []
    steps = int(secs / DT)
    for i in range(steps):
        sim.step(DT, controllers.apply)
        if i % 25 == 0:   # 0.1s 采样
            S, F = sim.S, sim.F
            trace.append({
                't': round(i * DT, 4),
                'quat': [S['quat']['x'], S['quat']['y'], S['quat']['z'], S['quat']['w']],
                'omega': [S['omega']['x'], S['omega']['y'], S['omega']['z']],
                'vel': [F['vel']['x'], F['vel']['y'], F['vel']['z']],
                'pos': [F['pos']['x'], F['pos']['y'], F['pos']['z']],
                'thr': S['thr'],
                'dtAct': S['dtAct'], 'dfAct': S['dfAct'], 'dwAct': S['dwAct'],
                'intAlt': S['intAlt'], 'intTh': S['intTh'], 'intPhi': S['intPhi'],
            })
    S = sim.S
    trace.append({
        't': round(steps * DT, 4),
        'quat': [S['quat']['x'], S['quat']['y'], S['quat']['z'], S['quat']['w']],
        'omega': [S['omega']['x'], S['omega']['y'], S['omega']['z']],
        'vel': [F['vel']['x'] for F in (sim.F,)],  # placeholder replaced below
        'pos': [sim.F['pos']['x'], sim.F['pos']['y'], sim.F['pos']['z']],
        'thr': S['thr'],
        'dtAct': S['dtAct'], 'dfAct': S['dfAct'], 'dwAct': S['dwAct'],
        'intAlt': S['intAlt'], 'intTh': S['intTh'], 'intPhi': S['intPhi'],
    })
    trace[-1]['vel'] = [sim.F['vel']['x'], sim.F['vel']['y'], sim.F['vel']['z']]
    return trace


SCENES = {
    'vtol': 8.0, 'vtol_pert': 8.0, 'vtol_alt': 12.0,
    'vtol_btrue': 8.0, 'cruise_sas': 5.0, 'vtol_dw': 8.0,
}

if __name__ == '__main__':
    OUT.mkdir(exist_ok=True, parents=True)
    for scene, secs in SCENES.items():
        trace = run(scene, secs)
        with open(OUT / f'py-{scene}.json', 'w') as f:
            json.dump(trace, f)
        last = trace[-1]
        print(f'[OK] py-{scene}: {len(trace)} 点, 末点 thr={last["thr"]:.4f} '
              f'omega=({last["omega"][0]:+.4f},{last["omega"][1]:+.4f},{last["omega"][2]:+.4f})')
