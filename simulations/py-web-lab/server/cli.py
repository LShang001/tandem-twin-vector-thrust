# -*- coding: utf-8 -*-
"""py-web-lab 命令行接口（调试 / 测试 / 分析，不依赖浏览器与 WebSocket）。

用法：
  python cli.py state [--mode cruise|vtol]                 # 初始状态快照
  python cli.py scene [--name vtol_pert] [--secs 8]        # 跑场景 → JSON 轨迹
                      [--out path.json] [--S thr=0.5,dt=0.1] [--pulse 0,0.3,0]
                      [--mode cruise|vtol] [--paused]
  python cli.py step [--n 100] [--dt 0.004] [--mode vtol]  # 逐步推进（打印控制律中间量）
                      [--S ...] [--pulse ...] [--print-every 25]
  python cli.py compare <a.json> <b.json> [--tolerance 1e-6]   # 轨迹对比

场景名（对齐 tests/cross）：vtol / vtol_pert / vtol_alt / vtol_btrue /
  cruise_sas / vtol_dw / cruise_trim
"""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim import Simulation  # noqa: E402
import controllers          # noqa: E402

DT = 0.004
D2R = 0.017453292519943295


# ------------------------------------------------------------
#  场景构建（与 tests/cross/run_py.py 对齐）
# ------------------------------------------------------------
SCENES = {
    'vtol': 8.0, 'vtol_pert': 8.0, 'vtol_alt': 12.0,
    'vtol_btrue': 8.0, 'cruise_sas': 5.0, 'vtol_dw': 8.0, 'cruise_trim': 5.0,
}


def build_scene(name, mode=None, S=None, pulse=None):
    sim = Simulation()
    if name.startswith('vtol') or mode == 'vtol':
        sim.reset_vtol()
    else:
        sim.reset_full()
    if S:
        for k, v in S.items():
            sim.S[k] = v
    if name == 'vtol_pert':
        sim.S['omega'] = {'x': 0.0, 'y': 0.3, 'z': 0.0}
    elif name == 'vtol_alt':
        sim.S['altHold'] = True
        sim.S['altRef'] = 5.0
    elif name == 'vtol_btrue':
        sim.S['useBtrue'] = True
        sim.S['omega'] = {'x': 0.0, 'y': 0.3, 'z': 0.2}
    elif name == 'cruise_sas':
        sim.S['dt'] = 3 * D2R
        sim.S['df'] = -2 * D2R
        sim.S['dw'] = 0.1
    elif name == 'vtol_dw':
        sim.S['dw'] = 30 * D2R
    if pulse:
        sim.S['omega'] = {'x': pulse[0], 'y': pulse[1], 'z': pulse[2]}
    return sim


def snap(sim):
    S, F = sim.S, sim.F
    return {
        't': round(S['time'], 4),
        'quat': [S['quat']['x'], S['quat']['y'], S['quat']['z'], S['quat']['w']],
        'omega': [S['omega']['x'], S['omega']['y'], S['omega']['z']],
        'vel': [F['vel']['x'], F['vel']['y'], F['vel']['z']],
        'pos': [F['pos']['x'], F['pos']['y'], F['pos']['z']],
        'thr': S['thr'], 'dtAct': S['dtAct'], 'dfAct': S['dfAct'], 'dwAct': S['dwAct'],
        'intAlt': S['intAlt'], 'intTh': S['intTh'], 'intPhi': S['intPhi'],
    }


def parse_S(pairs):
    """'thr=0.5,dt=0.1,dw=0.3' → dict"""
    out = {}
    if not pairs:
        return out
    for item in pairs.split(','):
        k, _, v = item.partition('=')
        k = k.strip()
        if k in ('aero', 'lockXY', 'vtolMode', 'useBtrue', 'altHold', 'paused'):
            out[k] = v.strip().lower() in ('1', 'true', 'yes', 'on')
        elif k == 'sasMode':
            out[k] = int(v)
        else:
            out[k] = float(v)
    return out


# ------------------------------------------------------------
#  子命令
# ------------------------------------------------------------
def cmd_state(args):
    sim = build_scene(args.name, args.mode)
    print(json.dumps(snap(sim), ensure_ascii=False, indent=1))
    print('# params:', len([k for k in sim.P]))


def cmd_scene(args):
    sim = build_scene(args.name, args.mode, parse_S(args.S),
                      tuple(float(x) for x in args.pulse.split(',')) if args.pulse else None)
    sim.S['paused'] = bool(args.paused)
    secs = args.secs if args.secs is not None else SCENES.get(args.name, 8.0)
    trace = []
    steps = int(secs / DT)
    for i in range(steps):
        sim.step(DT, controllers.apply)
        if i % 25 == 0:
            trace.append(snap(sim))
    trace.append(snap(sim))
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(json.dumps(trace), encoding='utf-8')
        print(f'[OK] {len(trace)} 点 → {args.out}')
    last = trace[-1]
    print(f'    末点 t={last["t"]} thr={last["thr"]:.4f} '
          f'omega=({last["omega"][0]:+.4f},{last["omega"][1]:+.4f},{last["omega"][2]:+.4f}) '
          f'pos=({last["pos"][0]:+.2f},{last["pos"][1]:+.2f},{last["pos"][2]:+.2f})')


def cmd_step(args):
    sim = build_scene(args.name, args.mode, parse_S(args.S),
                      tuple(float(x) for x in args.pulse.split(',')) if args.pulse else None)
    sim.S['paused'] = bool(args.paused)
    print(f"# 逐步推进 n={args.n} dt={args.dt} mode={'vtol' if sim.S['vtolMode'] else 'cruise'} "
          f"sasMode={sim.S['sasMode']} useBtrue={sim.S['useBtrue']}")
    for i in range(args.n):
        sim.step(args.dt, controllers.apply)
        if i % args.print_every == 0 or i == args.n - 1:
            S, F = sim.S, sim.F
            dbg = S.get('dbg') or {}
            line = (f'[{i:5d}] t={S["time"]:.3f} '
                    f'quat=({S["quat"]["w"]:.4f},{S["quat"]["x"]:+.4f},{S["quat"]["y"]:+.4f},{S["quat"]["z"]:+.4f}) '
                    f'ω=({S["omega"]["x"]:+.4f},{S["omega"]["y"]:+.4f},{S["omega"]["z"]:+.4f}) '
                    f'dtAct={S["dtAct"]:+.4f} dfAct={S["dfAct"]:+.4f} dwAct={S["dwAct"]:+.4f} '
                    f'thr={S["thr"]:.4f} h={-F["pos"]["z"]:+.3f}')
            if dbg:
                line += (f' | qe=({dbg.get("qe_x", 0):+.4f},{dbg.get("qe_y", 0):+.4f},{dbg.get("qe_z", 0):+.4f}) '
                         f'ωdes=({dbg.get("wdy", 0):+.3f},{dbg.get("wdz", 0):+.3f})')
                if 'du' in dbg:
                    line += f' du=({dbg["du"][0]:+.4f},{dbg["du"][1]:+.4f},{dbg["du"][2]:+.4f})'
            print(line)


def cmd_compare(args):
    a = json.loads(Path(args.a).read_text(encoding='utf-8'))
    b = json.loads(Path(args.b).read_text(encoding='utf-8'))
    if len(a) != len(b):
        print(f'[FAIL] 点数不一致 {len(a)} vs {len(b)}')
        sys.exit(1)
    vec_fields = ['quat', 'omega', 'vel', 'pos']
    worst = (0.0, '', 0.0)
    for i in range(len(a)):
        for f in vec_fields:
            for k in range(len(a[i][f])):
                dv = abs(a[i][f][k] - b[i][f][k])
                if dv > worst[0]:
                    worst = (dv, f'{f}[{k}]', a[i]['t'])
        for f in ('thr', 'dtAct', 'dfAct', 'dwAct', 'intAlt', 'intTh', 'intPhi'):
            dv = abs(a[i][f] - b[i][f])
            if dv > worst[0]:
                worst = (dv, f, a[i]['t'])
    tol = args.tolerance
    ok = worst[0] < tol
    print(f'{"[PASS]" if ok else "[FAIL]"} 最大偏差 {worst[0]:.3e} '
          f'({worst[1]} @t={worst[2]}) 门槛 {tol:g}')
    sys.exit(0 if ok else 1)


# ------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description='py-web-lab CLI')
    sub = ap.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('state', help='初始状态快照')
    p.add_argument('--name', default='cruise_trim', choices=sorted(SCENES))
    p.add_argument('--mode', choices=['cruise', 'vtol'], default=None)
    p.set_defaults(fn=cmd_state)

    p = sub.add_parser('scene', help='跑场景 → JSON 轨迹')
    p.add_argument('--name', default='vtol', choices=sorted(SCENES))
    p.add_argument('--secs', type=float, default=None)
    p.add_argument('--out', default=None)
    p.add_argument('--S', default=None, help="'thr=0.5,dt=0.1'")
    p.add_argument('--pulse', default=None, help="'0,0.3,0' = x,y,z rad/s")
    p.add_argument('--mode', choices=['cruise', 'vtol'], default=None)
    p.add_argument('--paused', action='store_true')
    p.set_defaults(fn=cmd_scene)

    p = sub.add_parser('step', help='逐步推进（打印控制律中间量）')
    p.add_argument('--n', type=int, default=100)
    p.add_argument('--dt', type=float, default=DT)
    p.add_argument('--name', default='vtol', choices=sorted(SCENES))
    p.add_argument('--S', default=None)
    p.add_argument('--pulse', default=None)
    p.add_argument('--mode', choices=['cruise', 'vtol'], default=None)
    p.add_argument('--paused', action='store_true')
    p.add_argument('--print-every', type=int, default=25)
    p.set_defaults(fn=cmd_step)

    p = sub.add_parser('compare', help='轨迹对比')
    p.add_argument('a')
    p.add_argument('b')
    p.add_argument('--tolerance', type=float, default=1e-6)
    p.set_defaults(fn=cmd_compare)

    args = ap.parse_args()
    args.fn(args)


if __name__ == '__main__':
    main()
