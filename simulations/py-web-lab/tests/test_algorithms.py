# -*- coding: utf-8 -*-
"""控制算法回归：INDI/LQR/ADRC 收敛断言（5°/s 扰动，8s 内 |ω|<0.05 rad/s）。

运行：cd simulations/py-web-lab && python tests/test_algorithms.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'server'))
from sim import Simulation  # noqa: E402
import controllers          # noqa: E402

DT = 0.004
D2R = 0.017453292519943295

CASES = [
    # (名称, 构建, ctrl, 扰动, 断言)
    ('INDI 巡航 5°/s', lambda s: s.reset_full(), 'indi', (0.0, 5 * D2R, 0.0)),
    ('LQR 悬停 5°/s', lambda s: s.reset_vtol(), 'lqr', (0.0, 5 * D2R, 0.0)),
    ('ADRC 悬停 5°/s', lambda s: s.reset_vtol(), 'adrc', (0.0, 5 * D2R, 0.0),
     None, None, (0.5, lambda w: abs(w['y']) < 0.12)),   # 带宽断言：0.5s 内衰减过半（防 b0/ESO 回归）
    ('ADRC 悬停滚转 5°/s', lambda s: s.reset_vtol(), 'adrc', (5 * D2R, 0.0, 0.0)),
    ('LQR 悬停航向指令 30°/s', lambda s: s.reset_vtol(), 'lqr', None,
     {'dw': 30 * D2R}, {'x': 30 * D2R, 'y': 0.0, 'z': 0.0}),
    ('INDI 俯仰姿态指令 5°', lambda s: s.reset_full(), 'indi', None, {'dt': 5 * D2R}),
]


def run_case(name, build, ctrl, pulse, extra=None, expect_w=None, early_check=None):
    """expect_w: 期望末点角速度（None=归零收敛；dict 可覆盖单通道）
    early_check: (t, 条件) 早期衰减断言（防 ESO/带宽降级类回归）"""
    sim = Simulation()
    build(sim)
    sim.S['ctrl'] = ctrl
    if extra:
        for k, v in extra.items():
            sim.S[k] = v
    if pulse:
        sim.S['omega'] = {'x': pulse[0], 'y': pulse[1], 'z': pulse[2]}
    secs = 8.0
    steps = int(secs / DT)
    for i in range(steps):
        sim.step(DT, controllers.apply)
        if i > steps * 0.5 and max(abs(sim.S['omega'][k]) for k in ('x', 'y', 'z')) > 5.0:
            return False, f'发散（t={i * DT:.1f}s |ω|={max(abs(sim.S["omega"][k]) for k in ("x", "y", "z")):.2f}）'
        if early_check is not None and i * DT >= early_check[0] and not early_check[1](sim.S['omega']):
            return False, f'早期衰减未达标（t={i * DT:.3f}s ω={sim.S["omega"]}）'
    w = sim.S['omega']
    if expect_w is None:
        peak = max(abs(w[k]) for k in ('x', 'y', 'z'))
        ok = peak < 0.05
        return ok, f'末点 |ω|max={peak:.4f} rad/s, quat=({sim.S["quat"]["w"]:.3f},{sim.S["quat"]["y"]:+.3f})'
    ok = True
    parts = []
    for k, target in expect_w.items():
        dv = abs(w[k] - target)
        ok = ok and dv < 0.05
        parts.append(f'ω.{k}={w[k]:+.4f}(目标{target:+.4f},差{dv:.4f})')
    return ok, ' '.join(parts)


def main():
    fails = 0
    for case in CASES:
        name, build, ctrl, pulse = case[0], case[1], case[2], case[3]
        extra = case[4] if len(case) > 4 else None
        expect_w = case[5] if len(case) > 5 else None
        early = case[6] if len(case) > 6 else None
        ok, msg = run_case(name, build, ctrl, pulse, extra, expect_w, early)
        print(f'{"[PASS]" if ok else "[FAIL]"} {name}: {msg}')
        fails += 0 if ok else 1
    print(f'\n{len(CASES) - fails}/{len(CASES)} 通过')
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
