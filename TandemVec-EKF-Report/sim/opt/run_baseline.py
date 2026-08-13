# -*- coding: utf-8 -*-
"""
基线蒙特卡洛：固件现值参数下 8 类场景的统计性能 + NIS 一致性诊断
输出 sim-data/opt/baseline_mc.json
"""
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from opt_ekf import Scenario, build_cfg, build_p0, OptEkf

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                   "sim-data", "opt")
os.makedirs(OUT, exist_ok=True)

GNSS_DEF = dict(noise=(2.0, 3.0, 0.15, 0.25), hz=10, age=0.015)
AID_DEF = dict(enable=True, tier=True, zupt_adapt=True)


def base_scenarios():
    """返回场景定义：默认全部用固件现值参数"""
    return {
        "SC1_static_align": dict(
            dur=120.0,
            phases=[("static", 120.0, {})],
            ba=[0.08, -0.05, 0.12], bg=[0.003, -0.002, 0.004],
            gnss=dict(**GNSS_DEF, outage=(0, 120.0)),
            static_aid=dict(AID_DEF),
        ),
        "SC2_cruise_gnss": dict(
            dur=120.0,
            phases=[("static", 2.0, {}),
                    ("cruise", 118.0, dict(v=[5.0, 1.0, 0.0], accel_t=2.0))],
            ba=[0.08, -0.05, 0.12], bg=[0.003, -0.002, 0.004],
            gnss=GNSS_DEF,
            yaw_aid=dict(enable=True),
            static_aid=dict(AID_DEF),
        ),
        "SC3_cruise_turn": dict(
            dur=90.0,
            phases=[("static", 2.0, {}),
                    ("cruise", 30.0, dict(v=[5.0, 0.0, 0.0], accel_t=2.0)),
                    ("turn", 20.0, dict(yaw_rate=0.2)),
                    ("cruise", 38.0, dict(v=[5.0, 0.0, 0.0], accel_t=2.0))],
            ba=[0.08, -0.05, 0.12], bg=[0.003, -0.002, 0.004],
            gnss=GNSS_DEF,
            yaw_aid=dict(enable=True),
            static_aid=dict(AID_DEF),
        ),
        "SC4_transition": dict(
            dur=60.0,
            phases=[("static", 3.0, {}),
                    ("transition", 12.0, dict(p0=0.0, p1=90.0,
                                              a=[1.0, 0.0, -0.2],
                                              v_out=[12.0, 0.0, 0.0])),
                    ("hover", 45.0, dict(disturb=None, decel_t=6.0))],
            ba=[0.08, -0.05, 0.12], bg=[0.003, -0.002, 0.004],
            gnss=GNSS_DEF,
            yaw_aid=dict(enable=True),
            static_aid=dict(AID_DEF),
        ),
        "SC5_outage_reacq": dict(
            dur=120.0,
            phases=[("static", 2.0, {}),
                    ("cruise", 118.0, dict(v=[5.0, 0.0, 0.0], accel_t=2.0))],
            ba=[0.08, -0.05, 0.12], bg=[0.003, -0.002, 0.004],
            gnss=dict(**GNSS_DEF, outage=(30.0, 80.0),
                      multipath=(80.0, 95.0, 2.5)),
            ahrs_fallback=dict(enable=True, gyro_thr=0.5, noise=0.08),
            static_aid=dict(AID_DEF),
        ),
        "SC6_outliers": dict(
            dur=90.0,
            phases=[("static", 2.0, {}),
                    ("cruise", 88.0, dict(v=[5.0, 0.0, 0.0], accel_t=2.0))],
            ba=[0.08, -0.05, 0.12], bg=[0.003, -0.002, 0.004],
            gnss=dict(**GNSS_DEF, outliers=[(20.0, 0.2, 30.0, 0.0, 0.0),
                                            (40.0, 0.2, 0.0, -25.0, 0.0),
                                            (60.0, 0.2, 0.0, 0.0, -40.0)]),
            static_aid=dict(AID_DEF),
        ),
        "SC7_hover_vib": dict(
            dur=60.0,
            phases=[("hover", 60.0, dict(disturb=None))],
            ba=[0.08, -0.05, 0.12], bg=[0.003, -0.002, 0.004],
            gnss=GNSS_DEF,
            vib_amp=0.03,
            yaw_aid=dict(enable=True),
            static_aid=dict(AID_DEF),
            init_hover=True,
        ),
        "SC8_bias_jump": dict(
            dur=60.0,
            phases=[("static", 2.0, {}),
                    ("cruise", 58.0, dict(v=[5.0, 0.0, 0.0], accel_t=2.0))],
            ba=[0.08, -0.05, 0.12], bg=[0.003, -0.002, 0.004],
            gnss=GNSS_DEF,
            bias_jumps=[dict(t=30.0, ba=[0.8, 0.0, 0.0])],
            static_aid=dict(AID_DEF),
        ),
    }


def run_one(name, spec, seed):
    c = build_cfg()
    p0 = build_p0()

    def make_ekf():
        return OptEkf(c)

    sc = Scenario(spec, make_ekf)
    sc.run(seed=seed)
    m = sc.metrics()
    return m


def mc(scenarios, seeds, only=None):
    results = {}
    t_all = time.time()
    for name, spec in scenarios.items():
        if only and name not in only:
            continue
        t0 = time.time()
        rows = []
        for seed in seeds:
            try:
                m = run_one(name, spec, seed)
                rows.append(m)
            except Exception as ex:
                print(f"  [{name}] seed={seed} FAILED: {ex!r}")
        keys = sorted({k for r in rows for k in r.keys()})
        agg = {}
        for k in keys:
            vals = [r[k] for r in rows if k in r]
            agg[k] = dict(mean=float(np.mean(vals)),
                          std=float(np.std(vals)),
                          min=float(np.min(vals)),
                          max=float(np.max(vals)),
                          n=int(len(vals)))
        results[name] = dict(n=len(rows), metrics=agg)
        print(f"[{name}] {len(rows)} seeds, {time.time()-t0:.1f}s "
              f"| pos_ne_rmse={agg['pos_ne_rmse']['mean']:.3f}±{agg['pos_ne_rmse']['std']:.3f}")
    print(f"[总耗时] {time.time()-t_all:.1f}s")
    return results


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--only", default=None)
    ap.add_argument("--out", default=os.path.join(OUT, "baseline_mc.json"))
    args = ap.parse_args()
    seeds = list(range(1, args.seeds + 1))
    res = mc(base_scenarios(), seeds, only=args.only)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=2)
    print(f"\n[完成] 写入 {args.out}")
