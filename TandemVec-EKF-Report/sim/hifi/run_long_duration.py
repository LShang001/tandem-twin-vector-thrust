#!/usr/bin/env python3
"""run_long_duration.py — 长航时退化仿真（30 min 试点，L1）。

@file    run_long_duration.py
@module  analysis/长航时与慢变退化（hifi 6DOF + 固件控制/EKF）
@brief   跑 30 min 定高+位置保持+磁矢量+光流，叠加 GNSS/气压慢漂、阵风、
         60 s GNSS 失联，统计分段位置/姿态/高度误差与漂移趋势。
@details 输入：--seconds（默认 1800）、--seeds；输出 long-duration.json。
@theory  N/A（长航时退化观测；非理论推导）。
@frame   NED + FRD；姿态 deg、位置 m。
@units   SI。
@note    L1 hifi：无气动/电池/温漂模型，慢漂是人工注入；不能替代长航时实测。
@see     run_full_envelope.py、run_param_mc.py、FULL-ENVELOPE-STATUS.md
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "opt"))
sys.path.insert(0, str(HERE))

import run_scenes_opt as rs  # noqa: E402
from opt_ekf import build_cfg  # noqa: E402


def metrics(r: dict, t0: float, t1: float) -> dict:
    """统计窗口内位置/姿态/高度 RMSE 与最大值。"""
    m = (r["t"] >= t0) & (r["t"] <= t1)
    pe = r["pos_err"][m]
    return {
        "pos_ne_rmse": float(np.sqrt(np.mean(pe[:, 0] ** 2 + pe[:, 1] ** 2))),
        "pos_ne_max": float(np.max(np.sqrt(pe[:, 0] ** 2 + pe[:, 1] ** 2))),
        "att_deg_rmse": float(np.sqrt(np.mean(r["att_qerr"][m] ** 2)) * rs.R2D),
        "att_deg_max": float(np.max(r["att_qerr"][m]) * rs.R2D),
        "alt_err_rmse": float(np.sqrt(np.mean((r["ealt"][m] - r["alt"][m]) ** 2))),
        "alt_err_max": float(np.max(np.abs(r["ealt"][m] - r["alt"][m]))),
    }


def run_seed(seed: int, seconds: float) -> dict:
    """跑一个种子的长航时场景，返回全段与 6 个分段的指标。"""
    cfgd = build_cfg({})
    nav = rs.NavV(cfgd, {}, p0_hdg=0.2, static_gate=True)
    scene = {
        "duration": seconds,
        "fault": {
            "gps_drift": {"t": seconds / 3.0, "rate": [0.02, 0.01, 0.0]},
            "baro_drift": {"t": seconds / 3.0, "rate_mps": 0.005},
            "gps_lost": (seconds * 2.0 / 3.0, seconds * 2.0 / 3.0 + 60.0),
            "wind_gust": (seconds / 2.0, seconds / 2.0 + 60.0),
        },
        "ref": {"alt_hold": 10.0, "pos_hold": True},
        "mag_vector": True,
        "flow_aiding": True,
        "baro_hz": 2.0,
        "baro_phase": 2,
    }
    r = rs.run_scene(scene, seconds, lambda: nav, seed=seed, seed_dyn=seed)
    bins = {}
    for i in range(6):
        t0 = seconds * i / 6.0
        t1 = seconds * (i + 1) / 6.0
        bins[f"bin{i}"] = metrics(r, t0, t1)
    return {"seed": seed, "all": metrics(r, 0.0, seconds), "bins": bins}


def main() -> int:
    """CLI 入口：逐 seed 跑长航时并写 JSON。"""
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=1800.0)
    ap.add_argument("--seeds", default="1,2")
    ap.add_argument("--out", default=str(HERE.parent.parent / "sim-data" /
                                          "campaign-2026-10-24" / "long-duration.json"))
    args = ap.parse_args()
    out = {"seconds": args.seconds, "seeds": {}, "note":
           "L1 hifi; injected GPS/baro drift, gust, 60s GNSS outage; not HIL/real"}
    for seed in [int(s) for s in args.seeds.split(",") if s]:
        print(f"[run] seed={seed} seconds={args.seconds}", flush=True)
        out["seeds"][str(seed)] = run_seed(seed, args.seconds)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, ensure_ascii=False, indent=2),
                              encoding="utf-8")
    print(json.dumps(out, ensure_ascii=False, indent=2)[:2000])
    print("out:", args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
