# -*- coding: utf-8 -*-
"""32 g 量程 vs 16 g/8 g 的高过载包线试点（宿主 EKF 台架）。
@file    run_highg_fs.py
@brief   协调转弯 6.4–25 g × IMU 量程 8/16/32 g，3 seeds。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 调用 test_ekf_high_dynamic.exe（纯 Ekf15State，无健康/Q 膨胀），
         评估“换 32 g 量程能买到多少包线”，不是固件端到端 SIL。
"""
import argparse
import json
import os
import re
import subprocess
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "sim-data")
JSON_RE = re.compile(r"HIGH_DYN_JSON (\{.*\})")
DEFAULT_EXE = os.path.join(tempfile.gettempdir(), "host_bin_v4", "highdyn.exe")
LOADS = [6.4, 8.0, 12.0, 15.0, 20.0, 25.0]
FS = [8.0, 16.0, 32.0]


def one(exe, load, fs, seed, speed, duration):
    cmd = [exe, "--profile", "turn", "--load", f"{load:g}",
           "--speed", f"{speed:g}", "--duration", f"{duration:g}",
           "--accel-fs-g", f"{fs:g}", "--seed", str(seed), "--json"]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    mm = JSON_RE.findall(r.stdout + r.stderr)
    if not mm:
        return dict(load=load, fs_g=fs, seed=seed, no_json=True)
    d = json.loads(mm[-1])
    d.update(fs_g=fs)
    return d


def stat(v):
    v = np.array([x for x in v if isinstance(x, (int, float))
                  and np.isfinite(x)], float)
    if not len(v):
        return float("nan")
    return float(np.median(v))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", default=DEFAULT_EXE)
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--speed", type=float, default=300.0)
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--tag", default="2026-10-27")
    args = ap.parse_args()

    rows = []
    out_jsonl = os.path.join(DATA, f"highg_fs_{args.tag}.jsonl")
    with open(out_jsonl, "w", encoding="utf-8") as f:
        for load in LOADS:
            for fs in FS:
                for seed in range(1, args.seeds + 1):
                    d = one(args.exe, load, fs, seed, args.speed,
                            args.duration)
                    f.write(json.dumps(d, ensure_ascii=False) + "\n")
                    f.flush()
                    rows.append(d)
                    print(f"{load:5.1f} g fs={fs:4.0f} seed={seed} "
                          f"pos={d.get('pos_rmse', float('nan')):8.3f} "
                          f"att={d.get('att_rmse_deg', float('nan')):6.2f} "
                          f"rej={d.get('rejected', -1)} fail={d.get('fail', -1)}",
                          flush=True)

    summary = {}
    for load in LOADS:
        for fs in FS:
            rs = [r for r in rows
                  if r.get("load") == load and r.get("fs_g") == fs]
            summary[f"{load:g}g_fs{fs:g}"] = dict(
                n=len(rs),
                pos_rmse=stat([r.get("pos_rmse") for r in rs]),
                att_rmse=stat([r.get("att_rmse_deg") for r in rs]),
                att_max=max((r.get("att_max_deg", float("-inf")) for r in rs),
                            default=float("nan")),
                rejected=stat([r.get("rejected") for r in rs]),
                fail=stat([r.get("fail") for r in rs]),
            )
    out = os.path.join(DATA, f"highg_fs_{args.tag}.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
