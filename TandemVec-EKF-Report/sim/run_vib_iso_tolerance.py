# -*- coding: utf-8 -*-
"""减震座参数容差网格：fn × ζ × 振动量级。

@file    run_vib_iso_tolerance.py
@brief  评估减震座制造/温度引起的 fn/ζ 偏差对导航姿态的影响。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 2 g/120 Hz/8.4 g 与 1 g/80 Hz/6.4 g，5 seeds/格；
         输出 sim-data/vib-iso-tol-<tag>.{json,jsonl}。
@frame   NED
@units   Hz、无量纲、g、°
"""
import argparse
import concurrent.futures as cf
import json
import os
import re
import subprocess

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
JSON_RE = re.compile(r"SIL_REAL_JSON (\{.*\})")
BASE = ["--real-nav", "--missile", "--seconds", "45", "--profile",
        "mag_clean", "--imu-grade", "icm"]
FNS = [15.0, 20.0, 25.0, 30.0, 35.0]
ZETAS = [0.05, 0.10, 0.20]
SEEDS = 5


def resolve_bin(path):
    if os.path.exists(path):
        return path
    if path.startswith("/tmp/"):
        alt = os.path.join(os.environ.get("TEMP", ""), path[5:])
        if os.path.exists(alt):
            return alt
    return path


def one(job):
    name, exe, seed, vib_g, vib_hz, fn, zeta, thrust = job
    cmd = [exe] + BASE + ["--seed", str(seed), "--vib", f"{vib_g:.2f}",
                          "--vib-hz", f"{vib_hz:.1f}", "--vib-start", "1.5",
                          "--vib-iso-fn", f"{fn:.1f}",
                          "--vib-iso-zeta", f"{zeta:.2f}"]
    if thrust:
        cmd += ["--ms-thrust", str(thrust)]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
    except subprocess.TimeoutExpired:
        return dict(config=name, seed=seed, no_json=True, timeout=True)
    mm = JSON_RE.findall(r.stdout + r.stderr)
    if not mm:
        return dict(config=name, seed=seed, no_json=True)
    d = json.loads(mm[-1].replace("nan", "NaN").replace("inf", "Infinity"))
    d.update(config=name, seed=seed, iso_fn=fn, iso_zeta=zeta,
             vib_g=vib_g, vib_hz=vib_hz)
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", default="/tmp/tv_fcs_missile_prod_fresh_iso/fw_real.exe")
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--tag", default="2026-10-27")
    args = ap.parse_args()
    exe = resolve_bin(args.bin)
    jobs = []
    for fn in FNS:
        for z in ZETAS:
            for s in range(1, SEEDS + 1):
                jobs.append((f"2g120_fn{fn:02.0f}_z{z:0.2f}".replace(".", "p"),
                             exe, s, 2.0, 120.0, fn, z, 8000))
                jobs.append((f"1g80_fn{fn:02.0f}_z{z:0.2f}".replace(".", "p"),
                             exe, s, 1.0, 80.0, fn, z, 0))
    print("jobs", len(jobs), flush=True)
    rows = []
    with cf.ProcessPoolExecutor(max_workers=args.workers) as pool:
        for r in pool.map(one, jobs, chunksize=2):
            rows.append(r)
    out = dict(tag=args.tag, bin=exe, seeds=SEEDS, n_jobs=len(rows),
               n_ok=sum(1 for r in rows if not r.get("no_json")), groups={})
    for r in rows:
        if r.get("no_json"):
            continue
        g = out["groups"].setdefault(r["config"], [])
        g.append(r)
    summary = {}
    for name, rs in out["groups"].items():
        att = [r["nav_att_rmse"] for r in rs if r.get("nav_att_rmse", -1) >= 0]
        pos = [r["pos_rms"] for r in rs if r.get("pos_rms", -1) >= 0]
        vel = [r["nav_vel_rms"] for r in rs if r.get("nav_vel_rms", -1) >= 0]
        summary[name] = dict(
            n=len(rs),
            att_med=float(np.median(att)) if att else -1,
            att_max=float(np.max(att)) if att else -1,
            pos_med=float(np.median(pos)) if pos else -1,
            vel_med=float(np.median(vel)) if vel else -1)
    out["summary"] = summary
    d = os.path.join(HERE, "..", "sim-data")
    with open(os.path.join(d, f"vib-iso-tol-{args.tag}.jsonl"), "w",
              encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
    with open(os.path.join(d, f"vib-iso-tol-{args.tag}.json"), "w",
              encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    print("out", os.path.join(d, f"vib-iso-tol-{args.tag}.json"))


if __name__ == "__main__":
    main()
