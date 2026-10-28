# -*- coding: utf-8 -*-
"""SIL：气压健康门控 A/B（GNSS 垂直重锚仲裁配套）。
@file    run_baro_health_ab.py
@brief   同一二进制 `--baro-health-gate 0|1` 的气压故障 A/B 驱动。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 场景臂：
         nominal         正常（回归，期望逐项相同）
         drift0.5        气压 0.5 m/s 漂移（30 m/min）
         spoofz100       GNSS 垂直 +100 m 台阶（气压真实）
         drift+spoof     气压漂移 + GNSS 垂直 spoof（无仲裁真值）
         Outage+drift    GNSS 20 s 失联 + 气压漂移（无参照，健康监测暂停）
         指标：final_alt/pos_max/hold_max/max_roll/bh_skips。
@frame   N/A
@units   m、m/s、deg
"""
import argparse
import json
import os
import re
import subprocess
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
JSON_RE = re.compile(r"SIL_REAL_JSON (\{.*\})")
# Windows Python 不认 Git Bash 的 /tmp；用系统临时目录拼 Windows 路径。
DEFAULT_EXE = os.path.join(tempfile.gettempdir(), "tv_bh", "fw_real.exe")
TAG = "2026-10-27"

ARMS = {
    "nominal": [],
    "drift0.5": ["--baro-drift", "0.5"],
    "spoofz100": ["--gnss-spoof-z", "100"],
    "drift+spoof": ["--baro-drift", "0.5", "--gnss-spoof-z", "100"],
    "outage+drift": ["--baro-drift", "0.5", "--gnss-outage", "20:40"],
}


def one(exe, arm, gate, seed, seconds, core):
    cmd = [exe, "--real-nav", "--profile", "mag_clean", "--pos-hold",
           "--flow", "--gnss-spoof", "0", "--gnss-spoof-start", "25",
           "--gnss-v-gate", "1", "--baro-health-gate", "1" if gate else "0",
           "--seed", str(seed), "--seconds", f"{seconds:g}"]
    cmd += ARMS[arm]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, text=True)
    if core >= 0:
        try:
            import psutil
            psutil.Process(proc.pid).cpu_affinity([core])
        except Exception:
            pass
    try:
        out, err = proc.communicate(timeout=240)
    except subprocess.TimeoutExpired:
        proc.kill()
        out, err = proc.communicate()
    mm = JSON_RE.findall(out + err)
    if not mm:
        return dict(arm=arm, gate=gate, seed=seed, no_json=True)
    d = json.loads(mm[-1].replace("nan", "NaN"))
    d.update(arm=arm, gate=gate, seed=seed)
    return d


def stats(vals):
    v = np.array([x for x in vals
                  if isinstance(x, (int, float)) and np.isfinite(x)], float)
    if not len(v):
        return dict(med=float("nan"), p95=float("nan"),
                    mx=float("nan"), n=0)
    return dict(med=float(np.median(v)), p95=float(np.percentile(v, 95)),
                mx=float(v.max()), n=int(v.size))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", default=os.environ.get("BH_EXE", DEFAULT_EXE))
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--seed-start", type=int, default=1)
    ap.add_argument("--seconds", type=float, default=50.0)
    ap.add_argument("--arms", default=",".join(ARMS.keys()))
    ap.add_argument("--tag", default=TAG)
    ap.add_argument("--core", type=int, default=2,
                    help="SIL 绑定的逻辑核（-1=不绑定）")
    args = ap.parse_args()

    out_jsonl = os.path.join(HERE, "..", "sim-data",
                             f"baro_health_ab_{args.tag}.jsonl")
    rows = []
    with open(out_jsonl, "w", encoding="utf-8") as f:
        for arm in args.arms.split(","):
            for gate in (0, 1):
                for seed in range(args.seed_start,
                                  args.seed_start + args.seeds):
                    d = one(args.exe, arm, gate, seed, args.seconds,
                            args.core)
                    f.write(json.dumps(d, ensure_ascii=False) + "\n")
                    f.flush()
                    rows.append(d)
                    print(f"{arm:14s} gate={gate} seed={seed:3d} "
                          f"alt={d.get('final_alt', float('nan')):7.2f} "
                          f"pos={d.get('pos_max', float('nan')):8.2f} "
                          f"roll={d.get('max_roll', float('nan')):6.1f} "
                          f"bh={d.get('bh_sus', -1)}/{d.get('bh_skips', -1)}",
                          flush=True)

    summary = {}
    for arm in args.arms.split(","):
        summary[arm] = {}
        for gate in (0, 1):
            rs = [r for r in rows
                  if r.get("arm") == arm and r.get("gate") == gate]
            summary[arm][f"gate{gate}"] = dict(
                n=len(rs),
                final_alt=stats([r.get("final_alt") for r in rs]),
                pos_max=stats([r.get("pos_max") for r in rs]),
                hold_max=stats([r.get("hold_max") for r in rs]),
                hold_end=stats([r.get("hold_end_rms") for r in rs]),
                max_roll=stats([r.get("max_roll") for r in rs]),
                landed=sum(1 for r in rs
                           if isinstance(r.get("final_alt"), (int, float))
                           and r["final_alt"] < 0.5),
                bh_skips=stats([r.get("bh_skips") for r in rs]),
                bh_sus=sum(1 for r in rs if r.get("bh_sus") == 1),
                vgsus=sum(1 for r in rs if r.get("vgsus") == 1),
            )
    out_sum = os.path.join(HERE, "..", "sim-data",
                           f"baro_health_ab_{args.tag}.json")
    with open(out_sum, "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
