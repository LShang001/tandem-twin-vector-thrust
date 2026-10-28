# -*- coding: utf-8 -*-
"""SIL 在飞 NEES：有色 vs 白 GNSS 误差同二进制 A/B。
@file    run_nees_sil_ab.py
@brief   SIL 在飞 NEES 白噪声/有色 A/B。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 驱动 EKF_NAV_TELEMETRY 二进制的噪声臂。
@frame   N/A
@units   N/A

用法：NEES_EXE=/tmp/tv_fcs_nees/fw_real.exe py -3.12 sim/run_nees_sil_ab.py
"""
import json, os, re, subprocess, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TAG = os.environ.get("NEES_TAG", "2026-10-26")
OUT = os.path.join(HERE, "..", "sim-data", f"nees_sil_ab_{TAG}.jsonl")
SUM = os.path.join(HERE, "..", "sim-data", f"nees_sil_ab_{TAG}.json")
EXE = os.environ.get("NEES_EXE", "/tmp/tv_fcs_nees/fw_real.exe")
JSON_RE = re.compile(r"SIL_REAL_JSON (\{.*\})")
SEEDS = list(range(1, int(os.environ.get("NEES_SEEDS", "11"))))
SECONDS = 40.0
ARMS = {
    "colored": [],
    "white": ["--gnss-white"],
}


def one(arm, seed):
    cmd = [EXE, "--real-nav", "--profile", "mag_clean", "--pos-hold",
           "--flow", "--seed", str(seed), "--seconds", str(SECONDS)]
    cmd += ARMS[arm]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    text = r.stdout + r.stderr
    mm = JSON_RE.findall(text)
    if not mm:
        return dict(arm=arm, seed=seed, no_json=True)
    d = json.loads(mm[-1].replace("nan", "NaN"))
    d.update(arm=arm, seed=seed)
    return d


def med(vals):
    vals = [v for v in vals if isinstance(v, (int, float)) and np.isfinite(v)]
    return float(np.median(vals)) if vals else float("nan")


def main():
    rows = []
    for arm in ARMS:
        for seed in SEEDS:
            d = one(arm, seed)
            rows.append(d)
            if d.get("no_json"):
                print("no json", arm, seed, flush=True)
            else:
                print(f"{arm:8s} s{seed:2d} nees_p={d['nees_pos_med']:7.2f} "
                      f"p95={d['nees_pos_p95']:7.2f} pos_rms={d['pos_rms']:.2f} "
                      f"hold={d['hold_end_rms']:.2f}", flush=True)
    with open(OUT, "w", encoding="utf-8") as f:
        for d in rows:
            f.write(json.dumps(d, ensure_ascii=False) + "\n")
    keys = ["nees_pos_med", "nees_pos_p95", "nees_pos_max",
            "nees_vel_med", "nees_vel_p95", "pos_rms", "hold_end_rms",
            "max_roll"]
    summary = {}
    for arm in ARMS:
        rs = [d for d in rows if d["arm"] == arm and not d.get("no_json")]
        summary[arm] = {k + "_median": med([d.get(k) for d in rs]) for k in keys}
        summary[arm]["n"] = len(rs)
    summary["paired"] = {
        "nees_pos_med_median": [med([d["nees_pos_med"] for d in rows
                                      if d["arm"] == "colored"]),
                                 med([d["nees_pos_med"] for d in rows
                                      if d["arm"] == "white"])],
    }
    json.dump(summary, open(SUM, "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    print("wrote", OUT)


if __name__ == "__main__":
    main()
