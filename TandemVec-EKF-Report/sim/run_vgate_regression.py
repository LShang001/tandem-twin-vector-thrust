# -*- coding: utf-8 -*-
"""垂直重锚门控回归：正常/失联/风下 gate 0 vs 1（同二进制）。
@file    run_vgate_regression.py
@brief   垂直 spoof 门控回归。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 驱动门控开/关的常规场景 A/B。
@frame   N/A
@units   N/A
"""
import json, os, re, subprocess
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TAG = os.environ.get("GATEREG_TAG", "2026-10-26")
OUT = os.path.join(HERE, "..", "sim-data", f"gnss_vgate_reg_{TAG}.jsonl")
SUM = os.path.join(HERE, "..", "sim-data", f"gnss_vgate_reg_{TAG}.json")
EXE = os.environ.get("GATEREG_EXE", "/tmp/tv_fcs_vgate/fw_real.exe")
JSON_RE = re.compile(r"SIL_REAL_JSON (\{.*\})")
SEEDS = list(range(1, int(os.environ.get("GATEREG_SEEDS", "11"))))
SCEN = {
    "nominal": (40.0, []),
    "outage5": (40.0, ["--gnss-outage", "20:25"]),
    "outage20": (50.0, ["--gnss-outage", "20:40"]),
    "wind": (40.0, ["--wind-window", "20:30", "--wind-force", "1.0"]),
}


def one(gate, scen, seed):
    secs, extra = SCEN[scen]
    cmd = [EXE, "--real-nav", "--profile", "mag_clean", "--pos-hold",
           "--flow", "--gnss-v-gate", str(gate), "--seed", str(seed),
           "--seconds", str(secs)] + extra
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=240)
    mm = JSON_RE.findall(r.stdout + r.stderr)
    if not mm:
        return dict(gate=gate, scen=scen, seed=seed, no_json=True)
    d = json.loads(mm[-1].replace("nan", "NaN"))
    d.update(gate=gate, scen=scen, seed=seed)
    return d


def stats(v):
    v = np.array([x for x in v if isinstance(x, (int, float)) and np.isfinite(x)],
                 float)
    if not len(v):
        return dict(med=float("nan"), p90=float("nan"), mx=float("nan"))
    return dict(med=float(np.median(v)), p90=float(np.percentile(v, 90)),
                mx=float(v.max()))


def main():
    rows = []
    for gate in (0, 1):
        for scen in SCEN:
            for seed in SEEDS:
                d = one(gate, scen, seed)
                rows.append(d)
                if not d.get("no_json"):
                    print(f"g{gate} {scen:8s} s{seed:2d} alt={d['final_alt']:6.2f} "
                          f"pos={d['pos_rms']:.2f} hold={d['hold_end_rms']:.2f} "
                          f"roll={d['max_roll']:5.1f} land={d['land_t']:5.1f} "
                          f"rst={d['reacquire_resets']} vg={d['vgsus']}/{d['vgfr']}",
                          flush=True)
    with open(OUT, "w", encoding="utf-8") as f:
        for d in rows:
            f.write(json.dumps(d, ensure_ascii=False) + "\n")
    keys = ["final_alt", "pos_rms", "hold_end_rms", "max_roll", "land_t",
            "reacquire_resets"]
    summary = {}
    for gate in (0, 1):
        for scen in SCEN:
            rs = [d for d in rows if d["gate"] == gate and d["scen"] == scen
                  and not d.get("no_json")]
            summary[f"g{gate}_{scen}"] = {k: stats([d.get(k) for d in rs])
                                          for k in keys}
            summary[f"g{gate}_{scen}"]["n_landed"] = int(
                sum(1 for d in rs if d.get("land_t", -1) >= 0))
            summary[f"g{gate}_{scen}"]["vgsus_seen"] = int(
                sum(1 for d in rs if d.get("vgsus", 0) == 1))
            summary[f"g{gate}_{scen}"]["n"] = len(rs)
    json.dump(summary, open(SUM, "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    print("wrote", OUT)


if __name__ == "__main__":
    main()
