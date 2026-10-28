# -*- coding: utf-8 -*-
"""SIL：放大 GNSS 自报精度（R）能否改善有色误差下的真值误差与 NEES。
@file    run_nees_r_scale.py
@brief   SIL R 缩放一致性扫描。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 驱动 --gnss-r-scale 并统计 NEES/保持误差。
@frame   N/A
@units   N/A

只改 UBX h_acc/v_acc/s_acc，不改实际噪声；1.0 为固件当前口径。
"""
import json, os, re, subprocess
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TAG = os.environ.get("RSCALE_TAG", "2026-10-26")
OUT = os.path.join(HERE, "..", "sim-data", f"nees_r_scale_{TAG}.jsonl")
SUM = os.path.join(HERE, "..", "sim-data", f"nees_r_scale_{TAG}.json")
EXE = os.environ.get("NEES_EXE", "/tmp/tv_fcs_nees/fw_real.exe")
JSON_RE = re.compile(r"SIL_REAL_JSON (\{.*\})")
SEEDS = list(range(1, int(os.environ.get("RSCALE_SEEDS", "21"))))
SCALES = [1.0, 1.5, 2.0, 3.0]


def one(scale, seed):
    cmd = [EXE, "--real-nav", "--profile", "mag_clean", "--pos-hold",
           "--flow", "--seed", str(seed), "--seconds", "40",
           "--gnss-r-scale", f"{scale:g}"]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    mm = JSON_RE.findall(r.stdout + r.stderr)
    if not mm:
        return dict(scale=scale, seed=seed, no_json=True)
    d = json.loads(mm[-1].replace("nan", "NaN"))
    d.update(scale=scale, seed=seed)
    return d


def stats(v):
    v = np.array([x for x in v if isinstance(x, (int, float)) and np.isfinite(x)],
                 float)
    if not len(v):
        return dict(med=float("nan"), p90=float("nan"), max=float("nan"))
    return dict(med=float(np.median(v)), p90=float(np.percentile(v, 90)),
                max=float(v.max()))


def main():
    rows = []
    for scale in SCALES:
        for seed in SEEDS:
            d = one(scale, seed)
            rows.append(d)
            if not d.get("no_json"):
                print(f"R x{scale:<4g} s{seed:2d} nees={d['nees_pos_med']:6.2f} "
                      f"p95={d['nees_pos_p95']:6.2f} rmse={d['pos_rms']:.2f} "
                      f"hold={d['hold_end_rms']:.2f} qf={d['quality_fail']}",
                      flush=True)
    with open(OUT, "w", encoding="utf-8") as f:
        for d in rows:
            f.write(json.dumps(d, ensure_ascii=False) + "\n")
    keys = ["nees_pos_med", "nees_pos_p95", "pos_rms", "hold_end_rms",
            "max_roll", "quality_fail"]
    summary = {}
    for scale in SCALES:
        rs = [d for d in rows if d["scale"] == scale and not d.get("no_json")]
        summary[f"{scale:g}"] = {k: stats([d.get(k) for d in rs]) for k in keys}
        summary[f"{scale:g}"]["n"] = len(rs)
    json.dump(summary, open(SUM, "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    print("wrote", OUT)


if __name__ == "__main__":
    main()
