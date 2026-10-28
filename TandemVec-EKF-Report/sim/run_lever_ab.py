# -*- coding: utf-8 -*-
"""GNSS 杆臂补偿 A/B：0=不补偿 1=仅位置预补偿 2=EKF 完整杆臂补偿。
@file    run_lever_ab.py
@brief   GNSS 杆臂补偿 A/B 矩阵。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 同种子对比补偿/不补偿的位置/速度误差。
@frame   N/A
@units   N/A
"""
import concurrent.futures as cf
import json, os, re, subprocess
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "sim-data", "high-dyn-lever-ab-2026-10-26.jsonl")
SUM = os.path.join(HERE, "..", "sim-data", "high-dyn-lever-ab-2026-10-26.json")
EXE = os.environ.get("HIGHDYN_EXE", "/tmp/highdyn.exe")
JSON_RE = re.compile(r"HIGH_DYN_JSON (\{.*\})")
SEEDS = list(range(1, 11))
CASES = [
    ("turn", 8.0, 0.0, 2.0),
    ("sturn", 6.0, 0.0, 2.0),
    ("pull", 6.0, 0.0, 2.0),
    ("roll", 2.0, 720.0, 1.0),
    ("coning", 1.0, 600.0, 1.0),
]


def one(args):
    prof, load, rate, lever, mode, seed = args
    cmd = [EXE, "--profile", prof, "--load", str(load), "--speed", "80",
           "--rate-dps", str(rate), "--duration", "30", "--latency-ms", "20",
           "--lever-m", str(lever), "--lever-comp", str(mode),
           "--seed", str(seed), "--json"]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    mm = JSON_RE.findall(r.stdout + r.stderr)
    d = json.loads(mm[-1]) if mm else {"no_json": True}
    d.update(profile=prof, load=load, rate=rate, lever=lever, mode=mode, seed=seed)
    return d


def stat(v):
    v = np.array([x for x in v if isinstance(x, (int, float)) and np.isfinite(x)], float)
    if not len(v):
        return {}
    return dict(n=int(len(v)), med=float(np.median(v)), p90=float(np.percentile(v, 90)),
                mx=float(v.max()))


def main():
    tasks = [(p, l, r, lv, m, s) for p, l, r, lv in CASES
             for m in (0, 1, 2) for s in SEEDS]
    with cf.ProcessPoolExecutor(max_workers=10) as ex:
        rows = list(ex.map(one, tasks, chunksize=2))
    with open(OUT, "w", encoding="utf-8") as f:
        for d in rows:
            f.write(json.dumps(d, ensure_ascii=False) + "\n")
    out = {}
    for p, l, r, lv in CASES:
        key = f"{p}_L{l:g}_R{r:g}_lev{lv:g}"
        out[key] = {}
        for m in (0, 1, 2):
            rs = [x for x in rows if x["profile"] == p and x["load"] == l and
                  x["rate"] == r and x["lever"] == lv and x["mode"] == m and
                  not x.get("no_json")]
            out[key][f"mode{m}"] = dict(
                pos_rmse=stat([x["pos_rmse"] for x in rs]),
                vel_rmse=stat([x["vel_rmse"] for x in rs]),
                att_rmse=stat([x["att_rmse_deg"] for x in rs]),
                nees_pos=stat([x["nees_pos_med"] for x in rs]),
                nees_vel=stat([x["nees_vel_med"] for x in rs]))
    json.dump(out, open(SUM, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    for k, v in out.items():
        print(k)
        for m in (0, 1, 2):
            q = v[f"mode{m}"]
            print(f"  mode{m} pos {q['pos_rmse'].get('med',0):6.2f}/"
                  f"{q['pos_rmse'].get('mx',0):7.2f} vel {q['vel_rmse'].get('med',0):5.2f} "
                  f"NEESp {q['nees_pos'].get('med',0):6.2f} NEESv {q['nees_vel'].get('med',0):7.2f}")
    print("wrote", OUT)


if __name__ == "__main__":
    main()
