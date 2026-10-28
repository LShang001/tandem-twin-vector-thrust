# -*- coding: utf-8 -*-
"""50-seed 定向验证：正过载基线、杆臂 A/B、20 s 失联 IMU 档、饱和边界。
@file    run_high_dynamic_validate.py
@brief   大过载/大机动 EKF 宿主台架 50-seed 验证。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 读取矩阵结果并做分组统计与可视化输入。
@frame   N/A
@units   N/A
"""
import concurrent.futures as cf
import json, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from run_high_dynamic_mc import case, one, stat  # noqa: E402

TAG = os.environ.get("HIGHDYN_TAG", "validate-2026-10-26")
OUT = os.path.join(HERE, "..", "sim-data", f"high-dyn-{TAG}.jsonl")
SUM = os.path.join(HERE, "..", "sim-data", f"high-dyn-{TAG}.json")
SEEDS = list(range(1, int(os.environ.get("HIGHDYN_SEEDS", "51"))))


def build():
    cs = []
    # 正过载基线
    for prof in ("turn", "sturn", "pull"):
        for load in (2.0, 4.0, 6.0, 8.0):
            for speed in (80.0, 150.0):
                for seed in SEEDS:
                    cs.append(dict(case(prof, load, speed), seed=seed))
    # 滚转/圆锥
    for load in (2.0, 4.0):
        for rate in (180.0, 360.0, 720.0, 1080.0):
            for seed in SEEDS:
                cs.append(dict(case("roll", load, 80.0, rate), seed=seed))
    for rate in (120.0, 300.0, 600.0, 900.0):
        for seed in SEEDS:
            cs.append(dict(case("coning", 1.0, 80.0, rate), seed=seed))
    # 负 g 边界
    for load in (2.0, 4.0, 6.0):
        for seed in SEEDS:
            cs.append(dict(case("negg", load), seed=seed))
    # 杆臂 A/B
    for prof, load, rate, lever in (("turn", 8.0, 0.0, 2.0),
                                    ("sturn", 6.0, 0.0, 2.0),
                                    ("pull", 6.0, 0.0, 2.0),
                                    ("roll", 2.0, 720.0, 1.0),
                                    ("coning", 1.0, 600.0, 1.0)):
        for mode in (0, 1, 2):
            for seed in SEEDS:
                cs.append(dict(case(prof, load, 80.0, rate, lever=lever,
                                    lever_comp=mode), seed=seed))
    # 20 s 失联 × IMU 误差档
    grades = ((1.0, 1.0, 0.0), (1.002, 1.001, 0.1),
              (1.005, 1.002, 0.2), (1.010, 1.005, 0.5))
    for prof, load, rate in (("turn", 4.0, 0.0), ("turn", 8.0, 0.0),
                             ("sturn", 6.0, 0.0), ("pull", 6.0, 0.0),
                             ("negg", 4.0, 0.0),
                             ("roll", 2.0, 720.0), ("coning", 1.0, 600.0)):
        for a_s, g_s, mis in grades:
            for seed in SEEDS:
                cs.append(dict(case(prof, load, 80.0, rate, duration=55.0,
                                    outage=(15.0, 35.0), a_scale=a_s,
                                    g_scale=g_s, misalign=mis), seed=seed))
    # 饱和边界
    for load in (9.0, 10.0, 12.0):
        for seed in SEEDS:
            cs.append(dict(case("turn", load, 80.0, duration=30.0), seed=seed))
    return cs


def main():
    cases = build()
    print("cases", len(cases), flush=True)
    rows = []
    with cf.ProcessPoolExecutor(max_workers=int(os.environ.get("HIGHDYN_WORKERS", "10"))) as ex:
        for i, r in enumerate(ex.map(one, cases, chunksize=4)):
            rows.append(r)
            if (i + 1) % 500 == 0:
                print("done", i + 1, flush=True)
    with open(OUT, "w", encoding="utf-8") as f:
        for d in rows:
            f.write(json.dumps(d, ensure_ascii=False) + "\n")
    ok = [r for r in rows if not r.get("no_json")]
    out = {"n": len(rows), "n_ok": len(ok),
           "pos_rmse": stat([r["pos_rmse"] for r in ok]),
           "pos_max": stat([r["pos_max"] for r in ok]),
           "att_rmse": stat([r["att_rmse_deg"] for r in ok]),
           "att_max": stat([r["att_max_deg"] for r in ok])}
    # 分组：杆臂 / 失联档 / 饱和
    groups = {}
    for r in ok:
        if r["lever"] > 0:
            key = f'lever_{r["profile"]}_lev{r["lever"]:g}_m{r["lever_comp"]}'
        elif r["outage"] and r["outage"][1] - r["outage"][0] == 20:
            key = f'out20_{r["profile"]}_a{r["a_scale"]:.3f}_m{r["misalign"]:g}'
        elif r["load"] > 8:
            key = f'sat_L{r["load"]:g}'
        else:
            key = f'base_{r["profile"]}_L{r["load"]:g}_V{r["speed"]:g}_R{r["rate"]:g}'
        groups.setdefault(key, []).append(r)
    out["groups"] = {}
    for k, rs in groups.items():
        out["groups"][k] = dict(
            n=len(rs),
            pos=stat([r["pos_rmse"] for r in rs]),
            pos_max=stat([r["pos_max"] for r in rs]),
            att=stat([r["att_rmse_deg"] for r in rs]),
            nees=stat([r["nees_pos_med"] for r in rs]))
    json.dump(out, open(SUM, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(json.dumps({k: out[k] for k in ("n", "n_ok", "pos_rmse", "pos_max",
                                         "att_rmse", "att_max")},
                     ensure_ascii=False, indent=1))
    print("wrote", OUT)


if __name__ == "__main__":
    main()
