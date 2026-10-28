#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24an：把 10 Hz 失效 seed 的分窗证据落到 GNSS 历元级通道上。

@file    check_missile_epoch_channels.py
@brief   读 `run_missile_phase_decomp.py --keep-csv` 留下的追踪，按 GNSS 历元
         （`msr_n/e` 变化）统计物理门的越门次数与新息幅值，回答"10–20 s 窗里
         先坏的是速度通道还是位置通道"
@module  TandemVec-EKF-Report/分析（L2 研究设施）
@details 动机（§4.24am 遗留）：分窗只证明误差集中在 10–20 s；整段 `outlier_rejects`
         的中位数会因"多数 seed 通过"而显示为 0，掩盖失效 seed 的越门事件
         （§4.24ak/§4.24ae 的"拒帧中位 0、子集无从下手"就是这种口径）。
         本脚本按**逐 seed 逐窗**统计，并显式报"失效簇 epoch 越门率"。
@frame   NED；残差列来自 EKF 只读诊断（`gnss_resid_pos_ne_m` / `..._vel_ne_mps`）
@units   长度 m、速度 m/s、时间 s
@note    预登记判据（跑前写死）：
         E1 每个 CSV 必须 ≥400 个 GNSS 历元（45 s×10 Hz），否则作废；
         E2 若失效簇在 10–20 s 的**速度门越门率** ≥ 3× 通过簇同窗、且位置门越门率
            也 ≥ 3× ⇒ 记 `VEL_LEADS`（同窗但速度更早更密）；
         E3 若仅位置门放大 ≥ 3× 而速度门 < 1.5× ⇒ 记 `POS_ONLY`；
         E4 若两簇在 0–10 s 有任何越门（>2% 历元）⇒ 记 `EARLY_TRIP`，
            说明失效信息早于 10 s 就出现，需要重划窗。
"""
import argparse
import csv
import glob
import json
import os
import re

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
WINDOWS = ((0, 10), (10, 20), (20, 30), (30, 45))


def epochs(csv_path):
    with open(csv_path, newline="") as f:
        r = csv.reader(f)
        h = next(r)
        ix = {k: i for i, k in enumerate(h)}
        rows = list(r)
    need = ["t", "msr_n", "msr_e", "gate_pos", "gate_vel",
            "gnss_resid_pos_ne", "gnss_resid_vel_ne", "true_n", "true_e",
            "ekf_n", "ekf_e"]
    for k in need:
        if k not in ix:
            raise SystemExit("缺失列 %s in %s" % (k, csv_path))
    t = np.array([float(r[ix["t"]]) for r in rows])
    msr = np.array([[float(r[ix["msr_n"]]), float(r[ix["msr_e"]])]
                    for r in rows])
    gv = np.array([float(r[ix["gate_vel"]]) for r in rows])
    gp = np.array([float(r[ix["gate_pos"]]) for r in rows])
    rv = np.array([float(r[ix["gnss_resid_vel_ne"]]) for r in rows])
    rp = np.array([float(r[ix["gnss_resid_pos_ne"]]) for r in rows])
    ch = np.where(np.any(np.diff(msr, axis=0) != 0, axis=1))[0] + 1
    return t[ch], gv[ch], gp[ch], rv[ch], rp[ch]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trace-dir",
                    default=os.path.join(os.environ.get("TEMP", "/tmp"),
                                         "phase_tr"))
    ap.add_argument("--phase-json", default=os.path.join(
        SD, "missile_phase_decomp_2026-10-28_am.json"))
    ap.add_argument("--tag", default="2026-10-28_an")
    args = ap.parse_args()
    ph = json.load(open(args.phase_json, encoding="utf-8"))
    cluster = {r["seed"]: ("fail" if r["pos_rms"] > 20.0 else "pass")
               for r in ph["rows"]}
    out_rows = []
    for p in sorted(glob.glob(os.path.join(args.trace_dir, "s*.csv"))):
        m = re.search(r"s(\d+)\.csv$", p)
        if not m:
            continue
        seed = int(m.group(1))
        if seed not in cluster:
            continue
        t, gv, gp, rv, rp = epochs(p)
        if len(t) < 400:
            continue
        row = {"seed": seed, "cluster": cluster[seed], "n_epochs": int(len(t)),
               "windows": {}}
        for w0, w1 in WINDOWS:
            s = (t >= w0) & (t < w1)
            if s.sum() == 0:
                continue
            row["windows"]["%d-%d" % (w0, w1)] = {
                "n": int(s.sum()),
                "gate_vel": int(gv[s].sum()),
                "gate_pos": int(gp[s].sum()),
                "resid_vel_med": float(np.median(rv[s])),
                "resid_vel_p90": float(np.percentile(rv[s], 90)),
                "resid_pos_med": float(np.median(rp[s])),
                "resid_pos_p90": float(np.percentile(rp[s], 90)),
            }
        out_rows.append(row)

    summary = {}
    for w0, w1 in WINDOWS:
        k = "%d-%d" % (w0, w1)
        f = [r["windows"][k] for r in out_rows
             if r["cluster"] == "fail" and k in r["windows"]]
        q = [r["windows"][k] for r in out_rows
             if r["cluster"] == "pass" and k in r["windows"]]
        if not f or not q:
            continue
        def med(v, kk):
            return float(np.median([x[kk] for x in v]))
        summary[k] = {
            "fail_gate_vel_rate": med(f, "gate_vel") / med(f, "n"),
            "pass_gate_vel_rate": med(q, "gate_vel") / med(q, "n"),
            "fail_gate_pos_rate": med(f, "gate_pos") / med(f, "n"),
            "pass_gate_pos_rate": med(q, "gate_pos") / med(q, "n"),
            "fail_resid_vel_med": med(f, "resid_vel_med"),
            "pass_resid_vel_med": med(q, "resid_vel_med"),
            "fail_resid_pos_med": med(f, "resid_pos_med"),
            "pass_resid_pos_med": med(q, "resid_pos_med"),
            "fail_gate_vel_med": med(f, "gate_vel"),
            "pass_gate_vel_med": med(q, "gate_vel"),
            "fail_gate_pos_med": med(f, "gate_pos"),
            "pass_gate_pos_med": med(q, "gate_pos"),
        }
    verdict = {}
    w = summary.get("10-20")
    if w:
        pr = w["pass_gate_pos_rate"]
        vr = w["pass_gate_vel_rate"]
        v_ratio = (w["fail_gate_vel_rate"] / vr) if vr > 0 else float("inf")
        p_ratio = (w["fail_gate_pos_rate"] / pr) if pr > 0 else float("inf")
        verdict["E2_E3"] = {
            "vel_rate_ratio": v_ratio, "pos_rate_ratio": p_ratio,
            "code": ("VEL_LEADS" if (v_ratio >= 3 and p_ratio >= 3)
                     else "POS_ONLY" if p_ratio >= 3 and v_ratio < 1.5
                     else "MIXED"),
        }
    early = summary.get("0-10")
    if early:
        verdict["E4"] = {
            "early_trip": (early["fail_gate_vel_rate"] > 0.02 or
                           early["fail_gate_pos_rate"] > 0.02),
            "fail_gate_vel_rate": early["fail_gate_vel_rate"],
            "fail_gate_pos_rate": early["fail_gate_pos_rate"],
        }
    outp = os.path.join(SD, "missile_epoch_channels_%s.json" % args.tag)
    json.dump({"summary": summary, "verdict": verdict, "rows": out_rows},
              open(outp, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(json.dumps({"summary": summary, "verdict": verdict},
                     ensure_ascii=False, indent=1))
    print("JSON:", outp)


if __name__ == "__main__":
    main()
