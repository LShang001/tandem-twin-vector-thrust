#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24ax：逐 seed 检验"补偿量抖动"是否比"均值偏差"更能预测失效。

@file    check_delay_jitter_vs_failure.py
@brief   读同档多 seed 的 200 Hz 追踪，按 seed 计算 τ̂ 的 p10/p90 抖动、
         应用次数、阻塞位图与该 seed 的位置误差，最后给出抖动 vs 误差的秩相关
@module  TandemVec-EKF-Report/分析（L2 研究设施）
@details 动机（§4.24aw，单 seed）：20 ms 档 seed73 的 τ̂ 中位 18.5（几乎准）但 p10/p90
         15.2/63.1，而 30 ms 档同一 seed 抖动仅 9 ms 且干净。若抖动是驱动量，则在**同一配置**
         的不同 seed 之间也应看到"抖动大 ⇒ 误差大"。本脚本只做相关，不做因果。
@frame   N/A
@units   ms / m
@note    预登记判据：
         J1 每个 CSV ≥8000 行、且 `ad_valid>0` 的历元 ≥200，否则该 seed 作废；
         J2 若 Spearman(抖动, pos_rms) ≥0.6 且 |Spearman(中位偏差, pos_rms)| <0.4
            ⇒ 记 `JITTER_DOMINANT`；
         J3 若两者都 <0.4 ⇒ 记 `NO_TREND`（单 seed 的对照不足以支持抖动说，须在报告里降级）；
         J4 其它 ⇒ `AMBIGUOUS`。
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


def spearman(a, b):
    a = np.asarray(a, float); b = np.asarray(b, float)
    ra = np.argsort(np.argsort(a)).astype(float)
    rb = np.argsort(np.argsort(b)).astype(float)
    ra -= ra.mean(); rb -= rb.mean()
    den = np.linalg.norm(ra) * np.linalg.norm(rb)
    return float(ra.dot(rb) / den) if den > 0 else 0.0


def one(path):
    with open(path, newline="") as f:
        r = csv.reader(f)
        h = next(r)
        ix = {k: i for i, k in enumerate(h)}
        rows = list(r)
    g = lambda k: np.array([float(x[ix[k]]) for x in rows])
    t = g("t"); tau = g("ad_tau_ms"); val = g("ad_valid")
    app = g("ad_app"); blk = g("ad_blk")
    dn = g("ekf_n") - g("true_n"); de = g("ekf_e") - g("true_e")
    pos = np.sqrt(dn ** 2 + de ** 2)
    m = (val > 0) & (tau > 0)
    if len(t) < 8000 or m.sum() < 200:
        return None
    p10, p50, p90 = np.percentile(tau[m], [10, 50, 90])
    return {
        "n": int(len(t)), "valid": int(m.sum()),
        "tau_p10": float(p10), "tau_med": float(p50), "tau_p90": float(p90),
        "jitter": float(p90 - p10),
        "ad_app": int(app[-1]), "blk_med": int(np.median(blk)),
        "pos_rms": float(np.sqrt(np.mean(pos ** 2))),
        "pos_max": float(np.max(pos)),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=os.path.join(os.environ.get("TEMP", "/tmp")))
    ap.add_argument("--prefix", default="j_s")
    ap.add_argument("--true-ms", type=float, default=20.0,
                    help="该档真实延迟，用于算中位偏差")
    ap.add_argument("--tag", default="2026-10-28_ax")
    args = ap.parse_args()
    rows = []
    for p in sorted(glob.glob(os.path.join(args.dir, args.prefix + "*.csv"))):
        m = re.search(re.escape(args.prefix) + r"(\d+)\.csv$", p)
        if not m:
            continue
        st = one(p)
        if st is None:
            print("J1 作废:", p)
            continue
        st["seed"] = int(m.group(1))
        st["bias"] = st["tau_med"] - args.true_ms
        rows.append(st)
    if len(rows) < 5:
        raise SystemExit("样本不足: %d" % len(rows))
    jit = [r["jitter"] for r in rows]
    bias = [abs(r["bias"]) for r in rows]
    pos = [r["pos_rms"] for r in rows]
    r_j = spearman(jit, pos)
    r_b = spearman(bias, pos)
    code = ("JITTER_DOMINANT" if r_j >= 0.6 and abs(r_b) < 0.4 else
            "NO_TREND" if abs(r_j) < 0.4 and abs(r_b) < 0.4 else "AMBIGUOUS")
    verdict = {"J2_J3": {"code": code, "spearman_jitter_pos": r_j,
                         "spearman_bias_pos": r_b, "n": len(rows)}}
    for r in rows:
        print("seed %2d  jitter %5.1f  bias %+5.1f  app %3d  blk %2d  pos_rms %7.2f"
              % (r["seed"], r["jitter"], r["bias"], r["ad_app"], r["blk_med"], r["pos_rms"]))
    print(json.dumps(verdict, ensure_ascii=False))
    out = os.path.join(SD, "delay_jitter_vs_failure_%s.json" % args.tag)
    json.dump({"verdict": verdict, "rows": rows}, open(out, "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)
    print("JSON:", out)


if __name__ == "__main__":
    main()
