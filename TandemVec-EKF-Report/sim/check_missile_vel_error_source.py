#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24ap：10–20 s 窗里 6–7 m/s 的速度误差由哪个状态承载。

@file    check_missile_vel_error_source.py
@brief   对 200 Hz 追踪按窗统计：速度误差、姿态误差、**加速度计零偏态**、陀螺零偏态、
         倾角 1σ，判定速度误差是被"加速度计零偏吸收"（abx/aby 与速度误差同量级）
         还是"姿态倾斜投影错"（ab 不动、姿态误差 × 比力解释速度误差）
@module  TandemVec-EKF-Report/分析（L2 研究设施）
@details 动机（§4.24an 遗留）：已知 10–20 s 窗速度门 58/100 历元、残差中位 7 m/s，
         但"谁把速度推偏"未知。两个候选在数据上可分：
         (a) 加速度计零偏/尺度误差被 EKF 吸进 `ba`（则 ab 分量窗内抬到 ~1 m/s² 级）；
         (b) 姿态（倾斜）误差把 ~30 g 比力投错方向（则 ab 不动，速度误差 ≈ ∫ g·δθ dt，
             δθ 只需 ~0.25°@30 g 就能积出 7 m/s/6 s）。
@frame   NED；`abx/aby` 为 EKF 零偏态（机体系 m/s²），`gbx/gby` deg/s
@units   长度 m、速度 m/s、加速度 m/s²、角度 deg
@note    预登记判据（跑前写死）：
         S1 每个 CSV 必须 ≥8000 行（200 Hz×45 s），否则作废；
         S2 若失效簇 10–20 s 的 |ab| 中位比通过簇高 ≥0.3 m/s² ⇒ 记 `BIAS_CARRIED`；
         S3 若 |ab| 两簇差 <0.15 m/s² 而姿态误差差 ≥0.5° ⇒ 记 `TILT_CARRIED`；
         S4 若两者都不满足 ⇒ 记 `INCONCLUSIVE`，不得据本脚本下结论。
         0–10 s 两簇必须几乎相同（前一轮 §4.24am 的阴性对照），否则说明这批种子条件不同。
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


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def one(path):
    with open(path, newline="") as f:
        r = csv.reader(f)
        h = next(r)
        ix = {k: i for i, k in enumerate(h)}
        rows = list(r)
    need = ["t", "truth_roll", "truth_pitch", "ekf_roll", "ekf_pitch",
            "ekf_n", "ekf_e", "true_n", "true_e", "ekf_vn", "ekf_ve",
            "abx_mps2", "aby_mps2", "gbx_dps", "gby_dps", "ekf_tilt_sigma"]
    for k in need:
        if k not in ix:
            raise SystemExit("缺失列 %s in %s" % (k, path))
    a = {k: np.array([float(x[ix[k]]) for x in rows]) for k in need}
    t = a["t"]
    dt = np.gradient(t)
    vtn = np.gradient(a["true_n"]) / dt
    vte = np.gradient(a["true_e"]) / dt
    verr = np.sqrt((a["ekf_vn"] - vtn) ** 2 + (a["ekf_ve"] - vte) ** 2)
    roll_e = a["ekf_roll"] - a["truth_roll"]
    pitch_e = a["ekf_pitch"] - a["truth_pitch"]
    att = np.sqrt(roll_e ** 2 + pitch_e ** 2)
    dn = a["ekf_n"] - a["true_n"]
    de = a["ekf_e"] - a["true_e"]
    pos = np.sqrt(dn ** 2 + de ** 2)
    ab = np.sqrt(a["abx_mps2"] ** 2 + a["aby_mps2"] ** 2)
    gb = np.sqrt(a["gbx_dps"] ** 2 + a["gby_dps"] ** 2)
    out = {}
    for w0, w1 in WINDOWS:
        s = (t >= w0) & (t < w1)
        out["%d-%d" % (w0, w1)] = {
            "vel_rms": float(np.sqrt(np.mean(verr[s] ** 2))),
            "vel_med": float(np.median(verr[s])),
            "att_rms": float(np.sqrt(np.mean(att[s] ** 2))),
            "att_max": float(np.max(att[s])),
            "pos_rms": float(np.sqrt(np.mean(pos[s] ** 2))),
            "ab_med": float(np.median(ab[s])),
            "ab_max": float(np.max(ab[s])),
            "abx_med": float(np.median(a["abx_mps2"][s])),
            "aby_med": float(np.median(a["aby_mps2"][s])),
            "gb_med": float(np.median(gb[s])),
            "tilt_sig_med": float(np.median(a["ekf_tilt_sigma"][s])),
        }
    out["_n"] = int(len(t))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trace-dir", default="/tmp")
    ap.add_argument("--prefix", default="d_s")
    ap.add_argument("--fail", default="73,63,59")
    ap.add_argument("--pass-seeds", default="76,57,55")
    ap.add_argument("--tag", default="2026-10-28_ap")
    args = ap.parse_args()
    fail = [int(x) for x in args.fail.split(",")]
    ok = [int(x) for x in args.pass_seeds.split(",")]
    rows = {}
    for group, seeds in (("fail", fail), ("pass", ok)):
        for s in seeds:
            p = os.path.join(args.trace_dir, "%s%d.csv" % (args.prefix, s))
            if not os.path.exists(p):
                raise SystemExit("缺追踪 %s" % p)
            st = one(p)
            if st["_n"] < 8000:
                raise SystemExit("S1 失败：%s 只有 %d 行" % (p, st["_n"]))
            rows[s] = {"group": group, "windows": st}
    summary = {}
    for w0, w1 in WINDOWS:
        k = "%d-%d" % (w0, w1)
        f = [r["windows"][k] for r in rows.values() if r["group"] == "fail"]
        q = [r["windows"][k] for r in rows.values() if r["group"] == "pass"]
        e = {}
        for fld in ("vel_rms", "vel_med", "att_rms", "att_max", "pos_rms",
                    "ab_med", "ab_max", "abx_med", "aby_med", "gb_med",
                    "tilt_sig_med"):
            e["fail_" + fld] = float(np.median([x[fld] for x in f]))
            e["pass_" + fld] = float(np.median([x[fld] for x in q]))
        e["d_ab"] = e["fail_ab_med"] - e["pass_ab_med"]
        e["d_att"] = e["fail_att_rms"] - e["pass_att_rms"]
        e["d_vel"] = e["fail_vel_rms"] - e["pass_vel_rms"]
        summary[k] = e
    w = summary["10-20"]
    if w["d_ab"] >= 0.3:
        code = "BIAS_CARRIED"
    elif abs(w["d_ab"]) < 0.15 and w["d_att"] >= 0.5:
        code = "TILT_CARRIED"
    else:
        code = "INCONCLUSIVE"
    verdict = {"S2_S3": {"code": code, "d_ab_10_20": w["d_ab"],
                         "d_att_10_20": w["d_att"],
                         "d_vel_10_20": w["d_vel"]},
               "S4_0_10_same": (abs(summary["0-10"]["d_ab"]) < 0.15 and
                                abs(summary["0-10"]["d_att"]) < 0.3)}
    outp = os.path.join(SD, "missile_vel_error_source_%s.json" % args.tag)
    json.dump({"summary": summary, "verdict": verdict, "rows": rows},
              open(outp, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(json.dumps({"summary": summary, "verdict": verdict},
                     ensure_ascii=False, indent=1))
    print("JSON:", outp)


if __name__ == "__main__":
    main()
