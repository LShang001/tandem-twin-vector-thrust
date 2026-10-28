# -*- coding: utf-8 -*-
"""30.5 g 尾部 seed 的失效时间线量化（SIL trace CSV）。
@file    analyze_tail_timeline.py
@brief   从 200 Hz trace 提取位置误差增长、门限越界时刻、重锚时刻、垂速误差。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 用于给"速度子集软恢复"定验收口径：误差以多少 m/s 累积、
         何时越过 30 m 位置野值门、速度残差是否先于位置残差越门。
@frame   NED（北/东为天向右手系水平分量）
@units   m / m/s / s

用法：py -3.12 sim/analyze_tail_timeline.py <csv...>
"""
import argparse
import json
import os

import numpy as np


def load(path):
    with open(path, encoding="utf-8") as fh:
        header = fh.readline().strip().split(",")
        rows = []
        for line in fh:
            parts = line.strip().split(",")
            if len(parts) == len(header):
                rows.append(parts)
    arr = np.array(rows, dtype=np.float64)
    return {name: arr[:, i] for i, name in enumerate(header)}


def tilt_consistency(col):
    """倾角（roll/pitch）一致性：|误差|/1σ。>1 持续即滤波器过信自己的姿态。

    需要 trace 里有 ekf_tilt_sigma 列（无则返回 None）。
    """
    if "ekf_tilt_sigma" not in col:
        return None
    t = col["t"]
    err = np.hypot(col["ekf_roll"] - col["truth_roll"],
                   col["ekf_pitch"] - col["truth_pitch"])
    sig = col["ekf_tilt_sigma"]
    out = {}
    for lo, hi in ((0.0, 10.0), (10.0, 16.0), (16.0, 21.4), (21.4, 30.0),
                   (30.0, 45.0)):
        m = (t >= lo) & (t < hi) & (sig > 1e-6)
        if m.sum() < 10:
            continue
        ratio = err[m] / sig[m]
        out[f"{lo:g}-{hi:g}s"] = {
            "tilt_err_med_deg": round(float(np.median(err[m])), 3),
            "tilt_sig_med_deg": round(float(np.median(sig[m])), 3),
            "ratio_med": round(float(np.median(ratio)), 3),
            "ratio_p95": round(float(np.percentile(ratio, 95)), 3),
            "ratio_max": round(float(ratio.max()), 3),
        }
    return out


def gate_summary(col):
    """物理野值门归因：哪一条门先越、各越多少次、残差峰值。

    需要 trace 里有 gate_pos/gate_vel/gnss_resid_* 列（旧 trace 返回 None）。
    """
    if "gate_pos" not in col or "gate_vel" not in col:
        return None
    t = col["t"]

    def first(mask):
        idx = np.flatnonzero(mask)
        return float(t[idx[0]]) if len(idx) else None

    gp = col["gate_pos"] > 0.5
    gv = col["gate_vel"] > 0.5
    return {
        "n_gate_pos_frames": int(gp.sum()),
        "n_gate_vel_frames": int(gv.sum()),
        "n_both": int((gp & gv).sum()),
        "vel_only": int((gv & ~gp).sum()),
        "pos_only": int((gp & ~gv).sum()),
        "t_first_gate_pos": first(gp),
        "t_first_gate_vel": first(gv),
        "resid_pos_ne_max_m": round(float(col["gnss_resid_pos_ne"].max()), 2),
        "resid_vel_ne_max_mps": round(float(col["gnss_resid_vel_ne"].max()), 2),
        "vel_sub_rec_final": int(col["vel_sub_rec"][-1]),
    }


def timeline(col, thr_ne=30.0, vel_thr=10.0):
    t = col["t"]
    err_ne = np.hypot(col["ekf_n"] - col["true_n"], col["ekf_e"] - col["true_e"])
    vel_err_d = np.abs(col["ekf_vz"] - col["truth_vz"])
    reanchor = np.diff(col["reanchor"], prepend=col["reanchor"][0]) > 0.5

    def first(mask):
        idx = np.flatnonzero(mask)
        return float(t[idx[0]]) if len(idx) else None

    def crossing(thr):
        idx = np.flatnonzero(err_ne > thr)
        if not len(idx):
            return None, None
        t0, i0 = t[idx[0]], idx[0]
        # 起点后 2 s 内的平均增长斜率（剔除单次尖峰）
        seg = (err_ne > thr * 0.2) & (t >= t0 - 2.0) & (t <= t0)
        if seg.sum() < 10:
            return t0, None
        p = np.polyfit(t[seg], err_ne[seg], 1)
        return t0, float(p[0])

    t30, slope30 = crossing(thr_ne)
    return {
        "dur_s": float(t[-1]),
        "err_ne_max": float(err_ne.max()),
        "err_ne_final": float(err_ne[-1]),
        "t_over_10m": first(err_ne > 10.0),
        "t_over_30m": t30,
        "growth_mps_to_30m": slope30,
        "t_over_vel10": first(vel_err_d > vel_thr),
        "vel_err_d_max": float(vel_err_d.max()),
        "reanchor_n": int(reanchor.sum()),
        "reanchor_t": [round(float(x), 2) for x in t[reanchor]][:6],
        "alt_err_max": float(np.abs(col["ctl_alt"] - col["truth_alt"]).max()),
        "roll_max": float(np.abs(col["truth_roll"]).max()),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--json", default="")
    args = ap.parse_args()
    out = {}
    for path in args.csv:
        name = os.path.splitext(os.path.basename(path))[0]
        col = load(path)
        rec = timeline(col)
        tc = tilt_consistency(col)
        if tc:
            rec["tilt_consistency"] = tc
        gs = gate_summary(col)
        if gs:
            rec["gate_attribution"] = gs
        out[name] = rec
        print(name, json.dumps(rec, ensure_ascii=False))
    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump(out, fh, ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
