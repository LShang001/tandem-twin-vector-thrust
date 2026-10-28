#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24am：30.06 g/10 Hz 失效 seed 的误差按时间窗分解。

@file    run_missile_phase_decomp.py
@brief   对同一过载档跑 N seeds 的 200 Hz 追踪，把滚转/俯仰误差、航向误差、
         位置误差按固定时间窗统计，回答"失效是整段抬升还是集中在某一飞行阶段"
@module  TandemVec-EKF-Report/仿真驱动（L2 研究设施）
@details 动机（§4.24al 遗留）：整段 `nav_att_rmse` 只能说明"失效 seed 姿态更差"，
         不能说明误差在**哪一段**产生。若失效 seed 的误差集中在某一窗而通过 seed
         在同窗保持干净，则下一步诊断应锁定该窗（此时真值俯仰/高度剖面用于
         标识阶段），而不是继续在整段统计量上找阈值。
         已知事实（勿重复验证）：
         0) 30.06 g（23 500 N）/10 Hz 生产路径 10 Hz 口径下失效与"整帧拒绝"无关
            （失效 seed 零拒帧，§4.24ak），只有姿态误差与位置误差相关（r=0.864）；
         1) 注入常数姿态失准 1°/2° 不能复现失效（§4.24ak 否证）⇒ 不是恒定偏差；
         2) 200 Hz 追踪列名见 `--trace-csv`；本脚本只用 t/truth_*/ekf_*/true_n/e。
@frame   NED；姿态角 deg；窗口边界 [0,10,20,30,45] s
@units   长度 m、姿态 deg、时间 s
@note    预登记判据（跑前写死）：
         P1 每 seed 必须有追踪 CSV 且行数 ≥8000（200 Hz×45 s≈9000），否则该 seed 作废；
         P2 簇划分用全程 pos_rms（>20 m=失效 / ≤20 m=通过），阈值取 §4.24 双峰空档
            10.7→22.3 m 的中间值，不按本批数据重选；
         P3 若失效簇在**某一窗**的姿态 RMS 中位数 ≥ 通过簇同窗的 2.0 倍，
            且该窗不是全程（0–45 s）——则"误差集中在某阶段"成立，报告该窗；
         P4 若失效簇在各窗的倍率都 <1.5×——则"集中"不成立，登记为整段抬升，
            不得用单窗诊断；
         P5 位置误差与姿态误差必须在**同一窗**同时抬升（倍率同向）才允许写
            "该窗姿态误差驱动位置误差"；否则只登记共现。
"""
import argparse
import csv
import json
import os
import re
import subprocess
from concurrent.futures import ThreadPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
JSON_RE = re.compile(r"SIL_REAL_JSON (\{.*\})")
BASE = ["--real-nav", "--missile", "--seconds", "45", "--imu-fs-g", "32",
        "--imu-health-accel-range", "313.8", "--profile", "mag_clean",
        "--imu-grade", "icm", "--gnss-rate-hz", "10"]
WINDOWS_S = [0.0, 10.0, 20.0, 30.0, 45.0]


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def run_one(exe, seed, thrust_n, extra, out_csv, bind_core):
    cmd = [exe] + BASE + ["--ms-thrust", str(thrust_n), "--seed", str(seed),
                          "--trace-csv", out_csv] + extra
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    m = JSON_RE.search(proc.stdout)
    if not m:
        return {"seed": seed, "ok": False,
                "err": (proc.stdout + proc.stderr)[-400:]}
    rec = json.loads(m.group(1))
    rec["seed"] = seed
    rec["ok"] = True
    rec["csv"] = out_csv
    return rec


def window_stats(csv_path):
    with open(csv_path, newline="") as f:
        rdr = csv.reader(f)
        hdr = next(rdr)
        idx = {k: i for i, k in enumerate(hdr)}
        rows = list(rdr)
    need = ["t", "truth_roll", "truth_pitch", "truth_yaw",
            "ekf_roll", "ekf_pitch", "ekf_yaw", "ekf_n", "ekf_e",
            "true_n", "true_e", "truth_alt", "truth_vz"]
    optional = ["ekf_vn", "ekf_ve", "gnss_resid_pos_ne", "gnss_resid_vel_ne",
                "gate_pos", "gate_vel", "vel_sub_rec", "ad_app", "ad_tau_ms",
                "vib_high", "ad_valid"]
    for k in need:
        if k not in idx:
            raise SystemExit("缺失列: %s" % k)
    arr = {k: np.array([float(r[idx[k]]) for r in rows]) for k in need}
    for k in optional:
        if k in idx:
            arr[k] = np.array([float(r[idx[k]]) for r in rows])
    t = arr["t"]
    # 真值速度：位置差分（200 Hz，cm/s 级）
    dt = np.gradient(t)
    vtn = np.gradient(arr["true_n"]) / dt
    vte = np.gradient(arr["true_e"]) / dt
    vel_err = None
    if "ekf_vn" in arr:
        vel_err = np.sqrt((arr["ekf_vn"] - vtn) ** 2 +
                          (arr["ekf_ve"] - vte) ** 2)
    # 累计计数器的逐帧增量
    counters = {}
    for k in ("vel_sub_rec", "ad_app"):
        if k in arr:
            d = np.diff(arr[k], prepend=arr[k][0])
            counters[k] = np.maximum(d, 0.0)
    roll_e = arr["ekf_roll"] - arr["truth_roll"]
    pitch_e = arr["ekf_pitch"] - arr["truth_pitch"]
    yaw_e = wrap_deg(arr["ekf_yaw"] - arr["truth_yaw"])
    att_deg = np.sqrt(roll_e ** 2 + pitch_e ** 2)
    dn = arr["ekf_n"] - arr["true_n"]
    de = arr["ekf_e"] - arr["true_e"]
    pos_m = np.sqrt(dn ** 2 + de ** 2)
    def chan(sel):
        c = {}
        if vel_err is not None:
            c["vel_rms"] = float(np.sqrt(np.mean(vel_err[sel] ** 2)))
        for k, short in (("gnss_resid_pos_ne", "resid_pos_med"),
                         ("gnss_resid_vel_ne", "resid_vel_med"),
                         ("ad_tau_ms", "ad_tau_med")):
            if k in arr:
                v = arr[k][sel]
                v = v[v > 0]
                c[short] = float(np.median(v)) if v.size else 0.0
        for k in ("gate_pos", "gate_vel", "vib_high", "ad_valid"):
            if k in arr:
                c[k + "_frac"] = float(np.mean(arr[k][sel]))
        for k, short in (("vel_sub_rec", "subset_fires"),
                         ("ad_app", "ad_applies")):
            if k in counters:
                c[short] = float(counters[k][sel].sum())
        return c

    out = {}
    for w0, w1 in zip(WINDOWS_S[:-1], WINDOWS_S[1:]):
        sel = (t >= w0) & (t < w1)
        if sel.sum() < 10:
            out["%g-%g" % (w0, w1)] = None
            continue
        out["%g-%g" % (w0, w1)] = {
            **chan(sel),
            "n": int(sel.sum()),
            "att_rms": float(np.sqrt(np.mean(att_deg[sel] ** 2))),
            "roll_bias": float(np.mean(roll_e[sel])),
            "pitch_bias": float(np.mean(pitch_e[sel])),
            "roll_rms": float(np.sqrt(np.mean(roll_e[sel] ** 2))),
            "pitch_rms": float(np.sqrt(np.mean(pitch_e[sel] ** 2))),
            "yaw_rms": float(np.sqrt(np.mean(yaw_e[sel] ** 2))),
            "pos_rms": float(np.sqrt(np.mean(pos_m[sel] ** 2))),
            "truth_pitch_mean": float(np.mean(arr["truth_pitch"][sel])),
            "truth_alt_mean": float(np.mean(arr["truth_alt"][sel])),
            "truth_vz_mean": float(np.mean(arr["truth_vz"][sel])),
        }
    sel = np.ones_like(t, dtype=bool)
    out["0-45"] = {
        **chan(sel),
        "n": int(sel.sum()),
        "att_rms": float(np.sqrt(np.mean(att_deg ** 2))),
        "roll_bias": float(np.mean(roll_e)),
        "pitch_bias": float(np.mean(pitch_e)),
        "roll_rms": float(np.sqrt(np.mean(roll_e ** 2))),
        "pitch_rms": float(np.sqrt(np.mean(pitch_e ** 2))),
        "yaw_rms": float(np.sqrt(np.mean(yaw_e ** 2))),
        "pos_rms": float(np.sqrt(np.mean(pos_m ** 2))),
        "truth_pitch_mean": float(np.mean(arr["truth_pitch"])),
        "truth_alt_mean": float(np.mean(arr["truth_alt"])),
        "truth_vz_mean": float(np.mean(arr["truth_vz"])),
    }
    out["_n_rows"] = int(len(t))
    # 200 Hz 时间线抽样保存（绘图用，降采样到 ~2 Hz）
    step = max(1, len(t) // 100)
    out["_timeline"] = {
        "t": t[::step].round(3).tolist(),
        "att_err": att_deg[::step].round(4).tolist(),
        "pos_err": pos_m[::step].round(4).tolist(),
        "truth_pitch": arr["truth_pitch"][::step].round(3).tolist(),
        "truth_alt": arr["truth_alt"][::step].round(3).tolist(),
    }
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", required=True)
    ap.add_argument("--seed-start", type=int, default=51)
    ap.add_argument("--seeds", type=int, default=30)
    ap.add_argument("--ms-thrust", type=float, default=23500.0)
    ap.add_argument("--extra-args", default="")
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--bind-core", type=int, default=5)
    ap.add_argument("--trace-dir",
                    default=os.path.join(os.environ.get("TEMP", "/tmp"),
                                         "phase_tr"),
                    help="追踪 CSV 输出目录（不入库）")
    ap.add_argument("--tag", default="2026-10-28_phase")
    ap.add_argument("--keep-csv", action="store_true",
                    help="保留 CSV（缺省处理完删除，只留 JSON）")
    args = ap.parse_args()
    if args.bind_core >= 0:
        import psutil
        psutil.Process().cpu_affinity([args.bind_core])
    os.makedirs(args.trace_dir, exist_ok=True)
    os.makedirs(SD, exist_ok=True)
    extra = args.extra_args.split() if args.extra_args else []
    seeds = list(range(args.seed_start, args.seed_start + args.seeds))

    def job(s):
        p = os.path.join(args.trace_dir, "s%d.csv" % s)
        return run_one(args.exe, s, args.ms_thrust, extra, p, args.bind_core)

    with ThreadPoolExecutor(max_workers=args.workers) as ex:
        recs = list(ex.map(job, seeds))

    rows = []
    bad = []
    for rec in recs:
        if not rec.get("ok"):
            bad.append(rec)
            continue
        ws = window_stats(rec["csv"])
        if ws["_n_rows"] < 8000:
            bad.append({"seed": rec["seed"], "err": "rows=%d" % ws["_n_rows"]})
            continue
        rows.append({"seed": rec["seed"],
                     "pos_rms": rec.get("pos_rms"),
                     "pos_max": rec.get("pos_max"),
                     "nav_att_rmse": rec.get("nav_att_rmse"),
                     "outlier_rejects": rec.get("outlier_rejects", 0),
                     "windows": {k: v for k, v in ws.items()
                                 if not k.startswith("_")},
                     "timeline": ws["_timeline"]})
        if not args.keep_csv:
            try:
                os.remove(rec["csv"])
            except OSError:
                pass

    keys = ["0-10", "10-20", "20-30", "30-45", "0-45"]
    # P2：簇划分用全程 pos_rms，阈值 20 m（§4.24 双峰空档 10.7–22.3 m 的中点）
    fail = [r for r in rows if r["pos_rms"] is not None and r["pos_rms"] > 20.0]
    ok = [r for r in rows if r["pos_rms"] is not None and r["pos_rms"] <= 20.0]
    summary = {"n_total": len(rows), "n_fail": len(fail), "n_pass": len(ok),
               "fail_seeds": [r["seed"] for r in fail],
               "pass_seeds": [r["seed"] for r in ok],
               "windows": {}}
    for k in keys:
        fv = [r["windows"][k] for r in fail if r["windows"].get(k)]
        pv = [r["windows"][k] for r in ok if r["windows"].get(k)]
        if not fv or not pv:
            continue
        def med(vals, key):
            return float(np.median([v[key] for v in vals]))
        entry = {
            "fail_att_med": med(fv, "att_rms"), "pass_att_med": med(pv, "att_rms"),
            "fail_pos_med": med(fv, "pos_rms"), "pass_pos_med": med(pv, "pos_rms"),
            "fail_yaw_med": med(fv, "yaw_rms"), "pass_yaw_med": med(pv, "yaw_rms"),
            "fail_roll_bias_med": med(fv, "roll_bias"),
            "pass_roll_bias_med": med(pv, "roll_bias"),
            "fail_pitch_bias_med": med(fv, "pitch_bias"),
            "pass_pitch_bias_med": med(pv, "pitch_bias"),
            "truth_pitch_med": med(fv, "truth_pitch_mean"),
            "truth_alt_med": med(fv, "truth_alt_mean"),
            "truth_vz_med": med(fv, "truth_vz_mean"),
        }
        entry["att_ratio"] = (entry["fail_att_med"] / entry["pass_att_med"]
                              if entry["pass_att_med"] > 0 else None)
        entry["pos_ratio"] = (entry["fail_pos_med"] / entry["pass_pos_med"]
                              if entry["pass_pos_med"] > 0 else None)
        for ch in ("vel_rms", "resid_pos_med", "resid_vel_med", "ad_tau_med",
                   "gate_pos_frac", "gate_vel_frac", "subset_fires",
                   "ad_applies"):
            if fv[0].get(ch) is not None:
                entry["fail_" + ch] = med(fv, ch)
                entry["pass_" + ch] = med(pv, ch)
        summary["windows"][k] = entry
    # P3/P4/P5 裁决
    inner = [k for k in keys if k != "0-45"]
    ratios = [(k, summary["windows"][k]["att_ratio"]) for k in inner
              if summary["windows"].get(k) and
              summary["windows"][k]["att_ratio"] is not None]
    verdict = {"P1": {"n_bad": len(bad), "bad": bad}}
    if ratios:
        best = max(ratios, key=lambda x: x[1])
        same_window = [k for k, rr in ratios if rr >= 1.5 and
                       summary["windows"][k]["pos_ratio"] >= 1.5]
        verdict["P3_P4"] = {
            "max_att_ratio_window": best[0],
            "max_att_ratio": best[1],
            "concentrated": best[1] >= 2.0,
            "same_window_pos_lift": same_window,
        }
        verdict["P5_coincident"] = bool(same_window)
    summary["verdict"] = verdict
    summary["thrust_n"] = args.ms_thrust
    outp = os.path.join(SD, "missile_phase_decomp_%s.json" % args.tag)
    with open(outp, "w", encoding="utf-8") as f:
        json.dump({"summary": summary, "rows": rows}, f, ensure_ascii=False,
                  indent=1)
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    print("JSON:", outp)


if __name__ == "__main__":
    main()
