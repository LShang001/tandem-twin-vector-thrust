# -*- coding: utf-8 -*-
"""用 200 Hz 迹线求"被应用的 τ̂ 分布"，取代 JSON 末帧 `ad_tau_ms`。

@file    check_delay_tau_trace.py
@brief   从 `--trace-csv` 的 `ad_tau_ms` + `ad_app`（累计计数）逐历元差分，
         给出 τ̂ 的中位/IQR/命中率和"应用占空比"
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 为什么必须做这件事：SIL JSON 里的 `ad_tau_ms` 是**运行结束那一帧**的
         `gnss_status.auto_delay_tau_s`，而估计器每秒重算一次、末帧恰好落在哪一档
         完全不代表整条弹道用了什么。本轮已经第 8 次踩"末帧/锁存统计量"坑：
         延 200 ms 档 G1 的末帧 τ̂=94.3 ms 看起来"没命中"，但 `ad_app` 从 20 涨到 220、
         位置 RMSE 38.6→22.8 m——判断"估得准不准"只能看逐历元分布。
         迹线里 `ad_app` 是累计计数 ⇒ 差分 > 0 的历元就是"这一拍真的用了 τ̂"，
         取这些历元上的 `ad_tau_ms` 分布才是正确答案。
"""
import json
import os
import re
import statistics as st
import subprocess
import sys

import psutil

JRE = re.compile(r"SIL_REAL_JSON (\{.*\})")
EXE = os.environ.get(
    "EXE", r"C:/Users/12631/AppData/Local/Temp/tv_hf3/fw_real.exe")
TRACES = ("D:/纵列双发矢量推力飞行器/TandemVec-EKF-Report/sim-data/delay_traces")
BASE = ["--real-nav", "--missile", "--seconds", "45", "--gnss-rate-hz", "10",
        "--profile", "mag_clean", "--imu-grade", "icm"]
CASES = [(150, 150), (150, 200), (300, 150), (300, 200), (300, 250),
         (150, 250), (300, 300), (150, 300),
         # 第二轮：定默认值需要的中间档与缓冲边缘
         (200, 200), (250, 250), (250, 300), (330, 300), (330, 330), (250, 200),
         (300, 100), (300, 0), (200, 150)]
SEEDS = [1, 2, 3]
BIND = int(os.environ.get("BIND", "6"))


def read_trace(path):
    with open(path, encoding="utf-8") as fh:
        hdr = fh.readline().strip().split(",")
        rows = [ln.strip().split(",") for ln in fh if ln.strip()]
    idx = {c: i for i, c in enumerate(hdr)}
    t = [float(r[idx["t"]]) for r in rows]
    tau = [float(r[idx["ad_tau_ms"]]) for r in rows]
    app = [float(r[idx["ad_app"]]) for r in rows]
    return t, tau, app


def run(grid, lat, seed):
    os.makedirs(TRACES, exist_ok=True)
    tp = os.path.join(TRACES, f"g{grid}_L{lat}_s{seed}.csv").replace("\\", "/")
    cmd = [EXE] + BASE + ["--gnss-latency-ms", str(lat)]
    if grid != 150:
        cmd += ["--delay-grid-max-ms", str(grid), "--delay-cap-ms", str(grid)]
    cmd += ["--seed", str(seed), "--trace-csv", tp]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                         text=True, env=os.environ.copy())
    try:
        psutil.Process(p.pid).cpu_affinity([BIND])
    except Exception:
        pass
    try:
        out, err = p.communicate(timeout=900)
    except subprocess.TimeoutExpired:
        p.kill()
        return None
    m = JRE.findall((out or "") + (err or ""))
    return (json.loads(m[-1]) if m else None), tp


def main():
    if not os.path.exists(EXE):
        print("MISSING EXE:", EXE)
        return 2
    print("EXE:", EXE)
    out_rows = {}
    for grid, lat in CASES:
        per = []
        for s in SEEDS:
            r = run(grid, lat, s)
            if not r:
                continue
            j, tp = r
            if not os.path.exists(tp):
                continue
            t, tau, app = read_trace(tp)
            applied = []
            for i in range(1, len(app)):
                if app[i] > app[i - 1] and t[i] > 3.0:
                    applied.append(tau[i])
            if not applied:
                per.append({"seed": s, "n_applied": 0, "med": float("nan"),
                            "p25": float("nan"), "p75": float("nan"),
                            "hit_frac": 0.0, "pos_rms": j.get("pos_rms"),
                            "last_tau": tau[-1]})
                continue
            a = sorted(applied)
            q = lambda f: a[min(len(a) - 1, int(f * len(a)))]
            per.append({"seed": s, "n_applied": len(a), "med": st.median(a),
                        "p25": q(0.25), "p75": q(0.75),
                        "hit_frac": sum(1 for v in a if abs(v - lat) <= 15.0) / len(a),
                        "pos_rms": j.get("pos_rms"), "last_tau": tau[-1]})
        if not per:
            print(f"grid {grid} lat {lat}: NO DATA")
            continue
        medm = [p["med"] for p in per if p["med"] == p["med"]]
        out_rows[f"{grid}_{lat}"] = per
        print(f"grid {grid:>4} lat {lat:>4} | 应用历元/seed "
              f"{[p['n_applied'] for p in per]}  τ̂(应用历元)中位 "
              f"{st.median(medm) if medm else float('nan'):>7.1f} ms  "
              f"IQR {st.median([p['p25'] for p in per if p['p25']==p['p25']] or [float('nan')]):.1f}"
              f"-{st.median([p['p75'] for p in per if p['p75']==p['p75']] or [float('nan')]):.1f}  "
              f"命中率(|τ̂−真|≤15ms) {st.median([p['hit_frac'] for p in per]):.2f}  "
              f"末帧 τ̂ {st.median([p['last_tau'] for p in per]):>6.1f}  "
              f"pos_rms {st.median([p['pos_rms'] for p in per]):.2f}")
    path = os.path.join(TRACES, "tau_by_epoch_2026-10-28.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(out_rows, fh, ensure_ascii=False, indent=1)
    print("json:", path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
