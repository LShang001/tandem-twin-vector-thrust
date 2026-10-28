# -*- coding: utf-8 -*-
"""振动自适应 Q 在弹道上的占空比审计 + 自动延时 τ̂ 死区判据取证。

@file    check_vib_q_duty.py
@brief   从导弹 SIL 200 Hz trace 统计 vib_q/高通加速度 RMS/陀螺 RMS 的时间占比
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 为什么必须做：`vib_high` 曾被证实恒真并静默封掉自动延时补偿；同一个
         `accel_rms_g` 还驱动 `vib_q=clamp(suggest,0.15,0.25)` 与 `gyro_std_suggest`。
         判据：0.5 Hz 高通的截止远低于导弹 27.8°/s（≈0.48 Hz）机动频率，
         若 `accel_rms_g` 随过载档位（零注入振动）上升，则"振动"其实是确定性机动，
         自适应 Q 在弹道上被机动而非振动决定。同时输出 τ̂ 分布用于选死区。
"""
import json
import os
import re
import statistics as st
import subprocess
import sys

import psutil

JRE = re.compile(r"SIL_REAL_JSON (\{.*\})")
BASE = ["--real-nav", "--missile", "--seconds", "45", "--gnss-rate-hz", "10",
        "--profile", "mag_clean", "--imu-grade", "icm"]
EXE = r"C:/Users/12631/AppData/Local/Temp/tv_vib/fw_real.exe"
TR = r"C:/Users/12631/AppData/Local/Temp/vibq"
# (标签, 追加参数, seeds)
CONDS = [
    ("6.4g_洁净", [], 3),
    ("8.4g", ["--ms-thrust", "8000"], 3),
    ("10.1g", ["--ms-thrust", "10000"], 3),
    ("6.4g_振1g80", ["--vib", "1.0", "--vib-hz", "80", "--vib-start", "1.5"], 3),
    ("6.4g_延100ms", ["--gnss-latency-ms", "100"], 3),
    ("6.4g_延200ms", ["--gnss-latency-ms", "200"], 3),
]


def run(extra, seed, tag):
    csv = f"{TR}/{tag}_s{seed}.csv"
    os.makedirs(TR, exist_ok=True)
    cmd = [EXE] + BASE + extra + ["--seed", str(seed), "--trace-csv", csv]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                         text=True, env=os.environ.copy())
    try:
        psutil.Process(p.pid).cpu_affinity([2])
    except Exception:
        pass
    out, err = p.communicate(timeout=900)
    m = JRE.findall((out or "") + (err or ""))
    return csv, (json.loads(m[-1]) if m else None)


def load(csv):
    with open(csv, encoding="utf-8") as fh:
        head = fh.readline().strip().split(",")
        rows = [dict(zip(head, ln.strip().split(","))) for ln in fh if ln.strip()]
    return rows


def duty(rows):
    """返回占空比字典（只统计飞行段 t>1.2 s，避开未初始化/助推瞬态）。"""
    fly = [r for r in rows if float(r["t"]) > 1.2]
    if not fly:
        return {}
    q = [float(r["vib_q"]) for r in fly]
    rms = [float(r["vib_rms_g"]) for r in fly]
    gy = [float(r["vib_gyr_dps"]) for r in fly]
    sug = [float(r["vib_sug"]) for r in fly]
    hi = [int(r["vib_high"]) for r in fly]
    gs = [min(0.010, max(0.0015, 0.0015 + 0.0025 * v)) for v in gy]
    n = len(q)
    tau = [float(r["ad_tau_ms"]) for r in fly if int(r["ad_valid"]) == 1]
    # 机体角速率幅值：trace 的 gyro_* 为滤波后机体系角速率 [rad/s]
    omg = [(float(r["gyro_x"]) ** 2 + float(r["gyro_y"]) ** 2 +
            float(r["gyro_z"]) ** 2) ** 0.5 * 57.29578 for r in fly]

    def pearson(x, y):
        mx, my = st.mean(x), st.mean(y)
        num = sum((a - mx) * (b - my) for a, b in zip(x, y))
        den = (sum((a - mx) ** 2 for a in x) *
               sum((b - my) ** 2 for b in y)) ** 0.5
        return num / den if den > 0 else float("nan")

    return {
        "n": n,
        "q_cap025": sum(1 for v in q if abs(v - 0.25) < 1e-6) / n,
        "q_floor015": sum(1 for v in q if abs(v - 0.15) < 1e-6) / n,
        "q_mid": sum(1 for v in q if 0.1501 < v < 0.2499) / n,
        "q_med": st.median(q),
        "rms_med": st.median(rms), "rms_p95": sorted(rms)[int(0.95 * n)],
        "rms_min": min(rms), "rms_max": max(rms),
        "sug_med": st.median(sug),
        "gy_med": st.median(gy), "gy_max": max(gy),
        "gs_cap": sum(1 for v in gs if v >= 0.00999) / n,
        "gs_med": st.median(gs),
        "hi_duty": sum(hi) / n,
        "omg_med": st.median(omg), "omg_max": max(omg),
        # 核心判据：高通"振动"RMS 是否由确定性机动 |ω| 解释
        "r_rms_omg": pearson(rms, omg),
        "tau_valid_n": len(tau),
        "tau_med": st.median(tau) if tau else -1,
        "tau_p5": sorted(tau)[int(0.05 * len(tau))] if tau else -1,
        "tau_p95": sorted(tau)[int(0.95 * len(tau))] if tau else -1,
    }


def main():
    print(f"exe md5-ish exists: {os.path.exists(EXE)}")
    all_res = {}
    for tag, extra, nseed in CONDS:
        acc = []
        for s in range(1, nseed + 1):
            csv, j = run(extra, s, tag)
            if not os.path.exists(csv):
                print(f"{tag} seed{s}: no trace")
                continue
            d = duty(load(csv))
            if d:
                d["pos_rms"] = (j or {}).get("pos_rms")
                acc.append(d)
        if not acc:
            continue
        k = ["q_cap025", "q_floor015", "q_mid", "q_med", "rms_med", "rms_max",
             "sug_med", "gy_med", "gy_max", "gs_cap", "gs_med", "hi_duty",
             "omg_med", "omg_max", "r_rms_omg",
             "tau_med", "tau_p5", "tau_p95", "pos_rms"]
        print(f"\n=== {tag} (n={len(acc)} seeds, {acc[0]['n']} frames/seed) ===")
        for key in k:
            vals = [a[key] for a in acc]
            med = st.median(vals)
            if key.endswith("duty") or key.startswith("q_cap") or \
               key.startswith("q_floor") or key == "q_mid" or key == "gs_cap":
                print(f"  {key:<11} {med*100:6.1f} %   "
                      f"seeds=[{' '.join(f'{v*100:.0f}' for v in vals)}]")
            else:
                print(f"  {key:<11} {med:9.4f}   "
                      f"seeds=[{' '.join(f'{v:.3f}' for v in vals)}]")
        all_res[tag] = acc
    path = ("D:/纵列双发矢量推力飞行器/TandemVec-EKF-Report/sim-data/"
            "vib_q_duty_2026-10-27.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(all_res, fh, ensure_ascii=False)
    print("\njson:", path)


if __name__ == "__main__":
    sys.exit(main())
