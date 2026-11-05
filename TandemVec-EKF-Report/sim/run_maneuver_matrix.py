# -*- coding: utf-8 -*-
"""机动导航矩阵驱动（真实导航 SIL）。
@file    run_maneuver_matrix.py
@brief   SIL 速度/机动导航矩阵驱动。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 驱动 fw_real.exe 的多方向/腿长/故障组合。每条用例：先定高飞到 t=20 s，
         再切 AUTO_POSITION 给一次性水平目标偏移（--pos-leg）；可选 5 s GNSS
         失联或横风在 t=25..30 s 与机动重叠。SIL 输出的 JSON 只含 EKF 对真值
         的误差与姿态，目标跟踪与速度从 stdout 打印的 1 Hz 真值样本回捞。
@frame   NED（水平位置目标 leg_n/leg_e；误差为 EKF 对真值）
@units   m、m/s、s、deg（姿态/航向误差）
"""
import argparse
import concurrent.futures as cf
import json
import math
import os
import re
import subprocess
import sys

EXE = r"C:\Users\12631\AppData\Local\Temp\inc_rate\fw_real.exe"
OUTDIR = r"C:\Users\12631\AppData\Local\Temp\man2"
DIRS = {"N": (1, 0), "E": (0, 1), "S": (-1, 0), "W": (0, -1), "NE": (1, 1)}
DISTS = [3.0, 10.0, 25.0, 50.0]
CONDS = ["nominal", "outage", "wind", "both"]
SEEDS = range(1, 41)
SECONDS = 45
STEP_AT = 20.0
FAULT = "25:30"

TRUTH_RE = re.compile(r"^t=\s*([0-9.]+).*?truth=\(\s*(-?[0-9.]+),\s*(-?[0-9.]+)\)")
JSON_RE = re.compile(r"SIL_REAL_JSON (\{.*\})")


def one(task):
    """跑一次 SIL 机动任务，返回一行结果 dict（含 leg_n/leg_e、峰值速度、终点误差与失效判据）。

    task = (工况, 方向, 腿长 m, seed)：腿长按方向单位向量分解成 N/E 目标偏移传给 --pos-leg。
    从 stdout 的 1 Hz 真值重算峰值速度与终点相对目标误差；JSON 指标原样展开。
    超时（180 s）或无 JSON 时返回带 timeout/no_json 标记的记录，不抛异常以免中断整批。
    """
    cond, direction, dist, seed = task
    dn, de = DIRS[direction]
    norm = math.hypot(dn, de)
    leg_n = dist * dn / norm
    leg_e = dist * de / norm
    tag = f"{cond}_{direction}_{int(dist)}_s{seed}"
    log = os.path.join(OUTDIR, tag + ".log")
    cmd = [EXE, "--real-nav", "--profile", "mag_clean", "--pos-hold",
           "--pos-hold-at", str(STEP_AT), "--pos-leg", f"{leg_n:.3f}:{leg_e:.3f}",
           "--flow", "--seed", str(seed), "--seconds", str(SECONDS)]
    if cond in ("outage", "both"):
        cmd += ["--gnss-outage", FAULT]
    if cond in ("wind", "both"):
        cmd += ["--wind-window", FAULT, "--wind-force", "1.0"]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
        text = r.stdout + r.stderr
    except subprocess.TimeoutExpired:
        return dict(tag=tag, cond=cond, dir=direction, dist=dist, seed=seed,
                    timeout=True)
    with open(log, "w", encoding="utf-8") as f:
        f.write(text)
    mm = JSON_RE.findall(text)
    if not mm:
        return dict(tag=tag, cond=cond, dir=direction, dist=dist, seed=seed,
                    no_json=True)
    d = json.loads(mm[-1].replace("nan", "NaN"))
    truth = []
    for line in text.splitlines():
        m = TRUTH_RE.match(line)
        if m:
            truth.append((float(m.group(1)), float(m.group(2)), float(m.group(3))))
    peak_speed = 0.0
    for a, b in zip(truth, truth[1:]):
        dt = b[0] - a[0]
        if dt > 0 and a[0] >= 10.0:
            peak_speed = max(peak_speed, math.hypot(b[1] - a[1], b[2] - a[2]) / dt)
    end_err = None
    if truth:
        end_err = math.hypot(truth[-1][1] - leg_n, truth[-1][2] - leg_e)
    out = dict(tag=tag, cond=cond, dir=direction, dist=dist, seed=seed,
               leg_n=leg_n, leg_e=leg_e, peak_speed=peak_speed,
               end_target_err=end_err)
    for k in ("pos_rms", "pos_max", "max_roll", "max_pitch", "final_alt",
              "heading_aiding", "pos_degraded", "outlier_rejects",
              "reacquire_resets", "gnss_susp", "gnss_crej", "mag_gate"):
        out[k] = d.get(k)
    out["nav_fail"] = bool(d["pos_rms"] > 3.0 or d["pos_max"] > 10.0
                           or abs(d["final_alt"] - 10.0) > 3.0
                           or d["max_roll"] > 45.0)
    out["stab_fail"] = bool(abs(d["final_alt"] - 10.0) > 3.0
                            or d["max_roll"] > 45.0)
    return out


def main():
    """入口：枚举全矩阵（4 工况 × 21 个方向-腿长组合 × 40 seeds = 3360 次），进程池跑并写 jsonl。

    --limit 只截前 N 个任务用于冒烟；结果逐行 flush 到 OUTDIR/results.jsonl，便于长跑中断续看。
    """
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--limit", type=int, default=0)
    args = ap.parse_args()
    os.makedirs(OUTDIR, exist_ok=True)
    tasks = []
    for cond in CONDS:
        for direction in DIRS:
            for dist in ([0.0] if direction == "N" else []) + DISTS:
                for seed in SEEDS:
                    tasks.append((cond, direction, dist, seed))
    if args.limit:
        tasks = tasks[:args.limit]
    print(f"tasks={len(tasks)} workers={args.workers}", flush=True)
    out_path = os.path.join(OUTDIR, "results.jsonl")
    done = 0
    with open(out_path, "w", encoding="utf-8") as f, \
            cf.ProcessPoolExecutor(max_workers=args.workers) as ex:
        for res in ex.map(one, tasks, chunksize=4):
            f.write(json.dumps(res) + "\n")
            f.flush()
            done += 1
            if done % 100 == 0:
                print(f"{done}/{len(tasks)}", flush=True)
    print("DONE", out_path, flush=True)


if __name__ == "__main__":
    sys.exit(main())
