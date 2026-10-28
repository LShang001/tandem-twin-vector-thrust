#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24u Z 组：新默认上限（25 m）下的可靠性交互矩阵——故障格不该因为子集而变差。

@file    run_vel_subset_fault_matrix.py
@brief   30.5 g / 20 Hz，四类故障格 × 两臂（子集关 / 子集开·默认 cap 25）× N seeds，
         逐格记录 pos_rms / pos_max / 拒帧 / 重锚，产出与 §4.16 同口径的故障交互表
@module  TandemVec-EKF-Report/仿真驱动（L2 研究设施）
@details 为什么必须在这一轮做：§4.16 的教训是"新特性收益 A/B 通过 ≠ 安全"——
         速度子集在 30.5 g + 1 s IMU NaN 上曾经造成 332→551 m 的回归（已由 5 s 故障抑制锁存修掉）。
         本轮把触发上限从 15 m 改到 25 m，等于放宽了子集的触发条件（更多帧可以触发），
         因此**必须重跑同一类故障格**，确认放宽没有把那类回归重新引回来。
         四格覆盖"预测通道失效"（nan/stuck）、"量测缺失"（GNSS 失联 5/20 s）、
         "量测被振动污染"（1 g/80 Hz）——正好是 §4.16 里出现过分歧的几类。
@frame   NED；pos_rms / pos_max [m]、nav_att_rmse [deg]
@units   米 / 度 / 次 / 个 seed
@note    口径：两臂共用同一二进制 `tv_32g_q2`（md5 499495e5…，编译期默认 cap=25 m），
         只差运行时 `--gnss-vel-subset 0|1` ⇒ 同二进制 A/B。
         JSON 的 `ad_tau_ms` 等末帧统计量不参与判定（本工程已多次踩末帧坑）。
@see     sim/check_vel_subset_fault.py（Z1–Z3 裁决）、报告 §4.24u
"""
import argparse
import json
import os
import re
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
JSON_RE = re.compile(r"SIL_REAL_JSON (\{.*\})")
BASE = ["--real-nav", "--missile", "--seconds", "45", "--imu-fs-g", "32",
        "--imu-health-accel-range", "313.8", "--profile", "mag_clean",
        "--imu-grade", "icm", "--gnss-rate-hz", "20", "--ms-thrust", "23900"]
# (格名, 追加参数)
CELLS = [("nan1s", "--imu-fault nan --imu-fault-window 12:13"),
         ("out5", "--gnss-outage 20:25"),
         ("out20", "--gnss-outage 25:45"),
         ("vib1g", "--vib 1 --vib-hz 80 --vib-start 2")]
ARMS = [("off", "--gnss-vel-subset 0"), ("on", "--gnss-vel-subset 1")]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", required=True)
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--seed-start", type=int, default=51)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--bind-core", type=int, default=5)
    ap.add_argument("--tag", default="2026-10-28_z")
    args = ap.parse_args()
    if args.bind_core >= 0:
        import psutil
        psutil.Process().cpu_affinity([args.bind_core])

    from concurrent.futures import ThreadPoolExecutor

    def one(job):
        cell, arm, seed = job
        cmd = [args.exe] + BASE + ["--seed", str(seed)]
        cmd += dict(CELLS)[cell].split() + dict(ARMS)[arm].split()
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
        m = JSON_RE.findall(r.stdout + r.stderr)
        if not m:
            return None
        d = json.loads(m[-1].replace("nan", "NaN").replace("inf", "Infinity"))
        d["cell"] = cell
        d["arm"] = arm
        d["seed"] = seed
        return d

    seeds = list(range(args.seed_start, args.seed_start + args.seeds))
    jobs = [(c, a, s) for c, _ in CELLS for a, _ in ARMS for s in seeds]
    rows = []
    with ThreadPoolExecutor(max_workers=args.workers) as ex:
        for d in ex.map(one, jobs):
            if d is not None:
                rows.append(d)
    out = os.path.join(SD, "vel_subset_fault_%s.jsonl" % args.tag)
    with open(out, "w", encoding="utf-8") as f:
        for d in rows:
            f.write(json.dumps(d, ensure_ascii=False) + "\n")
    print("rows", len(rows), "expect", len(jobs))
    print("written", out)


if __name__ == "__main__":
    main()
