#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24y Y 组：配额门的故障交互与 10 Hz 惰性（同二进制 A/B）。

@file    run_vel_subset_quota_fault.py
@brief   30.5 g 下四故障格 + 10 Hz 主格，配额 1（新默认）vs 配额 255（取消）各 20 seeds
@module  TandemVec-EKF-Report/仿真驱动（L2 研究设施）
@details 为什么必须做：§4.16 的纪律是"每加一个门都要过故障矩阵"——§4.24w 只覆盖了
         20 Hz 的两段干净 seeds（结果：配额门完全惰性）。故障格与 10 Hz 都还没测：
         ① 故障格（IMU NaN / GNSS 失联 5·20 s / 振动）：若配额门在故障时咬到不该咬的
            触发，会把"本可以修的 seed"变成失效；
         ② 10 Hz：§4.24q 表明 10 Hz 的触发本来就是每 seed 1 次，配额理应惰性，
            但"理应"不是证据。
         ★ 两臂共用同一二进制（`tv_32g_w2` md5 `fbac7fe9…`），只差 `--gnss-vel-subset-maxfires`。
@frame   NED；pos_rms / pos_max [m]、nav_att_rmse [deg]
@units   米 / 度 / 次 / 个 seed
@note    口径：SIL 未识别参数会硬失败（§4.21.3），所以每个臂的参数都显式给出。
@see     sim/check_vel_subset_quota_fault.py（Y1–Y3）、报告 §4.24y
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
        "--imu-grade", "icm", "--ms-thrust", "23900"]
# (格名, 刷新率, 追加参数)
CELLS = [("nan1s", 20, "--imu-fault nan --imu-fault-window 12:13"),
         ("out5", 20, "--gnss-outage 20:25"),
         ("out20", 20, "--gnss-outage 25:45"),
         ("vib1g", 20, "--vib 1 --vib-hz 80 --vib-start 2"),
         ("hz10", 10, "")]
ARMS = [("q1", "--gnss-vel-subset-maxfires 1"), ("q255", "--gnss-vel-subset-maxfires 255")]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", required=True)
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--seed-start", type=int, default=51)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--bind-core", type=int, default=5)
    ap.add_argument("--tag", default="2026-10-28_y")
    args = ap.parse_args()
    if args.bind_core >= 0:
        import psutil
        psutil.Process().cpu_affinity([args.bind_core])

    from concurrent.futures import ThreadPoolExecutor

    def one(job):
        cell, rate, extra, arm, aextra, seed = job
        cmd = [args.exe] + BASE + ["--seed", str(seed), "--gnss-rate-hz", str(rate),
                                   "--gnss-vel-subset", "1"]
        if extra:
            cmd += extra.split()
        cmd += aextra.split()
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
        m = JSON_RE.findall(r.stdout + r.stderr)
        if not m:
            return None
        d = json.loads(m[-1].replace("nan", "NaN").replace("inf", "Infinity"))
        d.update({"cell": cell, "arm": arm, "seed": seed})
        return d

    seeds = list(range(args.seed_start, args.seed_start + args.seeds))
    jobs = [(c, r, e, a, ae, s) for c, r, e in CELLS for a, ae in ARMS for s in seeds]
    rows = []
    with ThreadPoolExecutor(max_workers=args.workers) as ex:
        for d in ex.map(one, jobs):
            if d is not None:
                rows.append(d)
    out = os.path.join(SD, "vel_subset_quota_fault_%s.jsonl" % args.tag)
    with open(out, "w", encoding="utf-8") as f:
        for d in rows:
            f.write(json.dumps(d, ensure_ascii=False) + "\n")
    print("rows", len(rows), "expect", len(jobs))
    print("written", out)


if __name__ == "__main__":
    main()
