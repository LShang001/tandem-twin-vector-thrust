#!/usr/bin/env python3
"""run_worst_case_search.py — 参数边界差分进化（DE）最坏情况搜索。

@file    run_worst_case_search.py
@module  analysis/最坏情况搜索（hifi 6DOF + 固件控制/EKF）
@brief   在 STRESS_PARAMS 边界内用 DE 主动最大化归一化位置/姿态/高度误差，
         输出最坏参数组合、对应轨迹指标与评估历史。
@details 输入：--pop/--gens/--workers/--seed/--out；处理：每代 16 个候选并行跑
         40 s 复合场景（定高+位置保持+磁矢量+光流+20°倾斜+阵风+GNSS失联），
         目标函数 = 归一化误差加权和；输出 worst.json + history.csv + manifest。
@theory  Differential Evolution (Storn & Price 1997)；目标是工程设计边界搜索，
         不是概率估计。随机 MC 给分布，DE 给边界；两者不能互相替代。
@frame   NED + FRD；姿态 deg、位置 m。
@units   SI。
@note    这是 L1 hifi 模型 + 理想磁/光流；边界参数是搜索产物，不代表真实
         飞机允许范围，也不能外推 HIL/实机。
@see     run_param_mc.py、FULL-ENVELOPE-STATUS.md
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent.parent
sys.path.insert(0, str(HERE))

import run_param_mc as mc  # noqa: E402

NAMES = [p[0] for p in mc.STRESS_PARAMS]
BOUNDS = [(p[1], p[2], p[3]) for p in mc.STRESS_PARAMS]
D = len(NAMES)


def unit_to_row(u: np.ndarray) -> dict:
    """把 DE 单位超立方体候选映射为物理参数行。"""
    row = {}
    for j, (name, lo, hi, log) in enumerate(mc.STRESS_PARAMS):
        row[name] = mc.map_unit(float(u[j]), lo, hi, log)
    return row


def severity(r: dict) -> float:
    """归一化最坏严重度：位置/姿态/高度误差相对门限的最大值。"""
    if not r.get("finite", 0):
        return 10.0
    return float(max(r["pos_ne"] / mc.LIMITS["pos_ne"],
                     r["att_deg"] / mc.LIMITS["att_deg"],
                     r["alt_err"] / mc.LIMITS["alt_err"]))


def evaluate(pool: ProcessPoolExecutor, pop: np.ndarray, base_id: int) -> list[dict]:
    """并行评估一个种群；返回 run_one 指标字典列表。"""
    items = [(base_id + i, unit_to_row(pop[i]), True) for i in range(len(pop))]
    return list(pool.map(mc.run_one, items))


def main() -> int:
    """CLI 入口：DE 初始化→逐代变异/交叉/选择→worst.json。"""
    ap = argparse.ArgumentParser()
    ap.add_argument("--pop", type=int, default=16)
    ap.add_argument("--gens", type=int, default=10)
    ap.add_argument("--workers", type=int, default=max(1, min(10, os.cpu_count() or 1)))
    ap.add_argument("--seed", type=int, default=20261024)
    ap.add_argument("--f", type=float, default=0.7)
    ap.add_argument("--cr", type=float, default=0.9)
    ap.add_argument("--out", default=str(HERE.parent.parent / "sim-data" /
                                          "campaign-2026-10-24" / "worst-case-de"))
    args = ap.parse_args()
    outdir = Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(args.seed)
    pop = rng.random((args.pop, D))
    t0 = time.time()
    history = []
    best = None
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for gen in range(args.gens + 1):
            res = evaluate(pool, pop, gen * args.pop)
            for r in res:
                r["score"] = severity(r)
                r["gen"] = gen
                history.append(r)
                if best is None or r["score"] > best["score"]:
                    best = r
            scores = np.array([severity(r) for r in res])
            print(f"[gen {gen}] best={best['score']:.3f} "
                  f"mean={scores.mean():.3f} elapsed={time.time()-t0:.1f}s",
                  flush=True)
            if gen == args.gens:
                break
            trial = np.empty_like(pop)
            for i in range(args.pop):
                idx = [j for j in range(args.pop) if j != i]
                r1, r2, r3 = rng.choice(idx, 3, replace=False)
                mutant = np.clip(pop[r1] + args.f * (pop[r2] - pop[r3]), 0.0, 1.0)
                cross = rng.random(D) < args.cr
                if not np.any(cross):
                    cross[rng.integers(D)] = True
                trial[i] = np.where(cross, mutant, pop[i])
            trial_res = evaluate(pool, trial, 100000 + gen * args.pop)
            trial_scores = np.array([severity(r) for r in trial_res])
            for i in range(args.pop):
                if trial_scores[i] > scores[i]:
                    pop[i] = trial[i]
    keys = ["gen", "score", "case", "pos_ne", "att_deg", "alt_err", "finite",
            "fail", "w_hover"] + NAMES
    with (outdir / "history.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        w.writerows(history)
    try:
        head = subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD"],
                                       text=True).strip()
    except Exception:  # noqa: BLE001
        head = "unknown"
    worst = {k: best.get(k) for k in keys if k in best}
    summary = {
        "best": worst,
        "evaluations": len(history),
        "generations": args.gens,
        "pop": args.pop,
        "elapsed_s": time.time() - t0,
        "git_head": head,
        "seed": args.seed,
        "note": "L1 hifi + ideal mag/flow; boundary search, not HIL/real",
        "params": [{"name": n, "low": lo, "high": hi, "log": lg}
                   for n, lo, hi, lg in mc.STRESS_PARAMS],
    }
    (outdir / "worst.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(worst, ensure_ascii=False, indent=2))
    print("out:", outdir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
