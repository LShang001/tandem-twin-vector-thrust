# -*- coding: utf-8 -*-
"""validation_stability.py — 修复后研究仿真的多 seed 稳定性回归（50 seeds）。

@file    validation_stability.py
@module  analysis/EKF 研究仿真（稳定性回归）
@brief   在恒定速度直线场景上跑 50 个随机种子，统计位置/速度误差、NIS 与拒绝率，
          确认 A-01/A-02/A-03 修复后没有引入默认配置的回归。
@details 输入：50 个种子（GNSS 噪声随机）；
          处理：调用 `run_scenarios.simulate_gnss(60 s)`（恒定速度 5 m/s 北 + 1 m/s 东，
          10 Hz GNSS，R 地板 2/3/0.15/0.25）；
          输出：`TandemVec-EKF-Report/sim-data/p0-stability-2026-10-23.json`
          与控制台分位数。
@theory  GNSS 位置/速度融合与 NIS 门控 @ref REF-121 §2.2.1 p.33、§7.3.1 p.255–259；
          P0 航向默认 0.2 rad @ref AGENTS.md（EKF 三风险，2026-08-13）。
@frame  NED + FRD。
@units   SI：m、m/s；NIS 无量纲。
@note    本场景为“直线匀速”，不覆盖强机动/失锁；强机动对照见
          `validation_p0_campaign.py`，两者结论不可互相替代。
@see     run_scenarios.py、validation_p0_campaign.py、DEC-002、MOD-004
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from run_scenarios import simulate_gnss  # noqa: E402

OUT = os.path.join(HERE, "..", "sim-data")


def one(seed: int) -> dict:
    """跑一个种子的 60 s 直线场景，返回 RMSE/NIS/拒绝统计。"""
    r = simulate_gnss(60.0, truth_vel=[5.0, 1.0, 0.0], rng_seed=seed)
    m = r["ts"] > 10.0
    pe = r["pos_err"][m]
    ve = r["vel_err"][m]
    nis = r["nis"][:, 1] if len(r["nis"]) else np.array([np.nan])
    return {
        "seed": seed,
        "pos_rmse": float(np.sqrt(np.mean(np.sum(pe ** 2, axis=1)))),
        "vel_rmse": float(np.sqrt(np.mean(np.sum(ve ** 2, axis=1)))),
        "final_pos": float(np.linalg.norm(r["pos_err"][-1])),
        "nis_mean": float(np.nanmean(nis)),
        "nis_frames": int(len(nis)),
    }


def main() -> int:
    """命令行入口：多进程跑种子集合并写 JSON。"""
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=50)
    ap.add_argument("--workers", type=int, default=min(8, os.cpu_count() or 4))
    ap.add_argument("--out", default=os.path.join(
        OUT, "p0-stability-2026-10-23.json"))
    args = ap.parse_args()
    from concurrent.futures import ProcessPoolExecutor
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        rows = list(ex.map(one, range(args.seeds)))
    p = np.array([r["pos_rmse"] for r in rows])
    v = np.array([r["vel_rmse"] for r in rows])
    n = np.array([r["nis_mean"] for r in rows])
    summary = {
        "seeds": args.seeds,
        "pos_rmse_mean": float(p.mean()),
        "pos_rmse_p95": float(np.percentile(p, 95)),
        "pos_rmse_max": float(p.max()),
        "vel_rmse_mean": float(v.mean()),
        "vel_rmse_p95": float(np.percentile(v, 95)),
        "nis_mean": float(np.nanmean(n)),
        "fail_over_5m": int((p > 5.0).sum()),
    }
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump({"summary": summary, "runs": rows}, f,
                  ensure_ascii=False, indent=2)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"-> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
