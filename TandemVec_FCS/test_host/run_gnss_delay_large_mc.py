#!/usr/bin/env python3
"""run_gnss_delay_large_mc.py — 无 PPS GNSS 延时批处理的大规模拉偏 MC 驱动。

@file    run_gnss_delay_large_mc.py
@module  固件/宿主研究（fw-tests：GNSS 延时估计 MC 驱动）
@brief   对 test_gnss_delay_batch 类可执行体做大样本拉偏，统计延时估计误差分布。
@details 输入：--cases/--seed/--bin 等命令行参数；每例随机速度、真实延时、
         GNSS 位置/速度噪声、三轴加计零偏、失锁窗口。
         处理：subprocess 调用被测可执行体，正则解析
         `best_tau=… ms (error … ms), cost=…`；收集误差后算均值/P95/最大值与通过率。
         输出：TandemVec-EKF-Report/sim-data/ 下的 CSV 与 JSON 摘要。
@theory  批处理延时估计：用 IMU 加速度积分出参考轨迹，对候选 τ 做
         “初位置+初速度+加计零偏”线性拟合，取残差最小者；仅在机动段可观测，
         匀速/悬停段理论上不可观测（残差对 τ 不敏感）。
         @ref REF-120 §9.6 (9.6.9) p.298（速度/位置递推）
@frame   NED（参考轨迹与 GNSS 位置均为导航系）。
@units   延时 ms，速度 m/s，位置 m，加计零偏 m/s²。
@note    输出 CSV 列与历史报告绑定的字段顺序不要随意改；统计门限（20 ms 通过）
         属于验收口径，写死在报告侧，不在本脚本内。
@see     TandemVec_FCS/test_host/test_gnss_delay_batch.cpp
"""

from __future__ import annotations

import argparse
import json
import math
import random
import re
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT_DIR = ROOT.parent / "TandemVec-EKF-Report" / "sim-data"
CSV = OUT_DIR / "gnss-delay-large-mc-2026-10-22.csv"
JSON = OUT_DIR / "gnss-delay-large-mc-2026-10-22.json"


def run_case(bin_path: str, args: list[str]) -> tuple[float, float, float]:
    p = subprocess.run([bin_path, *args], capture_output=True, text=True,
                       timeout=20)
    out = p.stdout
    m = re.search(r"best_tau=([0-9.]+) ms \(error ([-0-9.]+) ms\), cost=([0-9.]+)",
                  out)
    if not m:
        raise RuntimeError("无法解析输出:\n" + out + "\n" + p.stderr)
    return float(m.group(1)), float(m.group(2)), float(m.group(3))


def pct(values: list[float], q: float) -> float:
    if not values:
        return float("nan")
    values = sorted(values)
    idx = min(len(values) - 1, max(0, int(round(q * (len(values) - 1)))))
    return values[idx]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases", type=int, default=2000)
    ap.add_argument("--seed", type=int, default=20261022)
    ap.add_argument("--bin", default="/tmp/gnss_batch")
    ap.add_argument("--out", default=str(CSV))
    ap.add_argument("--every", type=int, default=100)
    args = ap.parse_args()

    rng = random.Random(args.seed)
    rows = []
    t0 = time.time()
    for i in range(args.cases):
        maneuver = rng.choice([1, 2])
        speed = rng.uniform(5.0, 60.0)
        true_delay = rng.uniform(0.005, 0.120)
        gnss_sigma = 10 ** rng.uniform(math.log10(0.3), math.log10(5.0))
        vel_sigma = 10 ** rng.uniform(math.log10(0.02), math.log10(0.20))
        # 三轴加计零偏：单轴最大约 100 mg，随机符号
        bx = rng.uniform(-0.5, 0.5)
        by = rng.uniform(-0.5, 0.5)
        bz = rng.uniform(-0.5, 0.5)
        # 20% 案例有 3–10 s GNSS 失锁
        dropout = rng.random() < 0.20
        drop_start = rng.uniform(8.0, 35.0) if dropout else -1.0
        drop_len = rng.uniform(3.0, 10.0) if dropout else 0.0

        cli = [
            str(maneuver), str(i + 1), f"{true_delay:.6f}",
            f"{gnss_sigma:.6f}", f"{bx:.6f}", f"{vel_sigma:.6f}",
            f"{speed:.3f}", f"{by:.6f}", f"{bz:.6f}",
            f"{drop_start:.3f}", f"{drop_len:.3f}",
        ]
        try:
            best_tau, err, cost = run_case(args.bin, cli)
            ok = 1
        except Exception as exc:  # noqa: BLE001
            best_tau = err = cost = float("nan")
            ok = 0
            print(f"[WARN] case {i}: {exc}", file=sys.stderr)
        if (i + 1) % args.every == 0 or (i + 1) == args.cases:
            elapsed = time.time() - t0
            rate = (i + 1) / max(elapsed, 1e-9)
            eta = (args.cases - i - 1) / max(rate, 1e-9)
            print(f"[progress] {i+1}/{args.cases} "
                  f"({100.0*(i+1)/args.cases:5.1f}%) "
                  f"elapsed={elapsed:6.1f}s eta={eta:6.1f}s",
                  flush=True)
        rows.append({
            "case": i,
            "maneuver": maneuver,
            "speed_mps": speed,
            "true_delay_ms": true_delay * 1000.0,
            "gnss_sigma_m": gnss_sigma,
            "vel_sigma_mps": vel_sigma,
            "bias_x": bx,
            "bias_y": by,
            "bias_z": bz,
            "bias_norm": math.sqrt(bx * bx + by * by + bz * bz),
            "dropout": int(dropout),
            "drop_start_s": drop_start,
            "drop_len_s": drop_len,
            "best_tau_ms": best_tau,
            "error_ms": err,
            "abs_error_ms": abs(err) if math.isfinite(err) else float("nan"),
            "cost": cost,
            "pass_20ms": int(ok and abs(err) <= 20.0),
        })

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8", newline="") as f:
        cols = list(rows[0].keys())
        f.write(",".join(cols) + "\n")
        for r in rows:
            f.write(",".join(
                f"{r[c]:.6g}" if isinstance(r[c], float) else str(r[c])
                for c in cols) + "\n")

    errs = [abs(r["abs_error_ms"]) for r in rows if math.isfinite(r["abs_error_ms"])]
    passes = sum(r["pass_20ms"] for r in rows)
    summary = {
        "cases": len(rows),
        "pass_20ms": passes,
        "pass_rate": passes / len(rows),
        "mean_abs_error_ms": sum(errs) / len(errs),
        "p50_abs_error_ms": pct(errs, 0.50),
        "p90_abs_error_ms": pct(errs, 0.90),
        "p95_abs_error_ms": pct(errs, 0.95),
        "p99_abs_error_ms": pct(errs, 0.99),
        "max_abs_error_ms": max(errs),
        "by_maneuver": {},
        "by_delay_bin": {},
        "by_speed_bin": {},
        "by_gnss_sigma_bin": {},
        "by_bias_norm_bin": {},
        "by_dropout": {},
    }

    def group(key: str, buckets: dict[str, list[dict]]) -> None:
        for name, rs in buckets.items():
            es = [abs(r["abs_error_ms"]) for r in rs
                  if math.isfinite(r["abs_error_ms"])]
            summary[key][name] = {
                "n": len(rs),
                "pass_rate": sum(r["pass_20ms"] for r in rs) / len(rs),
                "p95_abs_error_ms": pct(es, 0.95),
                "mean_abs_error_ms": sum(es) / len(es),
            }

    group("by_maneuver", {
        "longitudinal": [r for r in rows if r["maneuver"] == 1],
        "3d_lateral_climb": [r for r in rows if r["maneuver"] == 2],
    })
    group("by_dropout", {
        "no_dropout": [r for r in rows if not r["dropout"]],
        "with_dropout": [r for r in rows if r["dropout"]],
    })

    def bucket(key: str, edges: list[float], param: str) -> None:
        buckets: dict[str, list[dict]] = {}
        for r in rows:
            v = float(r[param])
            for a, b in zip(edges[:-1], edges[1:]):
                if a <= v < b:
                    buckets.setdefault(f"[{a:g},{b:g})", []).append(r)
                    break
        group(key, buckets)

    bucket("by_delay_bin", [0, 20, 50, 80, 120, 121], "true_delay_ms")
    bucket("by_speed_bin", [0, 10, 30, 50, 61], "speed_mps")
    bucket("by_gnss_sigma_bin", [0, 0.5, 1.5, 3.0, 5.01], "gnss_sigma_m")
    bucket("by_bias_norm_bin", [0, 0.1, 0.3, 0.6, 0.87], "bias_norm")

    with JSON.open("w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"CSV:  {out}")
    print(f"JSON: {JSON}")
    return 0 if summary["pass_rate"] >= 0.90 else 1


if __name__ == "__main__":
    raise SystemExit(main())
