#!/usr/bin/env python3
"""run_vibration_policy_mc.py — 振动自适应 Q 策略的大规模 MC（带进度与检查点）。

@file    run_vibration_policy_mc.py
@module  固件/宿主研究（fw-tests：振动自适应策略 MC 驱动）
@brief   固定/自适应成对跑同一场景，输出进度行、CSV 检查点与最终 JSON 摘要。
@details 输入：--cases/--seed/--every/--flush-every/--bin 等参数；每例随机速度、
         延时、机动、振动（g/模式/频率）、GNSS 噪声、失锁、野值比例、
         加计尺度误差、安装失准、y 与 z 轴零偏。
         处理：同一组参数分别以“策略关/开”调用被测可执行体，解析 pos3d；
         输出：CSV 逐例写盘（可断点续跑）+ JSON 汇总 ratio 分布。
@theory  安全版 Policy C：只在振动门限内把 Q 从 0.25 收到 0.15，延时 >50 ms、
         失锁/恢复、hAcc >2 m 时回退 0.25；判据是“不出现 >5% 的成对退化”。
         @ref REF-121 §2.2.1 p.33、@ref AGENTS.md（振动自适应策略与默认开关）
@frame   NED（3D 位置误差范数）。
@units   位置 m，速度 m/s，延时 ms，振动 g，频率 Hz，角度 deg。
@note    大纲：CSV 字段顺序与历史 sim-data 兼容，改字段会破坏已有分析脚本。
@see     TandemVec_FCS/include/ins_vibration_adaptive.h
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import re
import statistics
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
OUT = ROOT / "TandemVec-EKF-Report" / "sim-data"
FIELDS = [
    "case", "seed", "speed_mps", "latency_ms", "maneuver", "vib_g",
    "vib_mode", "vib_hz", "gyro_vib_dps", "gnss_pos_m", "gnss_vel_mps",
    "outage", "outage_len_s", "outlier_frac", "accel_scale", "misalign",
    "bias_y", "bias_z", "pos_fixed_m", "pos_adapt_m", "ratio",
]


def parse_pos(out: str) -> float:
    m = re.search(r"age-comp:.*pos3d=([0-9.]+)", out)
    if not m:
        raise RuntimeError("parse failed:\n" + out)
    return float(m.group(1))


def run_pair(bin_path: str, args: list[str]) -> tuple[float, float]:
    p0 = subprocess.run([bin_path, *map(str, args), "0"],
                        capture_output=True, text=True, timeout=30)
    p1 = subprocess.run([bin_path, *map(str, args), "1"],
                        capture_output=True, text=True, timeout=30)
    return parse_pos(p0.stdout), parse_pos(p1.stdout)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases", type=int, default=10000)
    ap.add_argument("--seed", type=int, default=20261027)
    ap.add_argument("--every", type=int, default=100)
    ap.add_argument("--flush-every", type=int, default=500)
    ap.add_argument("--bin", default=r"C:\Users\12631\AppData\Local\Temp\hsreal.exe")
    ap.add_argument("--csv", default=str(OUT / "vibration-policy-10k.csv"))
    ap.add_argument("--json", default=str(OUT / "vibration-policy-10k.json"))
    args = ap.parse_args()

    rng = random.Random(args.seed)
    rows: list[dict] = []
    t0 = time.time()
    csv_path = Path(args.csv)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    f = csv_path.open("w", encoding="utf-8", newline="")
    writer = csv.DictWriter(f, fieldnames=FIELDS)
    writer.writeheader()
    f.flush()

    for i in range(args.cases):
        seed = rng.randint(1, 10_000_000)
        speed = rng.uniform(5.0, 60.0)
        lat = rng.uniform(0.0, 0.08)
        man = rng.choice([0, 1, 2, 3])
        vg = rng.uniform(0.0, 2.5)
        mode = rng.choice([0, 1])
        hz = rng.uniform(20.0, 200.0)
        gv = rng.uniform(0.0, 2.0)
        gp = rng.uniform(1.5, 5.0)
        gvel = rng.uniform(0.05, 0.20)
        outage = rng.random() < 0.15
        os_ = rng.uniform(5.0, 30.0) if outage else 1e9
        oe = os_ + rng.uniform(5.0, 12.0) if outage else 1e9
        of = rng.uniform(0.0, 0.02)
        scale = rng.uniform(0.97, 1.03)
        mis = rng.uniform(-0.01, 0.01)
        by = rng.uniform(-0.2, 0.2)
        bz = rng.uniform(-0.2, 0.2)
        common = [
            speed, lat, 0.25, 0.0015, 0.00524, gp, gvel, seed, man, 0.01, vg,
        ]
        sensor = [0, 0, 0, 0, scale, scale, mis, by, bz, of]
        # hsreal 参数顺序：
        # base 11 + adaptive + mode/gyro_vib/hz/outage/os/oe
        # + lever_m/lever_comp/latency_jitter/auto_delay/scale/gyro_scale/
        #   misalign/bias_y/bias_z/outlier
        def one(adapt: int) -> float:
            cli = common + [adapt, mode, gv, hz, os_, oe] + sensor
            p = subprocess.run([args.bin, *map(str, cli)], capture_output=True,
                               text=True, timeout=30)
            return parse_pos(p.stdout)

        fixed = one(0)
        adapt = one(1)
        ratio = adapt / fixed if fixed > 1e-9 else float("nan")
        row = {
            "case": i, "seed": seed, "speed_mps": speed, "latency_ms": lat * 1000.0,
            "maneuver": man, "vib_g": vg, "vib_mode": mode, "vib_hz": hz,
            "gyro_vib_dps": gv, "gnss_pos_m": gp, "gnss_vel_mps": gvel,
            "outage": int(outage), "outage_len_s": (oe - os_) if outage else 0.0,
            "outlier_frac": of, "accel_scale": scale, "misalign": mis,
            "bias_y": by, "bias_z": bz, "pos_fixed_m": fixed, "pos_adapt_m": adapt,
            "ratio": ratio,
        }
        rows.append(row)
        writer.writerow(row)

        if (i + 1) % args.every == 0 or (i + 1) == args.cases:
            elapsed = time.time() - t0
            rate = (i + 1) / max(elapsed, 1e-9)
            eta = (args.cases - i - 1) / max(rate, 1e-9)
            fm = statistics.mean(r["pos_fixed_m"] for r in rows)
            am = statistics.mean(r["pos_adapt_m"] for r in rows)
            print(
                f"[progress] {i+1}/{args.cases} "
                f"({100.0*(i+1)/args.cases:5.1f}%) "
                f"elapsed={elapsed:6.1f}s eta={eta:6.1f}s "
                f"mean_fixed={fm:.3f} mean_adapt={am:.3f}",
                flush=True,
            )
        if (i + 1) % args.flush_every == 0:
            f.flush()

    f.close()
    fixed = [r["pos_fixed_m"] for r in rows]
    adapt = [r["pos_adapt_m"] for r in rows]
    ratios = [r["ratio"] for r in rows if math.isfinite(r["ratio"])]
    summary = {
        "cases": len(rows),
        "fixed_mean": statistics.mean(fixed),
        "adapt_mean": statistics.mean(adapt),
        "mean_improve_pct": 100.0 * (1.0 - statistics.mean(adapt) / statistics.mean(fixed)),
        "fixed_p50": statistics.median(fixed),
        "adapt_p50": statistics.median(adapt),
        "fixed_p95": sorted(fixed)[int(0.95 * (len(fixed) - 1))],
        "adapt_p95": sorted(adapt)[int(0.95 * (len(adapt) - 1))],
        "median_ratio": statistics.median(ratios),
        "worse_5pct": sum(r > 1.05 for r in ratios),
        "worse_10pct": sum(r > 1.1 for r in ratios),
        "worst_ratio": max(ratios),
        "best_ratio": min(ratios),
    }
    with Path(args.json).open("w", encoding="utf-8") as jf:
        json.dump(summary, jf, ensure_ascii=False, indent=2)
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    print("CSV:", csv_path, flush=True)
    print("JSON:", args.json, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
