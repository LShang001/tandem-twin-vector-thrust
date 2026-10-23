#!/usr/bin/env python3
"""run_vibration_adaptive_mc.py — 振动自适应 EKF 过程噪声 Q 的随机拉偏 MC。

@file    run_vibration_adaptive_mc.py
@module  固件/宿主研究（fw-tests：振动自适应 Q 的 MC 驱动）
@brief   在同一场景下成对比较固定 Q 与振动自适应 Q 的 3D 位置/速度误差。
@details 输入：--cases/--seed/--bin/--out；使用 test_ekf_highspeed_realistic 的
         高保真传感器/动力学模型（ICM-42688 噪声、UBX PVT、机动、振动、
         GNSS 失锁、延时、杆臂）。
         处理：subprocess 调用可执行体，正则解析 `age-comp: … pos3d=… vel=…`；
         输出：CSV 明细 + JSON 分位统计。
@theory  自适应过程噪声：把 IMU 振动量级映射成加计方差 Q，抑制“振动被当成
         真实加速度”导致的航向/速度踢动；判据为位置误差的配对差。
         @ref REF-121 §2.2.1 p.33（Q 在预测步的作用）、@ref AGENTS.md（Q 0.25 不可无实测下调）
@frame   NED（3D 位置误差范数）。
@units   位置 m，速度 m/s，振动 g，频率 Hz。
@note    这是研究脚本，不参与 run_all.sh；成对跑同一 seed/场景才可比较，
         单独跑一组没有统计意义。
@see     TandemVec_FCS/test_host/test_ekf_highspeed_realistic.cpp、
         TandemVec_FCS/include/ins_vibration_adaptive.h
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
OUT = ROOT / "TandemVec-EKF-Report" / "sim-data"
CSV = OUT / "vibration-adaptive-mc-2026-10-22.csv"
JSON = OUT / "vibration-adaptive-mc-2026-10-22.json"


def run(bin_path: str, args: list[str]) -> tuple[float, float]:
    p = subprocess.run([bin_path, *map(str, args)], capture_output=True,
                       text=True, timeout=30)
    out = p.stdout
    m = re.search(r"age-comp:.*pos3d=([0-9.]+).*vel=([0-9.]+)", out)
    if not m:
        raise RuntimeError("parse failed:\n" + out + "\n" + p.stderr)
    return float(m.group(1)), float(m.group(2))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases", type=int, default=500)
    ap.add_argument("--seed", type=int, default=20261022)
    ap.add_argument("--bin", default=r"C:\Users\12631\AppData\Local\Temp\hsreal.exe")
    ap.add_argument("--out", default=str(CSV))
    args = ap.parse_args()
    rng = random.Random(args.seed)
    rows = []

    for i in range(args.cases):
        seed = rng.randint(1, 10_000_000)
        speed = rng.uniform(5.0, 60.0)
        latency = rng.uniform(0.0, 0.10)
        maneuver = rng.choice([0, 1, 2, 3])
        vib_g = rng.uniform(0.0, 2.5)
        vib_mode = rng.choice([0, 1])
        vib_hz = rng.uniform(20.0, 200.0)
        gyro_vib = rng.uniform(0.0, 2.0)
        gnss_pos = rng.uniform(1.5, 5.0)
        gnss_vel = rng.uniform(0.05, 0.20)
        outage = rng.random() < 0.20
        outage_start = rng.uniform(5.0, 30.0) if outage else 1.0e9
        outage_end = outage_start + rng.uniform(5.0, 15.0) if outage else 1.0e9

        base = [
            f"{speed:.6f}", f"{latency:.6f}", "0.25", "0.0015",
            "0.00524", f"{gnss_pos:.6f}", f"{gnss_vel:.6f}",
            str(seed), str(maneuver), "0.01", f"{vib_g:.6f}",
        ]
        common = [
            "0", str(vib_mode), f"{gyro_vib:.6f}", f"{vib_hz:.6f}",
            f"{outage_start:.6f}", f"{outage_end:.6f}",
        ]
        try:
            p_fixed, v_fixed = run(args.bin, base + ["0"] + common)
            p_adapt, v_adapt = run(args.bin, base + ["1"] + common)
            ok = True
        except Exception as exc:  # noqa: BLE001
            p_fixed = p_adapt = v_fixed = v_adapt = float("nan")
            ok = False
            print(f"[WARN] case {i}: {exc}")

        ratio = (p_adapt / p_fixed) if (math.isfinite(p_fixed) and p_fixed > 1e-9) else float("nan")
        rows.append({
            "case": i,
            "seed": seed,
            "speed_mps": speed,
            "latency_ms": latency * 1000.0,
            "maneuver": maneuver,
            "vib_g": vib_g,
            "vib_mode": vib_mode,
            "vib_hz": vib_hz,
            "gyro_vib_dps": gyro_vib,
            "gnss_pos_m": gnss_pos,
            "gnss_vel_mps": gnss_vel,
            "outage": int(outage),
            "outage_len_s": (outage_end - outage_start) if outage else 0.0,
            "pos3d_fixed_m": p_fixed,
            "pos3d_adapt_m": p_adapt,
            "vel_fixed_mps": v_fixed,
            "vel_adapt_mps": v_adapt,
            "ratio": ratio,
            "adapt_worse_5pct": int(ok and ratio > 1.05),
            "adapt_worse_10pct": int(ok and ratio > 1.10),
            "ok": int(ok),
        })

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8", newline="") as f:
        cols = list(rows[0].keys())
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)

    ok_rows = [r for r in rows if r["ok"]]
    ratios = [r["ratio"] for r in ok_rows if math.isfinite(r["ratio"])]
    fixed = [r["pos3d_fixed_m"] for r in ok_rows]
    adapt = [r["pos3d_adapt_m"] for r in ok_rows]
    summary = {
        "cases": len(rows),
        "ok": len(ok_rows),
        "fixed_mean": sum(fixed) / len(fixed),
        "adapt_mean": sum(adapt) / len(adapt),
        "fixed_p50": sorted(fixed)[len(fixed) // 2],
        "adapt_p50": sorted(adapt)[len(adapt) // 2],
        "fixed_p95": sorted(fixed)[int(0.95 * (len(fixed) - 1))],
        "adapt_p95": sorted(adapt)[int(0.95 * (len(adapt) - 1))],
        "mean_improve_pct": 100.0 * (1.0 - sum(adapt) / sum(fixed)),
        "median_ratio": sorted(ratios)[len(ratios) // 2],
        "worse_5pct": sum(r["adapt_worse_5pct"] for r in ok_rows),
        "worse_10pct": sum(r["adapt_worse_10pct"] for r in ok_rows),
        "worst_ratio": max(ratios),
        "best_ratio": min(ratios),
    }
    with JSON.open("w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print("CSV:", out)
    print("JSON:", JSON)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
