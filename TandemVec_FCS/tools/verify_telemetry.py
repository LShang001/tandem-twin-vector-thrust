#!/usr/bin/env python3
"""@file    verify_telemetry.py
@module  tools/数传链路验收
@brief   读取 tvc record CSV，检查遥测帧率、帧间隔、IMU ring 溢出。
@details 通过条件：行率≥35Hz；gap p99≤60ms；最大gap≤100ms；无>100ms gap；
          imu_ring_drop=0。仅读 CSV，不连接设备。
"""
from __future__ import annotations

import argparse
import csv
import sys


def percentile(values, p):
    if not values:
        return 0.0
    s = sorted(values)
    idx = min(len(s) - 1, int(len(s) * p))
    return s[idx]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    args = ap.parse_args()
    try:
        rows = list(csv.DictReader(open(args.csv, newline="", encoding="utf-8")))
    except OSError as e:
        print(f"[FAIL] 读取失败: {e}")
        return 2
    t = sorted(float(r["t_ms"]) for r in rows if r.get("t_ms") not in ("", None))
    if len(t) < 10:
        print("[FAIL] 样本不足")
        return 2
    span_s = (t[-1] - t[0]) / 1000.0
    rate = (len(t) - 1) / span_s if span_s > 0 else 0.0
    gaps = [b - a for a, b in zip(t, t[1:])]
    p95, p99, mx = percentile(gaps, 0.95), percentile(gaps, 0.99), max(gaps)
    over100 = sum(1 for g in gaps if g > 100.0)
    over500 = sum(1 for g in gaps if g > 500.0)
    drops = [float(r.get("imu_ring_drop", 0)) for r in rows
             if r.get("imu_ring_drop") not in ("", None)]
    max_drop = max(drops) if drops else 0.0
    print(f"rows={len(t)} span={span_s:.1f}s rate={rate:.1f}Hz "
          f"gap_p95={p95:.1f}ms p99={p99:.1f}ms max={mx:.1f}ms "
          f">100ms={over100} >500ms={over500} imu_ring_drop={max_drop:.0f}")
    errs = []
    if rate < 35.0:
        errs.append(f"rate<35Hz ({rate:.1f})")
    if p99 > 60.0:
        errs.append(f"gap_p99>60ms ({p99:.1f})")
    if mx > 100.0 or over100:
        errs.append(f"gap>100ms (max={mx:.1f}, count={over100})")
    if max_drop != 0.0:
        errs.append(f"imu_ring_drop={max_drop:.0f}")
    print("[PASS] 数传链路稳定" if not errs else "[FAIL] " + "; ".join(errs))
    return 0 if not errs else 1


if __name__ == "__main__":
    sys.exit(main())
