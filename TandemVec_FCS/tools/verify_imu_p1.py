#!/usr/bin/env python3
"""@file    verify_imu_p1.py
@module  tools/IMU P1验收
@brief   读取 tvc record CSV，判定 IMU 2kHz/P1 是否达标。
@details 只读 CSV；要求 imu_rate_hz 均值≥1950、std≤100、imu_ring_drop 全 0。
"""
from __future__ import annotations

import argparse
import csv
import statistics
import sys


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    args = ap.parse_args()
    try:
        with open(args.csv, newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
    except OSError as e:
        print(f"[FAIL] 读取失败: {e}")
        return 2
    rates = []
    drops = []
    for r in rows:
        try:
            rates.append(float(r.get("imu_rate_hz", "")))
            drops.append(float(r.get("imu_ring_drop", "")))
        except (TypeError, ValueError):
            pass
    if not rates:
        print("[FAIL] 无 imu_rate_hz 数据（需新固件 0xF4 遥测）")
        return 2
    mean = statistics.mean(rates)
    std = statistics.pstdev(rates) if len(rates) > 1 else 0.0
    max_drop = max(drops) if drops else 0.0
    ok = mean >= 1950.0 and std <= 100.0 and max_drop == 0.0
    print(f"imu_rate mean={mean:.1f} Hz std={std:.1f} "
          f"min={min(rates):.1f} max={max(rates):.1f} "
          f"ring_drop_max={max_drop:.0f} n={len(rates)}")
    print("[PASS] P1 2kHz" if ok else "[FAIL] 未达 mean≥1950/std≤100/drop=0")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
