#!/usr/bin/env python3
"""@file    verify_preflight.py
@module  tools/飞前状态验收
@brief   读取 tvc record CSV，检查未解锁状态安全 + IMU P1 采样。
@details 默认只检查软件/输出安全；--expect-powered 额外要求电池>0且RC有脉宽。
"""
from __future__ import annotations

import argparse
import csv
import statistics
import sys


def fvals(rows, key):
    out = []
    for r in rows:
        try:
            out.append(float(r.get(key, "")))
        except (TypeError, ValueError):
            pass
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--expect-powered", action="store_true")
    args = ap.parse_args()
    try:
        rows = list(csv.DictReader(open(args.csv, newline="", encoding="utf-8")))
    except OSError as e:
        print(f"[FAIL] 读取失败: {e}")
        return 2
    if not rows:
        print("[FAIL] 空 CSV")
        return 2
    errs = []
    rate = fvals(rows, "imu_rate_hz")
    drop = fvals(rows, "imu_ring_drop")
    if not rate:
        errs.append("无 imu_rate_hz")
    else:
        mean = statistics.mean(rate)
        std = statistics.pstdev(rate) if len(rate) > 1 else 0.0
        print(f"imu_rate mean={mean:.1f} std={std:.1f} n={len(rate)}")
        if mean < 1950.0 or std > 100.0:
            errs.append(f"imu_rate未达标 mean={mean:.1f}/std={std:.1f}")
    if drop and max(drop) != 0.0:
        errs.append(f"ring_drop={max(drop):.0f}")
    for key, limit in (("flight_mode", 0.01), ("unlocked", 0.01), ("tvc_upper_deg", 0.1),
                       ("tvc_lower_deg", 0.1), ("motor_front_pct", 0.1),
                       ("motor_rear_pct", 0.1)):
        v = fvals(rows, key)
        if not v:
            errs.append(f"缺列 {key}")
            continue
        if max(abs(x) for x in v) > limit:
            errs.append(f"{key}非零 max={max(v):.2f}")
    # 供电/RC只报告，默认不作为软件失败；实机前用 --expect-powered 强制
    bat = max(fvals(rows, "bat_voltage_v") or [0.0])
    rc = max(fvals(rows, "rc3") or [0.0])
    print(f"battery_max={bat:.2f}V rc3_max={rc:.0f}")
    if args.expect_powered:
        if bat <= 1.0:
            errs.append("电池电压为0（未接动力电池）")
        if rc <= 0:
            errs.append("RC无有效脉宽（接收机未接）")
    if errs:
        print("[FAIL] " + "; ".join(errs))
        return 1
    print("[PASS] 未解锁输出安全 + IMU P1")
    return 0


if __name__ == "__main__":
    sys.exit(main())
