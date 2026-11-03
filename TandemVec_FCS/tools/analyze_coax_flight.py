#!/usr/bin/env python3
"""@file    analyze_coax_flight.py
@module  tools/共轴飞行记录分析
@brief   汇总 `tvc record` CSV 的姿态/TVC/电机/饱和证据。
@details 共轴飞行记录快速分析（TandemVec GCS `tvc record` CSV）。

用法:
  py -3.12 TandemVec_FCS/tools/analyze_coax_flight.py flight.csv
  py -3.12 ... flight.csv --json

输出：姿态峰值、气压/融合高度范围、TVC 两轴/电机范围、饱和计数、
解锁/模式分布；数据缺失列自动跳过。不做控制结论，只做证据汇总。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from pathlib import Path


def fnum(row, key):
    try:
        v = float(row.get(key, ""))
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def stats(rows, key):
    vals = [v for r in rows if (v := fnum(r, key)) is not None]
    if not vals:
        return None
    return {
        "min": min(vals),
        "max": max(vals),
        "mean": sum(vals) / len(vals),
        "std": statistics.pstdev(vals) if len(vals) > 1 else 0.0,
    }


def count_true(rows, key):
    n = 0
    for r in rows:
        v = str(r.get(key, "")).strip().lower()
        if v in ("1", "true", "yes"):
            n += 1
    return n


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", type=Path)
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()
    if not args.csv.is_file():
        print(f"[FAIL] 文件不存在: {args.csv}")
        return 2
    with args.csv.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print("[FAIL] CSV 为空")
        return 2

    out = {
        "rows": len(rows),
        "duration_s": (fnum(rows[-1], "t_ms") or 0) / 1000.0,
        "roll_deg": stats(rows, "roll_deg"),
        "pitch_deg": stats(rows, "pitch_deg"),
        "heading_deg": stats(rows, "heading_deg"),
        "alt_bar_m": stats(rows, "alt_bar_m"),
        "alt_fu_m": stats(rows, "alt_fu_m"),
        "tvc_upper_deg": stats(rows, "tvc_upper_deg"),
        "tvc_lower_deg": stats(rows, "tvc_lower_deg"),
        "motor_front_pct": stats(rows, "motor_front_pct"),
        "motor_rear_pct": stats(rows, "motor_rear_pct"),
        "sat_df": count_true(rows, "sat_df"),
        "sat_dt": count_true(rows, "sat_dt"),
        "sat_dw": count_true(rows, "sat_dw"),
        "unlocked_rows": count_true(rows, "unlocked"),
        "modes": sorted({str(r.get("flight_mode", "")) for r in rows}),
    }
    if args.json:
        print(json.dumps(out, ensure_ascii=False, indent=2))
        return 0

    def line(name, st, unit=""):
        if st is None:
            print(f"{name:18s}  —")
        else:
            print(f"{name:18s} min={st['min']:8.3f} max={st['max']:8.3f} "
                  f"mean={st['mean']:8.3f} std={st['std']:6.3f} {unit}")

    print(f"rows={out['rows']}  duration={out['duration_s']:.2f}s  modes={out['modes']}  "
          f"unlocked_rows={out['unlocked_rows']}")
    line("roll_deg", out["roll_deg"], "deg")
    line("pitch_deg", out["pitch_deg"], "deg")
    line("heading_deg", out["heading_deg"], "deg")
    line("alt_bar_m", out["alt_bar_m"], "m")
    line("alt_fu_m", out["alt_fu_m"], "m")
    line("tvc_upper_deg", out["tvc_upper_deg"], "deg")
    line("tvc_lower_deg", out["tvc_lower_deg"], "deg")
    line("motor_front_pct", out["motor_front_pct"], "%")
    line("motor_rear_pct", out["motor_rear_pct"], "%")
    print(f"saturation: df={out['sat_df']} dt={out['sat_dt']} dw={out['sat_dw']} rows")
    return 0


if __name__ == "__main__":
    sys.exit(main())
