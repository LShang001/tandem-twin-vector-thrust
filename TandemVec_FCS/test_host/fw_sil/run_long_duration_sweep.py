#!/usr/bin/env python3
"""run_long_duration_sweep.py — 单机型长航时失效起始点扫描。

@file    run_long_duration_sweep.py
@module  固件/宿主 SIL（长航时边界）
@brief   对同一机型/场景跑 40/60/80/100/120 s，记录高度/位置/姿态/IMU 健康
@details 只调用现有 SIL 二进制，不改固件；用于找“从哪个时长开始失效”。
         结果仅对该 SIL 场景有效，不是实机长航时结论。
"""
import argparse
import json
import math
import re
import subprocess
from pathlib import Path


def finite(v):
    return isinstance(v, (int, float)) and math.isfinite(float(v))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", required=True)
    ap.add_argument("--profile", default="mag_clean")
    ap.add_argument("--extra", default="--no-wind --mag")
    ap.add_argument("--durations", default="40,60,80,100,120")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    bin_path = Path(args.bin)
    if not bin_path.exists():
        print(f"[FAIL] binary not found: {bin_path}")
        return 2

    rows = []
    for seconds in [float(x) for x in args.durations.split(",") if x]:
        cmd = [str(bin_path), "--real-nav", "--profile", args.profile,
               "--pos-hold", "--seconds", str(seconds), "--seed",
               str(args.seed), *args.extra.split()]
        proc = subprocess.run(cmd, capture_output=True, text=True,
                              timeout=int(seconds * 10 + 120))
        data = None
        for line in reversed(proc.stdout.splitlines()):
            if line.startswith("SIL_REAL_JSON "):
                raw = re.sub(r"(?<=[:\[,])-?(?:nan|inf)(?=[,}\]])", "null",
                             line[len("SIL_REAL_JSON "):], flags=re.IGNORECASE)
                data = json.loads(raw)
                break
        row = dict(seconds=seconds, ok=0)
        if data is not None:
            row.update(dict(
                final_alt=data.get("final_alt"),
                hold_end=data.get("hold_end_rms"),
                max_roll=data.get("max_roll"),
                max_pitch=data.get("max_pitch"),
                imu_state=data.get("imu_state"),
                imu_frz=data.get("imu_frz"),
                pos_degraded=data.get("pos_degraded")))
            ok = (data.get("nav_init") == 1
                  and finite(row["final_alt"])
                  and 8.0 <= row["final_alt"] <= 11.0
                  and finite(row["hold_end"]) and row["hold_end"] <= 6.0
                  and finite(row["max_roll"]) and row["max_roll"] <= 30.0
                  and finite(row["max_pitch"]) and row["max_pitch"] <= 30.0
                  and row["imu_state"] != 2)
            row["ok"] = 1 if ok else 0
        rows.append(row)
        print(f"[{'PASS' if row['ok'] else 'FAIL'}] {seconds:5.0f} s "
              f"alt={row.get('final_alt')} hold={row.get('hold_end')} "
              f"roll={row.get('max_roll')} pitch={row.get('max_pitch')} "
              f"imu={row.get('imu_state')} frz={row.get('imu_frz')}")

    out = dict(bin=str(bin_path), profile=args.profile, seed=args.seed,
               extra=args.extra, rows=rows)
    out_path = Path(args.out) if args.out else (
        Path("sim-data") / "long-duration-2026-10-24" /
        f"long_duration_{args.profile}_seed{args.seed}.json")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(out, ensure_ascii=False, indent=2),
                        encoding="utf-8")
    print(f"saved: {out_path}")
    return 0 if all(r["ok"] for r in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
