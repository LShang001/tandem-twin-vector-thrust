#!/usr/bin/env python3
"""run_real_acceptance.py — 真实导航任务 SIL 多配置验收。

@file    run_real_acceptance.py
@module  固件/宿主 SIL（fw-tests：real-nav 验收门）
@brief   对 clean/nominal/degraded 三套传感器/环境配置各跑 3 seeds，
         解析被测体输出的 SIL_REAL_JSON 行并按阈值判定。
@details 输入：--bin 被测可执行体；--seconds 仿真时长（默认 40 s）。
         处理：subprocess 运行 `--real-nav --profile <p> --pos-hold`；
         从 stdout 取最后一行 `SIL_REAL_JSON {...}`；
         输出：逐例 PASS/FAIL 表；任何失败退出码非 0。
@theory  N/A（验收驱动；阈值来自 campaign 报告与当前 SIL 基线）
@frame   NED 水平位置/高度；阈值均为米。
@units   m、s。
@note    degraded 的阈值是“故障/退化下仍不崩”的宽松门，不是飞行品质指标。
@see     test_host/fw_sil/README.md、run_real_acceptance.sh
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

PROFILES = {
    # profile: (extra args, gates)
    "clean": (
        ["--no-wind"],
        {"final_alt": (8.0, 11.0), "hold_end_rms": 1.5, "pos_rms": 1.0},
    ),
    "nominal": (
        [],
        {"final_alt": (8.0, 11.0), "hold_end_rms": 8.0, "pos_rms": 5.0},
    ),
    "degraded": (
        [],
        {"final_alt": (6.0, 12.0), "hold_end_rms": 25.0, "pos_rms": 25.0},
    ),
}


def parse_json_line(stdout: str) -> dict:
    for line in reversed(stdout.splitlines()):
        if line.startswith("SIL_REAL_JSON "):
            return json.loads(line[len("SIL_REAL_JSON "):])
    raise RuntimeError("no SIL_REAL_JSON line in output")


def finite(value) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", required=True)
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--seeds", default="1,2,3")
    ap.add_argument("--profiles", default="clean,nominal,degraded")
    args = ap.parse_args()
    seeds = [int(s) for s in args.seeds.split(",") if s]
    profiles = [p for p in args.profiles.split(",") if p]
    bin_path = Path(args.bin)
    if not bin_path.exists():
        print(f"[FAIL] binary not found: {bin_path}")
        return 2

    failures = 0
    rows = []
    for profile in profiles:
        if profile not in PROFILES:
            print(f"[FAIL] unknown profile: {profile}")
            return 2
        extra, gates = PROFILES[profile]
        for seed in seeds:
            cmd = [
                str(bin_path), "--real-nav", "--profile", profile,
                "--pos-hold", "--seconds", str(args.seconds),
                "--seed", str(seed), *extra,
            ]
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
            try:
                data = parse_json_line(proc.stdout)
            except Exception as exc:  # noqa: BLE001
                print(f"[FAIL] {profile} seed{seed}: {exc}")
                print(proc.stdout[-2000:])
                print(proc.stderr[-1000:])
                failures += 1
                continue

            errors = []
            if proc.returncode != 0:
                errors.append(f"returncode={proc.returncode}")
            if data.get("nav_init") != 1:
                errors.append("nav_init=0")
            if data.get("fix3d", 0) <= 0:
                errors.append("fix3d=0")
            for key in ("final_alt", "hold_end_rms", "pos_rms", "pos_max",
                        "max_roll", "max_pitch", "rms_roll", "rms_pitch"):
                if not finite(data.get(key)):
                    errors.append(f"{key}=nonfinite")
            lo, hi = gates["final_alt"]
            if not (lo <= float(data.get("final_alt", -1e9)) <= hi):
                errors.append(f"final_alt={data.get('final_alt')} not in [{lo},{hi}]")
            for key in ("hold_end_rms", "pos_rms"):
                if float(data.get(key, 1e9)) > gates[key]:
                    errors.append(f"{key}={data.get(key)} > {gates[key]}")

            status = "PASS" if not errors else "FAIL"
            if errors:
                failures += 1
            rows.append((status, profile, seed, data, errors))
            print(f"[{status}] {profile:8s} seed{seed} "
                  f"alt={data.get('final_alt'):.2f} "
                  f"hold_end={data.get('hold_end_rms'):.2f} "
                  f"hold_max={data.get('hold_max'):.2f} "
                  f"pos_rms={data.get('pos_rms'):.2f} "
                  f"roll_max={data.get('max_roll'):.2f} "
                  f"resets={data.get('reacquire_resets')} "
                  f"outliers={data.get('outlier_rejects')}")
            for e in errors:
                print(f"       - {e}")

    print(f"\n=== real-nav acceptance: {len(rows)} cases, {failures} failed ===")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
