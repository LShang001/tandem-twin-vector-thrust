#!/usr/bin/env python3
"""run_real_acceptance.py — 真实导航任务 SIL 多配置验收。

@file    run_real_acceptance.py
@module  固件/宿主 SIL（fw-tests：real-nav 验收门）
@brief   对 clean/nominal/degraded 三套传感器/环境配置各跑 3 seeds，
         解析被测体输出的 SIL_REAL_JSON 行并按阈值判定。
@details 输入：--bin 被测可执行体；--seconds 仿真时长（默认 40 s）；
         --profiles 可选 baseline（clean/nominal/degraded）、磁（mag_*）、
         IMU 故障（imu_*）与 GNSS 欺骗（gnss_spoof）。
         处理：subprocess 运行 `--real-nav --profile <p> --pos-hold`；
         从 stdout 取最后一行 `SIL_REAL_JSON {...}`（NaN/Inf 转 null）；
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
import re
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
    # 磁生产链独立 profile：磁默认不混入 clean/nominal/degraded 基线。
    "mag_clean": (
        ["--no-wind"],
        {"final_alt": (8.0, 11.0), "hold_end_rms": 1.5, "pos_rms": 1.0,
         "mag_fused_min": 100, "require_mag_cal": True,
         "max_roll_max": 20.0, "max_pitch_max": 20.0},
    ),
    "mag_nominal": (
        [],
        {"final_alt": (8.0, 11.0), "hold_end_rms": 8.0, "pos_rms": 5.0,
         "mag_fused_min": 30, "require_mag_cal": True,
         "max_roll_max": 20.0, "max_pitch_max": 20.0},
    ),
    "mag_disturbed": (
        [],
        {"final_alt": (6.0, 12.0), "hold_end_rms": 25.0, "pos_rms": 25.0,
         "mag_gate_min": 1, "require_mag_cal": True,
         "max_roll_max": 25.0, "max_pitch_max": 25.0},
    ),
    # IMU 故障路径只验证“检测到并隔离”，不把故障后的降落品质当验收门。
    "imu_stuck": (
        ["--imu-fault", "stuck", "--imu-fault-window", "10:11"],
        {"skip_motion": True, "require_imu_detected": True},
    ),
    "imu_nan": (
        ["--imu-fault", "nan", "--imu-fault-window", "10:11"],
        {"skip_motion": True, "require_imu_detected": True},
    ),
    "imu_drop": (
        ["--imu-dropout", "10:11"],
        {"skip_motion": True, "require_imu_detected": True},
    ),
    # 慢漂移/欺骗：默认只告警；本 profile 显式启用拒绝，只验证监测/拒绝路径。
    "gnss_spoof": (
        ["--gnss-spoof", "0", "--gnss-spoof-ramp", "0.5",
         "--gnss-spoof-start", "15", "--gnss-consistency-reject"],
        {"skip_motion": True, "gnss_crej_min": 1},
    ),
}


def parse_json_line(stdout: str) -> dict:
    for line in reversed(stdout.splitlines()):
        if line.startswith("SIL_REAL_JSON "):
            raw = line[len("SIL_REAL_JSON "):]
            # 故障场景可能产生 NaN/Inf；Python json 不接受裸 nan，转 null 后
            # 由下面的 skip_motion 逻辑区分“故障导致的非有限值”和解析失败。
            raw = re.sub(r"(?<=[:\[,])-?(?:nan|inf)(?=[,}\]])", "null", raw,
                         flags=re.IGNORECASE)
            return json.loads(raw)
    raise RuntimeError("no SIL_REAL_JSON line in output")


def finite(value) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def fmt(value, digits: int = 2) -> str:
    return f"{float(value):.{digits}f}" if finite(value) else "nan"


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
                value = data.get(key)
                if value is None:
                    if not gates.get("skip_motion", False):
                        errors.append(f"{key}=missing")
                elif not finite(value):
                    errors.append(f"{key}=nonfinite")
            if not gates.get("skip_motion", False):
                lo, hi = gates["final_alt"]
                if not (lo <= float(data.get("final_alt", -1e9)) <= hi):
                    errors.append(f"final_alt={data.get('final_alt')} not in [{lo},{hi}]")
                for key in ("hold_end_rms", "pos_rms"):
                    if float(data.get(key, 1e9)) > gates[key]:
                        errors.append(f"{key}={data.get(key)} > {gates[key]}")
                if "mag_fused_min" in gates and data.get("mag_fused", 0) < gates["mag_fused_min"]:
                    errors.append(f"mag_fused={data.get('mag_fused')} < {gates['mag_fused_min']}")
                if "mag_gate_min" in gates and data.get("mag_gate", 0) < gates["mag_gate_min"]:
                    errors.append(f"mag_gate={data.get('mag_gate')} < {gates['mag_gate_min']}")
                if gates.get("require_mag_cal") and data.get("mag_cal") != 1:
                    errors.append(f"mag_cal={data.get('mag_cal')}")
                for key in ("max_roll", "max_pitch"):
                    limit_key = f"{key}_max"
                    if limit_key in gates and float(data.get(key, 1e9)) > gates[limit_key]:
                        errors.append(f"{key}={data.get(key)} > {gates[limit_key]}")
            if gates.get("require_imu_detected") and data.get("imu_detected") != 1:
                errors.append(f"imu_detected={data.get('imu_detected')}")
            if "gnss_crej_min" in gates and data.get("gnss_crej", 0) < gates["gnss_crej_min"]:
                errors.append(f"gnss_crej={data.get('gnss_crej')} < {gates['gnss_crej_min']}")

            status = "PASS" if not errors else "FAIL"
            if errors:
                failures += 1
            rows.append((status, profile, seed, data, errors))
            print(f"[{status}] {profile:8s} seed{seed} "
                  f"alt={fmt(data.get('final_alt'))} "
                  f"hold_end={fmt(data.get('hold_end_rms'))} "
                  f"hold_max={fmt(data.get('hold_max'))} "
                  f"pos_rms={fmt(data.get('pos_rms'))} "
                  f"roll_max={fmt(data.get('max_roll'))} "
                  f"resets={data.get('reacquire_resets')} "
                  f"outliers={data.get('outlier_rejects')} "
                  f"mag={data.get('mag_fused')}/{data.get('mag_gate')} "
                  f"imu_det={data.get('imu_detected')}")
            for e in errors:
                print(f"       - {e}")

    print(f"\n=== real-nav acceptance: {len(rows)} cases, {failures} failed ===")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
