#!/usr/bin/env python3
"""run_real_acceptance.py — 真实导航任务 SIL 多配置验收。

@file    run_real_acceptance.py
@module  固件/宿主 SIL（fw-tests：real-nav 验收门）
@brief   对磁基线（mag_clean/mag_nominal/mag_disturbed）各跑 3 seeds，
         解析被测体输出的 SIL_REAL_JSON 行并按阈值判定。
@details 输入：--bin 被测可执行体；--seconds 仿真时长（默认 40 s）；
         --profiles 默认磁基线；也可选无磁 clean/nominal/degraded（观测性诊断，
         40 s 位置保持会因航向不可观发散，不作验收）、IMU 故障（imu_*）与
         GNSS 欺骗（gnss_spoof）。
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
    # 磁干扰 + nominal 的 GNSS 失联组合：只验收“磁门控确实拒绝 + 高度/姿态
    # 有界”，不把位置保持当判据——同时失去绝对航向与水平姿态辅助时，
    # 位置保持本身已不可保证。
    "mag_disturbed": (
        [],
        {"final_alt": (6.0, 12.0), "pos_rms": 25.0,
         "skip_hold_end": True,
         "mag_gate_min": 1, "require_mag_cal": True,
         "max_roll_max": 60.0, "max_pitch_max": 60.0},
    ),
    # 偏航振荡门控验证：只要求“确实降级 + 高度/姿态有界”，不要求水平位置保持。
    "mag_gate": (
        [],
        {"final_alt": (6.0, 12.0), "pos_rms": 30.0,
         "skip_hold_end": True, "require_pos_degraded": True,
         "mag_fused_min": 30, "require_mag_cal": True,
         "max_roll_max": 30.0, "max_pitch_max": 30.0},
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
    # 无磁（无绝对航向）降级路径：不再要求位置保持，只验证高度保持、
    # 姿态有界和“确实发生 AUTO_POSITION→AUTO_ALTITUDE 降级”。
    "nomag_degraded": (
        ["--no-wind"],
        {"final_alt": (8.0, 11.0), "pos_rms": 5.0,
         "skip_hold_end": True, "require_pos_degraded": True,
         "max_roll_max": 25.0, "max_pitch_max": 25.0},
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
    ap.add_argument(
        "--profiles", default="mag_clean,mag_nominal,mag_disturbed",
        help="默认磁基线；无磁 clean/nominal/degraded 仅用于观测性诊断")
    ap.add_argument("--gain-scale", type=float, default=1.0,
                    help="测试用 att/rate kp 缩放")
    ap.add_argument("--scale-ki", action="store_true",
                    help="同时缩放 rate ki")
    ap.add_argument("--aaf-gain", type=float, default=None,
                    help="测试用：AAF 滚转/俯仰增益（需 -DBFS_RATE_ACCEL_FB）")
    ap.add_argument("--aaf-yaw-gain", type=float, default=None,
                    help="测试用：AAF 偏航增益（0 关闭偏航）")
    ap.add_argument("--aaf-tau", type=float, default=None,
                    help="测试用：AAF 差分低通时间常数 [s]")
    ap.add_argument("--pl-gain", type=float, default=None,
                    help="测试用：相位超前增益（需 -DBFS_RATE_PHASE_LEAD）")
    ap.add_argument("--pl-yaw-gain", type=float, default=None,
                    help="测试用：相位超前偏航增益（0 关闭偏航）")
    ap.add_argument("--pl-tau", type=float, default=None,
                    help="测试用：相位超前低通时间常数 [s]")
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
            if args.gain_scale != 1.0:
                cmd += ["--gain-scale", str(args.gain_scale)]
            if args.scale_ki:
                cmd += ["--scale-ki"]
            if args.aaf_gain is not None:
                cmd += ["--aaf-gain", str(args.aaf_gain)]
            if args.aaf_yaw_gain is not None:
                cmd += ["--aaf-yaw-gain", str(args.aaf_yaw_gain)]
            if args.aaf_tau is not None:
                cmd += ["--aaf-tau", str(args.aaf_tau)]
            if args.pl_gain is not None:
                cmd += ["--pl-gain", str(args.pl_gain)]
            if args.pl_yaw_gain is not None:
                cmd += ["--pl-yaw-gain", str(args.pl_yaw_gain)]
            if args.pl_tau is not None:
                cmd += ["--pl-tau", str(args.pl_tau)]
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
                final_alt = data.get("final_alt")
                if finite(final_alt) and not (lo <= float(final_alt) <= hi):
                    errors.append(f"final_alt={final_alt} not in [{lo},{hi}]")
                for key in ("hold_end_rms", "pos_rms"):
                    if key == "hold_end_rms" and gates.get("skip_hold_end", False):
                        continue
                    value = data.get(key)
                    if finite(value) and float(value) > gates[key]:
                        errors.append(f"{key}={value} > {gates[key]}")
                if "mag_fused_min" in gates and data.get("mag_fused", 0) < gates["mag_fused_min"]:
                    errors.append(f"mag_fused={data.get('mag_fused')} < {gates['mag_fused_min']}")
                if "mag_gate_min" in gates and data.get("mag_gate", 0) < gates["mag_gate_min"]:
                    errors.append(f"mag_gate={data.get('mag_gate')} < {gates['mag_gate_min']}")
                if gates.get("require_mag_cal") and data.get("mag_cal") != 1:
                    errors.append(f"mag_cal={data.get('mag_cal')}")
                for key in ("max_roll", "max_pitch"):
                    limit_key = f"{key}_max"
                    value = data.get(key)
                    if limit_key in gates and finite(value) and float(value) > gates[limit_key]:
                        errors.append(f"{key}={value} > {gates[limit_key]}")
            if gates.get("require_imu_detected") and data.get("imu_detected") != 1:
                errors.append(f"imu_detected={data.get('imu_detected')}")
            if gates.get("require_pos_degraded") and data.get("pos_degraded") != 1:
                errors.append(f"pos_degraded={data.get('pos_degraded')}")
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
