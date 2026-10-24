#!/usr/bin/env python3
"""run_coaxial_sweep.py — 共轴 SIL plant 参数/故障扰动扫描。

@file    run_coaxial_sweep.py
@module  固件/宿主 SIL（共轴 plant 参数/故障扫描）
@brief   共轴 SIL 的 plant 参数与执行器故障扰动扫描
@details 只改变 plant 的质量/推力/惯量/力臂/差速力矩与执行器故障，固件控制律不变；
         输出逐案例 PASS/FAIL 与指标，失败退出码非 0。

只改变 plant 的质量/推力/惯量/力臂/差速力矩与执行器故障，固件控制律不变；
用于找“实飞结构控制律在 L1 plant 参数偏差下是否仍有界”。不是实机结论。
"""
import argparse
import json
import re
import subprocess
from pathlib import Path

CASES = [
    # name, mass, thrust, lever, iy, iz, yawk, extra
    ("nominal", 0.84, 14.0, 0.15, 0.05, 0.08, 0.0005, []),
    ("mass_low", 0.70, 14.0, 0.15, 0.05, 0.08, 0.0005, []),
    ("mass_high", 1.00, 14.0, 0.15, 0.05, 0.08, 0.0005, []),
    ("thrust_low", 0.84, 12.0, 0.15, 0.05, 0.08, 0.0005, []),
    ("thrust_high", 0.84, 16.0, 0.15, 0.05, 0.08, 0.0005, []),
    ("lever_low", 0.84, 14.0, 0.08, 0.05, 0.08, 0.0005, []),
    ("lever_high", 0.84, 14.0, 0.22, 0.05, 0.08, 0.0005, []),
    ("iy_low", 0.84, 14.0, 0.15, 0.03, 0.08, 0.0005, []),
    ("iy_high", 0.84, 14.0, 0.15, 0.08, 0.08, 0.0005, []),
    ("iz_low", 0.84, 14.0, 0.15, 0.05, 0.04, 0.0005, []),
    ("iz_high", 0.84, 14.0, 0.15, 0.05, 0.12, 0.0005, []),
    ("yawk_low", 0.84, 14.0, 0.15, 0.05, 0.08, 0.0002, []),
    ("yawk_high", 0.84, 14.0, 0.15, 0.05, 0.08, 0.0010, []),
    ("combined_low", 0.70, 12.0, 0.08, 0.03, 0.04, 0.0002, []),
    ("combined_high", 1.00, 16.0, 0.22, 0.08, 0.12, 0.0010, []),
    ("motor_loss_30", 0.84, 14.0, 0.15, 0.05, 0.08, 0.0005,
     ["--fault", "3", "--fault-at", "15"]),
    ("servo_jam", 0.84, 14.0, 0.15, 0.05, 0.08, 0.0005,
     ["--fault", "4", "--fault-at", "15"]),
]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", required=True)
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()
    bin_path = Path(args.bin)
    if not bin_path.exists():
        print(f"[FAIL] binary not found: {bin_path}")
        return 2

    failures = 0
    for name, mass, thrust, lever, iy, iz, yawk, extra in CASES:
        cmd = [str(bin_path), "--real-nav", "--profile", "mag_clean",
               "--pos-hold", "--seconds", str(args.seconds),
               "--seed", str(args.seed), "--no-wind", "--mag",
               "--coax-mass", str(mass), "--coax-thrust", str(thrust),
               "--coax-lever", str(lever), "--coax-iy", str(iy),
               "--coax-iz", str(iz), "--coax-yawk", str(yawk), *extra]
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
        data = None
        for line in reversed(proc.stdout.splitlines()):
            if line.startswith("SIL_REAL_JSON "):
                raw = re.sub(r"(?<=[:\[,])-?(?:nan|inf)(?=[,}\]])", "null",
                             line[len("SIL_REAL_JSON "):], flags=re.IGNORECASE)
                data = json.loads(raw)
                break
        if data is None:
            print(f"[FAIL] {name}: no JSON / rc={proc.returncode}")
            failures += 1
            continue
        alt = data.get("final_alt")
        hold = data.get("hold_end_rms")
        roll = data.get("max_roll")
        pitch = data.get("max_pitch")
        ok = (data.get("nav_init") == 1 and isinstance(alt, (int, float))
              and 8.0 <= alt <= 11.0
              and isinstance(hold, (int, float)) and hold <= 6.0
              and isinstance(roll, (int, float)) and roll <= 30.0
              and isinstance(pitch, (int, float)) and pitch <= 30.0)
        status = "PASS" if ok else "FAIL"
        if not ok:
            failures += 1
        print(f"[{status}] {name:14s} mass={mass:.2f} T={thrust:.0f} "
              f"L={lever:.2f} Iy={iy:.2f} Iz={iz:.2f} "
              f"alt={alt if alt is not None else 'nan'} "
              f"hold={hold if hold is not None else 'nan'} "
              f"roll={roll if roll is not None else 'nan'} "
              f"pitch={pitch if pitch is not None else 'nan'}")
    print(f"\n=== coaxial sweep: {len(CASES)} cases, {failures} failed ===")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
