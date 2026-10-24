#!/usr/bin/env python3
"""run_liquid_sweep.py — 液体火箭 L1 回收 SIL 参数扰动扫描。

@file    run_liquid_sweep.py
@module  固件/宿主 SIL（液体推进参数扫描）
@brief   液体火箭 L1 回收 SIL 的推进参数扰动扫描
@details 只改变推进模型参数（Isp/Tmax/Tmin/阀门/推力速率/高度/推进剂），
         制导与 plant 不变；输出逐案例 PASS/FAIL，失败退出码非 0。

只改变推进模型参数（Isp/Tmax/Tmin/阀门/推力速率/高度/推进剂），制导与 plant
不变；用于找模型参数偏差下的回收包线。不是实机/台架结论。
"""
import argparse
import subprocess
from pathlib import Path

CASES = [
    # name, h0, prop, isp, tmax, tmin, valve, thrust_rate
    ("nominal", 30, 3.0, 200, 200, 80, 10, 1800),
    ("isp_low", 30, 3.0, 150, 200, 80, 10, 1800),
    ("isp_high", 30, 3.0, 250, 200, 80, 10, 1800),
    ("tmax_low", 30, 3.0, 200, 180, 80, 10, 1800),
    ("tmax_high", 30, 3.0, 200, 220, 80, 10, 1800),
    ("tmin_low", 30, 3.0, 200, 200, 60, 10, 1800),
    ("tmin_high", 30, 3.0, 200, 200, 100, 10, 1800),
    ("valve_slow", 30, 3.0, 200, 200, 80, 5, 1800),
    ("valve_fast", 30, 3.0, 200, 200, 80, 20, 1800),
    ("thrust_slow", 30, 3.0, 200, 200, 80, 10, 900),
    ("thrust_fast", 30, 3.0, 200, 200, 80, 10, 2700),
    ("combined_low", 20, 2.5, 150, 180, 60, 5, 900),
    ("combined_high", 40, 4.0, 250, 220, 100, 20, 2700),
    ("h20_prop25", 20, 2.5, 200, 200, 80, 10, 1800),
    ("h40_prop40", 40, 4.0, 200, 200, 80, 10, 1800),
]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", required=True)
    args = ap.parse_args()
    bin_path = Path(args.bin)
    if not bin_path.exists():
        print(f"[FAIL] binary not found: {bin_path}")
        return 2

    failures = 0
    for name, h0, prop, isp, tmax, tmin, valve, thrust_rate in CASES:
        cmd = [str(bin_path), "--h0", str(h0), "--prop", str(prop),
               "--isp", str(isp), "--tmax", str(tmax), "--tmin", str(tmin),
               "--valve-rate", str(valve), "--thrust-rate", str(thrust_rate)]
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
        ok = proc.returncode == 0 and "landed=1" in proc.stdout
        if not ok:
            failures += 1
        line = next((l for l in reversed(proc.stdout.splitlines())
                     if l.startswith("L-SIL ")), "no L-SIL line")
        print(f"[{'PASS' if ok else 'FAIL'}] {name:14s} h0={h0} prop={prop} "
              f"isp={isp} T[{tmin},{tmax}] valve={valve} rate={thrust_rate} | {line}")
    print(f"\n=== liquid sweep: {len(CASES)} cases, {failures} failed ===")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
