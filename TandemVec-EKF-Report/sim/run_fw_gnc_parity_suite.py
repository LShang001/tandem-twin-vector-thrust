# -*- coding: utf-8 -*-
"""
@file    run_fw_gnc_parity_suite.py
@brief   固件控制链对拍套件（5 个 profile）
@module  仿真验证（固件-仿真一体化）
@details SIL 跑 5 条轨迹并逐拍重放 fwapi.Gnc，输出证据 JSON。
@frame   NED / FRD（透传固件约定）
@units   SI + 执行器 % + 摆角 deg


@file    run_fw_gnc_parity_suite.py
@brief   一体化回归入口：一次跑完 VTOL/自动/手动共 5 条轨迹，输出证据 JSON。
@details 每条轨迹用 SIL `--trace-ctrl` 落盘逐拍输入/输出，再交给
          `check_fw_gnc_parity.py` 用 `fwapi.Gnc` 逐拍重放比较。
          判据（预登记，见 CTL-004 §6）：C1–C4a + C6 必须全过；
          C4b（严格 1e-2）的结果也落盘保留，不做"事后放宽"。
@usage   py -3.12 sim/run_fw_gnc_parity_suite.py [--exe PATH] [--out-dir DIR]
@see     sim/check_fw_gnc_parity.py、TandemVec_FCS/test_host/fw_algo/fw_gnc_api.h
"""
import argparse
import json
import os
import pathlib
import subprocess
import sys
import tempfile

HERE = pathlib.Path(__file__).resolve().parent

CASES = [
    # name, SIL 参数（--trace-ctrl 由驱动补）
    ("vtol", ["--seconds", "20", "--takeoff-at", "3"]),
    ("nominal", ["--real-nav", "--profile", "nominal",
                 "--seconds", "15", "--takeoff-at", "3"]),
    ("mag_nominal", ["--real-nav", "--profile", "mag_nominal",
                     "--seconds", "15", "--takeoff-at", "3"]),
    ("degraded", ["--real-nav", "--profile", "degraded",
                  "--seconds", "15", "--takeoff-at", "3"]),
    ("manual", ["--manual", "--seconds", "8", "--takeoff-at", "3"]),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", default=os.path.join(tempfile.gettempdir(),
                                                  "inc", "fw_real.exe"))
    ap.add_argument("--out-dir", default=str(HERE.parent / "sim-data"))
    ap.add_argument("--tag", default="2026-10-29")
    ap.add_argument("--only", default="", help="只跑指定 profile（逗号分隔）")
    args = ap.parse_args()

    only = {s.strip() for s in args.only.split(",") if s.strip()}
    out_dir = pathlib.Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    tmp = pathlib.Path(tempfile.gettempdir())

    results = {}
    failed = []
    for name, extra in CASES:
        if only and name not in only:
            continue
        trace = tmp / f"ctrl_{name}.csv"
        print(f"\n=== [{name}] SIL → {trace} ===")
        proc = subprocess.run([args.exe, *extra, "--seed", "1",
                               "--trace-ctrl", str(trace)],
                              text=True, capture_output=True, timeout=600)
        tail = proc.stdout.strip().splitlines()
        print("  " + (tail[-1] if tail else "(no stdout)"))
        if proc.returncode != 0:
            failed.append(name)
            print(proc.stderr.rstrip()[-2000:])
            continue
        ev = out_dir / f"fw_gnc_parity_{args.tag}_{name}.json"
        proc = subprocess.run(
            [sys.executable, str(HERE / "check_fw_gnc_parity.py"),
             "--trace", str(trace), "--json", str(ev)],
            text=True, capture_output=True)
        print(proc.stdout.rstrip())
        if proc.returncode != 0:
            failed.append(name)
            print(proc.stderr.rstrip())
        if ev.exists():
            results[name] = json.loads(ev.read_text(encoding="utf-8"))

    print("\n=== 汇总（判据 C1–C4a + C6）===")
    for name, d in results.items():
        mad = d["max_abs_diff"]
        print(f"{name:12s} rows={d['rows']:5d} mode_bad={d['mode_mismatch']} "
              f"step_fail={d['step_fail']} max|Δch|={max(mad['ch1'], mad['ch2'], mad['ch3'], mad['ch4']):.4g}% "
              f"max|Δα|={max(mad['ar0'], mad['ar1'], mad['ar2']):.4g} dps²")
    if failed:
        print(f"\n[FAIL] 未通过：{', '.join(failed)}")
        return 1
    print(f"\n[PASS] {len(results)} 个 profile 全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
