# -*- coding: utf-8 -*-
"""@file    serial_check.py
@module  工具
@brief   serial_check.py 模块说明。
TandemVec FCS serial health check (COM port -> GCS CLI -> DBG).

Usage:
  py -3.12 TandemVec_FCS/tools/serial_check.py --port COM9
  py -3.12 TandemVec_FCS/tools/serial_check.py --port COM9 --uplink-seconds 10

It runs the GCS CLI (single implementation of the protocol) instead of
re-implementing the AnoCom framing here. Use this after flashing to verify:
  health  -> IMU/baro/GNSS/EKF/Serial5 uplink summary
  ubxstat -> PVT/EOE counts and last UBX message
  tasks   -> task timing / CPU load
"""
import argparse
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CLI = os.path.join(ROOT, "GCS", "server", "cli.py")


def run_cli(port, *args, timeout=45):
    cmd = [sys.executable, CLI, "--port", port] + list(args)
    env = dict(os.environ, PYTHONUTF8="1", PYTHONIOENCODING="utf-8")
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout,
                       env=env, encoding="utf-8", errors="replace")
    return (p.stdout or "") + (p.stderr or "")


def show(title, text, keep=16):
    print(f"\n=== {title} ===")
    lines = [ln for ln in text.splitlines() if ln.strip()]
    for ln in lines[:keep]:
        print("  " + ln[:180])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM9")
    ap.add_argument("--uplink-seconds", type=float, default=0.0,
                    help="if >0, print uplink counters twice to estimate frame rate")
    a = ap.parse_args()

    show("health", run_cli(a.port, "dbg", "health"))
    show("ubxstat", run_cli(a.port, "dbg", "ubxstat"))
    show("monver", run_cli(a.port, "dbg", "ubxcfg monver"))
    show("gpsproto", run_cli(a.port, "dbg", "gpsproto"))
    show("tasks", run_cli(a.port, "dbg", "tasks"), keep=8)

    if a.uplink_seconds > 0:
        before = run_cli(a.port, "dbg", "health")
        time.sleep(a.uplink_seconds)
        after = run_cli(a.port, "dbg", "health")
        show("uplink before", before, keep=12)
        show("uplink after", after, keep=12)
        print(f"\nuplink window = {a.uplink_seconds:.1f} s "
              f"(compare 'uplink rx=' lines to get frames/s)")


if __name__ == "__main__":
    main()
