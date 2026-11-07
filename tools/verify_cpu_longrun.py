#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    verify_cpu_longrun.py
@module  工具/长时运行 CPU 统计验证
@brief   等到板子 uptime 跨过 micros() 的旧回绕点后，检查 CPU 统计是否仍在合理区间
@details 输入：串口名、目标 uptime（默认 4400 s，须 > 4260 s）；处理：轮询 health，
         到达后读 cpu_permille 并与阈值比较；输出：判据结果 + 退出码。
@theory  32 位 micros() 的满量程是 4294.9673 s（71.58 min）。2026-11-08 前固件
         用它做统计窗口基准，导致跑过该点后窗口被算小、CPU 被放大成 400%+。
         修复改用 millis()（49.7 天）后，跨过 4260 s 仍应正常。
         本工具就是那个「跨点」判据的自动化版本。
@ref     VER-010 §5.5（长时运行后的 CPU 统计失真）、task_scheduler.cpp 的窗口实现
@frame   N/A（只读遥测）
@units   uptime_s / win_s 为秒；cpu_permille 为千分比（1000 = 100%）
@note    ★ 阈值取 850‰（85%）：正常负载约 500‰，留出裕度；超过即判 FAIL。
         若板子 uptime 已超过目标，直接读取不等待。
@see     tools/verify_board.py、tools/verify_param_names.py
@test    本脚本即自检：退出码 0 表示跨点后 CPU 仍正常
"""
from __future__ import annotations

import re
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
GCS_DIR = ROOT / "TandemVec_FCS" / "GCS"

OLD_WRAP_S = 4260.0      # micros() 的 32 位回绕点（4294.97 s，留一点余量）
DEFAULT_TARGET_S = 4400.0
CPU_LIMIT_PERMILLE = 850


def read_health(port: str) -> str:
    proc = subprocess.run(
        [sys.executable, "-X", "utf8", "server/cli.py", "--port", port, "dbg", "health"],
        cwd=str(GCS_DIR), capture_output=True, text=True,
        encoding="utf-8", errors="replace", timeout=120,
    )
    return (proc.stdout or "") + (proc.stderr or "")


def grab(pattern: str, text: str, cast=float):
    m = re.search(pattern, text)
    return cast(m.group(1)) if m else None


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM6"
    target = float(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_TARGET_S

    print(f"目标：等 uptime 跨过 {target:.0f} s（micros 旧回绕点 {OLD_WRAP_S:.0f} s）")

    deadline = time.monotonic() + 5 * 3600
    while True:
        out = read_health(port)
        up = grab(r"uptime_s=(\d+)", out, int)
        if up is None:
            print("  [warn] 本次未读到 uptime，5 s 后重试")
            time.sleep(5)
            if time.monotonic() > deadline:
                print("[FAIL] 超时仍未读到 uptime")
                return 2
            continue

        cpu = grab(r"cpu_permille=(\d+)", out, int)
        win = grab(r"win_s=(\d+)", out, int)
        mx = grab(r"max_pass_us=(\d+)", out, int)
        print(f"  uptime={up}s  win={win}s  cpu={cpu}‰  max_pass={mx}us", flush=True)

        if up >= target:
            print()
            print(f"已跨过旧回绕点。判据：cpu_permille ≤ {CPU_LIMIT_PERMILLE}")
            if cpu is None:
                print("[FAIL] 未读到 cpu_permille")
                return 2
            if cpu > CPU_LIMIT_PERMILLE:
                print(f"[FAIL] cpu_permille={cpu} 超过 {CPU_LIMIT_PERMILLE}，"
                      f"长时统计失真可能仍未修好")
                return 1
            print(f"[PASS] cpu_permille={cpu}（{cpu / 10.0:.1f}%），跨点后正常")
            return 0

        time.sleep(60)


if __name__ == "__main__":
    sys.exit(main())
