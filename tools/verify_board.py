#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    verify_board.py
@module  工具/实机烧录后自检（一键）
@brief   对已连接的飞控板跑一组只读检查，按判据给出 PASS/FAIL
@details 输入：串口名（如 COM6）、可选期望机型；
         处理：调用 GCS CLI 依次读 device / health / imu_rate / magcal，解析关键字段；
         输出：逐项 PASS/FAIL + 汇总退出码（0 全过，1 有失败）。
@theory  N/A（检查流程编排，不含算法）
@ref     docs/07-验证与确认/VER-010-实机烧录验证记录-2026-11-07.md
@frame   N/A
@units   见各项判据注释
@note    ★ 只读：不写参数、不解锁、不给油。可以随时在任意一块已烧录的板上跑。
         ★ 必须在 GCS 目录下运行（复用 server/cli.py），且目标机需 pyserial。
         ★ IMU 速率用 `vars watch imu_rate`（非 DBG 路径）——用 tasks/health
           的数字会因 DBG 打印本身加剧丢拍而失真（AGENTS.md §调度统计口径）。
@see     docs/07-验证与确认/VER-010-实机烧录验证记录-2026-11-07.md
@test    本脚本即自检：全部 PASS 返回 0
"""
from __future__ import annotations

import re
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
GCS = HERE.parent / "TandemVec_FCS" / "GCS"

PY = sys.executable or "py"

# 判据：期望的固件参数数量（改参数表时同步这里与 docs/registers）
EXPECT_PARAM_COUNT = 169
# 判据：IMU 真实采样率允许区间 [Hz]
IMU_RATE_MIN, IMU_RATE_MAX = 1800.0, 2200.0
# 判据：CPU 占空比上限（%）
CPU_MAX_PCT = 85.0


def cli(port: str, *args: str, timeout: int = 60) -> str:
    """调用 GCS CLI，返回 stdout+stderr。"""
    cmd = [PY, "-X", "utf8", "server/cli.py", "--port", port, *args]
    p = subprocess.run(cmd, cwd=str(GCS), capture_output=True, text=True,
                       encoding="utf-8", errors="replace", timeout=timeout)
    return (p.stdout or "") + (p.stderr or "")


def check(name: str, ok: bool, detail: str, results: list) -> None:
    results.append((name, ok, detail))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM6"
    expect_veh = sys.argv[2] if len(sys.argv) > 2 else ""

    results: list[tuple[str, bool, str]] = []
    print(f"=== 实机自检 port={port} ===")

    # 1) 固件参数表在位（版本判据：数量对得上才算当前固件）
    # ★ 2026-11-08 加重试：device 偶发读不到（串口忙 / 首帧丢失）时原来直接
    #   记 -1 并误报 FAIL。实测连续两次调用，第一次 -1、第二次 169。
    #   这类暂时性失败不该判成版本不对。
    got = -1
    for attempt in range(3):
        out = cli(port, "device")
        m = re.search(r"(\d+)\s*参数", out)
        if m:
            got = int(m.group(1))
            break
        if attempt < 2:
            time.sleep(1.0)
    check("参数表数量", got == EXPECT_PARAM_COUNT,
          f"{got} 项（期望 {EXPECT_PARAM_COUNT}）", results)

    # 2) health：机型 / IMU / EKF / CPU
    out = cli(port, "dbg", "health")
    veh_m = re.search(r"veh=(\w+)", out)
    veh = veh_m.group(1) if veh_m else "?"
    if expect_veh:
        check("机型匹配", veh == expect_veh, f"{veh}（期望 {expect_veh}）", results)
    else:
        check("机型可读", veh != "?", f"{veh}", results)

    rate_m = re.search(r"rate=([\d.]+)Hz", out)
    hz = float(rate_m.group(1)) if rate_m else -1.0
    check("ISR 采样率", IMU_RATE_MIN <= hz <= IMU_RATE_MAX, f"{hz:.1f} Hz", results)

    cal_m = re.search(r"cal=(\d+)", out)
    cal = cal_m.group(1) if cal_m else "?"
    check("IMU 已校准", cal == "1", f"cal={cal}", results)

    drop_m = re.search(r"drop=([\d.]+)s", out)
    drop = float(drop_m.group(1)) if drop_m else -1.0
    check("IMU 无掉帧", drop == 0.0, f"drop={drop} s", results)

    ring_m = re.search(r"ring_drop=(\d+)", out)
    ring = int(ring_m.group(1)) if ring_m else -1
    check("SPSC ring 无丢包", ring == 0, f"ring_drop={ring}", results)

    cpu_m = re.search(r"cpu_permille=(\d+)", out)
    cpu = int(cpu_m.group(1)) / 10.0 if cpu_m else -1.0
    check("CPU 占空", 0 <= cpu <= CPU_MAX_PCT, f"{cpu:.1f}%", results)

    ekf_m = re.search(r"fake_origin=(\d+)", out)
    fake = ekf_m.group(1) if ekf_m else "?"
    print(f"       [信息] fake_origin={fake}（1=无 GNSS 伪原点，室内常见）")

    # 3) IMU 真实速率（非 DBG 路径，避免打印副作用）
    out = cli(port, "vars", "watch", "imu_rate", "-t", "5", timeout=90)
    rate_m = re.search(r"imu_rate\s*=\s*\+?([\d.]+)", out)
    real_hz = float(rate_m.group(1)) if rate_m else -1.0
    check("IMU 真实速率", IMU_RATE_MIN <= real_hz <= IMU_RATE_MAX,
          f"{real_hz:.1f} Hz（非 DBG 路径）", results)

    # 4) 磁力计（只报告，不判 PASS/FAIL——装不装取决于机型配置）
    out = cli(port, "dbg", "magcal")
    mag_m = re.search(r"chip=(\w+)", out)
    chip = mag_m.group(1) if mag_m else "?"
    print(f"       [信息] 磁力计 chip={chip}"
          + ("（未检测到，航向无绝对参考）" if chip == "NONE" else ""))

    # ---- 汇总 ----
    failed = [n for n, ok, _ in results if not ok]
    print()
    if failed:
        print(f"[FAIL] {len(failed)} 项未通过：{', '.join(failed)}")
        return 1
    print(f"[PASS] {len(results)} 项全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
