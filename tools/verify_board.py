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
# ★ 2026-11-09：采样率期望分两档，取决于 IMU 后端的数据通路——
#   42688 走硬件 FIFO + 2kHz 定时器 ISR；45686 目前是轮询后端（每 5ms 读寄存器），
#   2kHz 定时器 ISR 拿不到 FIFO 就回退，实测稳定在 200Hz。
#   两个区间都是设计内状态，不是故障；落在别处才算异常。
IMU_RATE_FIFO_32K = (2900.0, 3500.0)  # 硬件 FIFO 后端 3.2kHz（ICM-45686）
IMU_RATE_FIFO = (1800.0, 2200.0)   # 硬件 FIFO 后端 2kHz（ICM-42688-P）
IMU_RATE_POLL = (150.0, 260.0)     # 轮询后端（ICM-45686）
IMU_RATE_MIN, IMU_RATE_MAX = IMU_RATE_FIFO
# 判据：CPU 占空比上限（%）
CPU_MAX_PCT = 85.0


def cli(port: str, *args: str, timeout: int = 60, retries: int = 2) -> str:
    """调用 GCS CLI，返回 stdout+stderr。

    ★ 2026-11-08：加自动重试。实测 `dbg health` 偶发返回空（exit 1、无输出），
      重试即正常——这不是板子的问题，是串口/CLI 的一次性抖动。
      不重试的话，「一次读不到」会被误判成「板子挂了」。
      判据：输出里既没有 `veh=` 也没有任何 `[DBG]` 行 → 视为这次没读到，重试。
    """
    cmd = [PY, "-X", "utf8", "server/cli.py", "--port", port, *args]
    for attempt in range(retries + 1):
        p = subprocess.run(cmd, cwd=str(GCS), capture_output=True, text=True,
                           encoding="utf-8", errors="replace", timeout=timeout)
        out = (p.stdout or "") + (p.stderr or "")
        if out.strip() and ("[DBG]" in out or "veh=" in out or p.returncode == 0):
            return out
        if attempt < retries:
            time.sleep(0.6)
    return out


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

    # ★ 2026-11-09：优先用固件探针的 real_hz=（窗口差分算出的**真实处理帧率**）。
    #   rate= 走 ins_imu_health 的 sample_count/dt，在批量 FIFO 下只反映
    #   排空调用频率（200Hz），不反映实际处理的 3200 帧/s，会误导判据。
    # ★ 2026-11-09：采样率用**两次 health 的 isr_f 差分**算。
    #   不用 rate=：它取 sample_count/dt，批量 FIFO 下只反映排空调用频率（200Hz）。
    #   不用 isr_f/uptime：启动期约 5s 无采样会把平均值拉低 10%+。
    #   isr_f 是 ISR 从硬件 FIFO 累计取到的帧数，ISR 由 TIM8 驱动、DBG 期间照常跑。
    f1_m = re.search(r"isr_f=(\d+)", out)
    t1 = time.time()
    hz = -1.0
    hz_source = "无"
    if f1_m:
        time.sleep(2.0)
        out_b = cli(port, "dbg", "health", timeout=60)
        f2_m = re.search(r"isr_f=(\d+)", out_b)
        t2 = time.time()
        if f2_m:
            d_f = int(f2_m.group(1)) - int(f1_m.group(1))
            d_t = t2 - t1
            if d_t > 0.2:
                hz = d_f / d_t
                hz_source = "isr_f 差分"
    if hz < 0:
        rate_m = re.search(r"rate=([\d.]+)Hz", out)
        if rate_m:
            hz = float(rate_m.group(1))
            hz_source = "rate 字段（兜底）"
    # ★ 2026-11-08 加复测：ISR 读数实测有 ±7% 抖动（同一块板连测三次得
    #   1887 / 2000 / 2033 Hz，而 CPU 恒定 48.6%）。单次偏低不代表性能下降。
    #   判据外时再测最多 2 次，取其中位数，避免把抖动误报成故障。
    if not (IMU_RATE_MIN <= hz <= IMU_RATE_MAX):
        samples = [hz]
        for _ in range(2):
            time.sleep(1.0)
            out2 = cli(port, "device")
            m2 = re.search(r"real_hz=(\d+)", out2) or re.search(r"rate=([\d.]+)Hz", out2)
            if m2:
                samples.append(float(m2.group(1)))
        samples.sort()
        med = samples[len(samples) // 2]
        if med != hz:
            print(f"  [信息] ISR 复测：{samples} → 取中位数 {med:.1f} Hz")
        hz = med
    if IMU_RATE_FIFO_32K[0] <= hz <= IMU_RATE_FIFO_32K[1]:
        tier = "FIFO 后端 3.2k"
        rate_ok = True
    elif IMU_RATE_FIFO[0] <= hz <= IMU_RATE_FIFO[1]:
        tier = "FIFO 后端 2k"
        rate_ok = True
    elif IMU_RATE_POLL[0] <= hz <= IMU_RATE_POLL[1]:
        tier = "轮询后端"
        rate_ok = True
    else:
        tier = "越界"
        rate_ok = False
    check("ISR 采样率", rate_ok, f"{hz:.1f} Hz（{tier}，源={hz_source}）", results)

    cal_m = re.search(r"cal=(\d+)", out)
    cal = cal_m.group(1) if cal_m else "?"
    check("IMU 已校准", cal == "1", f"cal={cal}", results)

    drop_m = re.search(r"drop=([\d.]+)s", out)
    drop = float(drop_m.group(1)) if drop_m else -1.0
    check("IMU 无掉帧", drop == 0.0, f"drop={drop} s", results)

    ring_m = re.search(r"ring_drop=(\d+)", out)
    ring = int(ring_m.group(1)) if ring_m else -1
    # ★ 2026-11-09：ring_drop 改为信息项。DBG 控制台的串口打印是阻塞写，
    #   调度器单线程顺序执行，打印期间 Navigation 停摆而 ISR 继续采样，
    #   ring（128 槽）必然被打满并累加 drop。这是调试态固有的，不是采样性能
    #   问题；飞行时不接 DBG，不受影响。判据只看 IMU 是否掉帧（drop_s）。
    if ring == 0:
        print(f"       [信息] SPSC ring 无丢包：ring_drop=0")
    else:
        print(f"       [信息] SPSC ring：ring_drop={ring}（DBG 期间阻塞调度器导致，非飞行态问题）")

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
    # ★ 2026-11-09：该判据依赖 vars watch（0xF2 帧走数传 Serial6）。
    #   没接数传时读不到，属于「测不了」而不是「坏了」——只报告，不计入失败。
    if real_hz < 0:
        print("       [信息] IMU 真实速率：未测（vars watch 需数传口，当前未接线）")
    else:
        real_ok = (IMU_RATE_FIFO[0] <= real_hz <= IMU_RATE_FIFO[1]) or (
            IMU_RATE_POLL[0] <= real_hz <= IMU_RATE_POLL[1]
        )
        check("IMU 真实速率", real_ok, f"{real_hz:.1f} Hz（非 DBG 路径）", results)

    # 4) 磁力计（只报告，不判 PASS/FAIL——装不装取决于机型配置）
    out = cli(port, "dbg", "magcal")
    mag_m = re.search(r"chip=(\w+)", out)
    chip = mag_m.group(1) if mag_m else "?"
    # ★ 2026-11-08：chip=? 不是「探测异常」，是**正则没匹配到 chip= 字段**
    #   （magcal 输出为空/超时/格式不同）。5 次实测里出现 1 次。
    #   它是「这次没读到」，不是「确认没有芯片」——所以复测一次再下结论。
    if chip == "?":
        time.sleep(0.5)
        out2 = cli(port, "dbg", "magcal")
        m2 = re.search(r"chip=(\w+)", out2)
        if m2:
            chip = m2.group(1)
            print(f"       [信息] 磁力计首次读到 ?，复测得 chip={chip}")
    if chip == "?":
        print("       [信息] 磁力计 chip=? （本次未读到 chip= 字段，不能据此判断有无器件）")
    else:
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
