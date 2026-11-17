#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    verify_gnss_stream.py
@module  工具/GNSS 串口与 UBX 解析健康验收
@brief   通过 DBG 读取 UBX 与串口状态，对观察窗口内的增量错误给出 PASS/FAIL
@details 只读进入/退出 DBG，采样 ubxstat 并做计数差分；校验 UART4 的硬件 FIFO、
         HAL FIFO 状态和优先级 0 始终有效，PVT/EOE 持续前进。
@theory  N/A（硬件状态与协议计数检查，不含算法）
@ref     docs/memory/2026-11-11-GNSS串口逐字节中断丢帧.md
@frame   N/A
@units   时间 s、串口波特率 bps、错误与历元计数为整数
@note    用法：py -3.13 tools/verify_gnss_stream.py --port COM9 --seconds 120
         判据：cksum_fail/bad_pvt/frame_timeout/eoe_no_pvt 均不增加。
@test    本脚本即板载检查：全部 PASS 返回 0，发现新增错误或字段缺失返回非 0。
"""
from __future__ import annotations

import argparse
import re
import sys
import time

import serial

FIELDS = (
    "rx_bytes", "valid_msgs", "cksum_fail", "oversize", "frame_timeout",
    "bad_pvt", "bad_other", "pvt", "eoe", "eoe_no_pvt", "pvt_dup",
    "tow_mismatch", "queue_overflow", "fifo_hw", "fifo_hal", "irq", "cfg_fail",
)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", required=True, help="飞控 Serial6 串口，例如 COM9")
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--seconds", type=int, default=120)
    return p.parse_args()


def read_for(ser: serial.Serial, seconds: float) -> bytes:
    data = bytearray()
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        data.extend(ser.read(4096))
    return bytes(data)


def ubxstat(ser: serial.Serial) -> str:
    ser.reset_input_buffer()
    ser.write(b"exit\n")
    read_for(ser, 0.25)
    for _ in range(5):
        ser.reset_input_buffer()
        ser.write(b"DBG\n")
        if b"Debug mode ON" in read_for(ser, 0.45):
            break
    else:
        return ""
    ser.write(b"ubxstat\n")
    text = read_for(ser, 1.8).decode("utf-8", "replace")
    ser.write(b"exit\n")
    read_for(ser, 0.15)
    return text


def sample(ser: serial.Serial) -> dict[str, int] | None:
    text = ubxstat(ser)
    values = {}
    for key in FIELDS:
        match = re.search(r"\b" + re.escape(key) + r"=(\d+)", text)
        if match:
            values[key] = int(match.group(1))
    required = {"pvt", "eoe", "fifo_hw", "fifo_hal", "irq", "cfg_fail"}
    return values if required.issubset(values) else None


def main() -> int:
    args = parse_args()
    if args.seconds < 30:
        raise SystemExit("--seconds 至少 30，避免用启动瞬时数据下结论")
    baseline = None
    failures: list[str] = []
    started = time.monotonic()
    next_sample = started
    try:
        with serial.Serial(args.port, args.baud, timeout=0.25) as ser:
            while True:
                elapsed = time.monotonic() - started
                if elapsed > args.seconds:
                    break
                current = sample(ser)
                if current is None:
                    failures.append("DBG/ubxstat 字段不完整")
                    break
                first_sample = baseline is None
                if first_sample:
                    baseline = current
                delta = {k: current.get(k, 0) - baseline.get(k, 0) for k in FIELDS}
                config_ok = (current["fifo_hw"] == 1 and current["fifo_hal"] == 1
                             and current["irq"] == 0 and current["cfg_fail"] == 0)
                if not config_ok:
                    failures.append("UART4 FIFO/IRQ 配置失效")
                for key in ("cksum_fail", "bad_pvt", "frame_timeout", "eoe_no_pvt"):
                    if delta[key] != 0:
                        failures.append(f"{key} 新增 {delta[key]}")
                if not first_sample and (delta["pvt"] <= 0 or delta["eoe"] <= 0):
                    failures.append("PVT/EOE 未持续增长")
                print(f"t={elapsed:5.0f}s delta={delta} config_ok={config_ok}", flush=True)
                if failures:
                    break
                next_sample += 30.0
                time.sleep(max(0.0, next_sample - time.monotonic()))
    except serial.SerialException as exc:
        print(f"串口错误: {exc}", file=sys.stderr)
        return 2

    if failures:
        print("FAIL: " + "；".join(dict.fromkeys(failures)))
        return 1
    print(f"PASS: {args.seconds}s 内无新增校验失败/坏PVT/缺历元/帧超时，FIFO+IRQ 配置稳定")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
