#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    rc_monitor.py
@brief   遥控通道实时监视：边打杆边看 CH1-CH8 与舵机输出，结束打印每通道 min/max
@module  工具链 / 实机调试
@details 进 DBG 控制台后循环发 `vanes`，解析通道原始值与 vane pct 单行刷新。
         结束时打印 8 个通道与两轴舵机的极值——打杆到底能直接看出哪个通道在动、
         哪个通道是恒定常量（未配置通道的典型特征）。
         典型用法：
           py -3.12 tools/rc_monitor.py --port COM12 --seconds 20
           py -3.12 tools/rc_monitor.py --port COM9 --seconds 30 --interval 0.2
@note    需要带 BFS_DBG_USB_SERIAL8 的构建才能用 USB 口（CoaxialVTVL_ICM45686_Dbg）；
         串口独占，运行期间别的程序不能开同一个口。
@see     TandemVec_FCS/src/communication.cpp（vanes 命令）、docs/registers/固件调试工具链-2026-11-01.md
"""

import argparse
import re
import sys
import time

import serial

BAUD = 921600

RE_CH14 = re.compile(r"ch1-4 roll/pitch/thr/yaw=(\d+)/(\d+)/(\d+)/(\d+)")
RE_CH58 = re.compile(r"ch5-8 arm/ign/mode/manual=(\d+)/(\d+)/(\d+)/(\d+)")
RE_MANUAL = re.compile(r"unlock=(\d+) link=(\d+) manual_tvc=(\d+)")
RE_VANE = re.compile(r"vane pct=(-?\d+(?:\.\d+)?)/(-?\d+(?:\.\d+)?)")

CH_NAMES = ["CH1 roll", "CH2 pitch", "CH3 thr", "CH4 yaw",
            "CH5 arm", "CH6 ign", "CH7 mode", "CH8 manual"]


def drain(ser, seconds):
    """在给定秒数内读空串口，返回解码文本。"""
    buf = bytearray()
    end = time.time() + seconds
    while time.time() < end:
        n = ser.in_waiting or 0
        if n:
            buf += ser.read(n)
        else:
            time.sleep(0.005)
    return buf.decode("utf-8", "replace")


def parse(text):
    """从一段文本里取最后一次出现的通道/舵机读数。"""
    ch = None
    m = RE_CH14.search(text)
    if m:
        ch = [int(x) for x in m.groups()] + [0, 0, 0, 0]
        m2 = RE_CH58.search(text)
        if m2:
            ch[4:8] = [int(x) for x in m2.groups()]
    man = RE_MANUAL.search(text)
    vane = RE_VANE.search(text)
    return ch, man.groups() if man else None, vane.groups() if vane else None


def main():
    ap = argparse.ArgumentParser(description="遥控通道实时监视（边打杆边看）")
    ap.add_argument("--port", required=True, help="串口，如 COM12（UART8）/ COM9（Serial6）")
    ap.add_argument("--seconds", type=float, default=20.0, help="监视时长（默认 20 s）")
    ap.add_argument("--interval", type=float, default=0.3, help="每次采样间隔（默认 0.3 s）")
    args = ap.parse_args()

    ser = serial.Serial(args.port, BAUD, timeout=0.05)
    ch_min = [None] * 8
    ch_max = [None] * 8
    roll_pct_min = roll_pct_max = None
    samples = 0
    try:
        drain(ser, 0.4)
        ser.write(b"DBG\n")
        ser.write(b"\n")
        time.sleep(0.8)
        drain(ser, 0.6)

        t0 = time.time()
        print("端口 %s  时长 %.0fs  间隔 %.2fs —— 现在开始打杆，Ctrl+C 提前结束"
              % (args.port, args.seconds, args.interval))
        print("     CH1  CH2  CH3  CH4 | CH5  CH6  CH7  CH8 | unlock link manual | roll%%")
        while time.time() - t0 < args.seconds:
            ser.write(b"vanes\n")
            time.sleep(args.interval)
            text = drain(ser, 0.15)
            ch, man, vane = parse(text)
            if not ch:
                continue
            samples += 1
            for i in range(8):
                ch_min[i] = ch[i] if ch_min[i] is None else min(ch_min[i], ch[i])
                ch_max[i] = ch[i] if ch_max[i] is None else max(ch_max[i], ch[i])
            roll_pct = float(vane[0]) if vane else 0.0
            if vane:
                roll_pct_min = roll_pct if roll_pct_min is None else min(roll_pct_min, roll_pct)
                roll_pct_max = roll_pct if roll_pct_max is None else max(roll_pct_max, roll_pct)
            man_s = "%s/%s/%s" % man if man else "?/?/?"
            print("%7d%5d%5d%5d |%5d%5d%5d%5d | %s | %6.2f"
                  % (ch[0], ch[1], ch[2], ch[3], ch[4], ch[5], ch[6], ch[7], man_s, roll_pct),
                  end="\r")
        print()
    except KeyboardInterrupt:
        print("\n（提前结束）")
    finally:
        try:
            ser.write(b"exit\n")
            time.sleep(0.2)
            drain(ser, 0.2)
        except Exception:
            pass
        ser.close()

    if samples == 0:
        print("没有解析到任何一帧 vanes 输出，检查端口/波特率/构建是否带 DBG 入口")
        return 2

    print("\n==== %d 个采样，通道极值（跨度 >20 说明该通道在动）====" % samples)
    for i in range(8):
        span = ch_max[i] - ch_min[i]
        flag = "  <-- 在动" if span > 20 else ("  （恒定）" if span == 0 else "")
        print("  %-10s min=%4d max=%4d span=%4d%s" % (CH_NAMES[i], ch_min[i], ch_max[i], span, flag))
    if roll_pct_min is not None:
        print("  roll 舵机输出 min=%.2f%% max=%.2f%% span=%.2f%%"
              % (roll_pct_min, roll_pct_max, roll_pct_max - roll_pct_min))
    return 0


if __name__ == "__main__":
    sys.exit(main())