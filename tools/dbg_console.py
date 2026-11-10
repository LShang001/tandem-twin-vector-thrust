#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    dbg_console.py
@brief   飞控 DBG 调试控制台驱动（Serial6，默认 921600）
@module  工具链 / 实机调试
@details 把一条或多条 DBG 命令发到飞控并回收文本输出，用于上板功能自检。
         典型用法：
           py -3.12 tools/dbg_console.py COM12 health
           py -3.12 tools/dbg_console.py COM6 "param get 0"
           py -3.12 tools/dbg_console.py COM6 "wsmode 0;wsstat;wsmode 1;wsstat"

         ★ 关键约定（2026-11-10 踩坑）：所有 argv 会被用空格拼成**一整行**再发送，
         多条命令用 ';' 分隔。若不这样拼，shell 会先把 `param get 0` 拆成三个
         argv，逐个发送就变成三条命令，固件回
         `param count=169` + usage + `unknown: 'get'` + `unknown: '0'`，
         看起来像固件命令解析坏了——实际是调用方式错。

@theory  N/A（调试工具，不含算法）
@frame   N/A
@units   N/A
@note    端口归属：Serial8(UART8, USB-C) 打印 [NAVP] 等调试文本；DBG 控制台在
         Serial6(USART6, 数传)。两者是不同 COM 口，发错口只会看到打印没有响应。
         datalog / flowdump 是阻塞型，回收窗口自动放宽到 5.5 s。
@test    2026-11-10 实测通过：`param get 0` → att_roll.kp=4.2500；
         `wsmode 0/1` → driver 0↔1 双向切换；`ws 0 255 0` → R0 G255 B0；
         `flash id` → JEDEC 0xEF 0xAA 0x21（W25N01GV）；`health` → IMU 3276.8Hz。
@see     docs/registers/固件调试工具链-2026-11-01.md、TandemVec_FCS/src/communication.cpp
         （handleDebugConsole）
"""

import sys
import time

import serial

BAUD = 921600
BLOCKING_CMDS = ('datalog', 'flowdump')


def drain(ser, seconds):
    """在给定秒数内读空串口，返回解码后的文本。"""
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < seconds:
        chunk = ser.read(8192)
        if chunk:
            buf += chunk
    return buf.decode('utf-8', 'replace')


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2

    port = sys.argv[1]
    raw = ' '.join(sys.argv[2:])
    cmds = [c.strip() for c in raw.split(';') if c.strip()]

    ser = serial.Serial(port, BAUD, timeout=0.2)
    try:
        drain(ser, 0.4)

        ser.write(b'DBG\n')
        time.sleep(0.5)
        print('===== enter DBG =====')
        print(drain(ser, 1.0))

        for c in cmds:
            ser.write((c + '\n').encode('utf-8'))
            print('===== %s =====' % c)
            wait = 5.5 if c.split(' ')[0] in BLOCKING_CMDS else 1.0
            time.sleep(0.35)
            print(drain(ser, wait))

        ser.write(b'exit\n')
        time.sleep(0.3)
        drain(ser, 0.3)
    finally:
        ser.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
