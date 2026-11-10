#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    read_serial.py
@brief   读指定串口 N 秒的原始字节并打印（飞控调试/数传口通用）
@module  工具链 / 实机调试
@details 用法：py -3.12 read_serial.py COM12 15
         用于抓飞控启动日志（Serial8/USB-C 调试口，921600）或确认数传口
         是否真的有数据流（Serial6，AnoCom 二进制）。
@theory  N/A
@frame   N/A
@units   N/A
@note    ★ 端口归属别搞混：[NAVP]/[Init] 等调试文本走 Serial8(UART8, USB-C)；
         DBG 控制台走 Serial6(USART6, 数传)。抓启动日志用前者，发 DBG 命令用后者。
         控制台输出用 `-X utf8`（或 PYTHONIOENCODING=utf-8），否则中文/替换字符
         在 GBK 终端会抛 UnicodeEncodeError。
@test    2026-11-10 实测：COM12 抓 45686 启动日志 3330 字节，含
         "Onboard IMU initialized successfully: ICM-45686 @3200Hz"。
@see     docs/registers/固件调试工具链-2026-11-01.md
"""

import sys
import time

import serial

BAUD = 921600


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2

    port = sys.argv[1]
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0

    ser = serial.Serial(port, BAUD, timeout=0.2)
    buf = bytearray()
    t0 = time.time()
    try:
        while time.time() - t0 < secs:
            chunk = ser.read(8192)
            if chunk:
                buf += chunk
    finally:
        ser.close()

    print('PORT=%s BYTES=%d' % (port, len(buf)))
    text = buf.decode('utf-8', 'replace')
    print(text[:8000] if text else '(no data)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
