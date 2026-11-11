#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    ano_rate.py
@brief   统计 AnoCom 下行各功能码帧率（实测遥测频率/链路健康）
@module  工具链 / 实机调试
@details 输入：串口名（数传口 Serial6，如 COM14）、采样秒数；处理：按帧头
         AB 05 FF 同步解析 AnoCom 帧（func + len + data + SC/AC），按功能码计数；
         输出：总流量 B/s + 各功能码帧数与 Hz。用于核对 communication.cpp 分频
         是否生效、无线侧是否过载丢帧（过载时各码会齐降到同一低 Hz）。
         典型用法（在 Flash 节点上）：
           python -X utf8 ano_rate.py COM14 6
@theory  N/A（统计工具，不含算法）
@frame   N/A
@units   流量 B/s；帧率 Hz
@note    端口归属：数传口 Serial6（AnoCom 二进制）；调试文本口 Serial8 不是本工具
         对象。帧率低于固件发送率即无线侧丢帧。当前频率口径见 COMM-001「当前遥测频率」。
@see     TandemVec_FCS/docs/COMM-001-数传通信链路优化方案.md、tools/fw_flash/README.md
"""
import serial
import sys
import time

port = sys.argv[1]
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
s = serial.Serial(port, 921600, timeout=0.2)
buf = bytearray()
counts = {}
total = 0
t0 = time.time()
while time.time() - t0 < secs:
    chunk = s.read(8192)
    if chunk:
        buf.extend(chunk)
        total += len(chunk)
    while len(buf) >= 8:
        if buf[0] != 0xAB or buf[1] != 0x05 or buf[2] != 0xFF:
            del buf[0]
            continue
        ln = buf[4] | (buf[5] << 8)
        if ln > 512:
            del buf[0]
            continue
        frame_len = 6 + ln + 2
        if len(buf) < frame_len:
            break
        func = buf[3]
        counts[func] = counts.get(func, 0) + 1
        del buf[:frame_len]
el = time.time() - t0
s.close()
print('elapsed=%.2fs bytes=%d (%.0f B/s)' % (el, total, total / el))
names = {0x01: 'IMU', 0x03: 'Euler', 0x05: 'Alt', 0x06: 'Mode', 0x07: 'Speed',
         0x08: 'Pos', 0x0A: 'TgtAtt', 0x0B: 'TgtSpd', 0x0D: 'Volt', 0x20: 'PWM',
         0x21: 'Ctrl', 0x30: 'GPS', 0x40: 'RC', 0xF1: 'Act', 0xF2: 'Vars',
         0xF4: 'Health', 0xF5: 'Diag'}
for f in sorted(counts):
    print('0x%02X %-7s: %5d = %6.1f Hz' % (f, names.get(f, '?'), counts[f], counts[f] / el))