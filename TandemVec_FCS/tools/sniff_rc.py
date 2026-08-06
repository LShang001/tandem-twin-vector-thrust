# -*- coding: utf-8 -*-
"""抓取 COM10 (2M 数传) AnoCom 帧：模式/解锁 + 8 通道输出值"""
import serial, time

s = serial.Serial('COM10', 2000000, timeout=0.3)
time.sleep(0.3)
s.reset_input_buffer()
seen = {}
t0 = time.time()
while time.time() - t0 < 4.0:
    data = s.read(8192)
    if not data:
        continue
    i = 0
    while i < len(data) - 2:
        if data[i] == 0xAB and data[i+1] == 0x05 and data[i+2] == 0xFF:
            func = data[i+3]
            ln = data[i+4] | (data[i+5] << 8)
            if i + 6 + ln + 2 <= len(data):
                payload = data[i+6:i+6+ln]
                seen.setdefault(func, payload)
                i += 6 + ln + 2
                continue
        i += 1
s.close()

def u16(b, off):
    return int.from_bytes(b[off:off+2], 'little')

if 0x06 in seen:
    p = seen[0x06]
    print(f'模式帧: mode={p[0]} 解锁={p[1]}')
if 0x20 in seen:
    p = seen[0x20]
    ch = [u16(p, i*2) for i in range(min(8, len(p)//2))]
    names = ['S1', 'S2', 'S3', 'S4', 'S5', 'S6', 'S7', 'S8']
    for n, v in zip(names, ch):
        print(f'  {n}: {v}')
else:
    print('未收到 0x20 输出帧')
