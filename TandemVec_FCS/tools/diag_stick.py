# -*- coding: utf-8 -*-
"""诊断：打杆无反应。抓 解锁/CH/控制输出，10s 内请打杆（CH1/CH2/CH4）"""
import serial, time, struct

s = serial.Serial('COM10', 2000000, timeout=0.3)
time.sleep(0.3)
s.reset_input_buffer()
print('开始采集 10s —— 请打杆（CH1滚转/CH2俯仰/CH4偏航）…')
t0 = time.time()
last = 0.0
ch = {}
mode = unlock = None
while time.time() - t0 < 10.0:
    data = s.read(8192)
    if not data:
        continue
    i = 0
    while i < len(data) - 2:
        if data[i] == 0xAB and data[i+1] == 0x05 and data[i+2] == 0xFF:
            func = data[i+3]
            ln = data[i+4] | (data[i+5] << 8)
            if i + 6 + ln + 2 <= len(data):
                p = data[i+6:i+6+ln]
                now = time.time()
                if func == 0x06 and ln >= 2:
                    mode, unlock = p[0], p[1]
                if func == 0x20 and ln >= 16:
                    ch = [int.from_bytes(p[j*2:j*2+2], 'little') for j in range(8)]
                if func == 0x21 and ln >= 8 and now - last > 0.7:
                    ctrl = [struct.unpack('<h', p[o:o+2])[0] / 10.0 for o in (0, 2, 4, 6)]
                    cs = ' '.join(f'{c:6.1f}' for c in ctrl)
                    chs = ' '.join(f'{c:5d}' for c in ch) if ch else '?'
                    print(f'mode={mode} 解锁={unlock} | CH1-8: {chs} | 控制输出(r/p/t/y): {cs}')
                    last = now
                i += 6 + ln + 2
                continue
        i += 1
s.close()
print('采集结束')
