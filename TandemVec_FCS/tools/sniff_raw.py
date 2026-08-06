# -*- coding: utf-8 -*-
"""原始字节嗅探 COM10"""
import serial, time, collections

s = serial.Serial('COM10', 2000000, timeout=0.3)
time.sleep(0.5)
s.reset_input_buffer()
data = b''
t0 = time.time()
while time.time() - t0 < 5.0:
    c = s.read(8192)
    if c:
        data += c
s.close()
print(f'收到 {len(data)} 字节')
if data:
    cnt = collections.Counter(data)
    print('高频:', [(f'0x{b:02X}', n) for b, n in cnt.most_common(8)])
    print('前 60 字节:', data[:60].hex(' '))
