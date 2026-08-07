# -*- coding: utf-8 -*-
"""抓 0x03 姿态欧拉帧，观察机头朝天静止时 Heading 是否可靠（欧拉奇异点检验）"""
import serial, time, struct

s = serial.Serial('COM10', 2000000, timeout=0.3)
time.sleep(0.3)
s.reset_input_buffer()
print('抓取欧拉角 8s（机头朝天静止）…')
t0 = time.time()
last = 0.0
vals = []
while time.time() - t0 < 8.0:
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
                if func == 0x03 and ln >= 6:
                    r = struct.unpack('<h', p[0:2])[0] / 100.0
                    pt = struct.unpack('<h', p[2:4])[0] / 100.0
                    y = struct.unpack('<h', p[4:6])[0] / 100.0
                    vals.append((r, pt, y))
                    if now - last > 0.8:
                        print(f'roll={r:+8.2f}  pitch={pt:+8.2f}  yaw/Heading={y:+8.2f}')
                        last = now
                i += 6 + ln + 2
                continue
        i += 1
s.close()
if vals:
    import statistics as st
    for idx, name in enumerate(('roll', 'pitch', 'Heading')):
        col = [v[idx] for v in vals]
        print(f'{name:8s}: 均值={st.mean(col):+8.2f}  标准差={st.pstdev(col):6.2f}  '
              f'范围=[{min(col):+.2f}, {max(col):+.2f}]')
    print(f'样本数 {len(vals)}')
