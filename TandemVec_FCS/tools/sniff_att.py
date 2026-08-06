# -*- coding: utf-8 -*-
"""连续抓取 0x02 姿态欧拉 + 0x03 IMU 帧，观察数值是否实时变化"""
import serial, time

s = serial.Serial('COM10', 2000000, timeout=0.3)
time.sleep(0.3)
s.reset_input_buffer()
t0 = time.time()
last_print = 0.0
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
                if func == 0x03 and ln >= 12 and time.time() - last_print > 0.8:
                    def f32(off):
                        import struct
                        return struct.unpack('<f', p[off:off+4])[0]
                    roll, pitch, yaw = f32(0), f32(4), f32(8)
                    print(f'欧拉: roll={roll:+.2f} pitch={pitch:+.2f} yaw={yaw:+.2f}')
                    last_print = time.time()
                if func == 0x21 and ln >= 8 and time.time() - last_print > 0.8:
                    import struct
                    ctrl = [struct.unpack('<h', p[o:o+2])[0] / 10.0 for o in (0, 2, 4, 6)]
                    print(f'控制输出: roll={ctrl[0]:+.1f} pitch={ctrl[1]:+.1f} thr={ctrl[2]:+.1f} yaw={ctrl[3]:+.1f}')
                    last_print = time.time()
                i += 6 + ln + 2
                continue
        i += 1
s.close()
print('采样结束')
