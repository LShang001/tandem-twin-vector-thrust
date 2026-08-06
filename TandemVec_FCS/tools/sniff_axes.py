# -*- coding: utf-8 -*-
"""陀螺/欧拉轴方向实时测试工具
用法：解锁通电后运行，然后按提示操作：
  1) 绕竖直轴(x_b)右转机头 → gyro_x 应变正
  2) 机头向前倒(绕y_b)     → gyro_y 应变正
  3) 右侧机翼下沉(绕z_b)   → gyro_z 应变正
连续打印 0x01(IMU) + 0x03(欧拉) 帧
"""
import serial, time, struct

s = serial.Serial('COM10', 2000000, timeout=0.3)
time.sleep(0.3)
s.reset_input_buffer()
print('轴方向测试：连续打印 欧拉(°)/陀螺(rad/s)。开始操作…')
t0 = time.time()
last = 0.0
while time.time() - t0 < 20.0:
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
                if func == 0x01 and ln >= 13 and now - last > 0.6:
                    ax = struct.unpack('<h', p[0:2])[0] * 0.01 / 9.81
                    ay = struct.unpack('<h', p[2:4])[0] * 0.01 / 9.81
                    az = struct.unpack('<h', p[4:6])[0] * 0.01 / 9.81
                    gx = struct.unpack('<h', p[6:8])[0] * (2000.0/32768.0)
                    gy = struct.unpack('<h', p[8:10])[0] * (2000.0/32768.0)
                    gz = struct.unpack('<h', p[10:12])[0] * (2000.0/32768.0)
                    print(f'acc(g): x={ax:+.3f} y={ay:+.3f} z={az:+.3f} | gyro(dps): x={gx:+.3f} y={gy:+.3f} z={gz:+.3f}', end='  ')
                    last = now
                if func == 0x03 and ln >= 12 and now - last < 0.3:
                    r = struct.unpack('<f', p[0:4])[0] * 57.29578
                    p_ = struct.unpack('<f', p[4:8])[0] * 57.29578
                    y = struct.unpack('<f', p[8:12])[0] * 57.29578
                    print(f'欧拉: roll={r:+.1f} pitch={p_:+.1f} yaw={y:+.1f}')
                i += 6 + ln + 2
                continue
        i += 1
s.close()
print('采样结束')
