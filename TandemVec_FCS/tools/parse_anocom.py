# AnoCom 遥测流解析：帧头 0xAB 0x05 0xFF | func | len_lo len_hi | data | sum1 | sum2
import serial
import time
import collections
import struct

s = serial.Serial('COM10', 2000000, timeout=0.3)
time.sleep(0.5)
s.reset_input_buffer()
data = b''
t0 = time.time()
while time.time() - t0 < 4.0:
    c = s.read(16384)
    if c:
        data += c
s.close()
print(f'4s 收到 {len(data)} 字节')

FUNC_NAMES = {
    0x00: '数据校验', 0x01: 'IMU数据', 0x02: '磁力/气压/温度', 0x03: '姿态欧拉',
    0x04: '姿态四元数', 0x05: '高度数据', 0x06: '飞行模式', 0x07: '飞行速度',
    0x08: '位置偏移', 0x09: '风速估计', 0x0A: '目标姿态', 0x0B: '目标速度',
    0x0C: '回航信息', 0x0D: '电压电流', 0x0E: '外接模块状态', 0x20: 'PWM输出',
    0x21: '姿态控制输出', 0x30: 'GPS信息1', 0xE3: '设备信息',
}

# 逐字节扫帧（帧头 0xAB 0x05 0xFF 后取 func/len）
i = 0
frames = []
while i < len(data) - 7:
    if data[i] == 0xAB and data[i + 1] == 0x05 and data[i + 2] == 0xFF:
        func = data[i + 3]
        length = data[i + 4] | (data[i + 5] << 8)
        if i + 6 + length + 2 <= len(data):
            payload = data[i + 6:i + 6 + length]
            sum1 = data[i + 6 + length]
            calc1 = sum(data[i:i + 6 + length]) & 0xFF
            frames.append((func, length, payload, sum1 == calc1))
            i += 6 + length + 2
            continue
    i += 1

ok = sum(1 for f in frames if f[3])
print(f'解析出 {len(frames)} 帧, 校验通过 {ok} ({100 * ok / max(len(frames), 1):.0f}%)')
fc = collections.Counter(f[0] for f in frames)
print('功能码分布:', [(FUNC_NAMES.get(k, hex(k)), n) for k, n in fc.most_common(10)])

# 解析关键帧内容（各类首个有效帧）
shown = set()
for func, length, payload, valid in frames:
    if not valid or func in shown:
        continue
    shown.add(func)
    if func == 0x03 and length >= 7:   # 姿态欧拉 int16×100 + fusionStatus
        rs, ps, ys = struct.unpack('<hhh', payload[:6])
        print(f'[姿态欧拉] roll={rs / 100:+.2f}° pitch={ps / 100:+.2f}° yaw={ys / 100:+.2f}° fusion={payload[6]:#04x}')
    elif func == 0x04 and length >= 16:  # 姿态四元数
        w, x, y, z = struct.unpack('<ffff', payload[:16])
        print(f'[姿态四元数] w={w:.4f} x={x:+.4f} y={y:+.4f} z={z:+.4f}')
    elif func == 0x21 and length >= 16:  # 姿态控制输出
        r, p, t, y = struct.unpack('<ffff', payload[:16])
        print(f'[控制输出] roll={r:.2f} pitch={p:.2f} thr={t:.2f} yaw={y:.2f}')
    elif func == 0x05 and length >= 4:   # 高度
        a = struct.unpack('<f', payload[:4])[0]
        print(f'[高度] 气压高度={a:.2f}m')
    elif func == 0x06 and length >= 2:   # 飞行模式 + 解锁标志
        mode = payload[0]
        unlock = payload[1]
        mode_names = {0: 'MANUAL', 1: 'AUTO_POSITION', 2: 'AUTO_ALTITUDE', 3: 'GUIDED'}
        print(f'[飞行模式] mode={mode}({mode_names.get(mode, "?")}) 解锁={unlock}')
    elif func == 0x0D and length >= 4:   # 电压电流
        v = struct.unpack('<f', payload[:4])[0]
        print(f'[电压电流] 电压={v:.2f}V')
    elif func == 0x20 and length >= 8:   # PWM 输出
        vals = struct.unpack('<8H', payload[:16]) if length >= 16 else None
        if vals:
            print(f'[PWM输出] S1-S8={vals}')
    if len(shown) >= 6:
        break
