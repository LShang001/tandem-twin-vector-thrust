# 921600 抓帧分析：找帧头模式与帧长
import serial
import time
import collections

s = serial.Serial('COM10', 921600, timeout=0.3)
time.sleep(0.5)
s.reset_input_buffer()
data = b''
t0 = time.time()
while time.time() - t0 < 3.0:
    chunk = s.read(8192)
    if chunk:
        data += chunk
s.close()
print(f'3s 收到 {len(data)} 字节')

# 帧头假设 A：0x00 开头（AnoCom）。统计 "0x00 X" 的 X 分布
pairs = collections.Counter()
for i in range(len(data) - 1):
    if data[i] == 0x00:
        pairs[data[i + 1]] += 1
print('0x00 后字节分布 top8:', [(f'0x{b:02X}', n) for b, n in pairs.most_common(8)])

# 帧头假设 B：0xAA / 0xA5 / 0xFE（MAVLink 0xFD / CRSF 0xC8）
for h in (0xAA, 0xA5, 0xFD, 0xC8, 0x55, 0x7E):
    n = data.count(bytes([h]))
    print(f'帧头候选 0x{h:02X}: {n} 次')

# 打印一段连续帧（前 200 字节，按 0x00 分帧）
print('前 200 字节:', data[:200].hex(' '))
