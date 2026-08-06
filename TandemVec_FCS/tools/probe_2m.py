# 验证数传 2M 波特率假设
import serial
import time
import collections

s = serial.Serial('COM10', 2000000, timeout=0.3)
time.sleep(0.5)
s.reset_input_buffer()
data = b''
t0 = time.time()
while time.time() - t0 < 3.0:
    c = s.read(8192)
    if c:
        data += c
s.close()
print(f'2M 波特率 3s 收到 {len(data)} 字节')
if data:
    cnt = collections.Counter(data)
    print('高频字节:', [(f'0x{b:02X}', n) for b, n in cnt.most_common(6)])
    pos0 = [i for i, b in enumerate(data) if b == 0x00]
    if len(pos0) > 2:
        gaps = [pos0[i + 1] - pos0[i] for i in range(len(pos0) - 1)]
        print('0x00 间隔众数:', collections.Counter(gaps).most_common(4))
    print('前 100 字节:', data[:100].hex(' '))
