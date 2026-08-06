# 扫描波特率：找帧结构最规律的（AnoCom 遥测帧头 0x00 间隔应显著成簇）
import serial
import time
import collections

BAUDS = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
results = []
for baud in BAUDS:
    try:
        s = serial.Serial('COM10', baud, timeout=0.3)
    except Exception as e:
        print(f'{baud}: 打开失败 {e}')
        continue
    time.sleep(0.5)
    s.reset_input_buffer()
    data = b''
    t0 = time.time()
    while time.time() - t0 < 2.0:
        chunk = s.read(4096)
        if chunk:
            data += chunk
    s.close()
    if len(data) < 50:
        print(f'{baud:7d}: {len(data)} 字节（几乎无数据）')
        continue
    # 0x00 间隔分布
    pos0 = [i for i, b in enumerate(data) if b == 0x00]
    gaps = [pos0[i + 1] - pos0[i] for i in range(len(pos0) - 1)] if len(pos0) > 1 else []
    mode = collections.Counter(gaps).most_common(3) if gaps else []
    # 规律性评分：间隔众数占比
    score = mode[0][1] / max(len(gaps), 1) if mode else 0
    results.append((score, baud, len(data), mode))
    print(f'{baud:7d}: {len(data)} 字节, 0x00×{len(pos0)}, 间隔众数 {mode[:2]}, 规律性 {score:.2f}')

results.sort(reverse=True)
print('\n最可能波特率:', results[0][1], f'（规律性 {results[0][0]:.2f}）' if results else '')
