"""低频上行测试：10 Hz 发 20 帧，验证丢帧是速率问题还是物理层问题。

对比依据：
  - demo 模式 250 Hz 下发了 54 帧，飞控只收到 5 帧（约 9%）
  - 本脚本 10 Hz 发 20 帧（每 100ms 一帧，远低于串口和飞控处理能力）
若 10 Hz 下 20/20 全部到达 -> 高波特率下丢帧是缓冲/处理速率问题
若 10 Hz 仍大量丢失 -> 物理层问题（接线/电平/晶振偏差）
"""
import sys
import time

import serial

sys.path.insert(0, ".")
from onboard_link import pack_private_up  # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM11"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 921600
N = 20
PERIOD = 0.100


def main():
    s = serial.Serial(PORT, BAUD, timeout=0.2)
    t_start = time.time()
    try:
        time.sleep(0.3)
        sent = 0
        for i in range(N):
            # 指令：U=0.1 m/s² 恒定，E/N=0，便于在飞控侧区分
            s.write(pack_private_up(0.1, 0.0, 0.0))
            s.flush()
            sent += 1
            time.sleep(PERIOD)
        dt = time.time() - t_start
        print(f"[lowfreq] sent={sent} frames in {dt:.1f}s = {sent/dt:.1f} Hz")
        print(f"[lowfreq] expected uplink rx to grow by {sent}")
        return 0
    finally:
        s.close()


if __name__ == "__main__":
    sys.exit(main())
