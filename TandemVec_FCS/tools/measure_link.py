#!/usr/bin/env python3
"""链路帧率与完整性测量（匿名助手协议 / AnoCom）。

@file    measure_link.py
@module  工具/链路测量（TandemVec_FCS/tools）
@brief   AnoCom 链路各帧类型的实测频率、丢帧与带宽
@details 输入：串口名、测量时长、`--raw` 原始字节扫描开关；
         处理：流式解帧（半帧留在缓冲区等下一块）、按帧类型统计频率与完整性，
         `--raw` 走原始字节扫描做独立交叉验证；
         输出：逐帧类型的实测 Hz、字节率与异常计数。
@theory  AnoCom 帧 = 0xAB + LEN + ...；流式解帧必须消费掉已解析长度并保留余量
@ref     PROTOCOL-001, docs/registers/固件调试工具链-2026-11-01.md
@frame   N/A（只做字节与时间统计，不解算姿态/位置）
@units   频率 Hz、带宽 B/s、时长 s
@note    ★ 2026-11-06 血的教训：本工具的前身写成 `extract_frames(chunk)` 并丢掉返回值，
  每个 512B 读块边界上的帧全被丢弃，测出「20Hz 只剩 10Hz」「6.9KB/s 只过 21%」，
  误判成链路带宽不足、白改了一轮固件分频。extract_frames 是**流式**接口，必须：

      buf += chunk
      frames, consumed = extract_frames(buf)
      buf = buf[consumed:]          # 半帧留在 buf 里等下一块

  用原始字节扫描（只认 0xAB + LEN）交叉验证后确认：降载前固件一直在按设计
  全速发，链路 100% 校验通过。**测量脚本的 bug 不能当成产品缺陷。**

用法：
    py measure_link.py COM6                    # 默认 30s
    py measure_link.py COM6 --seconds 60
    py measure_link.py COM6 --raw              # 原始字节扫描（独立交叉验证）
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

_GCS = Path(__file__).resolve().parents[1] / "GCS" / "server"
sys.path.insert(0, str(_GCS))

import serial  # noqa: E402

NAME = {
    0x01: "0x01 姿态(IMU)", 0x02: "0x02 罗盘/气压", 0x03: "0x03 欧拉角",
    0x05: "0x05 飞行模式", 0x06: "0x06 速度", 0x07: "0x07 高度", 0x08: "0x08 位置",
    0x0A: "0x0A 目标姿态", 0x0B: "0x0B 姿态速率", 0x0D: "0x0D 电池",
    0x20: "0x20 PWM", 0x21: "0x21 控制量", 0x30: "0x30 状态", 0x40: "0x40 遥控",
    0xF1: "0xF1 执行器", 0xF4: "0xF4 IMU 健康", 0xF5: "0xF5 链路诊断",
}


def scan_raw(buf: bytes):
    """原始字节扫描：只认 0xAB，LEN = u16 LE（buf[4] | buf[5]<<8）。

    返回 (counts, good, consumed)。用于与 anocom.extract_frames 交叉验证。
    """
    counts: dict[int, int] = {}
    good = 0
    i = 0
    n = len(buf)
    consumed = 0
    while i + 8 <= n:
        if buf[i] != 0xAB:
            i += 1
            continue
        length = buf[i + 4] | (buf[i + 5] << 8)
        end = i + 6 + length + 2
        if end > n:
            break
        if (sum(buf[i:end - 2]) & 0xFF) == buf[end - 2]:
            good += 1
            fid = buf[i + 3]
            counts[fid] = counts.get(fid, 0) + 1
        i = end
        consumed = end
    return counts, good, consumed


def main() -> int:
    """命令行入口：解析参数 → 打开串口 → 按时长统计各帧类型 → 打印实测结果。"""
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--raw", action="store_true", help="原始字节扫描（独立验证）")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    time.sleep(5.0)                     # 等固件稳定、跳过上电抖动
    ser.reset_input_buffer()

    buf = b""
    counts: dict[int, int] = {}
    stamps: dict[int, list[float]] = {}
    good = bad = 0
    t0 = time.time()

    if not args.raw:
        import anocom  # noqa: F401  (仅非 raw 模式需要)

    while time.time() - t0 < args.seconds:
        chunk = ser.read(1024)
        if not chunk:
            continue
        buf += chunk
        if args.raw:
            cnt, g, consumed = scan_raw(buf)
            for k, v in cnt.items():
                counts[k] = counts.get(k, 0) + v
                stamps.setdefault(k, []).append(time.time())
            good += g
            buf = buf[consumed:]
        else:
            import anocom
            frames, consumed = anocom.extract_frames(buf)   # ★ 流式用法
            now = time.time()
            for f in frames:
                if f.valid:
                    good += 1
                else:
                    bad += 1
                counts[f.func] = counts.get(f.func, 0) + 1
                stamps.setdefault(f.func, []).append(now)
            buf = buf[consumed:]
    ser.close()

    total = sum(counts.values())
    print(f"端口 {args.port}  时长 {args.seconds:.0f}s  "
          f"模式 {'原始扫描' if args.raw else 'anocom.extract_frames'}")
    print(f"总帧 {total} = {total / args.seconds:.1f} Hz  校验通过 {good}  失败 {bad}\n")
    print(f"{'功能码':<18} {'帧数':>6} {'Hz':>8} {'gap均值ms':>10}")
    for fid, c in sorted(counts.items(), key=lambda kv: -kv[1]):
        ts = stamps.get(fid, [])
        gaps = [(b - a) * 1000 for a, b in zip(ts, ts[1:])]
        gm = statistics.mean(gaps) if gaps else 0.0
        print(f"{NAME.get(fid, f'0x{fid:02X}'):<18} {c:>6} {c / args.seconds:>8.2f} {gm:>10.1f}")
    if total and good:
        print(f"\n校验通过率 {good / (good + bad) * 100:.1f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
