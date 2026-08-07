#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
flash_export.py — 从 W25N01GV 黑匣子导出并解析飞行数据

用法:
  python flash_export.py <COM口> [startPage] [count] [-o out.csv]

流程:
  1. 连接串口（波特率 SERIAL6_BAUDRATE，默认 2000000）
  2. 进入调试模式 (DBG\\n)
  3. 发 `flash export <startPage> <count>`
  4. 接收原始帧字节流，按 W25N01GV_LOG_FRAME_SIZE (95B) 切块
  5. 校验 magic (0xAA55) + CRC8，解 21 个 float → CSV

帧格式（与固件 lib/W25N01GV 一致）:
  magic(2) + seq(4) + t_ms(4) + payload(84=21×float) + crc8(1)

CSV 列（与 Serial3 CSV 一致）:
  seq, t_ms, roll_deg, pitch_deg, heading_deg,
  accel_x_ms2, accel_y_ms2, accel_z_ms2,
  gyro_x_dps, gyro_y_dps, gyro_z_dps,
  vel_n_ms, vel_e_ms, vel_d_ms,
  rel_n_m, rel_e_m, rel_d_m,
  tvc1_deg, tvc2_deg, valve_ctrl, p1, p2
"""

import sys
import struct
import time
import argparse

import serial

FRAME_SIZE = 95          # W25N01GV_LOG_FRAME_SIZE
PAYLOAD_LEN = 84         # W25N01GV_LOG_PAYLOAD
MAGIC = b'\xAA\x55'

# CRC-8 (poly 0x31, 同固件 lib/crc8 类)
def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x31) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

CSV_HEADER = ("seq,t_ms,roll_deg,pitch_deg,heading_deg,"
              "accel_x_ms2,accel_y_ms2,accel_z_ms2,"
              "gyro_x_dps,gyro_y_dps,gyro_z_dps,"
              "vel_n_ms,vel_e_ms,vel_d_ms,"
              "rel_n_m,rel_e_m,rel_d_m,"
              "tvc1_deg,tvc2_deg,valve_ctrl,p1,p2")

def parse_frames(data: bytes):
    """按 magic 同步切块：扫描 0xAA55 定位帧起点，再按固定帧长切。
    容错：开头可能有调试文本（[DBG] ...），尾部可能有残余。"""
    frames = []
    i = 0
    n = len(data)
    while i < n - FRAME_SIZE + 1:
        if data[i:i+2] == MAGIC:
            f = data[i:i + FRAME_SIZE]
            if crc8(f[:-1]) == f[-1]:
                frames.append(f)
                i += FRAME_SIZE
                continue
            else:
                # CRC 错——可能错位，跳过 1 字节继续找
                i += 1
                continue
        i += 1
    return frames

def frame_to_row(f: bytes):
    seq, t_ms = struct.unpack('<II', f[2:10])
    payload = f[10:10 + PAYLOAD_LEN]
    vals = struct.unpack('<21f', payload)
    return (seq, t_ms) + vals

def main():
    ap = argparse.ArgumentParser(description="W25N01GV blackbox export")
    ap.add_argument("port", help="串口 (如 COM10)")
    ap.add_argument("start_page", type=int, nargs="?", default=0)
    ap.add_argument("count", type=int, nargs="?", default=200)
    ap.add_argument("-b", "--baud", type=int, default=2000000,
                    help="波特率 (默认 2000000 = SERIAL6_BAUDRATE)")
    ap.add_argument("-o", "--out", default="blackbox_export.csv")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    time.sleep(0.3)
    ser.reset_input_buffer()

    # 进入调试模式
    ser.write(b'DBG\n')
    time.sleep(0.3)
    ser.read(8192)   # 丢弃提示

    # 发导出命令
    cmd = f"flash export {args.start_page} {args.count}\n".encode()
    ser.write(cmd)
    time.sleep(0.2)

    # 收集直到超时（每页 ~1ms，count 页 + 余量）
    raw = bytearray()
    deadline = time.time() + 2.0 + args.count * 0.01
    while time.time() < deadline:
        chunk = ser.read(65536)
        if chunk:
            raw.extend(chunk)
        else:
            time.sleep(0.02)

    frames = parse_frames(bytes(raw))
    print(f"收到 {len(raw)} 字节, 解析出 {len(frames)} 帧")

    if not frames:
        print("无有效帧（检查 startPage/count 或 Flash 是否已写数据）")
        return

    with open(args.out, 'w', newline='') as f:
        f.write(CSV_HEADER + "\n")
        for fr in frames:
            row = frame_to_row(fr)
            f.write(",".join(f"{v:.4f}" if isinstance(v, float) else str(v)
                             for v in row) + "\n")

    print(f"已写出 {args.out}")
    # 打印前 3 行预览
    print("--- 预览 ---")
    print(CSV_HEADER)
    for fr in frames[:3]:
        row = frame_to_row(fr)
        print(",".join(f"{v:.4f}" if isinstance(v, float) else str(v)
                       for v in row))

if __name__ == "__main__":
    main()
