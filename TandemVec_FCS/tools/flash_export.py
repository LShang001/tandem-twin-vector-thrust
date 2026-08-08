#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
flash_export.py — 从 W25N01GV 黑匣子导出并解析飞行数据 (v2: I/P 差分帧)

用法:
  python flash_export.py <COM口> [startPage] [count] [-o out.csv]

流程:
  1. 连接串口（波特率 SERIAL6_BAUDRATE，默认 2000000）
  2. 进入调试模式 (DBG\\n)
  3. 发 `flash export <startPage> <count>`
  4. 接收原始字节流，按 magic 同步切帧
  5. 按 type 解析: I 帧(全量 95B) / P 帧(差分 51B)，差分还原 → CSV

帧格式 v2:
  I: magic(2) + type(1)=0x49 + seq(2) + t_ms(4) + 21×float(84) + crc16(2) = 95B
  P: magic(2) + type(1)=0x50 + seq(2) + dt_ms(2) + 21×int16(42) + crc16(2) = 51B
  差分: Δ = (cur - prev) * 100，I 帧每 32 帧重置参考，误差不累积

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

MAGIC = b'\xAA\x55'
TYPE_I = 0x49
TYPE_P = 0x50
IFRAME_SIZE = 95
PFRAME_SIZE = 51
PAYLOAD_LEN = 84          # 21 floats
DELTA_SCALE = 100

# CRC-16/CCITT (poly 0x1021, init 0xFFFF) — 与固件一致
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

CSV_HEADER = ("seq,t_ms,roll_deg,pitch_deg,heading_deg,"
              "accel_x_ms2,accel_y_ms2,accel_z_ms2,"
              "gyro_x_dps,gyro_y_dps,gyro_z_dps,"
              "vel_n_ms,vel_e_ms,vel_d_ms,"
              "rel_n_m,rel_e_m,rel_d_m,"
              "tvc1_deg,tvc2_deg,valve_ctrl,p1,p2")

def extract_frames(data: bytes):
    """magic 同步切帧，返回 (type, frame_bytes) 列表（CRC 校验通过）。"""
    frames = []
    i = 0
    n = len(data)
    while i < n - 2:
        if data[i:i+2] == MAGIC:
            # 尝试 I 帧
            if i + IFRAME_SIZE <= n and data[i+2] == TYPE_I:
                f = data[i:i+IFRAME_SIZE]
                if crc16(f[:-2]) == struct.unpack('<H', f[-2:])[0]:
                    frames.append((TYPE_I, f))
                    i += IFRAME_SIZE
                    continue
            # 尝试 P 帧
            if i + PFRAME_SIZE <= n and data[i+2] == TYPE_P:
                f = data[i:i+PFRAME_SIZE]
                if crc16(f[:-2]) == struct.unpack('<H', f[-2:])[0]:
                    frames.append((TYPE_P, f))
                    i += PFRAME_SIZE
                    continue
            # 都不匹配 → 假 magic，跳过
            i += 1
        else:
            i += 1
    return frames

def decode_frames(frames):
    """I/P 差分还原 → [(seq, t_ms, [21 floats])]。"""
    rows = []
    ref = [0.0] * 21
    ref_t = 0
    for typ, f in frames:
        seq = struct.unpack('<H', f[3:5])[0]
        if typ == TYPE_I:
            t_ms = struct.unpack('<I', f[5:9])[0]
            vals = list(struct.unpack('<21f', f[9:9+PAYLOAD_LEN]))
            ref = vals
            ref_t = t_ms
        else:  # TYPE_P
            dt = struct.unpack('<h', f[5:7])[0]
            t_ms = ref_t + dt
            deltas = struct.unpack('<21h', f[7:7+42])
            vals = [ref[i] + deltas[i] / DELTA_SCALE for i in range(21)]
            ref = vals
            ref_t = t_ms
        rows.append((seq, t_ms, vals))
    return rows

def main():
    ap = argparse.ArgumentParser(description="W25N01GV blackbox export (v2 I/P frames)")
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

    ser.write(b'DBG\n')
    time.sleep(0.3)
    ser.read(8192)

    cmd = f"flash export {args.start_page} {args.count}\n".encode()
    ser.write(cmd)
    time.sleep(0.2)

    raw = bytearray()
    deadline = time.time() + 2.0 + args.count * 0.01
    while time.time() < deadline:
        chunk = ser.read(65536)
        if chunk:
            raw.extend(chunk)
        else:
            time.sleep(0.02)

    frames = extract_frames(bytes(raw))
    print(f"收到 {len(raw)} 字节, 解析出 {len(frames)} 帧 "
          f"(I={sum(1 for t,_ in frames if t==TYPE_I)}, "
          f"P={sum(1 for t,_ in frames if t==TYPE_P)})")

    if not frames:
        print("无有效帧（检查 startPage/count 或 Flash 是否已写数据）")
        return

    rows = decode_frames(frames)
    with open(args.out, 'w', newline='') as f:
        f.write(CSV_HEADER + "\n")
        for seq, t_ms, vals in rows:
            line = f"{seq},{t_ms}," + ",".join(f"{v:.4f}" for v in vals)
            f.write(line + "\n")

    print(f"已写出 {args.out}")
    print("--- 预览 ---")
    print(CSV_HEADER)
    for seq, t_ms, vals in rows[:3]:
        print(f"{seq},{t_ms}," + ",".join(f"{v:.4f}" for v in vals))

if __name__ == "__main__":
    main()
