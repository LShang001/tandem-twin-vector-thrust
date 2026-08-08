#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
flash_export.py — 从 W25N01GV 黑匣子导出并解析飞行数据 (v2: I/P 差分帧 + S/E 飞行段)

用法:
  python flash_export.py <COM口> [startPage] [count] [-o out.csv]
  python flash_export.py <COM口> --auto [-o out.csv]     # 自动定位最新数据
  python flash_export.py <COM口> --auto --list            # 列出飞行段
  python flash_export.py <COM口> --auto --segment 2       # 只导出飞行段 2

帧格式 v2:
  I: magic(2)+type(1)=0x49+seq(2)+t_ms(4)+22xfloat(88)+crc16(2) = 99B
  P: magic(2)+type(1)=0x50+seq(2)+dt_ms(2)+21xint16(42)+crc16(2) = 51B
  S: magic(2)+type(1)=0x53+seg(2)+t_ms(4)+通道名表(\\0结尾)+crc16(2) 变长
  E: magic(2)+type(1)=0x45+seg(2)+dur_ms(4)+frames(4)+crc16(2) = 15B

列名自描述：S 帧携带通道名表，导出时自动识别列名（无需硬编码）。
"""

import struct
import time
import argparse

import serial

MAGIC = b'\xAA\x55'
TYPE_I = 0x49
TYPE_P = 0x50
TYPE_S = 0x53
TYPE_E = 0x45
IFRAME_SIZE = 99
PFRAME_SIZE = 51
EFRAME_SIZE = 15
SFRAME_MAX = 236   # 与固件 W25N01GV_LOG_SFRAME_MAX 一致（通道名 224 + 头 12）
PAYLOAD_LEN = 88          # 22 floats
DELTA_SCALE = 100


def crc16(data: bytes) -> int:
    """CRC-16/CCITT (poly 0x1021, init 0xFFFF) — 与固件一致"""
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
            t = data[i+2] if i+2 < n else 0
            if t == TYPE_I and i + IFRAME_SIZE <= n:
                f = data[i:i+IFRAME_SIZE]
                if crc16(f[:-2]) == struct.unpack('<H', f[-2:])[0]:
                    frames.append((TYPE_I, f))
                    i += IFRAME_SIZE
                    continue
            if t == TYPE_P and i + PFRAME_SIZE <= n:
                f = data[i:i+PFRAME_SIZE]
                if crc16(f[:-2]) == struct.unpack('<H', f[-2:])[0]:
                    frames.append((TYPE_P, f))
                    i += PFRAME_SIZE
                    continue
            if t == TYPE_E and i + EFRAME_SIZE <= n:
                f = data[i:i+EFRAME_SIZE]
                if crc16(f[:-2]) == struct.unpack('<H', f[-2:])[0]:
                    frames.append((TYPE_E, f))
                    i += EFRAME_SIZE
                    continue
            if t == TYPE_S and i + SFRAME_MAX <= n:
                f = data[i:i+SFRAME_MAX]
                if crc16(f[:-2]) == struct.unpack('<H', f[-2:])[0]:
                    frames.append((TYPE_S, f))
                    i += SFRAME_MAX
                    continue
            i += 1
        else:
            i += 1
    return frames


def decode_frames(frames):
    """I/P 差分还原 + S/E 段切分。返回 (rows, segments, columns)。"""
    rows = []
    N_F = PAYLOAD_LEN // 4   # float 通道数
    ref = [0.0] * N_F
    ref_t = 0
    cur_seg = 0
    columns = None
    default_cols = CSV_HEADER.split(',')
    segments = []

    for typ, f in frames:
        seq = struct.unpack('<H', f[3:5])[0]
        if typ == TYPE_S:
            seg_num = struct.unpack('<H', f[3:5])[0]
            t_ms = struct.unpack('<I', f[5:9])[0]
            names_raw = f[9:].split(b'\x00')[0]
            names = [x for x in names_raw.decode('ascii', errors='replace').split(',') if x]
            if names:
                columns = names
            cur_seg = seg_num
            segments.append({'num': seg_num, 'start_tms': t_ms,
                             'dur_ms': 0, 'frames': 0, 'columns': list(names)})
        elif typ == TYPE_E:
            seg_num = struct.unpack('<H', f[3:5])[0]
            dur_ms = struct.unpack('<I', f[5:9])[0]
            frames_cnt = struct.unpack('<I', f[9:13])[0]
            for sg in segments:
                if sg['num'] == seg_num:
                    sg['dur_ms'] = dur_ms
                    sg['frames'] = frames_cnt
        elif typ == TYPE_I:
            t_ms = struct.unpack('<I', f[5:9])[0]
            vals = list(struct.unpack(f'<{N_F}f', f[9:9+PAYLOAD_LEN]))
            ref = vals
            ref_t = t_ms
            rows.append((seq, t_ms, vals, cur_seg))
        elif typ == TYPE_P:
            dt = struct.unpack('<h', f[5:7])[0]
            t_ms = ref_t + dt
            deltas = struct.unpack(f'<{N_F}h', f[7:7+N_F*2])
            vals = [ref[i] + deltas[i] / DELTA_SCALE for i in range(N_F)]
            ref = vals
            ref_t = t_ms
            rows.append((seq, t_ms, vals, cur_seg))

    for sg in segments:
        sg['frames'] = max(sg['frames'], sum(1 for r in rows if r[3] == sg['num']))

    if columns is None:
        columns = default_cols
    return rows, segments, columns


def main():
    ap = argparse.ArgumentParser(description="W25N01GV blackbox export (v2 I/P+S/E frames)")
    ap.add_argument("port", help="串口 (如 COM10)")
    ap.add_argument("start_page", type=int, nargs="?", default=0)
    ap.add_argument("count", type=int, nargs="?", default=200)
    ap.add_argument("-b", "--baud", type=int, default=2000000,
                    help="波特率 (默认 2000000 = SERIAL6_BAUDRATE)")
    ap.add_argument("-o", "--out", default="blackbox_export.csv")
    ap.add_argument("--auto", action="store_true",
                    help="自动定位最新数据（flash stat 读 cursorPage）")
    ap.add_argument("--list", action="store_true",
                    help="列出所有飞行段（S/E 帧解析），不导出 CSV")
    ap.add_argument("--segment", type=int, default=0,
                    help="只导出指定飞行段号（0=全部）")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    time.sleep(0.3)
    ser.reset_input_buffer()

    ser.write(b'DBG\n')
    time.sleep(0.3)
    ser.read(8192)

    if args.auto:
        ser.write(b'flash stat\n')
        time.sleep(0.5)
        stat = ser.read(8192).decode('utf-8', errors='replace')
        cursor = 0
        for line in stat.split('\n'):
            if 'cursorPage=' in line:
                try:
                    cursor = int(line.split('cursorPage=')[1].strip())
                except ValueError:
                    pass
        if cursor == 0:
            print("无法获取 cursorPage，退回手动模式")
        else:
            args.start_page = max(0, cursor - args.count)
            print(f"[auto] cursorPage={cursor}, 导出 page {args.start_page}..{cursor}")
        time.sleep(0.2)
        ser.reset_input_buffer()

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
          f"P={sum(1 for t,_ in frames if t==TYPE_P)}, "
          f"S={sum(1 for t,_ in frames if t==TYPE_S)}, "
          f"E={sum(1 for t,_ in frames if t==TYPE_E)})")

    if not frames:
        print("无有效帧（检查 startPage/count 或 Flash 是否已写数据）")
        return

    rows, segments, columns = decode_frames(frames)

    if args.list:
        print(f"=== 飞行段列表（{len(segments)} 段）===")
        for sg in segments:
            cols = ','.join(sg['columns']) if sg['columns'] else '(默认)'
            print(f"  段 {sg['num']}: start_tms={sg['start_tms']} "
                  f"dur={sg['dur_ms']}ms frames={sg['frames']} cols=[{cols[:50]}]")
        return

    if args.segment > 0:
        rows = [r for r in rows if r[3] == args.segment]
        print(f"[segment] 段 {args.segment}: {len(rows)} 帧")

    if not rows:
        print("无有效数据帧")
        return

    header = (",".join(columns) if columns and len(columns) == len(rows[0][2])
              else CSV_HEADER)
    with open(args.out, 'w', newline='') as f:
        f.write(header + "\n")
        for seq, t_ms, vals, _seg in rows:
            line = f"{seq},{t_ms}," + ",".join(f"{v:.4f}" for v in vals)
            f.write(line + "\n")

    print(f"已写出 {args.out}（{len(rows)} 帧, 列名自描述: "
          f"{'是' if header != CSV_HEADER else '否(默认)'}）")
    print("--- 预览 ---")
    print(header)
    for seq, t_ms, vals, _seg in rows[:3]:
        print(f"{seq},{t_ms}," + ",".join(f"{v:.4f}" for v in vals))


if __name__ == "__main__":
    main()
