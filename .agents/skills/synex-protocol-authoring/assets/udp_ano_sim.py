#!/usr/bin/env python3
"""ANO 协议 UDP 仿真数据源，用于 Synex CustomBinary 解析端到端验证。

帧格式取自固件 AnoComProtocol::buildFrame：
    AB 05 <dest> <func> <lenL> <lenH> <payload...> <sum> <add>
    sum = 逐字节累加和（从帧头第 1 字节起算）
    add = 逐字节 sum 的再累加

支持两个功能码：
    0x03 姿态欧拉  : roll/pitch/yaw 各 int16（deg×100）+ fusionSta u8        → 帧长 15
    0xF1 执行器    : 前摆角 int16(deg×100)、后摆角 int16(deg×100)、
                     前电机 uint16(%×10)、尾电机 uint16(%×10)、
                     差速 int16(×1000)、饱和 u8、预留 u8                     → 帧长 20

一个 UDP 数据报 = 一帧；Synex 侧协议解析按固定帧长切，所以只发一种功能码。
"""

import argparse
import math
import socket
import struct
import sys
import time

ANO_HEAD = 0xAB
ANO_LOCAL_ADDR = 0x05
ANO_GND_ADDR = 0xFF


def build_frame(func: int, payload: bytes) -> bytes:
    """按 ANO 规则组帧并追加 sum/add 双校验。"""
    body = bytes([ANO_HEAD, ANO_LOCAL_ADDR, ANO_GND_ADDR, func]) + struct.pack("<H", len(payload)) + payload
    s = 0
    a = 0
    for b in body:
        s = (s + b) & 0xFF
        a = (a + s) & 0xFF
    return body + bytes([s, a])


def clamp_i16(v: float) -> int:
    return max(-32768, min(32767, int(round(v))))


def clamp_u16(v: float) -> int:
    return max(0, min(65535, int(round(v))))


def frame_0x03(t: float) -> bytes:
    """姿态欧拉：roll ±10°/0.5Hz、pitch ±8°/0.3Hz、yaw ±30°/0.1Hz。"""
    roll = 10.0 * math.sin(2 * math.pi * 0.5 * t)
    pitch = 8.0 * math.sin(2 * math.pi * 0.3 * t)
    yaw = 30.0 * math.sin(2 * math.pi * 0.1 * t)
    payload = struct.pack("<hhhB", clamp_i16(roll * 100), clamp_i16(pitch * 100),
                          clamp_i16(yaw * 100), 3)
    return build_frame(0x03, payload)


def frame_0xF1(t: float) -> bytes:
    """执行器：摆角 ±5°/±4°，两电机 50% 基准对摆，差速 ±20。"""
    front_vane = 5.0 * math.sin(2 * math.pi * 0.5 * t)
    rear_vane = 4.0 * math.sin(2 * math.pi * 0.3 * t)
    mot_front = 50.0 + 8.0 * math.sin(2 * math.pi * 0.2 * t)
    mot_rear = 50.0 + 8.0 * math.sin(2 * math.pi * 0.2 * t + math.pi)
    diff = 20.0 * math.sin(2 * math.pi * 0.1 * t)
    payload = struct.pack("<hhHHhBB",
                          clamp_i16(front_vane * 100), clamp_i16(rear_vane * 100),
                          clamp_u16(mot_front * 10), clamp_u16(mot_rear * 10),
                          clamp_i16(diff * 1000), 0, 0)
    return build_frame(0xF1, payload)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1", help="目标地址")
    ap.add_argument("--port", type=int, default=19000, help="目标 UDP 端口")
    ap.add_argument("--func", default="0x03", choices=["0x03", "0xF1"], help="功能码")
    ap.add_argument("--rate", type=float, default=50.0, help="发送频率 Hz")
    ap.add_argument("--count", type=int, default=0, help="发送帧数，0=一直发")
    args = ap.parse_args()

    make = frame_0x03 if args.func == "0x03" else frame_0xF1
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / args.rate
    sent = 0
    t0 = time.perf_counter()
    next_t = t0

    print(f"UDP -> {args.host}:{args.port}  func={args.func}  {args.rate:.0f} Hz", flush=True)
    try:
        while args.count == 0 or sent < args.count:
            now = time.perf_counter()
            if now < next_t:
                time.sleep(min(next_t - now, 0.005))
                continue
            t = now - t0
            frame = make(t)
            sock.sendto(frame, (args.host, args.port))
            sent += 1
            if sent == 1:
                print(f"首帧 {len(frame)} 字节: {frame.hex(' ').upper()}", flush=True)
            next_t += period
            if now - next_t > 0.5:
                next_t = now
    except KeyboardInterrupt:
        pass
    finally:
        elapsed = max(1e-9, time.perf_counter() - t0)
        print(f"已发送 {sent} 帧 / {elapsed:.2f}s = {sent / elapsed:.1f} fps，帧长 {len(make(0))} 字节", flush=True)
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())