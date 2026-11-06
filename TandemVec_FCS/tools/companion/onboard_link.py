#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""onboard_link.py —— 机载电脑（香橙派）与飞控的链路参考实现。

支持两种协议，自动识别：
  · 私有协议（默认）：下行 74 B / 18 float，上行 14 B / 3 float 加速度
  · MAVLink v2（onboard_link_proto=1）：下行 LOCAL_POSITION_NED + ATTITUDE，
    上行 SET_POSITION_TARGET_LOCAL_NED（只开加速度位）

为什么要自动识别：两种协议在开发期并存，接上串口后先看第一帧帧头
（0xAA = 私有，0xFD = MAVLink v2）就能确定走哪套，不需要手工配置。

安全设计（照搬 ~/mavlink_lan/control_link/pi_control.py 的思路）：
  · 三轴加速度限幅 ±10 m/s²，与固件侧一致
  · NaN/Inf 拦截
  · 指令失效超时后自动回零，不保持最后一条指令
  · 所有下发指令都过一遍 _sanitize()

用法：
  python3 onboard_link.py --port /dev/ttyACM0 --baud 921600 --probe
  python3 onboard_link.py --port /dev/ttyACM0 --baud 921600 --monitor
  python3 onboard_link.py --port /dev/ttyACM0 --baud 921600 --demo
"""
from __future__ import annotations

import argparse
import math
import struct
import sys
import time

# ============================================================
#  私有协议
# ============================================================
PRIV_DOWN_MAGIC = 0xAA
PRIV_UP_MAGIC = 0xAA
PRIV_TAIL = 0x55
PRIV_DOWN_FLOATS = 18
PRIV_DOWN_LEN = 1 + PRIV_DOWN_FLOATS * 4 + 1   # 74
PRIV_UP_FLOATS = 3
PRIV_UP_LEN = 1 + PRIV_UP_FLOATS * 4 + 1       # 14

# 下行 18 个字段的含义（字段顺序 = 固件 sendPositionVelocityData 的数组顺序）
PRIV_DOWN_FIELDS = [
    "auto_intent", "traj_start", "time_s",
    "pos_n", "pos_e", "pos_u",
    "vel_n", "vel_e", "vel_u",
    "roll", "pitch", "heading",
    "gyro_x", "gyro_y", "gyro_z",
    "acc_u", "acc_e", "acc_n",
]


def pack_private_up(acc_up: float, acc_east: float, acc_north: float) -> bytes:
    """打包 14 字节上行帧：0xAA + 3×float32(LE) + 0x55。"""
    return struct.pack("<B3fB", PRIV_UP_MAGIC, acc_up, acc_east, acc_north, PRIV_TAIL)


def unpack_private_down(buf: bytes) -> dict | None:
    """解析 74 字节下行帧；帧头帧尾不对返回 None。"""
    if len(buf) != PRIV_DOWN_LEN:
        return None
    if buf[0] != PRIV_DOWN_MAGIC or buf[-1] != PRIV_TAIL:
        return None
    vals = struct.unpack("<18f", buf[1:-1])
    return dict(zip(PRIV_DOWN_FIELDS, vals))


# ============================================================
#  安全层
# ============================================================
ACC_LIMIT = 10.0          # m/s²，与固件 handleGuidanceCommands 的限幅一致
CMD_TIMEOUT = 0.5         # s，与固件 trajectoryPlanningStarted 的 500 ms 超时一致


def sanitize(acc_up: float, acc_east: float, acc_north: float) -> tuple[float, float, float]:
    """NaN/Inf 拦截 + 三轴限幅。与固件侧同一口径。"""
    out = []
    for v in (acc_up, acc_east, acc_north):
        if not math.isfinite(v):
            v = 0.0
        out.append(max(-ACC_LIMIT, min(ACC_LIMIT, v)))
    return out[0], out[1], out[2]


# ============================================================
#  MAVLink（惰性导入：没装 pymavlink 时私有协议仍可用）
# ============================================================
def _import_mav():
    try:
        from pymavlink.dialects.v20 import common as m
        return m
    except ImportError:
        return None


def pack_mavlink_cmd(m, acc_n: float, acc_e: float, acc_d: float) -> bytes:
    """打包 SET_POSITION_TARGET_LOCAL_NED，只让加速度位生效。

    type_mask 位定义：
      bit0 POSITION_IGNORE  bit1 VELOCITY_IGNORE  bit2 ACCELERATION_IGNORE
      bit3 FORCE_IGNORE     bit4 YAW_IGNORE       bit5 YAW_RATE_IGNORE
    只要加速度 → 位置忽略 | 速度忽略 | 偏航忽略 | 偏航率忽略 = 1+2+16+32 = 51
    """
    mask = 1 | 2 | 16 | 32   # 51
    msg = m.MAVLink_set_position_target_local_ned_message(
        time_boot_ms=int(time.time() * 1000) & 0xFFFFFFFF,
        target_system=1, target_component=1,
        coordinate_frame=m.MAV_FRAME_LOCAL_NED,
        type_mask=mask,
        x=0.0, y=0.0, z=0.0,
        vx=0.0, vy=0.0, vz=0.0,
        afx=acc_n, afy=acc_e, afz=acc_d,
        yaw=0.0, yaw_rate=0.0,
    )
    return msg.pack(m.MAVLink(0, 0))


# ============================================================
#  串口读写
# ============================================================
def make_serial(port: str, baud: int):
    import serial
    s = serial.Serial(port, baud, timeout=0.01)
    s.reset_input_buffer()
    return s


def probe(port: str, baud: int, secs: float = 3.0) -> str:
    """探测串口上是哪种协议。返回 'private' / 'mavlink' / 'unknown'。"""
    s = make_serial(port, baud)
    buf = b""
    t0 = time.time()
    while time.time() - t0 < secs:
        d = s.read(8192)
        if d:
            buf += d
    s.close()

    if not buf:
        print(f"[probe] {port} @ {baud}: {secs:.0f} 秒内 0 字节")
        return "unknown"

    n_priv = sum(1 for i in range(len(buf) - 1)
                 if buf[i] == 0xAA and buf[i + 1] != 0x55)
    n_mav2 = buf.count(0xFD)
    n_mav1 = buf.count(0xFE)

    print(f"[probe] {port} @ {baud}: {len(buf)} 字节 / {secs:.0f}s")
    print(f"[probe] 候选帧头: 私有(0xAA)={n_priv}  MAVLink v2(0xFD)={n_mav2}  v1(0xFE)={n_mav1}")

    # 用帧长验证：私有 74B 帧头+帧尾配对
    ok_priv = 0
    for i in range(len(buf) - PRIV_DOWN_LEN + 1):
        if buf[i] == PRIV_DOWN_MAGIC and buf[i + PRIV_DOWN_LEN - 1] == PRIV_TAIL:
            ok_priv += 1
    print(f"[probe] 74B 帧头尾配对数 = {ok_priv}")

    if ok_priv >= 3:
        print("[probe] 判定: 私有协议")
        return "private"
    if n_mav2 >= 3:
        print("[probe] 判定: MAVLink v2")
        return "mavlink"
    print("[probe] 判定: 未知（数据不足或波特率不对）")
    return "unknown"


def monitor_private(port: str, baud: int, secs: float) -> None:
    s = make_serial(port, baud)
    buf = b""
    nframe = 0
    bad = 0
    t0 = time.time()
    last = None
    try:
        while time.time() - t0 < secs:
            d = s.read(8192)
            if not d:
                continue
            buf += d
            while len(buf) >= PRIV_DOWN_LEN:
                if buf[0] != PRIV_DOWN_MAGIC:
                    buf = buf[1:]
                    continue
                frame = buf[:PRIV_DOWN_LEN]
                if frame[-1] != PRIV_TAIL:
                    buf = buf[1:]
                    bad += 1
                    continue
                rec = unpack_private_down(frame)
                if rec:
                    nframe += 1
                    last = rec
                buf = buf[PRIV_DOWN_LEN:]
    finally:
        s.close()

    dt = max(time.time() - t0, 1e-6)
    print(f"[monitor] {nframe} 帧 / {dt:.1f}s = {nframe/dt:.1f} Hz，坏帧 {bad}")
    if last:
        print("[monitor] 最后一帧:")
        for k in PRIV_DOWN_FIELDS:
            print(f"    {k:<12} = {last[k]:>12.4f}")


def monitor_mavlink(port: str, baud: int, secs: float) -> None:
    m = _import_mav()
    if m is None:
        print("[monitor] 没装 pymavlink，无法解析 MAVLink。用 ~/venvs/base/bin/python 跑。")
        return
    from pymavlink import mavutil

    conn = mavutil.mavlink_connection(port, baud=baud, source_system=255)
    counts: dict[str, int] = {}
    latest: dict[str, object] = {}
    t0 = time.time()
    while time.time() - t0 < secs:
        msg = conn.recv_match(blocking=True, timeout=0.2)
        if msg is None:
            continue
        t = msg.get_type()
        counts[t] = counts.get(t, 0) + 1
        latest[t] = msg
    conn.close()

    dt = max(time.time() - t0, 1e-6)
    print(f"[monitor] MAVLink {sum(counts.values())} 帧 / {dt:.1f}s")
    for t, c in sorted(counts.items(), key=lambda kv: -kv[1]):
        print(f"    {t:<26} {c/dt:>7.2f} Hz")
    for t in ("LOCAL_POSITION_NED", "ATTITUDE"):
        if t in latest:
            print(f"[monitor] {t}: {latest[t]}")


def demo(port: str, baud: int, proto: str, secs: float) -> None:
    """最小闭环演示：读状态 → 给一个缓慢变化的指令 → 打印实际下发值。

    注意：这只是通道连通性演示，不是控制律。真实轨迹规划算法替换
    _simple_command() 即可。
    """
    s = make_serial(port, baud)
    print(f"[demo] 协议={proto}，{secs:.0f}s。发出的加速度指令 = 0.3·sin(0.5t)，仅验证通道。")
    t0 = time.time()
    sent = 0
    try:
        while time.time() - t0 < secs:
            t = time.time() - t0
            acc = 0.3 * math.sin(0.5 * t)
            au, ae, an = sanitize(acc, 0.0, 0.0)

            if proto == "private":
                s.write(pack_private_up(au, ae, an))
            else:
                m = _import_mav()
                if m is None:
                    print("[demo] 没装 pymavlink")
                    return
                s.write(pack_mavlink_cmd(m, acc_n=an, acc_e=ae, acc_d=-au))
            sent += 1

            # 顺手把下行读掉，避免缓冲堆积
            s.read(8192)
            time.sleep(0.004)   # 250 Hz
    finally:
        s.close()
    print(f"[demo] 共发 {sent} 帧，{sent/max(time.time()-t0,1e-6):.0f} Hz")


def main() -> int:
    ap = argparse.ArgumentParser(description="机载电脑 ↔ 飞控 链路参考实现")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--proto", choices=("auto", "private", "mavlink"), default="auto")
    ap.add_argument("--probe", action="store_true", help="探测串口上是哪种协议")
    ap.add_argument("--monitor", action="store_true", help="解析并统计下行帧")
    ap.add_argument("--demo", action="store_true", help="最小闭环：发缓慢正弦加速度")
    ap.add_argument("--seconds", type=float, default=5.0)
    args = ap.parse_args()

    if args.probe:
        probe(args.port, args.baud, args.seconds)
        return 0

    proto = args.proto
    if proto == "auto":
        proto = probe(args.port, args.baud, max(args.seconds, 3.0))
        print(f"[auto] 采用协议: {proto}")
        if proto == "unknown":
            return 2

    if args.monitor:
        if proto == "private":
            monitor_private(args.port, args.baud, args.seconds)
        else:
            monitor_mavlink(args.port, args.baud, args.seconds)
        return 0

    if args.demo:
        demo(args.port, args.baud, proto, args.seconds)
        return 0

    print("请指定 --probe / --monitor / --demo 之一")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
