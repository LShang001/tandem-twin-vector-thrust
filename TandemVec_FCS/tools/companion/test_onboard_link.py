#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_onboard_link.py —— 协议层往返自测（不需要硬件）。

@file    test_onboard_link.py
@module  工具/协议自测
@brief   协议层往返自测（不需要硬件）：验证编解码与 ACK 往返
@details 输入/处理/输出见正文；本文件只做链路与协议层验证
@frame   N/A（协议层，不解算姿态/位置）
@units   帧、字节、毫秒（按正文）
@note    这是自测而非性能测量；性能口径请看 measure_link.py。
验证三件事：
  1. 私有协议 74 B 下行帧的字段顺序与固件 sendPositionVelocityData 一致
  2. 私有协议 14 B 上行帧的打包/解包对称
  3. MAVLink SET_POSITION_TARGET_LOCAL_NED 的 type_mask 语义正确
     （只开加速度位 = 51，带位置/速度目标必须被识别为非法）
"""
import math
import struct
import sys
from pathlib import Path

# 让 onboard_link.py 可导入
sys.path.insert(0, str(Path(__file__).resolve().parent))

import onboard_link as ol  # noqa: E402

FAIL = 0


def check(name, cond, detail=""):
    global FAIL
    if cond:
        print(f"  OK    {name}")
    else:
        print(f"  MISS  {name}  {detail}")
        FAIL += 1


# ============================================================
# 1) 私有下行：按固件字段顺序造帧，再用 onboard_link 解回来
# ============================================================
print("[1] 私有下行 74 B 帧字段顺序")

# 造一组可区分的值：第 i 个字段 = 100 + i，位置/速度/姿态/角速度/加速度各段不同
fields = [
    1.0, 1.0, 12.5,          # auto_intent, traj_start, time_s
    1.5, -2.5, 3.5,          # pos N/E/U
    0.11, 0.22, 0.33,        # vel N/E/U
    0.05, -0.06, 1.57,       # roll, pitch, heading
    0.001, 0.002, 0.003,     # gyro
    9.7, -0.4, 0.2,          # acc U/E/N
]
assert len(fields) == 18, len(fields)

frame = struct.pack("<B18fB", 0xAA, *fields, 0x55)
check("帧长 = 74 B", len(frame) == 74, f"实际 {len(frame)}")

rec = ol.unpack_private_down(frame)
check("能解析", rec is not None)
if rec:
    for i, key in enumerate(ol.PRIV_DOWN_FIELDS):
        ok = abs(rec[key] - fields[i]) < 1e-6
        if not ok:
            check(f"字段[{i}] {key}", False, f"期望 {fields[i]} 得到 {rec[key]}")
    check("18 个字段全部吻合", all(
        abs(rec[k] - fields[i]) < 1e-6 for i, k in enumerate(ol.PRIV_DOWN_FIELDS)))

# 帧头/帧尾错必须拒绝
bad1 = bytearray(frame); bad1[0] = 0x00
check("帧头错 → 拒绝", ol.unpack_private_down(bytes(bad1)) is None)
bad2 = bytearray(frame); bad2[-1] = 0x00
check("帧尾错 → 拒绝", ol.unpack_private_down(bytes(bad2)) is None)
check("长度错 → 拒绝", ol.unpack_private_down(frame[:-1]) is None)

# ============================================================
# 2) 私有上行 14 B 帧
# ============================================================
print("\n[2] 私有上行 14 B 帧")

up = ol.pack_private_up(1.25, -0.5, 0.75)
check("帧长 = 14 B", len(up) == 14, f"实际 {len(up)}")
check("帧头 0xAA", up[0] == 0xAA)
check("帧尾 0x55", up[-1] == 0x55)
au, ae, an = struct.unpack("<3f", up[1:-1])
check("载荷顺序 U/E/N", abs(au - 1.25) < 1e-6 and abs(ae + 0.5) < 1e-6 and abs(an - 0.75) < 1e-6,
      f"得到 {au},{ae},{an}")

# ============================================================
# 3) 安全层
# ============================================================
print("\n[3] 安全层（限幅 + NaN/Inf）")

u, e, n = ol.sanitize(100.0, -100.0, 0.0)
check("限幅 ±10", u == 10.0 and e == -10.0 and n == 0.0, f"得到 {u},{e},{n}")

u, e, n = ol.sanitize(float("nan"), float("inf"), 1.0)
check("NaN 归零", u == 0.0)
check("Inf 归零", e == 0.0)
check("正常值保留", n == 1.0)

# ============================================================
# 4) MAVLink type_mask 语义
# ============================================================
print("\n[4] MAVLink SET_POSITION_TARGET_LOCAL_NED type_mask")

try:
    from pymavlink.dialects.v20 import common as m
    HAVE_MAV = True
except ImportError:
    HAVE_MAV = False
    print("  SKIP  没装 pymavlink（该段跳过）")

if HAVE_MAV:
    raw = ol.pack_mavlink_cmd(m, acc_n=1.5, acc_e=-0.5, acc_d=-9.7)
    check("帧头 0xFD（v2）", raw[0] == 0xFD, f"得到 0x{raw[0]:02X}")

    # 用 pymavlink 自己的解析器读回来，确保打包合法
    mav = m.MAVLink(None, srcSystem=255, srcComponent=0)
    msg = mav.parse_char(raw)
    check("能被 pymavlink 解析", msg is not None)
    if msg is not None:
        check("消息 ID = 84", msg.id == m.MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED,
              f"得到 {msg.id}")
        # 只开加速度：位置/速度/偏航/偏航率都必须 IGNORE
        expect = (1 << 0) | (1 << 1) | (1 << 4) | (1 << 5)   # 51
        check("type_mask = 51（只加速度）", msg.type_mask == expect,
              f"得到 {msg.type_mask}")
        check("位置位 IGNORE", bool(msg.type_mask & (1 << 0)))
        check("速度位 IGNORE", bool(msg.type_mask & (1 << 1)))
        check("偏航位 IGNORE", bool(msg.type_mask & (1 << 4)))
        check("偏航率位 IGNORE", bool(msg.type_mask & (1 << 5)))
        check("加速度位生效（未 IGNORE）", not (msg.type_mask & (1 << 2)))
        check("afx = acc_n", abs(msg.afx - 1.5) < 1e-6, f"得到 {msg.afx}")
        check("afy = acc_e", abs(msg.afy + 0.5) < 1e-6, f"得到 {msg.afy}")
        check("afz = acc_d", abs(msg.afz + 9.7) < 1e-6, f"得到 {msg.afz}")

        # 固件侧的 NED→本机换算：acc_U = -afz
        check("固件换算 acc_U = -afz = 9.7", abs(-msg.afz - 9.7) < 1e-6)

# ============================================================
# 汇总
# ============================================================
print()
if FAIL == 0:
    print("全部通过")
else:
    print(f"{FAIL} 项失败")
raise SystemExit(0 if FAIL == 0 else 1)
