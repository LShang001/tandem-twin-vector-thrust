#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    check_unlock_gate.py
@module  工具/解锁门红线校验
@brief   锁定解锁判据（CH5 > 1500 + isLinkUp 门控 + 中值滤波），防止被放宽或绕过
@details 输入：src/flight_control.cpp；处理：提取 inputs.is_unlocked 的赋值表达式，
         校验三个要素同时存在；输出：逐项结果 + 退出码。
@theory  解锁是全机唯一的安全总闸：判据一旦放宽（阈值降低或去掉 isLinkUp），
         上电无信号时（raw_rc 全 0）就可能被误判为已解锁，电机随即可转。
         2026-08-08 已有同类教训：开关判定缺 isLinkUp 兜底导致舵机钳满偏。
@ref     CTL-001 §遥控通道语义、AGENTS.md §安全关键修改规则
@frame   N/A（只做判据形式校验）
@units   CH5 PWM 脉宽 µs；阈值 1500 µs
@note    ★ 为什么需要这条：SIL 在 fw_sil_env_main.cpp:1721 直接写
         `g_is_unlocked = launched`，**绕过了真实解锁判据**；宿主测试也没有
         任何文件引用 is_unlocked/unlockMedian。也就是说这个安全总闸
         此前没有任何自动化覆盖，只靠实机「解锁能起飞」间接验证。
@see     tools/check_imu_axis_map.py（同类的文本级红线）
@test    本脚本即自检：退出码 0 表示解锁判据完整
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "TandemVec_FCS" / "src" / "flight_control.cpp"

# 判据必须同时具备的三个要素
REQUIREMENTS = [
    ("isLinkUp 门控", r"isLinkUp\s*&&", "缺 isLinkUp 兜底：上电 raw_rc 全 0 可能被误判已解锁"),
    ("阈值 > 1500", r">\s*1500(?:\.0f?)?", "阈值被改动：放宽等于降低解锁门槛"),
    ("通道索引 [4]", r"raw_rc_values\[4\]", "取的通道不是 CH5（索引 4）"),
    ("中值滤波", r"unlockMedian\.filter", "去掉中值滤波：单帧抖动可致解锁状态跳变"),
]


def main() -> int:
    if not SRC.exists():
        print(f"[FAIL] 找不到 {SRC}")
        return 2
    text = SRC.read_text(encoding="utf-8", errors="replace")

    m = re.search(r"inputs\.is_unlocked\s*=\s*([^;]+);", text)
    if not m:
        print("[FAIL] 找不到 inputs.is_unlocked 的赋值语句（可能被重构，需人工核对）")
        return 1
    expr = m.group(1).strip()

    print("=" * 68)
    print("解锁门红线校验（flight_control.cpp is_unlocked）")
    print("=" * 68)
    print(f"  表达式：{expr}")
    print()

    problems: list[str] = []
    for name, pattern, why in REQUIREMENTS:
        ok = re.search(pattern, expr) is not None
        print(f"  {'OK  ' if ok else '★缺 '} {name}")
        if not ok:
            problems.append(f"{name} — {why}")

    print()
    if problems:
        print(f"[FAIL] {len(problems)} 项不满足：")
        for p in problems:
            print(f"  - {p}")
        print()
        print("⚠️ 解锁/故障保护属安全关键项。改判据前必须读 AGENTS.md")
        print("   §安全关键修改规则，并说明影响行为与已验证的故障路径。")
        return 1

    print("[PASS] 解锁判据完整：isLinkUp 门控 + CH5 > 1500 + 中值滤波")
    return 0


if __name__ == "__main__":
    sys.exit(main())
