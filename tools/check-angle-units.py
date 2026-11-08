#!/usr/bin/env python3
"""欧拉角单位混用审计。

@file    tools/check-angle-units.py
@module  工具/固件安全
@brief   检测「弧度值被当作度比较/加减」这类隐蔽 bug。
@details A-8 的教训：原始共轴固件写 `|AHRS_Packet.Roll| > 45.0f`，而该字段
         单位是弧度——阈值 45 弧度约 2578 度，永不触发，联锁从未生效。
         这类 bug 代码能跑、编译不警告、运行不报错，靠读逻辑读不出来。
         本脚本按「维度」而不是「逻辑」去查：找出所有把欧拉角字段与
         裸数字比较/相加的位置，逐个要求显式换算或白名单登记。
@theory  角度量有两套量纲（rad / deg），C++ 不带单位类型检查，混用无法在
         编译期暴露。唯一可靠的防线是静态扫描 + 人工核对每一处。
@frame   MOD-002 §0：欧拉角字段 `AHRS_Packet.Roll/Pitch/Yaw` 单位为 **弧度**；
         对外展示（`*_deg` 参数、MAVLink 以外的自定义协议）需显式换算。
@units   rad（内部字段） / deg（展示与阈值）
@note    白名单里每一条都必须写明「为什么这里不换算」，否则视为未审计。
@see     docs/07-验证与确认/VER-004-共轴机型原始固件对照-控制与安全.md A-8/A-10
@test    造一条 `if (AHRS_Packet.Roll > 45.0f)` 应报 FAIL；删除后 PASS
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FW = ROOT / "TandemVec_FCS"
SRC = FW / "src"

# 欧拉角字段（弧度）
FIELDS = ("AHRS_Packet.Roll", "AHRS_Packet.Pitch", "AHRS_Packet.Yaw",
          "AHRS_Packet.Heading")

# 已审计正确的位置：file:line → 为什么这里不换算
AUDITED = {
    "src/communication.cpp": "bbRoll/bbPitch 返回弧度，注册表带 RAD_TO_DEG 换算",
    "src/can_bus.cpp": "sendFloatPair 传原始弧度，与 CAN 协议侧约定一致",
}

# 出现这些标记的行视为已显式处理单位
CONVERTED = re.compile(
    r"RAD_TO_DEG|DEG_TO_RAD|\*\s*57\.29|\*\s*180\.0|\*\s*M_PI|/\s*M_PI"
    r"|_deg\b|_rad\b|rad_to_deg|deg_to_rad", re.IGNORECASE
)

# 与裸数字比较/运算
NUMERIC_CMP = re.compile(r"[<>]=?\s*[-+]?\d|[-+*/]\s*\d+\.?\d*")


def main() -> int:
    print("=" * 70)
    print("欧拉角单位混用审计（A-8 教训：弧度当度用）")
    print("=" * 70)

    problems: list[str] = []
    checked = 0
    skipped_files: list[str] = []

    for path in sorted(SRC.rglob("*.cpp")) + sorted(SRC.rglob("*.h")):
        rel = path.relative_to(FW).as_posix()
        try:
            lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError:
            continue
        file_skipped = False
        for lineno, line in enumerate(lines, 1):
            if not any(f in line for f in FIELDS):
                continue
            stripped = line.strip()
            if stripped.startswith("//") or stripped.startswith("*"):
                continue
            checked += 1
            if CONVERTED.search(line):
                continue
            # 直接赋值/传参给别的模块，未必是运算；只有出现数值比较才判高风险
            if not NUMERIC_CMP.search(line):
                continue
            if rel in AUDITED:
                file_skipped = True
                continue
            problems.append(f"{rel}:{lineno}: {stripped[:90]}")
        if file_skipped:
            skipped_files.append(rel)

    print(f"含欧拉角字段的代码行：{checked} 行")
    print(f"已审计正确用法（白名单）：{len(AUDITED)} 个文件")
    for f, why in AUDITED.items():
        print(f"  [已审计] {f}: {why}")
    print()

    if problems:
        for p in problems:
            print(f"  ★ {p}")
        print()
        print(f"[FAIL] {len(problems)} 处把弧度字段与裸数字比较，未显式换算")
        print("处理：改成 R_to_D(field)，或在 AUDITED 里登记为什么不用换算。")
        return 1

    print("[PASS] 没有发现「弧度当度用」的高风险比较")
    return 0


if __name__ == "__main__":
    sys.exit(main())
