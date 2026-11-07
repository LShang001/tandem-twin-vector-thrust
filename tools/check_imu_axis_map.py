#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    check_imu_axis_map.py
@module  工具/IMU 轴映射红线校验
@brief   锁定 sensor_imu.cpp 里的传感器系→机体系轴映射，防止被误改
@details 输入：src/sensor_imu.cpp；处理：提取 axBody/ayBody/azBody 三行映射，
         与规范值（bX=-sZ、bY=+sX、bZ=-sY）比对，并验证手性 bX×bY=bZ；
         输出：比对明细 + 退出码。
@theory  该映射是 2026-08-07 实机轴重映射的结论，与原始 VTVL 实飞存档版一致：
           机体X(前) = -传感器Z(后)
           机体Y(右) = +传感器X(右)
           机体Z(下) = -传感器Y(上)   ← z_b = 推力轴
         手性：bX×bY = (-sZ)×(sX) = -sY = bZ ✓ 右手系。
         ⚠️ 2026-08-07 曾误改为「x_b 竖直」，导致 pitch≈+89° 落进万向锁、
         roll 与 Heading 退化耦合（实测 roll=-110/Heading=-121），
         打杆与目标姿态关系错乱。所以这条必须在 CI 里锁死。
@ref     AGENTS.md「IMU 轴映射」条、src/sensor_imu.cpp 步骤 5
@frame   {S}(传感器系 RUB) → {A}(存档系/机体系 FRD)
@units   N/A（只校验符号与轴对应）
@note    ★ 本检查是文本级：只确认那三行赋值仍是规范形式。它不能替代实机
         「机头朝天静止 acc≈(0,0,-1)g 且 roll=pitch≈0」的判据，两者互补。
@see     tools/check-frame-refs.py、MOD-002 §0
@test    本脚本即自检：退出码 0 表示映射未被改动
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "TandemVec_FCS" / "src" / "sensor_imu.cpp"

# 规范：(轴名, 期望的符号*源轴)。源轴只有 x/y/z 三种可能。
EXPECT = {
    "axBody": ("-", "z"),
    "ayBody": ("+", "x"),
    "azBody": ("-", "y"),
}

ASSIGN = re.compile(r"\bfloat\s+(axBody|ayBody|azBody)\s*=\s*([-+]?)\s*([a-zA-Z_][A-Za-z0-9_]*)\s*;")


def main() -> int:
    if not SRC.exists():
        print(f"[FAIL] 找不到 {SRC}")
        return 2
    text = SRC.read_text(encoding="utf-8", errors="replace")

    found: dict[str, tuple[str, str]] = {}
    for m in ASSIGN.finditer(text):
        name, sign, src = m.group(1), m.group(2) or "+", m.group(3)
        # 源标识符形如 azFilteredRaw / axFilteredRaw，取轴字母
        axis_m = re.search(r"a([xyz])", src)
        if not axis_m:
            continue
        found.setdefault(name, (sign, axis_m.group(1)))

    print("=" * 64)
    print("IMU 轴映射红线校验（sensor_imu.cpp 步骤 5）")
    print("=" * 64)

    problems: list[str] = []
    for name, (want_sign, want_axis) in EXPECT.items():
        if name not in found:
            problems.append(f"{name}: 未找到赋值语句（可能被重构，需人工核对）")
            continue
        got_sign, got_axis = found[name]
        ok = (got_sign == want_sign) and (got_axis == want_axis)
        flag = "OK" if ok else "★ 不符"
        print(f"  {name} = {got_sign}{got_axis}    期望 {want_sign}{want_axis}   [{flag}]")
        if not ok:
            problems.append(f"{name}: 实际 {got_sign}{got_axis}，期望 {want_sign}{want_axis}")

    # 手性检查：把三轴映射写成置换矩阵，验 det=+1 且 bX×bY=bZ
    if not problems:
        vec = {n: (int(s + "1"), a) for n, (s, a) in found.items()}
        def to_xyz(v):
            sign, axis = v
            out = [0, 0, 0]
            out["xyz".index(axis)] = sign
            return out
        bx, by, bz = to_xyz(vec["axBody"]), to_xyz(vec["ayBody"]), to_xyz(vec["azBody"])
        # 叉积 bX × bY
        cross = [
            bx[1] * by[2] - bx[2] * by[1],
            bx[2] * by[0] - bx[0] * by[2],
            bx[0] * by[1] - bx[1] * by[0],
        ]
        if cross != bz:
            problems.append(f"手性不符：bX×bY={cross}，bZ={bz}（应为右手系）")
        else:
            print(f"  手性：bX×bY = {cross} = bZ ✓ 右手系")

    print()
    if problems:
        print(f"[FAIL] {len(problems)} 处不符：")
        for p in problems:
            print(f"  - {p}")
        print()
        print("⚠️ 改轴映射前必须读 AGENTS.md「IMU 轴映射」与 2026-08-07 的踩坑记录：")
        print("   曾误改为「x_b 竖直」导致 pitch≈89° 进万向锁、roll 与 Heading 耦合。")
        return 1

    print("[PASS] 轴映射与规范一致（bX=-sZ / bY=+sX / bZ=-sY，右手系）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
