#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    check-frame-refs.py
@module  工具/坐标系规范落地检查（@frame 声明审计）
@brief   扫描全仓 `@frame` 注释，检查是否明确标注了所属坐标系
@details 输入：源码目录（固件 include/src/test_host、仿真 core）；处理：抽取每个
         文件的 `@frame` 行，按 MOD-002 §0 的坐标系词汇判定是否已标注；
         输出：合规/待补清单 + 汇总 + 退出码。
@theory  坐标系约束（MOD-002 §0.5 第 1 条）：代码或文档里不得只写 x/y/z 而不标注
         属于哪套系。本脚本把这条规则变成可执行检查，而不是靠 review 记忆。
@ref     docs/04-数学建模/MOD-002-坐标系与符号约定.md §0（唯一正本）、§0.5（禁止事项）
@frame   检查工具本身不解算任何量；它检查的是被检文件的 @frame 声明
@units   N/A
@note   判据分两档：
           - 合规：出现「存档系 / 模型系 / 机体系 / NED / FRD / 经纬高」，
             或出现轴名符号（★ 这是**有意宽松**：文件里出现 z_b/{A} 这类
                    轴名即说明作者明确到轴级别，可替代 @frame。
                    代价是抽掉 @frame 但正文仍有轴名时不会报警——
                    2026-11-08 实测：删 FrameConventions.h 的 @frame 后
                    检查器仍 PASS，因为正文遍布 z_b。要严格判定需另设开关）
         或显式写 `N/A`（表示该文件不涉及坐标系），或出现轴名符号
             （x' / z_b / x_A / Mx' / R13 / R33 等）——轴名即隐含标注。
           - 待补：@frame 为空、或只有泛泛描述（如「同控制器」）而没有坐标系词。
         ★ 本脚本只查"有没有标注"，不查"标注对不对"。标注正确性由
         tools/verify_frame_map.py（置换推导）与 test_host/test_frame_conventions.cpp
         （几何性质）保证。
@see     TandemVec_FCS/include/FrameConventions.h、TandemVec_FCS/tools/verify_frame_map.py
@test    本脚本即自检：退出码 0 表示全部 @frame 已标注坐标系
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

SCAN_DIRS = [
    ROOT / "TandemVec_FCS" / "include",
    ROOT / "TandemVec_FCS" / "src",
    ROOT / "TandemVec_FCS" / "test_host",
    ROOT / "simulations" / "vector-thrust-lab" / "src" / "core",
]
SUFFIXES = {".h", ".hpp", ".c", ".cpp", ".mjs", ".js"}

# 出现任一即视为"已标注坐标系"
FRAME_WORDS = [
    "存档系", "模型系", "机体系", "NED", "FRD", "经纬高", "惯性系",
    "N/A", "n/a", "无坐标系", "标量",
    # 其它已成立的具体坐标系/轴系命名（同样是"说清了在哪套系里"）
    "WGS84", "ECEF", "ENU", "RUB", "FRD", "箭体", "竖直", "剖面",
    "大地坐标", "调用方约定", "原始轴序", "传感器原始",
]
# 轴名符号也算隐含标注（出现即说明作者明确到轴级别）
AXIS_TOKENS = ["x'", "y'", "z'", "x_b", "y_b", "z_b", "x_A", "y_A", "z_A",
               "R13", "R33", "Mx", "My", "Mz"]

FRAME_RE = re.compile(r"@frame\s+(.*)$")


def collect_files() -> list[Path]:
    out: list[Path] = []
    for d in SCAN_DIRS:
        if not d.exists():
            continue
        for p in d.rglob("*"):
            if p.suffix.lower() in SUFFIXES and p.is_file():
                out.append(p)
    return sorted(out)


def main() -> int:
    files = collect_files()
    total = 0
    ok = 0
    missing: list[tuple[Path, str]] = []

    for path in files:
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        # @frame 常常跨行写（正文续在下一行），必须读到下一个 @标签或注释结束为止，
        # 否则会把"只写了一半"的行误判为未标注。
        lines = text.splitlines()
        i = 0
        while i < len(lines):
            m = FRAME_RE.search(lines[i])
            if not m:
                i += 1
                continue
            total += 1
            body = m.group(1)
            j = i + 1
            while j < len(lines):
                nxt = lines[j].strip().lstrip("*").strip()
                if not nxt or nxt.startswith("@") or nxt.startswith("*/"):
                    break
                body += " " + nxt
                j += 1
                if j - i > 6:  # 最多续读 6 行，避免吞掉整段正文
                    break
            body = body.strip()
            has_word = any(w in body for w in FRAME_WORDS)
            has_axis = any(t in body for t in AXIS_TOKENS)
            if body and (has_word or has_axis):
                ok += 1
            else:
                missing.append((path.relative_to(ROOT), body))
            i = j

    print("=" * 70)
    print("坐标系规范落地检查：@frame 声明是否标注所属坐标系")
    print("=" * 70)
    print(f"扫描 {len(files)} 个文件，命中 @frame {total} 处")
    print(f"已标注 {ok} 处 / 待补 {len(missing)} 处")
    print()

    if missing:
        print("待补清单（@frame 未写清坐标系；按 MOD-002 §0.5 第 1 条须补）：")
        for path, body in missing:
            shown = body if body else "(空)"
            print(f"  - {path}: {shown[:70]}")
        print()
        print("[FAIL] 存在未标注坐标系的 @frame")
        return 1

    print("[PASS] 全部 @frame 均已标注坐标系（或有轴名符号 / 明示 N/A）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
