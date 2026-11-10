#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    check-memory-index.py
@brief   docs/memory/ 沉淀索引完整性守卫（AGENTS.md 原则 11「动手前先查沉淀」的可执行面）
@module  项目治理 / 开发流程
@details 为什么需要这个守卫：2026-11-09 做 ICM-45686 采样率时，自创「FIFO 批量读」
         方案并耗掉大半天，而 docs/memory/2026-11-04-IMU-2kHz攻关.md §4「无效路线
         （不要再试）」第一条就是它。根因不是文档没写，而是文档没写「什么时候该读」，
         且索引藏在文件末尾——下次自然想不起来查。
         所以把两件事变成硬约束：
           ① 每条 memory 文档首屏必须有「> **触发条件**：…」，写清何时该读它；
           ② 每个文件名必须登记进 AGENTS.md 的「★ 动手前必查的沉淀」索引表。
         任一条不满足即退出码 1，verify-all 会拦下来。

@theory  N/A（工程流程守卫，非算法）
@frame   N/A
@units   N/A
@note    反向也查：索引表里列了但磁盘上不存在的文件同样报错（防止改名后索引腐烂）。
@test    T1 触发条件缺失检出；T2 未登记索引检出；T3 索引指向不存在文件检出；T4 干净仓库全绿
@see     AGENTS.md §★ 动手前必查的沉淀、§原则 11；docs/memory/
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MEMORY_DIR = ROOT / "docs" / "memory"
AGENTS = ROOT / "AGENTS.md"

# 索引节标题（AGENTS.md 里那节的锚点）
SECTION_TITLE = "★ 动手前必查的沉淀"
# 首屏窗口：触发条件必须出现在文件开头这么多字符内
HEAD_WINDOW = 700
TRIGGER_MARKERS = ("触发条件", "何时读")


def read_text(path: Path) -> str:
    """按 UTF-8 读文本，容忍 BOM。"""
    return path.read_text(encoding="utf-8-sig")


def collect_memory_files() -> list[Path]:
    """收集 docs/memory/ 下的全部沉淀文档。"""
    if not MEMORY_DIR.is_dir():
        return []
    return sorted(p for p in MEMORY_DIR.glob("*.md") if p.is_file())


def extract_indexed_names(agents_text: str) -> set[str]:
    """
    从 AGENTS.md 的沉淀索引节里抽出被索引的文件名。

    只扫该节内的表格行，避免把正文里顺手提到的文件名也算作「已登记」
    —— 那样索引腐烂时守卫会漏报。
    """
    lines = agents_text.splitlines()
    start = None
    for i, line in enumerate(lines):
        if line.startswith("## ") and SECTION_TITLE in line:
            start = i + 1
            break
    if start is None:
        return set()

    names: set[str] = set()
    for line in lines[start:]:
        if line.startswith("## "):
            break  # 下一节，结束
        # 只认表格行：正文里顺带提到的文件名（如「踩坑记录-AGENTS.md 同理」）
        # 不算已登记，否则索引腐烂时守卫会漏报。
        if not line.lstrip().startswith("|"):
            continue
        for m in re.finditer(r"`([^`]+\.md)`", line):
            names.add(Path(m.group(1)).name)
    return names


def check() -> int:
    """执行全部检查，返回 0 表示全绿。"""
    if not AGENTS.is_file():
        print("[FAIL] 找不到 AGENTS.md", file=sys.stderr)
        return 1

    agents_text = read_text(AGENTS)
    files = collect_memory_files()
    if not files:
        print("[FAIL] docs/memory/ 下没有任何沉淀文档", file=sys.stderr)
        return 1

    indexed = extract_indexed_names(agents_text)
    if not indexed:
        print(
            f'[FAIL] AGENTS.md 里找不到「## {SECTION_TITLE}」索引表，或表内没有任何 .md 条目',
            file=sys.stderr,
        )
        return 1

    errors: list[str] = []

    # T1 + T2：逐条查触发条件与索引登记
    for f in files:
        head = read_text(f)[:HEAD_WINDOW]
        if not any(m in head for m in TRIGGER_MARKERS):
            errors.append(
                f"{f.name}：首屏缺「> **触发条件**：…」（{HEAD_WINDOW} 字符内未找到 "
                f"{'/'.join(TRIGGER_MARKERS)}）——读者不知道何时该读它"
            )
        if f.name not in indexed:
            errors.append(
                f"{f.name}：未登记进 AGENTS.md「## {SECTION_TITLE}」索引表——"
                f"不登记就等于没写，下次不会有人查"
            )

    # T3：索引指向的文件必须存在
    on_disk = {f.name for f in files}
    for name in sorted(indexed - on_disk):
        errors.append(
            f"AGENTS.md 索引表列出了 `{name}`，但 docs/memory/ 下没有这个文件"
            f"（改名/删除后索引没同步）"
        )

    if errors:
        print(f"[FAIL] 沉淀索引守卫：{len(errors)} 处问题", file=sys.stderr)
        for e in errors:
            print(f"  - {e}", file=sys.stderr)
        print(
            "\n修复：给文档首行加「> **触发条件**：<改什么/查什么时会读它>」，"
            "并把它登记到 AGENTS.md §★ 动手前必查的沉淀 表格。",
            file=sys.stderr,
        )
        return 1

    print(
        f"[PASS] 沉淀索引守卫：{len(files)} 篇文档全部有触发条件且已登记索引"
    )
    return 0


def main() -> int:
    """命令行入口。"""
    return check()


if __name__ == "__main__":
    sys.exit(main())
