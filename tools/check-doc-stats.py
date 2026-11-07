#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    check-doc-stats.py
@module  工具/文档统计数字一致性（单一事实源守卫）
@brief   把「散落在多份文档里的统计数字」与实测值比对，不一致就报错
@details 输入：各事实的实测方式 + 文档里的数字写法（正则）；
         处理：先算实测值，再扫全仓 Markdown 抓同义表述里的数字，逐一比对；
         输出：一致/不一致清单 + 退出码。
@theory  单一事实源原则：一个数字如果有权威来源（如 `node --test` 的实际输出），
         文档里任何复述都只是缓存。缓存要么被同步，要么被检查——否则必然腐烂。
         本项目已有多次教训：测试项数 91→92 散在 8 份文档（2026-11-07）、
         参数数量 133/164/169 散在多份文档（2026-11-07）。手工同步靠不住，
         所以把「比对」这一步自动化。
@ref     AGENTS.md §约定、docs/registers/需求注册表.md
@frame   N/A（只做文本比对）
@units   N/A
@note    ★ 本工具不修文档，只报不一致。修法有两个方向：
          ① 文档改成实际值（当前做法，需要人工）；
          ② 文档改写成「以实测为准」不写死数字（根治，但改动面大）。
          本工具的存在让方向 ① 至少不会漏。
@see     tools/verify-all.py（已并入）、tools/check-links.py
@test    本脚本即自检：退出码 0 表示所有已知统计数字与实测一致
"""
from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SKIP_PARTS = {".git", "node_modules", "vendor", "lib", "tmp", ".pio"}


def count_node_tests() -> int:
    """实测 Node 测试项数（跑一次 node --test，取其 tests 汇总行）。"""
    tests_dir = ROOT / "simulations" / "vector-thrust-lab" / "tests"
    files = sorted(str(p) for p in tests_dir.glob("*.test.mjs"))
    if not files:
        return -1
    proc = subprocess.run(["node", "--test", *files], capture_output=True,
                          text=True, encoding="utf-8", errors="replace")
    m = re.search(r"^ℹ tests (\d+)$", proc.stdout, re.MULTILINE)
    return int(m.group(1)) if m else -1


def count_host_segments() -> int:
    """实测宿主回归段数（run_all.sh 里 run_one 调用数）。"""
    sh = ROOT / "TandemVec_FCS" / "test_host" / "run_all.sh"
    if not sh.exists():
        return -1
    text = sh.read_text(encoding="utf-8", errors="replace")
    return len(re.findall(r"^run_one ", text, re.MULTILINE))


# 每项：名称 / 实测函数 / 文档中的写法（捕获组 1 是数字）
FACTS = [
    {
        "name": "Node 测试项数",
        "actual": count_node_tests,
        "patterns": [
            r"(\d+)\s*项（\d{4}-\d{2}-\d{2}\s*`node --test`\s*实测）",
            r"(\d+)\s*项（\d{4}-\d{2}-\d{2}\s*实测）",
            r"执行\s*(\d+)\s*项测试",
            r"<br/>(\d+)\s*项",
            r"(\d+)\s*项测试与\s*4\s*项回归基线",
            r"(\d+)\s*项测试、4\s*条回归基线",
            r"实测\s*(\d+)\s*项全绿",
            r"\.test\.mjs`\s*文件（(\d+)\s*项）",
            r"(\d+)\s*项测试性质与容差",
        ],
    },
    {
        "name": "宿主回归段数",
        "actual": count_host_segments,
        "patterns": [
            r"宿主回归全量（(\d+)\+?\s*段",
        ],
    },
]


def iter_markdown():
    for p in ROOT.rglob("*.md"):
        if any(part in SKIP_PARTS or part.startswith(".") for part in p.parts):
            continue
        yield p


def main() -> int:
    print("=" * 72)
    print("文档统计数字一致性（单一事实源守卫）")
    print("=" * 72)

    problems: list[str] = []
    for fact in FACTS:
        actual = fact["actual"]()
        if actual < 0:
            print(f"[SKIP] {fact['name']}：无法实测（工具缺失或文件不存在）")
            continue
        hits = 0
        for md in iter_markdown():
            try:
                text = md.read_text(encoding="utf-8", errors="replace")
            except OSError:
                continue
            for pat in fact["patterns"]:
                for m in re.finditer(pat, text):
                    hits += 1
                    got = int(m.group(1))
                    if got != actual:
                        rel = md.relative_to(ROOT)
                        problems.append(
                            f"{fact['name']}：{rel} 写 {got}，实测 {actual}"
                        )
        print(f"[实测] {fact['name']} = {actual}（文档命中 {hits} 处）")

    print()
    if problems:
        print(f"[FAIL] {len(problems)} 处统计数字与实际不符：")
        for p in problems:
            print(f"  - {p}")
        print()
        print("修法：把文档数字改成实测值，并同步实测日期。")
        return 1

    print("[PASS] 所有已知统计数字与实测一致")
    return 0


if __name__ == "__main__":
    sys.exit(main())
