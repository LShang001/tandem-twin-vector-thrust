#!/usr/bin/env python3
"""文档可达性守卫：从入口文档出发，检查是否有「写了但没人链」的孤儿文档。

@file     check-doc-reach.py
@module   工具/文档质量门禁
@brief   从 README.md / AGENTS.md / docs/README.md / docs/PROJECT-MAP.md 出发做
          Markdown 链接图的可达性 BFS，报告从入口走不到的项目文档。
@details  输入：`docs/**/*.md` 全部文档 + 4 个入口文件；
          处理：解析 `[text](path)` 形式的相对链接（跳过 http/mailto/纯锚点），
                按「相对当前文件」解析后 BFS；
          输出：孤儿文档清单，发现即退出码 1。
          豁免集合（豁免理由必须写在代码里，不能靠人工忽略）：
            ① MinerU 提取产物 —— `book.md` / `paper.md` / `full.md`，或位于
               `part*/` `chunk*/` `toc_probe/` `chapters/`；这些是转换中间产物，
               引用未入库的 images/，且入口走 `book.md` 或 `chapters/` 即可；
            ② `docs/reference/` —— 第三方 SDK/协议文档原文（vendor），非本项目产出。
@theory   N/A（工程检查）
@frame    N/A
@units    N/A

为什么需要这个守卫（2026-11-10）：
    `check-links.py` 只能证明「写出来的链接不指向空处」，无法证明「文档能被找到」。
    整理文档时实测：`docs/` 下 191 篇，从入口只能走到 **2 篇**——其余全部是孤儿。
    链接检查当时报「零断链」，因为它们压根没被链接，连断链的机会都没有。
    孤儿文档 = 对新会话等于不存在，正是「每次都要重新学一遍」的根因。

用法（仓库根目录）：
    py -3.12 tools/check-doc-reach.py
    py -3.12 tools/check-doc-reach.py --list   # 连可达的一起列出，用于人工核查

@test    造一个未被任何文档链接的新 md，应报 FAIL；在 docs/README.md 加一行链接后 PASS。
"""
from __future__ import annotations

import re
import sys
from collections import deque
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DOCS = ROOT / "docs"

# 入口文件：新会话实际会读的那几个。加新入口必须同时加到这里。
ENTRIES = [
    "README.md",
    "AGENTS.md",
    "docs/README.md",
    "docs/PROJECT-MAP.md",
]

# MinerU 提取产物的目录名（见文件头豁免①）
MINERU_DIRS = {"part0", "part1", "part2", "part3",
               "chunkA", "chunkB", "chunkC", "chunkD",
               "toc_probe", "chapters"}
MINERU_NAMES = {"book.md", "paper.md", "full.md"}

MD_LINK = re.compile(r"\[[^\]]*\]\(([^)\s]+)(?:\s+\"[^\"]*\")?\)")
SKIP_PREFIX = ("http://", "https://", "mailto:", "data:", "javascript:", "doc:", "#")


def is_exempt(path: Path) -> bool:
    """判断是否属于豁免集合（MinerU 提取产物 / 第三方原文）。"""
    rel = path.relative_to(ROOT)
    if "reference" in rel.parts and rel.parts[0] == "docs":
        return True                                    # 豁免②
    if path.name in MINERU_NAMES:                      # 豁免①
        return True
    if MINERU_DIRS & set(rel.parts):
        return True
    return False


def out_links(path: Path) -> list[Path]:
    """返回该 Markdown 指向的、仓库内存在的 .md 文件。"""
    try:
        text = path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError):
        return []
    out: list[Path] = []
    for m in MD_LINK.finditer(text):
        raw = m.group(1)
        if raw.startswith(SKIP_PREFIX):
            continue
        target = raw.split("#", 1)[0].strip()
        if target.startswith("<") and target.endswith(">"):
            target = target[1:-1].strip()
        if not target:
            continue
        for base in (path.parent, ROOT):
            cand = (base / target).resolve()
            try:
                cand.relative_to(ROOT)
            except ValueError:
                continue
            if cand.is_file() and cand.suffix == ".md":
                out.append(cand)
                break
    return out


def main() -> int:
    print("=" * 70)
    print("文档可达性守卫（从入口能否走到每一篇文档）")
    print("=" * 70)

    missing_entries = [e for e in ENTRIES if not (ROOT / e).exists()]
    if missing_entries:
        print(f"[FAIL] 入口文件缺失：{missing_entries}")
        return 1

    all_md = sorted(p.resolve() for p in DOCS.rglob("*.md"))
    required = [p for p in all_md if not is_exempt(p)]
    exempt = [p for p in all_md if is_exempt(p)]

    seen: set[Path] = set()
    queue: deque[Path] = deque()
    for e in ENTRIES:
        p = (ROOT / e).resolve()
        if p not in seen:
            seen.add(p)
            queue.append(p)
    while queue:
        cur = queue.popleft()
        for nxt in out_links(cur):
            if nxt not in seen:
                seen.add(nxt)
                queue.append(nxt)

    orphans = [p for p in required if p not in seen]

    print(f"  入口           {len(ENTRIES)} 个：{'、'.join(ENTRIES)}")
    print(f"  docs/ 文档     {len(all_md)} 篇")
    print(f"  需可达（去豁免）{len(required)} 篇")
    print(f"  豁免           {len(exempt)} 篇（MinerU 提取产物 / docs/reference 第三方原文）")
    print(f"  可达           {len(required) - len(orphans)}/{len(required)}")
    print()

    if "--list" in sys.argv:
        for p in required:
            mark = "  OK " if p in seen else "  ★  "
            print(f"{mark}{p.relative_to(ROOT)}")
        print()

    if orphans:
        print(f"[FAIL] {len(orphans)} 篇文档从入口走不到（孤儿文档 = 对新会话等于不存在）：")
        by_dir: dict[str, list[str]] = {}
        for p in orphans:
            by_dir.setdefault(str(p.parent.relative_to(ROOT)), []).append(p.name)
        for d in sorted(by_dir):
            print(f"  [{d}] {len(by_dir[d])} 篇")
            for n in sorted(by_dir[d]):
                print(f"      {n}")
        print()
        print("  修法：在 docs/README.md §2 对应小节（或该目录的 README）加一行链接 + 一句话说明。")
        print("  ★ 不要往豁免集合里塞目录来绕开——豁免只留给「不是本项目产出」的东西。")
        return 1

    print("[PASS] 所有项目文档都能从入口文档走到（无孤儿）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
