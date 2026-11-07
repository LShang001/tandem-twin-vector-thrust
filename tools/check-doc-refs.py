#!/usr/bin/env python3
"""文档交叉引用的编号完整性检查。

@file    tools/check-doc-refs.py
@module  工具/文档一致性
@brief   检查 Markdown 里「见 CTL-001 §3」这类编号引用是否都指向存在的文档。
@details 输入：docs/ 下全部 .md；处理：提取 `见/参见/详见` 附近的
         「模块前缀-三位数字」形式编号，与 docs/ 下实际文件名中的编号比对；
         输出：悬空引用清单 + 退出码。
@theory  文档编号是这套文档体系的骨架——断掉的编号引用等于把读者引向空地。
         与 check-links.py 互补：后者查 [text](path) 形式的文件链接，
         本脚本查正文里的「编号」引用（不一定是链接）。
@frame   N/A（只做编号存在性）
@units   N/A
@note    ★ 只认文档类前缀（CTL/VER/GOV/MOD/PROP/REQ/CFG/ASM/DEC/A）
         与 registers/ 下的编号；REF-xxx 属参考资料注册表、另由
         参考资料注册表维护，不在本脚本范围。
@see     tools/check-links.py、tools/check-doc-stats.py
@test    本脚本即自检：退出码 0 表示无悬空编号引用
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DOCS = ROOT / "docs"

# 只在「引用语境」里抓编号，避免把正文里讨论的编号误当引用
REF_CONTEXT = re.compile(r"(见|参见|详见|见\s*§|依据|按)\s*([^\n]{0,40})")
CODE = re.compile(r"\b([A-Z]{2,4}-[0-9]{3})\b")

DOC_PREFIXES = ("CTL", "VER", "GOV", "MOD", "PROP", "REQ", "CFG",
                "ASM", "DEC", "ANA", "FW")


def collect_existing() -> set[str]:
    """docs/ 下实际存在的编号（来自文件名 + registers 里的表格）。"""
    ex: set[str] = set()
    for p in DOCS.rglob("*.md"):
        m = re.match(r"([A-Z]{2,4}-[0-9]{3})-", p.name)
        if m:
            ex.add(m.group(1))
    # registers 目录里的编号（假设日志、决策记录等常以表格出现，读标题行）
    reg = DOCS / "registers"
    if reg.is_dir():
        for p in reg.rglob("*.md"):
            text = p.read_text(encoding="utf-8", errors="replace")
            for m in re.finditer(r"^\s*\|?\s*\**([A-Z]{2,4}-[0-9]{3})\**", text, re.MULTILINE):
                ex.add(m.group(1))
    return ex


def main() -> int:
    existing = collect_existing()
    print("=" * 66)
    print("文档交叉引用编号完整性")
    print("=" * 66)
    print(f"已存在编号：{len(existing)} 个")

    dangling: list[str] = []
    checked = 0
    for p in DOCS.rglob("*.md"):
        text = p.read_text(encoding="utf-8", errors="replace")
        for ctx in REF_CONTEXT.finditer(text):
            for m in CODE.finditer(ctx.group(2)):
                code = m.group(1)
                if code.split("-")[0] not in DOC_PREFIXES:
                    continue
                checked += 1
                if code not in existing:
                    dangling.append(f"{p.relative_to(ROOT)} → {code}")

    print(f"检查了 {checked} 处编号引用")

    if dangling:
        uniq = sorted(set(dangling))
        print(f"[FAIL] {len(uniq)} 处悬空引用：")
        for d in uniq[:30]:
            print(f"  - {d}")
        if len(uniq) > 30:
            print(f"  ... 另有 {len(uniq) - 30} 处")
        return 1

    print("[PASS] 所有文档类编号引用都指向存在的文档")
    return 0


if __name__ == "__main__":
    sys.exit(main())
