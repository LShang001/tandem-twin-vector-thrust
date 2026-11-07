#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    check-codeblock-paths.py
@module  工具/文档代码块路径有效性
@brief   检查 Markdown 代码块里引用的项目内文件路径是否还存在
@details 输入：全仓 Markdown；处理：提取 ``` 围栏块内的疑似文件路径，
         逐个验证（相对文档所在目录、项目根、以及若干常见基准）；
         输出：找不到的路径清单 + 退出码。
@theory  文档里最危险的不是「没写」，而是「给了跑不通的方法」——
         读者照着做，失败后会怀疑自己而不是怀疑文档。
         2026-11-07 实例：固件 README 给出两条「可复核命令」，指向的
         test/ekf_host_regression.cpp 早已删除；check-links.py 抓不到，
         因为它只管 [text](path) 形式，代码块里的路径是它的盲区。
@ref     AGENTS.md §约定（文档构建与检查）
@frame   N/A（只做路径存在性）
@units   N/A
@note    ★ 误报抑制：跳过含变量标记（$ % < > ~ *）、绝对路径（/ 开头、
          盘符开头、\\ 开头）、URL、以及带通配符的写法。
           只验证「看起来像项目内相对路径」的串。
@see     tools/check-links.py、tools/verify-all.py
@test    本脚本即自检：退出码 0 表示无失效路径
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SKIP_PARTS = {".git", "node_modules", "vendor", "lib", "tmp", ".pio",
              "simulations"}  # simulations 下是构建产物站点，路径基准不同

# 代码块围栏
FENCE = re.compile(r"^```.*?^```", re.MULTILINE | re.DOTALL)

# 可疑路径：至少一层目录 + 有扩展名，扩展名限定为源码/配置类
# ★ 顺序敏感：长扩展名必须排在它的前缀之前（json 在 js 前、mjs 在 js 前），
#   否则 a.json 会被截成 a.js —— 这曾导致一次误报
#   （docs/00-*GOV-002 的 models/aircraft-model.json 被报成 .js）。
PATH_RE = re.compile(
    r"(?<![\w/.-])((?:[A-Za-z0-9_.\u4e00-\u9fff-]+/)+[A-Za-z0-9_.\u4e00-\u9fff-]+"
    r"\.(?:cpp|hpp|h|py|mjs|json|js|ini|md|sh|bat|ps1|txt|yaml|yml))"
)

# 只检查这些明确的项目内目录前缀（减少误报：省略号、外部技能路径、
# 论文目录下的示意路径等都不是「项目内真实文件」）
KNOWN_PREFIX = ("test_host/", "src/", "include/", "tools/", "docs/",
                "models/", "simulations/vector-thrust-lab/src/",
                "simulations/vector-thrust-lab/tests/",
                "TandemVec_FCS/src/", "TandemVec_FCS/include/",
                "TandemVec_FCS/test_host/", "TandemVec_FCS/tools/")

# 含这些字符的跳过（变量、通配、模板）
BAD_CHARS = set("$%<>~*{}[]|\\")

# 绝对/系统路径前缀
ABS_PREFIX = ("/", "C:", "D:", "E:", "~/", "http")


def iter_markdown():
    for p in ROOT.rglob("*.md"):
        parts = set(p.relative_to(ROOT).parts)
        if parts & SKIP_PARTS:
            continue
        if any(part.startswith(".") for part in parts):
            continue
        yield p


def candidates(doc: Path):
    """产出该文档代码块里的候选路径。"""
    try:
        text = doc.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return
    for m in FENCE.finditer(text):
        block = m.group(0)
        for pm in PATH_RE.finditer(block):
            raw = pm.group(1).strip()
            if any(c in raw for c in BAD_CHARS):
                continue
            if raw.startswith(ABS_PREFIX):
                continue
            if "/" not in raw:
                continue
            # 省略号式的缩略路径（simulations/.../x）不是真实路径
            if "..." in raw:
                continue
            # 只认明确的项目内目录前缀
            if not raw.startswith(KNOWN_PREFIX):
                continue
            yield m.start(), raw


def resolves(doc: Path, rel: str) -> bool:
    """在几个常见基准下判断路径是否存在。

    基准不限于文档所在目录——技术文档常写「相对于某个子项目根」的路径
    （例：docs/07-*/VER-001 里写 tests/xxx.mjs，实际相对
    simulations/vector-thrust-lab/）。所以把文档的所有祖先目录都试一遍，
    再加上项目根下几个常见子项目根，宁可多试不可漏判。
    """
    bases = [doc.parent, *doc.parents]
    bases += [ROOT, ROOT / "TandemVec_FCS", ROOT / "tools",
              ROOT / "simulations" / "vector-thrust-lab",
              ROOT / "TandemVec_FCS" / "GCS"]
    seen: set[Path] = set()
    for b in bases:
        if b in seen:
            continue
        seen.add(b)
        try:
            if (b / rel).exists():
                return True
        except OSError:
            continue
    return False


def main() -> int:
    print("=" * 72)
    print("文档代码块路径有效性")
    print("=" * 72)

    bad: list[str] = []
    total = 0
    for doc in iter_markdown():
        for _pos, rel in candidates(doc):
            total += 1
            if not resolves(doc, rel):
                bad.append(f"{doc.relative_to(ROOT)} -> {rel}")

    print(f"扫描 {total} 处代码块内路径引用")

    if bad:
        # 去重后输出，按文档分组
        seen: set[str] = set()
        uniq = [x for x in bad if not (x in seen or seen.add(x))]
        print(f"[FAIL] {len(uniq)} 处找不到：")
        for b in uniq[:40]:
            print(f"  - {b}")
        if len(uniq) > 40:
            print(f"  ... 另有 {len(uniq) - 40} 处")
        return 1

    print("[PASS] 代码块内引用的项目路径全部存在")
    return 0


if __name__ == "__main__":
    sys.exit(main())
