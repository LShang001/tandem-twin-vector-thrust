#!/usr/bin/env python3
"""全站链接检查：Markdown 相对链接 + 仿真 HTML 站本地链接，零断链为通过。

用法（仓库根目录）：
    py -3.12 tools/check-links.py

规则：
- 跳过 http(s):、mailto:、data:、javascript: 与纯锚点（#...）链接；
- Markdown 链接先按相对当前文件解析，再按相对仓库根解析（文档中两种约定并存）；
- HTML href/src 仅按相对当前文件解析；
- 仅检查目标存在性，不校验锚点。
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SITE = ROOT / "simulations" / "vector-thrust-lab"

MD_LINK = re.compile(r'\[[^\]]*\]\(([^)\s]+)(?:\s+"[^"]*")?\)')
HTML_LINK = re.compile(r'(?:href|src)="([^"]+)"')
SKIP = ("http://", "https://", "mailto:", "data:", "javascript:")


def is_skip(target: str) -> bool:
    return target.startswith(SKIP) or target.startswith("#")


def check_md(path: Path, broken: list[str]) -> int:
    n = 0
    for m in MD_LINK.finditer(path.read_text(encoding="utf-8")):
        target = m.group(1).split("#", 1)[0]
        if not target or is_skip(m.group(1)):
            continue
        n += 1
        if not (path.parent / target).exists() and not (ROOT / target).exists():
            broken.append(f"{path.relative_to(ROOT)} -> {m.group(1)}")
    return n


def check_html(path: Path, broken: list[str]) -> int:
    n = 0
    for m in HTML_LINK.finditer(path.read_text(encoding="utf-8")):
        target = m.group(1).split("#", 1)[0]
        if not target or is_skip(m.group(1)):
            continue
        n += 1
        if not (path.parent / target).exists():
            broken.append(f"{path.relative_to(ROOT)} -> {m.group(1)}")
    return n


def main() -> int:
    broken: list[str] = []
    n_md = n_html = 0
    for md in sorted(ROOT.rglob("*.md")):
        if any(part in {"node_modules", ".git", ".reasonix", "vendor"} for part in md.parts):
            continue
        n_md += check_md(md, broken)
    for html in sorted(SITE.rglob("*.html")):
        if "vendor" in html.parts:
            continue
        n_html += check_html(html, broken)

    if broken:
        print(f"发现 {len(broken)} 处断链：")
        for b in broken:
            print(f"  {b}")
        return 1
    print(f"链接检查通过：Markdown {n_md} 处、HTML {n_html} 处，零断链。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
