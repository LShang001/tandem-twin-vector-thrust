#!/usr/bin/env python3
"""全站链接检查：Markdown 相对链接 + 仿真 HTML 站本地链接，零断链为通过。

@file     check-links.py
@module   工具/文档质量门禁
@brief    扫描 docs 与仿真站点，报告断链并以退出码表达结果。
@details  输入：仓库内全部 Markdown 与仿真 HTML；处理：解析相对链接并按
          当前文件/仓库根两种约定解析；输出：断链清单，发现即退出码 1。
@theory   N/A（工程检查）
@frame    N/A
@units    N/A

用法（仓库根目录）：
    py -3.12 tools/check-links.py

规则：
- 跳过 http(s):、mailto:、data:、javascript: 与纯锚点（#...）链接；
- Markdown 链接先按相对当前文件解析，再按相对仓库根解析（文档中两种约定并存）；
- HTML href/src 仅按相对当前文件解析；
- 跳过隐藏目录（.dsh-ops/.reasonix 等）与文献库 MinerU 提取稿（books/papers 的
  book.md/paper.md 引用未入库 images/，不属于站点文档）；
- 支持 CommonMark 的 `<path>` 链接目标（去除尖括号后检查）；
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
    """判断链接目标是否为外部协议/纯锚点，这类目标不检查。"""
    return target.startswith(SKIP) or target.startswith("#")


def check_md(path: Path, broken: list[str]) -> int:
    """检查单个 Markdown 文件的相对链接，返回检查数并记录断链。"""
    n = 0
    for m in MD_LINK.finditer(path.read_text(encoding="utf-8")):
        target = m.group(1).split("#", 1)[0]
        # CommonMark 允许 <path> 形式的链接目标，尖括号不属于路径
        if target.startswith("<") and target.endswith(">"):
            target = target[1:-1].strip()
        if not target or is_skip(m.group(1)):
            continue
        n += 1
        if not (path.parent / target).exists() and not (ROOT / target).exists():
            broken.append(f"{path.relative_to(ROOT)} -> {m.group(1)}")
    return n


def check_html(path: Path, broken: list[str]) -> int:
    """检查单个站点 HTML 的 href/src 相对链接，返回检查数并记录断链。"""
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
    """命令行入口：遍历文档与 HTML，零断链返回 0，否则返回 1。"""
    broken: list[str] = []
    n_md = n_html = 0
    for md in sorted(ROOT.rglob("*.md")):
        if any(
            part in {"node_modules", ".git", "vendor"} or part.startswith(".")
            for part in md.parts
        ):
            continue
        # 文献库的 MinerU 提取稿（book.md/paper.md）引用未入库的 images/，不属于站点文档，跳过
        if "08-参考资料" in md.parts and ({"books", "papers"} & set(md.parts)):
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
