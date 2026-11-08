#!/usr/bin/env python3
"""文档站点收录审计。

@file    tools/check-doc-site-coverage.py
@module  工具/文档
@brief   检查 docs/ 下每个编号文档是否被 build-docs.py 收录进站点；未收录的
         必须在 SKIP 白名单里写明原因。
@details 2026-11-08 发现：build-docs.py 的文档清单是**硬编码**的，
         VER-006~VER-012 写了很久却一直没进站点——文档在 git 里、在文件系统里，
         但读者打开网站根本看不到。构建输出「50 页」这个数字也掩盖了它。
         本脚本把「漏收」变成会报错的事。
@theory  文档的价值链是「写 → 审 → 发布」。前两步有守卫（注释门禁、链接检查），
         第三步此前没有。缺了它，写好的东西可能永远停在本地。
@frame   N/A
@units   N/A
@note    白名单里每一条都必须写明「为什么不进站点」，否则视为未审计。
@see     docs/07-验证与确认/VER-009-SIL验收脚本判据表-2026-11-07.md
@test    造一个 docs/ 下未被收录的 .md，应报 FAIL；加进白名单后 PASS
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DOCS = ROOT / "docs"
BUILD = ROOT / "tools" / "build-docs.py"

# 有意不进站点的文档：path（相对 ROOT）→ 原因
SKIP = {
    "docs/memory/": "攻坚记录，面向 agent 而非站点读者",
    "docs/registers/": "登记册/踩坑记录，体量大且非叙事性",
    "docs/08-参考资料/": "参考资料全文（教材 MinerU 产物），单独索引",
    "docs/00-项目治理/": "治理规范，面向协作者而非产品文档读者",
    "docs/reference/": "第三方 SDK/协议文档原文（vendor），非本项目产出",
    "docs/PROJECT-MAP.md": "仓库结构速查表，面向协作者",
    "docs/01-方案设计/CFG-001-py-web-lab统一架构设计.md":
        "py-web-lab 实验台架构说明，面向开发环境搭建，非产品文档",
}


# 该进站点但还没进的文档：path → 计划。这类只警告不判失败，
# 但**必须真的逐步清空**——它是待办清单，不是免死金牌。
PENDING = {
    "docs/04-数学建模/代码理论核查/":
        "代码理论核查深读 24 份，体量大；站点只收摘要索引，全文留在仓库",
}


def main() -> int:
    print("=" * 70)
    print("文档站点收录审计（build-docs.py 的清单是否覆盖 docs/）")
    print("=" * 70)

    # ① 编号唯一性：同一前缀+序号不能出现两份文档
    #   2026-11-08 发现两个 VER-007（场景矩阵 / 摄动测试），站点只收了前一个，
    #   而 VER-009 的引用指向后一个——读者点进去看到的是另一份文档。
    from collections import defaultdict
    import re as _re

    by_id: dict[str, list[str]] = defaultdict(list)
    for md in sorted(DOCS.rglob("*.md")):
        rel = md.relative_to(ROOT).as_posix()
        if rel.startswith("docs/08-参考资料/") or rel.startswith("docs/reference/"):
            continue
        m = _re.match(r"([A-Z]+-\d{3})", md.name)
        if m:
            by_id[m.group(1)].append(rel)

    dup = {k: v for k, v in by_id.items() if len(v) > 1}
    if dup:
        print("[FAIL] 文档编号重复：")
        for k, v in sorted(dup.items()):
            print(f"  ★ {k}:")
            for f in v:
                print(f"      {f}")
        print("处理：给其中一份换新编号，并更新所有引用。")
        return 1
    print(f"编号唯一性：{len(by_id)} 个编号，无重复")
    print()

    src = BUILD.read_text(encoding="utf-8", errors="replace")
    included = set(re.findall(r'"docs/([^"]+\.md)"', src))

    missing: list[str] = []
    pending: list[str] = []
    skipped = 0
    total = 0
    for md in sorted(DOCS.rglob("*.md")):
        rel = md.relative_to(ROOT).as_posix()
        total += 1
        if rel.startswith("docs/"):
            short = rel[len("docs/"):]
        else:
            short = rel
        if short in included:
            continue
        if any(rel.startswith(p) for p in SKIP):
            skipped += 1
            continue
        if any(rel.startswith(p) for p in PENDING):
            pending.append(rel)
            continue
        missing.append(rel)

    print(f"docs/ 下 .md 共 {total} 份")
    print(f"  已收录进站点   {len(included)}")
    print(f"  白名单跳过     {skipped}（{len(SKIP)} 条规则）")
    print(f"  待收录（PENDING）{len(pending)}")
    print(f"  **漏收**       {len(missing)}")
    print()

    for p, why in SKIP.items():
        print(f"  [跳过] {p}: {why}")
    for p, why in PENDING.items():
        print(f"  [待收录] {p}: {why}（{sum(1 for x in pending if x.startswith(p))} 份）")
    print()

    if missing:
        for m in missing:
            print(f"  ★ {m} 未收录且不在白名单")
        print()
        print(f"[FAIL] {len(missing)} 份文档写了但读者看不到")
        print("处理：加进 build-docs.py 的清单，或在 SKIP 里登记为什么不进站点。")
        return 1

    print("[PASS] docs/ 下的文档要么进了站点，要么白名单里写明了原因")
    return 0


if __name__ == "__main__":
    sys.exit(main())
