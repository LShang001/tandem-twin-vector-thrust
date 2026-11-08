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


def count_fw_params() -> int:
    """实测固件参数总数（ano_params.h 的 ANO_PARAMS_COUNT）。"""
    h = ROOT / "TandemVec_FCS" / "src" / "ano_params.h"
    if not h.exists():
        return -1
    m = re.search(r"#define\s+ANO_PARAMS_COUNT\s+(\d+)",
                  h.read_text(encoding="utf-8", errors="replace"))
    return int(m.group(1)) if m else -1


# 每项：名称 / 实测函数 / 文档中的写法（捕获组 1 是数字）
def count_verify_items() -> int:
    """数 verify-all.py 里快档守卫的项数（不含 --include-sil/--include-host 两项）。"""
    import re as _re
    src = (ROOT / "tools" / "verify-all.py").read_text(encoding="utf-8", errors="replace")
    # 守卫清单是以 (名字, 命令, cwd) 三元组形式列出的
    items = _re.findall(r'^\s{4,8}\(\s*"([^"]+)",\s*\[', src, _re.MULTILINE)
    # 去掉两个慢档。★ 用精确名称而不是 "SIL" 子串——"SIL 覆盖审计"是快档，
    #   之前用子串匹配把它也过滤了，导致算出来少 1 项（2026-11-08 踩过）。
    slow = {"三机型 SIL 跑批", "完整宿主回归"}
    return len([n for n in items if n not in slow])


FACTS = [
    {
        "name": "快档守卫项数",
        "actual": count_verify_items,
        "patterns": [
            r"(\d+)\s*项快档",
            r"(\d+)\s*项全绿再提交",
        ],
    },

    {
        # ★ 2026-11-08：VER-009 §2.16 的「能失败验证 N/M」会随守卫增加漂移。
        #   M 必须等于全量守卫数（快档 + 两个慢档），N 是人工维护的实测计数。
        "name": "能失败验证的分母",
        "actual": lambda: count_verify_items() + 2,
        "patterns": [
            r"能失败验证[^\n]{0,12}(\d+)/(\d+)",
            r"(\d+)/(\d+)\s*做过违规实验",
        ],
        "group": 2,
    },

    {
        # ★ 2026-11-08：档位数字曾漂移四次（AGENTS.md / VER-009 §2.16 / VER-009 L1172 /
        #   VER-012 L14），其中一处写成「21 项快档 + 2 项慢档 = 22 项」——纯算术错误。
        #   这里守「全量 = 快档 + 2」这个恒等关系：任何一处写错都会报。
        "name": "全量守卫项数",
        "actual": lambda: count_verify_items() + 2,
        "patterns": [
            r"快档\s*\+\s*2\s*项慢档\s*=\s*(\d+)\s*项",
            # ★ 非贪婪：{0,8} 贪婪会先把「全量 2」的 2 吃掉，(\d+) 只剩 3
            r"全量[^\n]{0,8}?(\d+)\s*项",
        ],
    },

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
    {
        "name": "固件参数总数",
        "actual": count_fw_params,
        "patterns": [
            # ★ 只匹配明确指「固件参数总数」的写法。早期用宽泛的 `(\d+) 项参数`
            #   会误报：`18 项参数扫描`（sweep 脚本的扫描维度）、
            #   `422 项参数断言`（cparams 测试的断言条数）都不是参数总数。
            r"参数总数以\s*`ANO_PARAMS_COUNT`\s*为准（\d{4}-\d{2}-\d{2}\s*时点\s*(\d+)\s*项",
            r"参数总数以\s*`ANO_PARAMS_COUNT`\s*为准（\s*(\d+)\s*项",
            r"`ano_params\.cpp`\s*(\d+)\s*项注册表",
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
                    # ★ 2026-11-08：支持 group 参数（N/M 这类只想核对分母的写法）
                    try:
                        got = int(m.group(fact.get("group", 1)))
                    except (IndexError, ValueError):
                        continue
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
