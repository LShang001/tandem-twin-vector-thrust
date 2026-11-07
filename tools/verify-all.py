#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    verify-all.py
@module  工具/总守卫（一键确认全仓一致性未退化）
@brief   依次运行所有一致性/规范性守卫，汇总成一张表并给出总退出码
@details 输入：`--quick`（跳过耗时的 SIL 跑批）、`--include-sil`（含三机型 SIL）；
         处理：按序调用各守卫脚本，记录退出码与耗时，任何一项失败即整体失败；
         输出：逐项 PASS/FAIL 表 + 汇总 + 退出码。
@theory  本脚本不重复实现任何检查，只做编排。每项守卫各自负责一类"矛盾"：
           参数层   仿真 JSON ⇄ 固件 Config.h 同一物理量是否同值
           推进层   propulsion.mjs ⇄ computeWrench 六维公式是否等价
           分配层   computeBTrue ⇄ computeEffectMatrix 3×3 是否等价
           坐标系   @frame 是否都标注了所属坐标系（MOD-002 §0.5 第 1 条）
           注释     GOV-004 五段式与引用标注格式
           仿真域   Node 91 项单测
           坐标系实现 test_frame_conventions（det/正交/往返）
@ref     MOD-002 §0（坐标系正本）、AGENTS.md §命令
@frame   N/A（编排器，不解算任何量）
@units   秒（耗时）
@note    ★ 设计原则：**一项失败不中断**，跑完再汇总——否则一次只能看到一个问题。
          ★ 快档约 1–2 分钟（前六项），慢档加三机型 SIL 约 10–20 分钟。
          ★ 本脚本只报"哪一项红了"，具体诊断去跑那一项自己的输出。
@see     tools/check-param-parity.py、tools/check-propulsion-parity.py、
         tools/check-allocation-parity.py、tools/check-frame-refs.py、
         tools/check-code-comments.py、TandemVec_FCS/test_host/fw_sil/run_all_airframes.sh
@test    本脚本即自检：退出码 0 表示全部守卫通过
"""
from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PY = sys.executable


def build_checks(include_sil: bool) -> list[tuple[str, list[str], Path]]:
    # ★ Node 的测试发现要显式给文件列表：在无 shell 的 subprocess 里，
    #   `node --test <目录>` 与 `*.test.mjs` 都不会被展开（2026-11-07 踩过）。
    import glob

    node_tests = sorted(
        glob.glob(str(ROOT / "simulations" / "vector-thrust-lab" / "tests" / "*.test.mjs"))
    )
    checks: list[tuple[str, list[str], Path]] = [
        ("参数层一致性", [PY, "tools/check-param-parity.py"], ROOT),
        ("推进层对拍", [PY, "tools/check-propulsion-parity.py"], ROOT),
        ("分配层对拍", [PY, "tools/check-allocation-parity.py"], ROOT),
        ("坐标系 @frame 标注", [PY, "tools/check-frame-refs.py"], ROOT),
        ("SIL 覆盖审计", [PY, "tools/check-sil-coverage.py"], ROOT),
        ("注释门禁 GOV-004", [PY, "tools/check-code-comments.py", "--check"], ROOT),
        ("文档链接完整性", [PY, "tools/check-links.py"], ROOT),
        ("文档产物同步", [PY, "tools/build-docs.py", "--check"], ROOT),
        ("文档统计数字", [PY, "tools/check-doc-stats.py"], ROOT),
        ("代码块路径", [PY, "tools/check-codeblock-paths.py"], ROOT),
        ("仿真域单测（Node）", ["node", "--test"] + node_tests, ROOT),
        ("坐标系实现自检",
         ["bash", "-c",
          "g++ -std=c++17 -ITandemVec_FCS/include "
          "TandemVec_FCS/test_host/test_frame_conventions.cpp "
          "-o tmp/_frame_check.exe && tmp/_frame_check.exe"], ROOT),
    ]
    if include_sil:
        # cwd 已经是 TandemVec_FCS，路径不能再带前缀（2026-11-07 首次跑慢档才发现）
        checks.append(
            ("三机型 SIL 跑批",
             ["bash", "test_host/fw_sil/run_all_airframes.sh"],
             ROOT / "TandemVec_FCS")
        )
    return checks


def main(argv: list[str]) -> int:
    include_sil = "--include-sil" in argv
    quick = "--quick" in argv
    if quick and include_sil:
        print("[FAIL] --quick 与 --include-sil 互斥")
        return 2

    # quick 是默认行为；只有显式 --include-sil 才跑慢档
    checks = build_checks(include_sil=include_sil)

    print("=" * 74)
    print("UniGNC 全仓总守卫" + ("（含 SIL 慢档）" if include_sil else "（快档）"))
    print("=" * 74)
    print()

    results: list[tuple[str, bool, float, str]] = []
    for name, cmd, cwd in checks:
        t0 = time.time()
        proc = subprocess.run(cmd, cwd=str(cwd), capture_output=True, text=True,
                              encoding="utf-8", errors="replace")
        dt = time.time() - t0
        ok = proc.returncode == 0
        tail = ""
        if not ok:
            out = (proc.stdout or "") + (proc.stderr or "")
            lines = [ln for ln in out.splitlines() if ln.strip()]
            tail = lines[-1][:90] if lines else "(无输出)"
        results.append((name, ok, dt, tail))
        mark = "PASS" if ok else "FAIL"
        print(f"[{mark}] {name:<20} {dt:6.1f}s" + (f"  ← {tail}" if tail else ""))

    print()
    print("-" * 74)
    passed = sum(1 for _, ok, _, _ in results if ok)
    print(f"通过 {passed}/{len(results)}　总耗时 {sum(d for _, _, d, _ in results):.1f}s")
    if passed != len(results):
        print("失败项：" + "、".join(n for n, ok, _, _ in results if not ok))
        return 1
    print("[PASS] 全部守卫通过：参数 / 推进 / 分配 / 坐标系 / 注释 / 仿真域 均未退化")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
