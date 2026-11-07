#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    verify_param_names.py
@module  工具/实机参数表与 GCS 期望列表核对
@brief   烧录后逐项核对板上参数名与 GCS 期望名字表是否完全一致
@details 输入：串口名；处理：经 GCS CLI 读 `param list`，解析名字后与
         server/params.py 的 expected_names() 求差集；
         输出：缺失/多出的名字清单 + 退出码。
@theory  参数表的名字集合是「固件版本」的语义指纹之一——
         名字对不上，说明板上固件与 GCS 期望不是同一代，
         后续任何按名字寻址的读写都可能落到错误的 ID 上。
@ref     AGENTS.md §跨机同步（烧录后必须读回验证）
@frame   N/A（只比名字集合）
@units   N/A
@note    ★ 解析要点：GCS `param list` 的行格式是
           `  [111] ang_alpha[0]   0.85…`
         ——名字可能带数组下标 `[0]~[N]`，正则必须覆盖方括号，
         否则会把 18 个数组参数误报成「板上缺失」。
@see     tools/verify_board.py（八项判据的整机自检）、TandemVec_FCS/GCS/server/params.py
@test    本脚本即自检：退出码 0 表示名字表完全一致
"""
from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
GCS_DIR = ROOT / "TandemVec_FCS" / "GCS"


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM6"

    sys.path.insert(0, str(GCS_DIR))
    try:
        from server.params import expected_names  # type: ignore
    except Exception as exc:  # pragma: no cover - 环境问题
        print(f"[FAIL] 无法导入 GCS 的 expected_names：{exc}")
        return 2

    want = list(expected_names())
    print(f"GCS 期望：{len(want)} 项")

    proc = subprocess.run(
        [sys.executable, "-X", "utf8", "server/cli.py", "--port", port, "param", "list"],
        cwd=str(GCS_DIR), capture_output=True, text=True,
        encoding="utf-8", errors="replace", timeout=300,
    )
    text = (proc.stdout or "") + (proc.stderr or "")

    got: list[str] = []
    for line in text.splitlines():
        m = re.match(r"\s*\[\s*\d+\]\s+([A-Za-z_][A-Za-z0-9_.]*(?:\[\d+\])?)\s+[-+]?[0-9.]", line)
        if m:
            got.append(m.group(1))

    print(f"实机读回：{len(got)} 项")
    if not got:
        print("[WARN] 未解析到任何参数行，原始输出前 20 行：")
        for line in text.splitlines()[:20]:
            print("  " + line)
        return 2

    missing = sorted(set(want) - set(got))
    extra = sorted(set(got) - set(want))

    if missing:
        print(f"[FAIL] 板上缺少 {len(missing)} 项：{missing[:10]}")
    if extra:
        print(f"[FAIL] 板上多出 {len(extra)} 项：{extra[:10]}")
    if missing or extra:
        return 1

    print(f"[PASS] 名字表完全一致：{len(want)} 项逐项对上")
    return 0


if __name__ == "__main__":
    sys.exit(main())
