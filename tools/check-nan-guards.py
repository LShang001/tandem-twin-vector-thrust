#!/usr/bin/env python3
"""安全告警类判据的 NaN 防护审计。

@file    tools/check-nan-guards.py
@module  工具/固件
@brief   检查「判据为假 = 不报警」这类安全告警模块，其浮点比较是否有 NaN 防护。
@details 2026-11-08 发现 ActuatorFaultMonitor 的 roll_bad/pitch_bad 用
         `std::fabs(x) > 阈值` 形式，而 |nan| > y 恒为 false。姿态发散到 NaN 时
         该判据永假 → 计时器每拍清零 → F4 永不触发。SIL 探针实测确认。
@theory  同一种写法在两类模块里含义相反：
           「判据为假 = 不使能」= 安全方向，NaN 无害（ActuatorOutputMap）
           「判据为假 = 不报警」= 危险方向，NaN 必须当故障（ActuatorFaultMonitor）
         本脚本只审计后者：名字里带 Monitor/Gate/Detector/Fault 的模块。
@frame   N/A
@units   N/A
@note    启发式检查，不看语义。命中不代表有 bug，但要求该行附近出现
         isfinite / is_finite / std::isfinite 之一。
@see     docs/07-验证与确认/VER-009-SIL验收脚本判据表-2026-11-07.md §2.26
@test    在 ActuatorFaultMonitor 里删掉 isfinite 应报；恢复后 PASS
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FW = ROOT / "TandemVec_FCS"

# 只审这些「告警类」模块（判据为假 = 不报警 = 危险方向）
ALERT_MODULES = ("ActuatorFaultMonitor", "HeadingAidingGate", "NavAidLossDetector")

# 浮点比较模式（可能被 NaN 短路）
CMP = re.compile(r"std::fabs\s*\(\s*([A-Za-z_][A-Za-z0-9_]*)\s*\)\s*[<>]=?")
# NaN 防护模式
GUARD = re.compile(r"isfinite|is_finite")


def main() -> int:
    print("=" * 70)
    print("安全告警判据的 NaN 防护审计")
    print("=" * 70)
    print()

    problems: list[str] = []
    for name in ALERT_MODULES:
        path = FW / "include" / f"{name}.h"
        if not path.exists():
            print(f"[跳过] {name}.h 不存在")
            continue
        lines = path.read_text(encoding="utf-8", errors="replace").split("\n")
        hits = 0
        for i, line in enumerate(lines, 1):
            if not CMP.search(line):
                continue
            hits += 1
            # ★ 必须**同一个变量名**出现在 isfinite(...) 里，不能靠「附近有 isfinite」
            #   2026-11-08 修：原实现只看 12 行窗口，去掉 roll 的 isfinite 后
            #   pitch 的 isfinite 仍在窗口内 → 假 PASS。
            var = CMP.search(line).group(1)
            guard_for_var = re.compile(
                r"(?:std::)?is_?finite\s*\(\s*" + re.escape(var) + r"\s*\)"
            )
            lo = max(0, i - 1 - 8)
            hi = min(len(lines), i + 8)
            window = "\n".join(lines[lo:hi])
            if not guard_for_var.search(window):
                problems.append(f"{name}.h:{i}  {line.strip()[:60]}")
        print(f"  {name}.h：浮点判据 {hits} 处")
    print()

    if problems:
        print(f"[FAIL] {len(problems)} 处告警判据无 NaN 防护：")
        for p in problems:
            print(f"  ★ {p}")
        print()
        print("理由：|nan| > 阈值 恒为 false。对「判据为假 = 不报警」的模块，")
        print("      这会让它在最需要报警时沉默。修法：非有限输入按故障方向处理。")
        return 1

    print("[PASS] 所有告警类判据附近都有 NaN 防护")
    return 0


if __name__ == "__main__":
    sys.exit(main())
