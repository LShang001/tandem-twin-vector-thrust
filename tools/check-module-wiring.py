#!/usr/bin/env python3
"""固件头文件的接线状态审计。

@file    tools/check-module-wiring.py
@module  工具/固件架构
@brief   检查 include/*.h 是否被固件调用链使用，区分「在用 / 仅测试 / 完全未用」。
@details 扫描 include/ 下每个 .h，按三级判据追引用：
         ① src/ 与 include/ 下是否有其它文件包含它（间接引用也算）
         ② test_host/ 下是否有测试引用
         ③ 两者都没有 → 完全未用
         输出三张表 + 退出码。
@theory  「有实现」不等于「有接线」。代码写了、单测过了、文档记了，
         固件却从不调用它——这三个维度都看不出问题。本脚本补这一层审计。
@frame   N/A（架构关系，与坐标系无关）
@units   N/A
@note    ★ 判据要点（2026-11-08 踩过）：只 grep --include=*.cpp src/ 会把
         IMU_AutoCalibrator / MedianFilter 这类「经 src/state_data.h 间接包含」
         的模块误判成死代码。必须把 .h 也算进引用面。
@see     docs/07-验证与确认/VER-011-未接线模块清单-2026-11-08.md
@test    退出码 0 表示没有「完全未用」模块；仅测试引用的模块只警告不失败
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FW = ROOT / "TandemVec_FCS"
INC = FW / "include"

# 已知「仅测试引用」的模块——登记后只警告，不判失败。
# 新增条目必须写清原因，否则本脚本会当成未接线问题报出来。
TEST_ONLY_OK = {
    "AdaptiveGainDamper": "自适应增益阻尼器，未接入固件（VER-011 B 类）",
    "AdrcController": "ADRC 控制器备选，未接入固件（VER-011 B 类）",
    "GenericControlAllocation": "通用分配器，当前用机型专属分配器（VER-011 B 类）",
    "LiquidRocketAllocator": "液体火箭机型专属，该机型未烧过（VER-011 B 类）",
    "NavAidLossDetector": "导航辅助丢失检测器，未接线（VER-011 B 类 / CTL-005 §5）",
    "tilt_compass_triad": "TRIAD 初始对准备选实现，未接入（VER-011 B 类）",
    "LiquidRocketModel": "液体火箭推进/质量/喷管模型，该机型未烧过；已被 "
                         "test_liquid_rocket_sil 覆盖（VER-011 B 类）",
    "Quat4f": "SIL 刚体仿真专用的 float 四元数工具，仅供 test_host 使用，"
              "不属于固件模块",
    "ins_gnss_pps_latency": "GNSS PPS 时延估计，仅在测试里验证过，未接入固件",
}

# 已知「完全未用」的模块——登记后只警告。这类应当是零，出现即说明欠账。
UNUSED_KNOWN = {
    "IncrementalPID": "增量式 PID 备选实现，从未接入且无测试（VER-011 A 类）",
    "TandemVec_ServoModel": "摆座舵机动态模型（速率/死区/一阶滞后），"
                            "SIL 的 plant 已在别处实现同类逻辑，固件不引用",
    "TandemVec_ADRC": "ADRC 速率环备选实现，零引用；与 AdrcController 并存待择一",
}


def _includes_of(token: str) -> set[str]:
    """找出所有包含 <token.h> 的文件（排除它自己）。"""
    hits: set[str] = set()
    for base in (FW / "src", FW / "include", FW / "test_host"):
        if not base.is_dir():
            continue
        for p in base.rglob("*"):
            if p.suffix not in (".h", ".hpp", ".cpp") or not p.is_file():
                continue
            if p.parent == INC and p.stem == token:
                continue
            try:
                text = p.read_text(encoding="utf-8", errors="replace")
            except OSError:
                continue
            if re.search(rf'#\s*include\s*["<]{re.escape(token)}\.h[">]', text):
                # ★ 统一用正斜杠：Windows 下 relative_to 返回反斜杠，
                #   而下面的 startswith("test_host/") 用的是正斜杠，
                #   不统一会让所有测试引用被误算成固件引用（2026-11-08 踩过）。
                hits.add(p.relative_to(FW).as_posix())
    return hits


def main() -> int:
    print("=" * 70)
    print("固件头文件接线状态审计")
    print("=" * 70)

    wired: list[str] = []
    test_only: list[tuple[str, list[str]]] = []
    unused: list[str] = []

    for h in sorted(INC.glob("*.h")):
        name = h.stem
        refs = _includes_of(name)
        fw_refs = [r for r in refs if not r.startswith("test_host/")]
        th_refs = [r for r in refs if r.startswith("test_host/")]
        if fw_refs:
            wired.append(name)
        elif th_refs:
            test_only.append((name, th_refs))
        else:
            unused.append(name)

    print(f"include/*.h 共 {len(wired) + len(test_only) + len(unused)} 个")
    print(f"  在用（固件引用）      {len(wired)}")
    print(f"  仅测试引用            {len(test_only)}")
    print(f"  完全未用              {len(unused)}")
    print()

    bad = 0

    for name, refs in test_only:
        if name in TEST_ONLY_OK:
            print(f"  [已登记] {name:<28} {TEST_ONLY_OK[name]}")
        else:
            print(f"  ★ {name}: 只被 {', '.join(refs)} 引用，但未登记")
            bad += 1
    print()

    for name in unused:
        if name in UNUSED_KNOWN:
            print(f"  [已登记] {name:<28} {UNUSED_KNOWN[name]}")
        else:
            print(f"  ★ {name}: 零引用（连测试都没有）且未登记")
            bad += 1
    print()

    # 反向检查：登记的模块是否真的还在那个状态
    for name in list(TEST_ONLY_OK) + list(UNUSED_KNOWN):
        if name in wired:
            print(f"  ★ {name} 已接线，请从登记表移除")
            bad += 1

    if bad:
        print(f"[FAIL] {bad} 处需要处理")
        print("处理：接入固件，或在 TEST_ONLY_OK / UNUSED_KNOWN 里登记原因。")
        return 1

    print("[PASS] 所有头文件的接线状态都已明确（在用 / 仅测试 / 未用）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
