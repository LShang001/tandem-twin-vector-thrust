#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    check-param-parity.py
@module  工具/参数台账（tools：单一事实源的漂移检测）
@brief   核对仿真参数源 `models/aircraft-model.json` 与固件机型参数
         `TandemVec_FCS/include/TandemVec_Config.h` 的 `kDefaultTandemVecParams`
         是否逐项一致
@details 输入：两个文件（只读）；处理：从 JSON 的 sections[].parameters[] 取
         name/value，从 C++ 的 `/* name */ value f,` 注释标号取同名数值；
         输出：逐项对照表 + 差异清单 + 退出码。
         判据：相对误差 > 1e-6 记为漂移；白名单内的键允许有意分离，
         但在输出中显式列出理由（不是静默跳过）。
@theory  这两个文件描述同一架飞机（纵列双发基型）：JSON 供仿真域消费，
         Config.h 供固件与宿主测试消费。同一物理量出现两个数就是漂移源头，
         SIL 与仿真对不上时首先怀疑这里。
@ref     DEC-002 A-04（dMax 两套口径的裁决）；AGENTS.md 关键路径表
@frame   无（标量参数）
@units   按 JSON 的 unit 字段，两边同量纲直接比数值
@note    白名单当前只有 dMax：
           仿真域 0.4363 rad（25°，SAS 概念窗口，有意保留）
           固件域 0.2618 rad（15°，2026-08-07 实机收紧）
         两者不可互推；除它之外任何差异都应当视为缺陷。
@see     models/aircraft-model.json、TandemVec_FCS/include/TandemVec_Config.h
@test    本脚本即自检：`py -3.12 tools/check-param-parity.py` 退出码 0 表示一致
"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
JSON_PATH = ROOT / "models" / "aircraft-model.json"
CFG_PATH = ROOT / "TandemVec_FCS" / "include" / "TandemVec_Config.h"

# 允许有意分离的键：键 -> 理由（会打印出来，不静默放过）
WHITELIST = {
    "dMax": "仿真 0.4363 rad(25°) 概念窗口 vs 固件 0.2618 rad(15°) 实机限幅，DEC-002 A-04",
}

REL_TOL = 1e-6


def load_json_params() -> dict[str, float]:
    data = json.loads(JSON_PATH.read_text(encoding="utf-8"))
    out: dict[str, float] = {}
    for section in data.get("sections", []):
        for p in section.get("parameters", []):
            name = p.get("name")
            value = p.get("value")
            if isinstance(name, str) and isinstance(value, (int, float)):
                out[name] = float(value)
    return out


def check_coax_parity() -> list[str]:
    """核对共轴参数在三处的默认值是否一致。

    三处分别定义（见 VER-009 §2.7）：固件参数表、分配器、SIL plant。
    plant 故意不依赖固件头，所以只能靠这种文本层校验来防漂移。

    ★ 实际只比较「三处都存在」的字段。当前能比的是：
        mass_kg（三处都有）
        lever_m（三处都有，2026-11-08 补）
      而 max_total_thrust_n / servo_half_angle_deg 在 SIL plant 里**不是
      结构体字段**（前者是 makeParams() 的入参、后者 plant 压根不用），
      所以只有 1/3 处有，会被跳过并打印 [跳过]。
      这两项的守护靠别处：编译期 static_assert + 宿主测试。
    """
    fw_path = ROOT / "TandemVec_FCS" / "include" / "FlightCtrlParams.h"
    alloc_path = ROOT / "TandemVec_FCS" / "include" / "CoaxialAllocator.h"
    plant_path = ROOT / "TandemVec_FCS" / "test_host" / "fw_sil" / "coaxial_dynamics_model.h"

    # 只比较三处都有的、语义相同的字段
    # 2026-11-08 补 lever_m：此前只比三项，力臂三处都定义了却没被守护。
    # ★ inertia 没进来：SIL plant 写成数组 `inertia_kgm2[3] = {..}`，
    #   而 grab() 只认 `name = value` 形式，抓不到数组；强行解析会增加
    #   误报风险，暂留在人工核对（三处当前都是 0.015/0.015/0.02，2026-11-12 对齐实机）。
    keys = ["mass_kg", "max_total_thrust_n", "servo_half_angle_deg", "lever_m"]

    def grab(path: Path, start_marker: str, stop_marker: str) -> dict[str, float]:
        text = path.read_text(encoding="utf-8", errors="replace")
        s = text.find(start_marker)
        if s < 0:
            return {}
        e = text.find(stop_marker, s + len(start_marker))
        body = text[s:e] if e > 0 else text[s:]
        out: dict[str, float] = {}
        for m in re.finditer(
            r"\b([A-Za-z_][A-Za-z0-9_]*)\s*=\s*([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)f?",
            body,
        ):
            out.setdefault(m.group(1), float(m.group(2)))
        return out

    a = grab(fw_path, "struct CoaxialCtrlParams", "struct ")
    # ★ 2026-11-08 修：原来从 "float servo_half_angle_deg" 起抓，而 lever_m
    #   定义在它之前（:39 vs :42），抓不到 → len(vals)<3 → 静默跳过不报。
    #   改从结构体开头抓，覆盖全部字段。
    b = grab(alloc_path, "struct CoaxialAllocatorParams", "};")
    # ★ 2026-11-08 修：stop_marker 用 "};" 会被数组结束符
    #   `float inertia_kgm2[3] = {0.05f, ...};` 命中最先的 `};`，
    #   导致只抓到 mass_kg 一个字段、lever_m 抓不到 → 静默跳过不比较。
    #   改用行首的 "\n};" 才对应结构体真正的收尾。
    c = grab(plant_path, "struct Params", "\n};")

    problems: list[str] = []
    for key in keys:
        vals = {name: d[key] for name, d in (("固件参数表", a), ("分配器", b), ("SIL plant", c))
                if key in d}
        if len(vals) < 3:
            # ★ 2026-11-08：这里原来只 continue 静默跳过，导致「某处抓不到字段」
            #   与「三处真的一致」在输出上完全一样——Round 134 就是被这个假 PASS
            #   骗了一次（lever_m 在 plant 侧没抓到，检查器照样报三处一致）。
            #   现在把「跳过」也打印出来，让少抓一处变得可见。
            have = "、".join(sorted(vals))
            print(f"  [跳过] {key}: 只有 {len(vals)}/3 处有这个字段（{have}）")
            continue
        lo, hi = min(vals.values()), max(vals.values())
        denom = max(abs(lo), abs(hi), 1e-12)
        if (hi - lo) / denom > REL_TOL:
            detail = " / ".join(f"{k}={v:g}" for k, v in vals.items())
            problems.append(f"{key}: {detail}")
    return problems


def load_cfg_params() -> dict[str, float]:
    """从 kDefaultTandemVecParams 抓 `/* name */ value f,` 形式的数值。"""
    text = CFG_PATH.read_text(encoding="utf-8", errors="replace")
    start = text.find("kDefaultTandemVecParams")
    if start < 0:
        raise SystemExit(f"[FAIL] {CFG_PATH.name} 里找不到 kDefaultTandemVecParams")
    body = text[start:]
    out: dict[str, float] = {}
    for m in re.finditer(
        r"/\*\s*([A-Za-z_][A-Za-z0-9_]*)\s*\*/\s*([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)f?",
        body,
    ):
        out[m.group(1)] = float(m.group(2))
    return out


def main() -> int:
    sim = load_json_params()
    fw = load_cfg_params()

    common = sorted(set(sim) & set(fw))
    only_sim = sorted(set(sim) - set(fw))
    only_fw = sorted(set(fw) - set(sim))

    drift: list[tuple[str, float, float]] = []
    whitelisted: list[tuple[str, float, float]] = []
    for key in common:
        a, b = sim[key], fw[key]
        denom = max(abs(a), abs(b), 1e-12)
        if abs(a - b) / denom <= REL_TOL:
            continue
        if key in WHITELIST:
            whitelisted.append((key, a, b))
        else:
            drift.append((key, a, b))

    print("=" * 66)
    print("仿真参数的单一事实源 ⇄ 固件机型参数 一致性核对")
    print("=" * 66)
    print(f"JSON  {JSON_PATH.relative_to(ROOT)}：{len(sim)} 项")
    print(f"固件  {CFG_PATH.relative_to(ROOT)}：{len(fw)} 项")
    print(f"两边都有：{len(common)} 项")
    print()

    print(f"{'参数':<10}{'仿真':>14}{'固件':>14}  结论")
    print("-" * 66)
    for key in common:
        a, b = sim[key], fw[key]
        denom = max(abs(a), abs(b), 1e-12)
        same = abs(a - b) / denom <= REL_TOL
        if same:
            verdict = "一致"
        elif key in WHITELIST:
            verdict = "有意分离（白名单）"
        else:
            verdict = "★ 漂移"
        print(f"{key:<10}{a:>14.6g}{b:>14.6g}  {verdict}")

    if whitelisted:
        print()
        print("白名单明细（允许不同，理由如下）：")
        for key, a, b in whitelisted:
            print(f"  - {key}: 仿真 {a:.6g} / 固件 {b:.6g} — {WHITELIST[key]}")

    if only_sim:
        print()
        print(f"仅仿真侧有（固件不消费，正常）：{', '.join(only_sim)}")
    if only_fw:
        print()
        print(f"仅固件侧有（仿真不消费，正常）：{', '.join(only_fw)}")

    print()
    if drift:
        print(f"[FAIL] {len(drift)} 项漂移，必须修（或登记进 WHITELIST 并写明理由）：")
        for key, a, b in drift:
            print(f"  - {key}: 仿真 {a:.6g} vs 固件 {b:.6g}")
        return 1
    print("[PASS] 除白名单外无漂移：两个事实源在同一量上给出同一个数")

    # ---- 共轴三方一致性（2026-11-08 补）------------------------------------
    # 共轴参数在三个地方各写一份，当前数值一致但没有校验。这里补上，
    # 防止某天只改了其中一处。
    #   A) TandemVec_FCS/include/FlightCtrlParams.h 的 CoaxialCtrlParams
    #   B) TandemVec_FCS/include/CoaxialAllocator.h 的分配器参数结构
    #   C) TandemVec_FCS/test_host/fw_sil/coaxial_dynamics_model.h 的 Params
    coax_drift = check_coax_parity()
    print()
    if coax_drift:
        print(f"[FAIL] 共轴参数 {len(coax_drift)} 项在三处之间不一致：")
        for item in coax_drift:
            print(f"  - {item}")
        return 1
    print("[PASS] 共轴参数三处一致（FlightCtrlParams / CoaxialAllocator / SIL plant）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
