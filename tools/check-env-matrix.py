#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    check-env-matrix.py
@brief   构建矩阵可解性守卫：每个 env 的 IMU ODR 都必须能选出合法的 ISR 轮询频率
@module  项目治理 / 构建配置
@details 为什么需要这个守卫（2026-11-12 实测教训）：
         2026-11-11 把 ISR 轮询频率改成硬编码 `2 × ODR` 并断言「必须整除 6400Hz 时基」。
         该写法只对 ICM-45686 的 1600 档成立（1600→3200，6400÷3200=2）；对
         ICM-42688 的 2000 档是 4000，6400÷4000 不整除 ⇒ 纵列/共轴/单涵道/单涵道实飞
         四个 42688 env 全部编译失败。而 `default_envs` 指向 45686，日常只编那一个，
         断裂两天无人发现，直到 2026-11-12 读板上 health 时才被追出来。
         本守卫不编译固件（四个 env 要 3~4 分钟），只按 platformio.ini 的宏组合复算
         「该 env 的 ODR 能否选出满足约束的 ISR 档」，几十毫秒出结果。

@theory  约束与 src/imu_odr_config.h 一一对应：
         ① ISR 必须整除调度时基 6400Hz（相位累加均匀，不产生 1/2 拍交替的长周期抖动）
         ② ISR ≥ ceil(1.5 × ODR)（芯片 RC 晶振实测 +5%，readFifo 每次只交一帧）
         ③ ISR ≤ 6400Hz（一拍最多抽一次）
         候选表是 6400 的 2 的幂因数：100/200/400/800/1600/3200/6400。

@frame   N/A
@units   Hz
@note    ★ 改 kSupportedHz / kOdrHz 默认值 / ISR 候选表 / 调度时基时，本守卫会把所有
         受影响的 env 一次列出来，而不是等你下次恰好编到那一个。
         ★ 新增 env 自动纳入，不需要维护名单。
         ★ `BFS_IMU_ODR_HZ` 的编译期覆盖（-DBFS_IMU_ODR_HZ=N）按 N 复算。
@test    T1 42688 默认 2000 → ISR 3200（1.6×）通过；
         T2 45686 默认 1600 → ISR 3200（2.0×）通过；
         T3 若改成旧 2× 规则（2000→4000）本守卫报 FAIL——复算的正是当前新规则。
@see     TandemVec_FCS/src/imu_odr_config.h、TandemVec_FCS/platformio.ini、AGENTS.md §命令
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
INI = ROOT / "TandemVec_FCS" / "platformio.ini"

# 与 imu_odr_config.h 的 kSupportedHz 保持一致（改那边必须同步这里）
MODELS = {
    "ICM-45686": {
        "default_odr": 1600,
        "supported": {50, 100, 200, 400, 800, 1600, 3200, 6400},
        "macro": "FCS_IMU_MODEL_ICM45686",
    },
    "ICM-42688-P": {
        "default_odr": 2000,
        "supported": {50, 100, 200, 500, 1000, 2000, 4000, 8000, 16000, 32000},
        "macro": "FCS_IMU_MODEL_ICM42688",
    },
}

SCHEDULER_TICK_HZ = 6400          # 与 task_scheduler_interval.h 的 kSchedulerTickHz 一致
ISR_CANDIDATES = (100, 200, 400, 800, 1600, 3200, 6400)
ISR_MIN_NUM, ISR_MIN_DEN = 3, 2   # 取数余量 1.5×


def parse_envs(text: str) -> dict[str, dict[str, str]]:
    """解析 platformio.ini 的 [env:*] 段（手写轻量解析，兼容 `;` 注释与续行）。"""
    envs: dict[str, dict[str, str]] = {}
    current: str | None = None
    collecting = False
    flags: dict[str, list[str]] = {}

    for raw in text.splitlines():
        line = raw.rstrip()
        stripped = line.strip()
        if stripped.startswith(";") or not stripped:
            continue

        m = re.match(r"^\[env:([^\]]+)\]$", stripped)
        if m:
            current = m.group(1)
            envs.setdefault(current, {})
            collecting = False
            continue

        if stripped.startswith("[") and not stripped.startswith("[env:"):
            current = None
            collecting = False
            continue

        if current is None:
            continue

        if collecting and raw[:1] in (" ", "\t") and not stripped.startswith(";"):
            flags.setdefault(current, []).append(stripped)
            continue

        if stripped.startswith("build_flags"):
            collecting = True
            value = stripped.split("=", 1)[1] if "=" in stripped else ""
            flags.setdefault(current, []).append(value)
            continue

        collecting = False
        if "=" in stripped:
            key, value = stripped.split("=", 1)
            envs[current][key.strip()] = value.strip()

    for name, chunks in flags.items():
        envs[name]["build_flags"] = " ".join(chunks)
    return envs


def resolve_macros(envs: dict[str, dict[str, str]], name: str, seen: set[str] | None = None) -> dict[str, str]:
    """沿 extends 链算出最终生效的宏（先父后子，子 env 的 -U 可撤销父的 -D）。"""
    seen = seen or set()
    if name in seen or name not in envs:
        return {}
    seen.add(name)

    env = envs[name]
    macros: dict[str, str] = {}

    extends = env.get("extends", "")
    for parent in re.findall(r"env:([A-Za-z0-9_]+)", extends):
        macros.update(resolve_macros(envs, parent, seen))

    raw = env.get("build_flags", "")
    for m in re.finditer(r"-U([A-Za-z_][A-Za-z0-9_]*)", raw):
        macros.pop(m.group(1), None)
    for m in re.finditer(r"-D([A-Za-z_][A-Za-z0-9_]*)(?:=([^\s]+))?", raw):
        macros[m.group(1)] = m.group(2) if m.group(2) is not None else "1"
    return macros


def ceil_ratio(value: int, num: int, den: int) -> int:
    return (value * num + den - 1) // den


def select_isr(odr: int) -> int:
    target = ceil_ratio(odr, ISR_MIN_NUM, ISR_MIN_DEN)
    for candidate in ISR_CANDIDATES:
        if candidate >= target:
            return candidate
    return 0


def check_env(env_name: str, macros: dict[str, str]) -> tuple[bool, str]:
    model = "ICM-42688-P"
    if macros.get("FCS_IMU_MODEL_ICM45686"):
        model = "ICM-45686"

    spec = MODELS[model]
    odr_raw = macros.get("BFS_IMU_ODR_HZ")
    if odr_raw is not None:
        try:
            odr = int(odr_raw)
        except ValueError:
            return False, f"{model}: BFS_IMU_ODR_HZ={odr_raw!r} 不是整数"
    else:
        odr = int(spec["default_odr"])

    if odr not in spec["supported"]:
        return False, f"{model}: ODR={odr}Hz 不在支持列表 {sorted(spec['supported'])}"

    if odr > SCHEDULER_TICK_HZ:
        return False, f"{model}: ODR={odr}Hz 超过调度时基 {SCHEDULER_TICK_HZ}Hz"

    isr = select_isr(odr)
    if isr == 0:
        return False, (f"{model}: ODR={odr}Hz 需要 ISR≥{ceil_ratio(odr, ISR_MIN_NUM, ISR_MIN_DEN)}Hz，"
                       f"候选表 {ISR_CANDIDATES} 无可用档（需提高调度时基）")

    if SCHEDULER_TICK_HZ % isr != 0:
        return False, f"{model}: ISR={isr}Hz 不整除调度时基 {SCHEDULER_TICK_HZ}Hz"

    if isr < ceil_ratio(odr, ISR_MIN_NUM, ISR_MIN_DEN):
        return False, f"{model}: ISR={isr}Hz 低于 1.5×ODR 下限"

    ratio = isr / odr
    return True, f"{model}: ODR={odr}Hz → ISR={isr}Hz（{ratio:.2f}×）"


def main() -> int:
    if not INI.exists():
        print(f"[FAIL] 找不到 {INI}")
        return 2

    envs = parse_envs(INI.read_text(encoding="utf-8", errors="replace"))
    rows: list[tuple[str, bool, str]] = []
    for name in sorted(envs):
        if name == "unignc":  # 基座不是可构建 env，只被继承
            continue
        ok, detail = check_env(name, resolve_macros(envs, name))
        rows.append((name, ok, detail))

    width = max(len(r[0]) for r in rows) if rows else 10
    print(f"{'env'.ljust(width)}  结果   说明")
    print("-" * (width + 44))
    for name, ok, detail in rows:
        print(f"{name.ljust(width)}  {'PASS' if ok else 'FAIL'}   {detail}")

    failed = [r for r in rows if not r[1]]
    print()
    if failed:
        print(f"[FAIL] {len(failed)} 个 env 的 IMU 采样配置不可出固件：")
        for name, _, detail in failed:
            print(f"  - {name}: {detail}")
        return 1
    print(f"[PASS] {len(rows)} 个 env 的 IMU 采样配置全部可解（ISR 整除 6400Hz 且 ≥1.5×ODR）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
