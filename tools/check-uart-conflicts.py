#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    check-uart-conflicts.py
@brief   串口用途冲突守卫：同一个 env 不得让两个功能抢同一个物理串口
@module  项目治理 / 构建配置
@details 为什么需要这个守卫（2026-11-09 实测教训）：
         调 MAVLink 时为了「在没有数传的条件下验证帧格式」，我往 USB 调试口
         （Serial8）抄了一份 MAVLink 二进制帧。结果 USB 口同时混了 DBG 文本、
         板载链路 MAVLink、镜像 MAVLink 三样东西，抓包完全没法区分，
         还一度误判成「固件不发 HEARTBEAT」。
         `platformio.ini` 里其实早写了「该 env 不能同时开 BFS_ONBOARD_LINK_ON_UART8
         和 BFS_DBG_USB_SERIAL8——DBG 文本会和二进制帧混流」，但那只是注释，
         没有任何东西会拦你。本守卫把这类「一个物理串口被两个用途占用」
         的组合变成可执行检查。

@theory  N/A（构建配置守卫）
@frame   N/A
@units   N/A
@note    继承链按 extends 解析；`-UX` 形式的 undefine 会从集合里移除同名宏。
         新增「抢串口」的宏时，把它加进 CONFLICT_GROUPS 即可。
@test    T1 同 env 同时开 UART8 机载链路 + USB DBG → 应报错；
         T2 只开其中之一 → 应通过；T3 extends 继承来的冲突也要能检出
@see     AGENTS.md §约定、TandemVec_FCS/platformio.ini 的 env 注释
"""

from __future__ import annotations

import configparser
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
INI = ROOT / "TandemVec_FCS" / "platformio.ini"

# 互斥组：同一个物理串口（这里是 USB 的 Serial8）被两种用途同时占用
# 每组列出「占这个口的宏」，同一 env 里出现 2 个以上即冲突。
CONFLICT_GROUPS: list[tuple[str, list[str], str]] = [
    (
        "USB 调试口 Serial8",
        [
            "BFS_DBG_USB_SERIAL8",        # DBG 文本控制台
            "BFS_ONBOARD_LINK_ON_UART8",  # 板载链路 MAVLink 二进制
            "BFS_MAVLINK_MIRROR_USB",     # MAVLink 帧镜像到 USB（调试用）
        ],
        "同一个 USB 口不能既跑 DBG 文本又跑二进制协议帧——混流后无法解析",
    ),
]

FLAG_RE = re.compile(r"-D([A-Za-z_][A-Za-z0-9_]*)")


def parse_envs(text: str) -> dict[str, dict[str, object]]:
    """
    解析 platformio.ini 的 [env:*] 段。

    configparser 对 platformio 的 `;` 注释和重复键处理不好，这里手写轻量解析：
    逐行扫描，遇到 `[env:xxx]` 开新段，收集 build_flags（含续行）。
    """
    envs: dict[str, dict[str, object]] = {}
    current: str | None = None
    collecting_flags = False
    flags_text: dict[str, list[str]] = {}

    for raw in text.splitlines():
        line = raw.rstrip()
        stripped = line.strip()

        if stripped.startswith(";") or not stripped:
            continue

        m = re.match(r"^\[env:([^\]]+)\]$", stripped)
        if m:
            current = m.group(1)
            envs.setdefault(current, {})
            collecting_flags = False
            continue

        if stripped.startswith("[") and not stripped.startswith("[env:"):
            current = None
            collecting_flags = False
            continue

        if current is None:
            continue

        # 续行（缩进且以 -D 或 ; 开头）归到 build_flags
        if collecting_flags and raw[:1] in (" ", "\t") and not stripped.startswith(";"):
            flags_text.setdefault(current, []).append(stripped)
            continue

        if stripped.startswith("build_flags"):
            collecting_flags = True
            value = stripped.split("=", 1)[1] if "=" in stripped else ""
            flags_text.setdefault(current, []).append(value)
            continue

        collecting_flags = False
        if "=" in stripped:
            key, value = stripped.split("=", 1)
            envs[current][key.strip()] = value.strip()

    for name, chunks in flags_text.items():
        envs[name]["build_flags"] = " ".join(chunks)
    return envs


def resolve_flags(envs: dict[str, dict[str, object]], name: str, seen: set[str] | None = None) -> set[str]:
    """沿 extends 链算出某个 env 最终生效的宏集合（处理 -D 与 -U）。"""
    seen = seen or set()
    if name in seen or name not in envs:
        return set()
    seen.add(name)

    env = envs[name]
    flags: set[str] = set()

    extends = env.get("extends")
    if isinstance(extends, str) and extends.startswith("env:"):
        flags |= resolve_flags(envs, extends[len("env:"):], seen)

    raw = env.get("build_flags")
    if isinstance(raw, str):
        for m in re.finditer(r"-U([A-Za-z_][A-Za-z0-9_]*)", raw):
            flags.discard(m.group(1))
        for m in FLAG_RE.finditer(raw):
            flags.add(m.group(1))
    return flags


def main() -> int:
    """跑全部冲突检查。"""
    if not INI.is_file():
        print(f"[FAIL] 找不到 {INI}", file=sys.stderr)
        return 1

    envs = parse_envs(INI.read_text(encoding="utf-8"))
    if not envs:
        print("[FAIL] platformio.ini 里没解析出任何 [env:*]", file=sys.stderr)
        return 1

    errors: list[str] = []
    for env_name in sorted(envs):
        flags = resolve_flags(envs, env_name)
        for port, macros, why in CONFLICT_GROUPS:
            hit = [m for m in macros if m in flags]
            if len(hit) > 1:
                errors.append(
                    f"[env:{env_name}] 同时启用了 {port} 的多个用途：{', '.join(hit)}\n"
                    f"    {why}"
                )

    if errors:
        print(f"[FAIL] 串口用途冲突：{len(errors)} 处", file=sys.stderr)
        for e in errors:
            print(f"  - {e}", file=sys.stderr)
        print(
            "\n修复：一个物理串口只留一种用途。要临时验证协议，单独开一个 env，"
            "在注释里写清它占用哪个口、何时撤掉，不要往正在跑的调试口里叠第二条流。",
            file=sys.stderr,
        )
        return 1

    print(f"[PASS] 串口用途守卫：{len(envs)} 个 env 无同口冲突")
    return 0


if __name__ == "__main__":
    sys.exit(main())
