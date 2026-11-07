#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    check-sil-coverage.py
@module  工具/仿真覆盖审计（固件源 ⇄ SIL/直调库覆盖清单）
@brief   核对固件 `src/*.cpp` 是否都在 SIL 核心源清单里，或已登记豁免及原因
@details 输入：`test_host/fw_algo/core_sources.sh` 的 `FW_CORE_SOURCES`、`src/*.cpp`；
         处理：逐一比对；未覆盖的源必须在下方 EXEMPT 白名单里且写明原因，否则报错；
         输出：覆盖/豁免/未登记 三分表 + 退出码。
@theory  SIL 的价值取决于「编进去的是不是真实的固件代码」。漏编一个模块不会报错，
         只会让仿真悄悄验证了另一份逻辑。本脚本把「哪些源没进仿真」变成显式清单。
@ref     docs/04-数学建模/代码理论核查/上板标定与验证清单-2026-11-01.md（宏与模块台账）
@frame   N/A（只做文件清单比对）
@units   N/A
@note    ★ 2026-11-07 首次运行时发现 sensor_imu.cpp 与 sensor_peripheral.cpp 都不在
         SIL 里——即 IMU 驱动链与 R33 斜距反投影**没有被仿真覆盖**。这是合理的
         设计边界（驱动层需要真实硬件），但之前没有一处文档把这条说清楚。
         本脚本的作用是：新增固件模块后，如果它既没进 SIL 也没登记豁免，立即红灯。
@see     test_host/fw_algo/core_sources.sh、test_host/fw_sil/README.md
@test    本脚本即自检：退出码 0 表示每个固件源都已覆盖或已登记豁免
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FW = ROOT / "TandemVec_FCS"
CORE_SOURCES = FW / "test_host" / "fw_algo" / "core_sources.sh"
SRC_DIR = FW / "src"

# 未进 SIL 的源：必须在这里登记原因，否则本脚本报错。
# 分类原则：① 需要真实硬件才能跑 ② 与算法无关的外设/通信 ③ SIL 有自己的等价实现
EXEMPT: dict[str, str] = {
    "sensor_imu.cpp": "IMU 驱动与校准链，需真实器件；SIL 用 fw_sil_stubs.cpp 注入理想 IMU",
    "sensor_peripheral.cpp": "气压/磁/光流/激光等外设读取，需真实器件；SIL 由传感器模型注入",
    "imu_factory.cpp": "器件检测与实例化，需真实 SPI；SIL 不实例化驱动",
    "communication.cpp": "AnoCom/串口遥测，与算法无关；SIL 直接读内部状态量",
    "mavlink_bridge.cpp": "MAVLink 桥，与算法无关；SIL 不启用",
    "can_bus.cpp": "CAN 外设，当前机型未使用",
    "ubx_config.cpp": "GNSS 接收机配置下发，需真实接收机；SIL 用 UBX 帧生成器",
    "main.cpp": "固件入口与 setup/loop，SIL 有自己的 main（fw_sil_*_main.cpp）",
    "task_scheduler.cpp": "TIM8 调度器，SIL 用确定步长循环替代",
    "ano_vars.cpp": "遥测变量表，与算法无关",
    "ano_params.cpp": "参数注册表；fw_algo 单独追加，SIL 侧不编",
}


def load_core_sources() -> list[str]:
    text = CORE_SOURCES.read_text(encoding="utf-8")
    m = re.search(r'FW_CORE_SOURCES="([^"]*)"', text)
    if not m:
        raise SystemExit(f"[FAIL] 在 {CORE_SOURCES} 里找不到 FW_CORE_SOURCES 定义")
    return [ln.strip() for ln in m.group(1).splitlines() if ln.strip()]


def main() -> int:
    core = load_core_sources()
    core_src = {p for p in core if p.startswith("src/")}
    all_src = {f"src/{p.name}" for p in sorted(SRC_DIR.glob("*.cpp"))}

    covered = sorted(core_src)
    not_covered = sorted(all_src - core_src)
    unregistered = [p for p in not_covered if Path(p).name not in EXEMPT]
    exempted = [p for p in not_covered if Path(p).name in EXEMPT]

    print("=" * 70)
    print("固件源 ⇄ SIL 覆盖清单审计")
    print("=" * 70)
    print(f"src/*.cpp 共 {len(all_src)} 个；清单里出现 {len(core_src)} 个；未覆盖 {len(not_covered)} 个")
    print()

    print(f"已进 SIL（{len(covered)}）：")
    for p in covered:
        print(f"  ✓ {p}")
    print()

    print(f"已登记豁免（{len(exempted)}）：")
    for p in exempted:
        print(f"  - {p:<28} {EXEMPT[Path(p).name]}")
    print()

    if unregistered:
        print(f"[FAIL] 有 {len(unregistered)} 个固件源既没进 SIL、也没登记豁免：")
        for p in unregistered:
            print(f"  ★ {p}")
        print()
        print("处理：若它是算法模块 → 加进 core_sources.sh 的 FW_CORE_SOURCES；")
        print("      若确实不需要仿真 → 在 tools/check-sil-coverage.py 的 EXEMPT 里写清原因。")
        return 1

    print("[PASS] 每个固件源要么进了 SIL、要么已登记豁免原因")
    return 0


if __name__ == "__main__":
    sys.exit(main())
