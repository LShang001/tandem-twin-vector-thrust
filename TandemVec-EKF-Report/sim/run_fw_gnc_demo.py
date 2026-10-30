#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""固件控制链直调冒烟（fwapi.Gnc）：Python 驱动 `runGNCExecutive()`。

@file    run_fw_gnc_demo.py
@brief   验证"固件=唯一算法源"的控制链通道：能力查询、参数注册表读写、
          定高模式的油门方向、解锁/上锁输出边界。
@module  TandemVec-EKF-Report/仿真驱动（固件直调）
@details 不建 plant，只给固定导航状态，检查控制链输出（PWM/模式/分配诊断）符合
          固件语义。plant 闭环请用 fw_sil（真实固件 + 弹体）或后续 GNC 适配器。
@frame   NED / FRD；四元数 FRD←NED
@units   SI；PWM µs
@note    固件控制层是进程内单例：本脚本只创建一个 Gnc。
@see     TandemVec_FCS/test_host/fw_algo/fw_gnc_api.h、fwapi/gnc.py
"""
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import fwapi  # noqa: E402

DT = 0.005
HOVER_THROTTLE_PWM = 1500  # 仅作占位；实际比较用"爬升命令 vs 中立命令"


def rc_frame(throttle=1500, mode=1500, arm=2000):
    """CH1 roll / CH2 pitch / CH3 thr / CH4 yaw / CH5 arm / CH7 mode / CH8 TVC / CH9 att."""
    rc = [1500] * 16
    rc[2] = throttle
    rc[4] = arm
    rc[6] = mode      # 1500 → AUTO_ALTITUDE
    rc[7] = 988       # 低档 = 飞控自稳（非手动 TVC）
    rc[8] = 1000      # 角度模式
    return rc


def fly(gnc, seconds, alt_up, target_alt, arm=2000):
    n = int(round(seconds / DT))
    outs = []
    for _ in range(n):
        o = gnc.step(
            quat=(1.0, 0.0, 0.0, 0.0),
            omega=(0.0, 0.0, 0.0),
            vel=(0.0, 0.0, 0.0),
            pos=(0.0, 0.0, -alt_up),
            alt_up=alt_up,
            rc=rc_frame(arm=arm),
            dt=DT,
            target_alt=target_alt,
        )
        outs.append(o)
    return outs


def main():
    fails = []

    def check(cond, msg):
        print(("[PASS] " if cond else "[FAIL] ") + msg)
        if not cond:
            fails.append(msg)

    caps = fwapi.caps()
    print("caps:", caps)
    check(caps["gnc_abi"] == 1, "GNC ABI v1")
    check("gnc_available" in caps["features"], "控制链编入库中")
    check(caps["param_count"] == 164, f"参数注册表 164 项（133 基础 + 31 共轴；实际 {caps['param_count']}）")

    # 参数注册表：读写往返（与 GCS/MAVLink 同一份）
    names = fwapi.param_names()
    check(len(names) == caps["param_count"], "参数名表长度一致")
    kp0 = fwapi.param_get("att_roll.kp")
    fwapi.param_set("att_roll.kp", kp0)
    check(abs(fwapi.param_get("att_roll.kp") - kp0) < 1e-9, "参数读写往返一致")

    gnc = fwapi.Gnc()

    # 1) 上锁：电机应停在低端、模式仍可解算
    locked = fly(gnc, 0.2, alt_up=0.0, target_alt=0.0, arm=988)
    check(all(o["unlocked"] is False for o in locked), "CH5 低档 → 未解锁")
    check(all(o["ch3_pct"] <= 5.0 and o["ch4_pct"] <= 5.0 for o in locked),
          "未解锁时电机输出在低端")

    # 2) 解锁 + 定高 5 m（当前高度 0）：油门应向爬升方向增
    climb = fly(gnc, 2.0, alt_up=0.0, target_alt=5.0)
    thr_climb = float(np.median([o["ch3_pct"] for o in climb[-100:]]))
    check(all(np.isfinite([o["ch1_pct"], o["ch3_pct"], o["dw"]]).all() for o in climb),
          "输出全部有限")

    # 3) 目标改为 0 m：油门应回落
    settle = fly(gnc, 2.0, alt_up=0.0, target_alt=0.0)
    thr_settle = float(np.median([o["ch3_pct"] for o in settle[-100:]]))
    check(thr_climb > thr_settle + 1.0,
          f"定高误差驱动油门方向正确（{thr_climb:.1f}% > {thr_settle:.1f}%）")

    for o in climb + settle:
        for k in ("ch1_pct", "ch2_pct", "ch3_pct", "ch4_pct"):
            if not (0.0 <= o[k] <= 100.0):
                check(False, f"{k} 越界: {o[k]}")
                break

    # 4) ABI 健壮性：版本/尺寸/数值非法必须被拒，且不得改动状态
    import ctypes
    import fwapi.library as fwl
    lib = fwl.load()
    inp = fwl.FwGncInput()
    inp.struct_size = ctypes.sizeof(fwl.FwGncInput)
    inp.abi_version = fwl.GNC_ABI_VERSION
    out = fwl.FwGncOutput()
    out.struct_size = ctypes.sizeof(fwl.FwGncOutput)
    h = lib.fw_gnc_create()
    inp.quat_wxyz[0] = 1.0
    inp.target_alt_m = 5.0
    inp.dt_s = 0.005
    check(lib.fw_gnc_step(h, ctypes.byref(inp), ctypes.byref(out)) == 0, "合法输入返回 0")
    inp.abi_version = 999
    check(lib.fw_gnc_step(h, ctypes.byref(inp), ctypes.byref(out)) == 2, "ABI 版本不匹配返回 2")
    inp.abi_version = fwl.GNC_ABI_VERSION
    inp.dt_s = 0.0
    check(lib.fw_gnc_step(h, ctypes.byref(inp), ctypes.byref(out)) == 5, "步长非正返回 5")
    inp.dt_s = 0.005
    inp.quat_wxyz[0] = float("nan")
    check(lib.fw_gnc_step(h, ctypes.byref(inp), ctypes.byref(out)) == 5, "NaN 输入返回 5")
    inp.quat_wxyz[0] = 1.0
    inp.struct_size = 4
    check(lib.fw_gnc_step(h, ctypes.byref(inp), ctypes.byref(out)) == 3, "输入结构过小返回 3")

    print("\n=== test_fw_gnc_demo:", "全部通过" if not fails else f"失败 {len(fails)} 项", "===")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
