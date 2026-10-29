#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""固件姿态目标直驱冒烟（fwapi.GncTrack）。

@file    run_fw_gnc_track_demo.py
@brief   验证"跳过遥控模式机、直接给目标姿态 + 油门"的固件控制通道：
          零误差居中、三轴误差符号、油门直通、饱和与返回码。
@module  TandemVec-EKF-Report/仿真驱动（固件直调）
@details 被测：`fw_gnc_track_step()`（内部调用固件 execute_attitude_controller /
          execute_yaw_controller / mix_and_output_commands，与 runGNCExecutive 同源）。
@frame   四元数均为 FRD←NED；欧拉 Z-Y'-X''、theta=−asin(R13)
@units   角度 deg（输出摆角）、油门 %、角加速度 dps²
@note    与 `run_fw_gnc_demo.py` 的区别：那条走完整模式机（RC 输入）；本脚本走
          姿态直驱（供设计/优化工具迁移用）。
@see     TandemVec_FCS/test_host/fw_algo/fw_gnc_api.h、docs/05-控制与分配/CTL-004
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import fwapi  # noqa: E402

DT = 0.005


def q_axis(axis: int, deg: float):
    """绕机体单轴旋转的四元数 (w,x,y,z)。"""
    h = math.radians(deg) * 0.5
    q = [math.cos(h), 0.0, 0.0, 0.0]
    q[1 + axis] = math.sin(h)
    return tuple(q)


def q_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def main():
    fails = []

    def check(cond, msg):
        print(("[PASS] " if cond else "[FAIL] ") + msg)
        if not cond:
            fails.append(msg)

    lib = fwapi.library.load()
    check(hasattr(lib, "fw_gnc_track_step"), "姿态直驱 ABI 已编入库")

    g = fwapi.GncTrack()
    level = (1.0, 0.0, 0.0, 0.0)

    # 1) 零误差 + 50% 油门：角加速度≈0、摆角居中、电机在油门附近
    out = g.step(quat=level, omega=(0, 0, 0), q_target=level,
                 throttle_pct=50.0, dt=DT)
    check(max(abs(a) for a in out["alpha_ref_dps2"]) < 1.0,
          f"零误差角加速度≈0（max {max(abs(a) for a in out['alpha_ref_dps2']):.3f} dps²）")
    check(abs(out["tvc_upper_deg"]) < 0.5 and abs(out["tvc_lower_deg"]) < 0.5,
          "零误差摆角居中")
    check(40.0 < out["ch3_pct"] < 60.0, f"50% 油门直通（{out['ch3_pct']:.1f}%）")

    # 2) 三轴误差符号：目标 +roll/+pitch/+yaw → 对应通道同号
    for axis, name in ((0, "roll"), (1, "pitch"), (2, "yaw")):
        o = g.step(quat=level, omega=(0, 0, 0), q_target=q_axis(axis, 5.0),
                   throttle_pct=50.0, dt=DT)
        a = o["alpha_ref_dps2"][axis]
        check(a > 0.5, f"+5° {name} 目标 → alpha_{name} 同号且有限（{a:.2f} dps²）")

    # 3) 油门直通单调
    thr = []
    for pct in (0.0, 30.0, 70.0, 100.0):
        o = g.step(quat=level, omega=(0, 0, 0), q_target=level,
                   throttle_pct=pct, dt=DT)
        thr.append(o["ch3_pct"])
    check(all(thr[i] <= thr[i + 1] + 1e-6 for i in range(len(thr) - 1)),
          f"油门直通单调（{['%.1f' % x for x in thr]}）")
    check(thr[0] < 5.0 and thr[-1] > 95.0, "0%/100% 油门到端点")

    # 4) 饱和：大误差下输出有界
    o = g.step(quat=level, omega=(0, 0, 0), q_target=q_mul(q_axis(0, 45.0), q_axis(1, 45.0)),
               throttle_pct=50.0, dt=DT)
    check(all(abs(v) <= 2000.0 for v in o["alpha_ref_dps2"]), "大误差输出有界")
    check(0.0 <= o["ch1_pct"] <= 100.0 and 0.0 <= o["ch2_pct"] <= 100.0,
          "摆角输出在 0–100%")

    # 5) 返回码：ABI 版本/数值非法
    import ctypes
    import fwapi.library as fwl
    inp = fwl.FwGncTrackInput()
    inp.struct_size = ctypes.sizeof(fwl.FwGncTrackInput)
    inp.abi_version = 999
    outp = fwl.FwGncTrackOutput()
    outp.struct_size = ctypes.sizeof(fwl.FwGncTrackOutput)
    h = lib.fw_gnc_create()
    check(lib.fw_gnc_track_step(h, ctypes.byref(inp), ctypes.byref(outp)) == 2,
          "ABI 版本不符返回 2")
    inp.abi_version = fwl.GNC_TRACK_ABI_VERSION
    inp.quat_wxyz[0] = float("nan")
    inp.q_target_wxyz[0] = 1.0
    inp.dt_s = 0.005
    check(lib.fw_gnc_track_step(h, ctypes.byref(inp), ctypes.byref(outp)) == 5,
          "NaN 输入返回 5")

    g.close()
    print("\n=== run_fw_gnc_track_demo:", "全部通过" if not fails else f"失败 {len(fails)} 项", "===")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
