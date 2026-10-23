# -*- coding: utf-8 -*-
"""run_firmware_suite.py — 飞控固件 SIL：复杂场景/故障矩阵（truth-stabilized 6DOF）。

@file    run_firmware_suite.py
@module  analysis/闭环仿真（固件故障矩阵）
@brief   用真值状态稳定 6DOF，注入执行器/传感器/环境故障，测控制律与导航降级行为。
@details 与 run_nav_envelope.py 的区别：
         - 控制器用真值状态稳定 6DOF（测控制律/执行器故障响应），EKF 仍吃传感器；
         - 新增执行器故障：单发推力损失、舵机卡死；
         - 新增传感器/环境故障：GNSS 丢失/跳变、陀螺跳变、加计跳变、80 Hz 振动放大、
           阵风/风切变；覆盖悬停/定高/倾斜/长时/故障降级。
         运行：cd TandemVec-EKF-Report/sim/hifi && py -3.12 run_firmware_suite.py
         输出：sim-data/opt/firmware_suite.json
@theory  故障响应与控制权限 @ref docs/08-参考资料/notes/08-F4单舵机卡死能力边界.md；
         执行器一阶动态与限幅 @ref MOD-001 §9；
         EKF 降级/隔离 @ref REF-121 §7.3.1 p.255–259。
@frame  NED + FRD。
@units   SI；故障量（推力比例、卡死角 rad、振动幅值 m/s²）。
@note    ★ 这不是"EKF 进控制"的闭环验收；EKF 进控制的 run_full_envelope.py 仍 10/10 FAIL。
         单前摆卡死是硬件单点（滚转唯一主控），只能检测 + 有界下降（F4b 72 m/102 m），
         不是可恢复故障。
@see     hifi_core.py、run_nav_envelope.py、@ref docs/08-参考资料/notes/08-F4单舵机卡死能力边界.md
"""
import json
import math
import os
import sys

import numpy as np

_BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _BASE)
sys.path.insert(0, os.path.join(_BASE, "opt"))
sys.path.insert(0, os.path.join(_BASE, "hifi"))

from run_nav_envelope import run_scene_truth_full  # noqa: E402
from run_scenes_opt import NavV  # noqa: E402
from opt_ekf import build_cfg  # noqa: E402

OUT = os.path.join(_BASE, "..", "sim-data", "opt")
os.makedirs(OUT, exist_ok=True)

SCENES = {
    "F0_nominal_hover": dict(duration=40.0, fault={}, ref={}),
    "F1_alt_step": dict(duration=40.0, fault={}, ref={"alt_hold": 10.0}),
    "F2_gust": dict(duration=40.0,
                    fault={"wind_gust": (10.0, 20.0)}, ref={}),
    "F3_thrust_loss30": dict(duration=40.0,
                             fault={"thrust_loss": {"t": 15.0, "motor": "front",
                                                    "scale": 0.70}},
                             ref={"alt_hold": 1.0}),
    "F4_servo_jam": dict(duration=30.0,
                         fault={"servo_jam": {"t": 15.0, "axis": "df",
                                              "value": 0.05}},
                         ref={"alt_hold": 1.0}),
    "F4b_servo_jam_failsafe": dict(duration=30.0,
                                    fault={"servo_jam": {"t": 15.0, "axis": "df",
                                                         "value": 0.05}},
                                    ref={"alt_hold": 1.0, "failsafe": True}),
    "F5_vibration": dict(duration=40.0,
                         fault={"vib_amp": 0.15}, ref={}),
    "F6_gyro_jump": dict(duration=40.0,
                         fault={"gyro_bias_jump": np.array([0.01, 0.0, 0.0]),
                                "gyro_bias_jump_t": 15.0}, ref={}),
    "F7_accel_jump": dict(duration=40.0,
                          fault={"accel_bias_jump": np.array([1.0, 0.0, 0.0]),
                                 "accel_bias_jump_t": 15.0}, ref={}),
    "F8_gps_loss": dict(duration=60.0,
                        fault={"gps_lost": (15.0, 45.0)}, ref={}),
    "F9_gps_jump": dict(duration=40.0,
                        fault={"gps_jump": np.array([8.0, -5.0, 0.0]),
                               "gps_jump_t": 15.0}, ref={}),
    "F10_tilt10": dict(duration=40.0, fault={}, ref={"tilt": (10.0, 10.0)}),
    "F11_long_hover": dict(duration=120.0, fault={}, ref={}),
    "F12_gps_slow_drift": dict(duration=60.0,
                               fault={"gps_drift": {"t": 15.0,
                                                    "rate": np.array([0.20, -0.15, 0.0])}},
                               ref={}),
    "F13_baro_drift": dict(duration=60.0,
                           fault={"baro_drift": {"t": 10.0, "rate_mps": 0.08}},
                           ref={"alt_hold": 10.0}),
}

# 诊断阈值：故障场景允许降级，但姿态/位置不能无限发散
LIMITS = {
    "F0_nominal_hover": dict(pos=4.0, att=4.0, alt=2.0),
    "F1_alt_step": dict(pos=4.0, att=4.0, alt=2.5),
    "F2_gust": dict(pos=3.0, att=6.0, alt=3.0),
    "F3_thrust_loss30": dict(pos=8.0, att=15.0, alt=8.0),
    "F4_servo_jam": dict(pos=15.0, att=25.0, alt=12.0),
    "F4b_servo_jam_failsafe": dict(pos=100.0, att=120.0, alt=500.0),
    "F5_vibration": dict(pos=4.0, att=6.0, alt=3.0),
    "F6_gyro_jump": dict(pos=5.0, att=12.0, alt=4.0),
    "F7_accel_jump": dict(pos=8.0, att=10.0, alt=5.0),
    "F8_gps_loss": dict(pos=15.0, att=6.0, alt=4.0),
    "F9_gps_jump": dict(pos=8.0, att=6.0, alt=4.0),
    "F10_tilt10": dict(pos=5.0, att=12.0, alt=4.0),
    "F11_long_hover": dict(pos=4.0, att=4.0, alt=3.0),
    "F12_gps_slow_drift": dict(pos=10.0, att=6.0, alt=4.0),
    "F13_baro_drift": dict(pos=4.0, att=6.0, alt=6.0),
}

WINDOWS = {
    "F0_nominal_hover": (10.0, 40.0),
    "F1_alt_step": (8.0, 40.0),
    "F2_gust": (5.0, 40.0),
    "F3_thrust_loss30": (10.0, 40.0),
    "F4_servo_jam": (10.0, 30.0),
    "F4b_servo_jam_failsafe": (10.0, 30.0),
    "F5_vibration": (10.0, 40.0),
    "F6_gyro_jump": (12.0, 40.0),
    "F7_accel_jump": (12.0, 40.0),
    "F8_gps_loss": (15.0, 45.0),
    "F9_gps_jump": (12.0, 40.0),
    "F10_tilt10": (12.0, 40.0),
    "F11_long_hover": (10.0, 120.0),
    "F12_gps_slow_drift": (15.0, 60.0),
    "F13_baro_drift": (10.0, 60.0),
}


def run_one(scene, seed):
    """跑一个故障场景的一个种子（真值稳控 + 固件语义 EKF），返回指标 dict。"""
    cfgd = build_cfg({})
    nav = NavV(cfgd, {}, p0_hdg=0.2, static_gate=True)
    r = run_scene_truth_full(scene, nav, seed=seed)
    t0, t1 = WINDOWS[scene["name"]]
    m = (r["t"] >= t0) & (r["t"] <= t1)
    pe = r["pos_err"][m]
    return dict(
        pos_ne_rmse=float(np.sqrt(np.mean(pe[:, 0] ** 2 + pe[:, 1] ** 2))),
        pos_d_rmse=float(np.sqrt(np.mean(pe[:, 2] ** 2))),
        att_q_rmse_deg=float(np.sqrt(np.mean(r["att_qerr"][m] ** 2)) *
                            180.0 / math.pi),
        alt_err_rmse=float(np.sqrt(np.mean((r["ealt"][m] - r["alt"][m]) ** 2))),
    )


def main():
    """程序入口：跑故障矩阵场景并写 firmware_suite.json。"""
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=1)
    ap.add_argument("--out", default=os.path.join(OUT, "firmware_suite.json"))
    args = ap.parse_args()
    seeds = list(range(1, max(1, args.seeds) + 1))
    out = {"variant": "firmware_truth_stabilized", "seeds": seeds,
           "scenes": {}}
    n_pass = 0
    for name, sc in SCENES.items():
        sc = dict(sc)
        sc["name"] = name
        rows = [run_one(sc, seed) for seed in seeds]
        keys = ["pos_ne_rmse", "att_q_rmse_deg", "alt_err_rmse"]
        row = {k: float(np.mean([r[k] for r in rows])) for k in keys}
        row["std"] = {k: float(np.std([r[k] for r in rows])) for k in keys}
        row["runs"] = rows
        lim = LIMITS[name]
        # 保守验收：所有种子都必须过阈值。
        row["pass"] = all(
            np.isfinite(r["pos_ne_rmse"]) and
            r["pos_ne_rmse"] <= lim["pos"] and
            r["att_q_rmse_deg"] <= lim["att"] and
            r["alt_err_rmse"] <= lim["alt"] for r in rows)
        n_pass += 1 if row["pass"] else 0
        out["scenes"][name] = row
        print(f"[{name}] pos_ne={row['pos_ne_rmse']:7.2f}±{row['std']['pos_ne_rmse']:6.2f} m  "
              f"att={row['att_q_rmse_deg']:6.2f}±{row['std']['att_q_rmse_deg']:5.2f}°  "
              f"alt_err={row['alt_err_rmse']:5.2f}±{row['std']['alt_err_rmse']:5.2f} m  "
              f"{'PASS' if row['pass'] else 'FAIL'}")

    out["pass"] = n_pass
    out["total"] = len(SCENES)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(f"\n[完成] {n_pass}/{len(SCENES)} PASS; 写入 {args.out}")


if __name__ == "__main__":
    main()
