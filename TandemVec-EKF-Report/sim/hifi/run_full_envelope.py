# -*- coding: utf-8 -*-
"""run_full_envelope.py — 全方位虚拟仿真：固件基线 + 全阶段场景（EKF 进控制闭环）。

@file    run_full_envelope.py
@module  analysis/闭环仿真（全包线）
@brief   6DOF + 固件控制律 + 固件语义 EKF 的全链路闭环，覆盖地/悬停/倾斜/故障各阶段。
@details 模型链：6DOF 动力学 + 固件控制律（hifi_core）→ IMU/GNSS/气压/光流传感器
         → 固件语义 EKF（OptEkf + 静止辅助 + 双矢量 + 延迟回放/重捕获）
         → 执行器动态 → 6DOF 闭环。
         覆盖 A0 静止 / A1 阵风 / A2 定高 / A3 GNSS 丢失 / A4 加计跳变 / A5 20° 倾斜 /
         A6 GNSS 位置跳变 / A7 陀螺跳变 / A8 90° 偏航 / A9 长时悬停。
         运行：cd TandemVec-EKF-Report/sim/hifi && py -3.12 run_full_envelope.py
@theory  EKF 传播/更新与 NIS @ref REF-121 §2.2.1 p.33、§7.3.1 p.255–259；
         固件控制律（误差四元数外环 + 速率内环 + 分配）@ref CTL-001 §4–§5；
         低速/悬停口径（pos-hold/mag-vector/flow-aiding 开关语义）见
         @ref TandemVec-EKF-Report/低速悬停闭环验收-2026-10-21.md。
@frame  NED + FRD；悬停 x_b 朝天。
@units   SI；姿态 deg、位置 m、时间 s。
@note    ★ 默认闭环只有姿态/定高，**没有水平位置环**，故 A2/A3/A5 的 pos_ne 不能单独当
         失败判据；开 --pos-hold --mag-vector --flow-aiding --a1-tilt-limit 5
         --a2-alt-ki 0.5 --nis-mag 10 后 3/5/10 种子均 9/9（A8 目标定义不成立，单列）。
         A2 还受气压更新率/相位耦合影响：仿真需 --baro-hz 2 --baro-phase 2，固件 baro
         实际速率不同，不能照搬。alt_vel.ki=0.5 只是仿真候选（固件仍 1.25）。
         这是 Python 固件语义闭环，不是 C++ 固件 HIL；C++ 侧由 test_host 与 `pio run` 验证。
@see     hifi_core.py、run_scenes_opt.py、opt/opt_ekf.py、
         @ref TandemVec-EKF-Report/低速悬停闭环验收-2026-10-21.md
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

from run_scenes_opt import NavV, run_scene  # noqa: E402
from opt_ekf import build_cfg  # noqa: E402

OUT = os.path.join(_BASE, "..", "sim-data", "opt")
os.makedirs(OUT, exist_ok=True)

Q_HOVER_YAW90 = None


def q_yaw90():
    """A8 场景的目标姿态：悬停基态 Q_HOVER 再绕体轴 z 转 90°（目标定义本身不成立，仅作对照）。"""
    from hifi_core import Q_HOVER, quat_mult, rotvec_to_quat
    return quat_mult(Q_HOVER, rotvec_to_quat(
        np.array([0.0, 0.0, 90.0 * math.pi / 180.0])))


SCENES = {
    "A0_ground_static": dict(duration=60.0, fault={}, ref={}),
    "A1_hover_gust": dict(duration=40.0,
                          fault={"wind_gust": (10.0, 20.0)}, ref={}),
    "A2_alt_hold": dict(duration=40.0, fault={}, ref={"alt_hold": 10.0}),
    "A3_gps_loss": dict(duration=60.0,
                        fault={"gps_lost": (15.0, 45.0)}, ref={}),
    "A4_accel_jump": dict(duration=40.0,
                          fault={"accel_bias_jump": np.array([1.0, 0, 0]),
                                 "accel_bias_jump_t": 10.0}, ref={}),
    "A5_tilt20": dict(duration=40.0, fault={}, ref={"tilt": (10.0, 20.0)}),
    "A6_gps_jump": dict(duration=40.0,
                        fault={"gps_jump": np.array([8.0, -5.0, 0.0]),
                               "gps_jump_t": 15.0}, ref={}),
    "A7_gyro_jump": dict(duration=40.0,
                         fault={"gyro_bias_jump": np.array([0.01, 0.0, 0.0]),
                                "gyro_bias_jump_t": 15.0}, ref={}),
    "A8_yaw90": dict(duration=40.0, fault={},
                     ref={"yaw_lock": 90.0 * math.pi / 180.0}),
    "A9_long_hover": dict(duration=120.0, fault={}, ref={}),
}

WINDOWS = {
    "A0_ground_static": (10.0, 50.0),
    "A1_hover_gust": (5.0, 40.0),
    "A2_alt_hold": (8.0, 40.0),
    "A3_gps_loss": (15.0, 45.0),
    "A4_accel_jump": (12.0, 40.0),
    "A5_tilt20": (12.0, 40.0),
    "A6_gps_jump": (12.0, 40.0),
    "A7_gyro_jump": (12.0, 40.0),
    "A8_yaw90": (10.0, 40.0),
    "A9_long_hover": (10.0, 120.0),
}

LIMITS = {
    "A0_ground_static": dict(pos=0.8, att=2.0, alt=1.0),
    "A1_hover_gust": dict(pos=3.0, att=6.0, alt=3.0),
    "A2_alt_hold": dict(pos=3.0, att=4.0, alt=2.0),
    "A3_gps_loss": dict(pos=15.0, att=6.0, alt=4.0),
    "A4_accel_jump": dict(pos=8.0, att=8.0, alt=4.0),
    "A5_tilt20": dict(pos=8.0, att=10.0, alt=4.0),
    "A6_gps_jump": dict(pos=8.0, att=6.0, alt=4.0),
    "A7_gyro_jump": dict(pos=8.0, att=8.0, alt=4.0),
    "A8_yaw90": dict(pos=3.0, att=6.0, alt=2.0),
    "A9_long_hover": dict(pos=3.0, att=3.0, alt=2.0),
}


def alt_err_rmse(r, t0, t1):
    """统计窗口 [t0,t1] 内的高度估计误差 RMSE [m]（r["ealt"]−r["alt"]）。"""
    m = (r["t"] >= t0) & (r["t"] <= t1)
    return float(np.sqrt(np.mean((r["ealt"][m] - r["alt"][m]) ** 2)))


def main():
    """程序入口：解析 CLI 开关并跑 A0–A9 全包线场景。"""
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--gravity-fast", action="store_true")
    ap.add_argument("--grav-noise", type=float, default=None)
    ap.add_argument("--pos-hold", action="store_true")
    ap.add_argument("--mag-vector", action="store_true")
    ap.add_argument("--flow-aiding", action="store_true")
    ap.add_argument("--a1-tilt-limit", type=float, default=None)
    ap.add_argument("--a2-alt-ki", type=float, default=None)
    ap.add_argument("--nis-mag", type=float, default=None)
    ap.add_argument("--baro-hz", type=float, default=None)
    ap.add_argument("--baro-phase", type=int, default=None)
    ap.add_argument("--gain-scale", type=float, default=1.0,
                    help="A/B only: scale att/rate kp (and optionally ki)")
    ap.add_argument("--scale-ki", action="store_true")
    ap.add_argument("--out", default=os.path.join(OUT, "hifi_full_envelope.json"))
    args = ap.parse_args()
    if args.gain_scale != 1.0:
        from hifi_core import CTRL
        for k in ("att_kp", "att_kp_yaw", "rate_kp_rp", "rate_kp_yaw"):
            CTRL[k] *= args.gain_scale
        if args.scale_ki:
            for k in ("rate_ki_rp", "rate_ki_yaw"):
                CTRL[k] *= args.gain_scale
    cfgd = build_cfg({})
    if args.grav_noise is not None:
        # 仅用于仿真重力增益 A/B；固件参数另走 FlightCtrlParams。
        from run_scenes_opt import FW
        FW["tier1_grav"] = float(args.grav_noise)
    opts = {"gravity_fast": bool(args.gravity_fast)}
    out = {"variant": "firmware", "gravity_fast": bool(args.gravity_fast),
           "grav_noise": args.grav_noise, "pos_hold": bool(args.pos_hold),
           "mag_vector": bool(args.mag_vector),
           "flow_aiding": bool(args.flow_aiding),
           "gain_scale": args.gain_scale, "scale_ki": bool(args.scale_ki),
           "a1_tilt_limit": args.a1_tilt_limit, "scenes": {}}
    all_pass = True
    for name, sc_orig in SCENES.items():
        sc = dict(sc_orig)
        sc["ref"] = dict(sc.get("ref", {}))
        if args.pos_hold:
            sc["ref"]["pos_hold"] = True
        if args.mag_vector:
            sc["mag_vector"] = True
        if args.flow_aiding and name == "A3_gps_loss":
            sc["flow_aiding"] = True
        if args.a1_tilt_limit is not None and name == "A1_hover_gust":
            sc["ref"]["pos_tilt_limit_deg"] = float(args.a1_tilt_limit)
        if args.a2_alt_ki is not None and name == "A2_alt_hold":
            sc["ref"]["alt_vel_ki"] = float(args.a2_alt_ki)
        nav = NavV(cfgd, opts, p0_hdg=0.2, static_gate=True)
        if args.nis_mag is not None:
            nav.ekf.opts["nis_mag"] = float(args.nis_mag)
        if args.baro_hz is not None:
            sc["baro_hz"] = float(args.baro_hz)
        if args.baro_phase is not None:
            sc["baro_phase"] = int(args.baro_phase)
        r = run_scene(sc, sc["duration"], lambda: nav)
        t0, t1 = WINDOWS[name]
        m = (r["t"] >= t0) & (r["t"] <= t1)
        pe = r["pos_err"][m]
        row = dict(
            pos_ne_rmse=float(np.sqrt(np.mean(pe[:, 0] ** 2 + pe[:, 1] ** 2))),
            pos_d_rmse=float(np.sqrt(np.mean(pe[:, 2] ** 2))),
            vel_ne_rmse=float(np.sqrt(np.mean(
                r["vel_err"][m, 0] ** 2 + r["vel_err"][m, 1] ** 2))),
            att_q_rmse_deg=float(np.sqrt(np.mean(r["att_qerr"][m] ** 2)) *
                                180.0 / math.pi),
            alt_err_rmse=alt_err_rmse(r, t0, t1),
        )
        lim = LIMITS[name]
        row["pass"] = bool(row["pos_ne_rmse"] <= lim["pos"] and
                           row["att_q_rmse_deg"] <= lim["att"] and
                           row["alt_err_rmse"] <= lim["alt"] and
                           np.isfinite(row["pos_ne_rmse"]))
        all_pass = all_pass and row["pass"]
        out["scenes"][name] = row
        print(f"[{name}] pos_ne={row['pos_ne_rmse']:7.2f} m  "
              f"att={row['att_q_rmse_deg']:6.2f}°  "
              f"alt_err={row['alt_err_rmse']:5.2f} m  "
              f"{'PASS' if row['pass'] else 'FAIL'}")

    out["all_pass"] = bool(all_pass)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(f"\n[完成] {'全部 PASS' if all_pass else '存在 FAIL'}; 写入 {args.out}")


if __name__ == "__main__":
    main()
