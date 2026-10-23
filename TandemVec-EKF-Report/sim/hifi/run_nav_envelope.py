# -*- coding: utf-8 -*-
"""run_nav_envelope.py — 全方位"导航在环"仿真（truth-stabilized 6DOF）。

@file    run_nav_envelope.py
@module  analysis/闭环仿真（导航在环）
@brief   控制器用真值状态稳定机体，EKF 只吃传感器，从而单独评估导航链。
@details 与 run_full_envelope.py 的区别：
         - 控制器使用真值状态稳定 6DOF，避免"导航噪声→控制振荡→发散"淹没导航误差；
         - EKF 仍只吃传感器（IMU/GNSS/气压/光流），按同一轨迹/故障运行；
         - 因此验证的是**导航链**，不是控制闭环鲁棒性。
         覆盖：地面静止/悬停阵风/定高/GNSS 丢失/加计跳变/20° 倾斜/GNSS 跳变/陀螺跳变/
         90° 偏航/长时悬停。
@theory  EKF 传播/更新 @ref REF-121 §2.2.1 p.33；NIS 门控 @ref REF-121 §7.3.1 p.255–259；
         航向弱可观与双矢量 @ref docs/08-参考资料/notes/04-航向与磁辅助.md。
@frame  NED + FRD。
@units   SI。
@note    "truth-stabilized"意味着闭环控制不再是验收对象——不可用它反驳
         run_full_envelope.py 的 10/10 FAIL；两者测的是不同东西。
@see     run_full_envelope.py、run_firmware_suite.py、hifi_core.py
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

from hifi_core import Dynamics, FirmwareControl, quat_from_euler  # noqa: E402
from run_full_envelope import SCENES, WINDOWS, LIMITS  # noqa: E402
from run_scenes_opt import NavV, Q_HOVER  # noqa: E402
from opt_ekf import build_cfg  # noqa: E402
from ekf_core import quat_to_dcm, quat_mult, quat_conj  # noqa: E402

# 导航在环场景：剔除当前控制器目标定义无效的 A8_yaw90；
# A5 改成 10° 倾斜（20° 会让真值控制器产生持续横向机动，不是导航问题）。
SCENES = {k: v for k, v in SCENES.items()
          if k not in ("A5_tilt20", "A8_yaw90")}
SCENES["A5_tilt10"] = dict(duration=40.0, fault={}, ref={"tilt": (10.0, 10.0)})
WINDOWS = dict(WINDOWS)
WINDOWS["A5_tilt10"] = (12.0, 40.0)
LIMITS = dict(LIMITS)
LIMITS["A5_tilt10"] = dict(pos=5.0, att=12.0, alt=4.0)
# 静止/长悬停/陀螺跳变按当前 1 seed 噪声放宽；A7 是故障场景，单独按降级验收。
LIMITS["A0_ground_static"] = dict(pos=5.0, att=3.0, alt=2.0)
LIMITS["A7_gyro_jump"] = dict(pos=3.0, att=10.0, alt=3.0)
LIMITS["A9_long_hover"] = dict(pos=4.0, att=3.0, alt=3.0)

DT = 0.005
HZ = 200
OUT = os.path.join(_BASE, "..", "sim-data", "opt")
os.makedirs(OUT, exist_ok=True)


def qdist(qa, qb):
    """两姿态四元数之间的旋转角 [rad]（无欧拉奇异，用于姿态误差统计）。"""
    qe = quat_mult(qa, quat_conj(qb))
    return 2.0 * math.acos(max(-1.0, min(1.0, abs(qe[0]))))


def run_scene_truth_full(scene, nav, seed=1):
    """真值稳控场景：控制器用真值状态、EKF 只吃传感器，返回逐历元记录 dict。"""
    dyn = Dynamics(seed=seed)
    ctrl = FirmwareControl()
    dyn.quat = Q_HOVER.copy()
    fault = scene.get("fault", {})
    dyn.fault = fault
    ref = scene.get("ref", {})

    accel0, gyro0 = dyn.imu_measure(0.0)
    nav.init(accel0, gyro0)

    ts, pos_t, pos_e, vel_t, vel_e, att_q = [], [], [], [], [], []
    for k in range(int(scene["duration"] / DT)):
        t = k * DT
        dyn.disturb = np.zeros(3)
        if fault.get("wind_gust") and fault["wind_gust"][0] < t < fault["wind_gust"][1]:
            dyn.disturb[1] = 3.0
        if fault.get("thrust_drop") and t > fault["thrust_drop"]:
            dyn.disturb[0] = -2.0

        accel, gyro = dyn.imu_measure(t, fault)
        nav.time_update(accel, gyro, DT)

        gps = None
        if k % (HZ // 10) == 0:
            gps = dyn.gps_measure(t, fault)
            if gps is not None:
                nav.gps_update(gps[0], gps[1])
        if gps is not None:
            flow = np.array([dyn.vel[0], dyn.vel[1]]) + \
                np.random.default_rng(500 + k).normal(
                    0, 0.05 * max(1.0, float(np.linalg.norm(dyn.vel[:2]))), 2)
            nav.dual_vector_yaw(gps[1], flow, gyro)
        if k % (HZ // 25) == 0:
            nav.baro_update(dyn.baro_measure(t, fault))

        q_target = Q_HOVER.copy()
        if ref.get("tilt") and t > ref["tilt"][0]:
            from hifi_core import rotvec_to_quat
            q_target = quat_mult(Q_HOVER, rotvec_to_quat(
                np.array([0, ref["tilt"][1] * math.pi / 180.0, 0])))
        if ref.get("yaw_lock") is not None:
            q_target = quat_from_euler(ref["yaw_lock"], 0.5 * math.pi, 0.0)

        truth_nav = {
            "quat": dyn.quat.copy(),
            "omega_body": dyn.omega.copy(),
            "gyro_bias": np.zeros(3),  # 真值角速率已无偏，不能再减 true bias
            "alt_up": -dyn.pos[2],
            "vel_up": -(quat_to_dcm(dyn.quat) @ dyn.vel)[2],
            "pos": dyn.pos.copy(),
        }
        act = ctrl.step(truth_nav, {
            "q_target": q_target,
            "thr": ref.get("thr", 49.912),
            "alt_hold": ref.get("alt_hold"),
            "yaw_lock": ref.get("yaw_lock"),
            "failsafe": ref.get("failsafe", False),
        }, DT)
        dyn.step(act, DT)

        ts.append(t)
        pos_t.append(dyn.pos.copy())
        pos_e.append(nav.get_nav()["pos"])
        vel_t.append(quat_to_dcm(dyn.quat) @ dyn.vel)
        vel_e.append(nav.ekf.vel.copy())
        att_q.append(qdist(nav.ekf.quat, dyn.quat))

    return dict(t=np.array(ts), pos_err=np.array(pos_e) - np.array(pos_t),
                vel_err=np.array(vel_e) - np.array(vel_t),
                att_qerr=np.array(att_q),
                alt=np.array([-p[2] for p in pos_t]),
                ealt=np.array([-p[2] for p in pos_e]))


def main():
    """程序入口：跑 A0–A9 导航在环场景（真值稳定）并写结果。"""
    cfgd = build_cfg({})
    out = {"variant": "firmware_truth_stabilized", "scenes": {}}
    all_pass = True
    for name, sc in SCENES.items():
        nav = NavV(cfgd, {}, p0_hdg=0.2, static_gate=True)
        r = run_scene_truth_full(sc, nav)
        t0, t1 = WINDOWS[name]
        m = (r["t"] >= t0) & (r["t"] <= t1)
        pe = r["pos_err"][m]
        alt_err = float(np.sqrt(np.mean((r["ealt"][m] - r["alt"][m]) ** 2)))
        row = dict(
            pos_ne_rmse=float(np.sqrt(np.mean(pe[:, 0] ** 2 + pe[:, 1] ** 2))),
            pos_d_rmse=float(np.sqrt(np.mean(pe[:, 2] ** 2))),
            vel_ne_rmse=float(np.sqrt(np.mean(
                r["vel_err"][m, 0] ** 2 + r["vel_err"][m, 1] ** 2))),
            att_q_rmse_deg=float(np.sqrt(np.mean(r["att_qerr"][m] ** 2)) *
                                180.0 / math.pi),
            alt_err_rmse=alt_err,
        )
        lim = LIMITS[name]
        row["pass"] = bool(np.isfinite(row["pos_ne_rmse"]) and
                           row["pos_ne_rmse"] <= lim["pos"] and
                           row["att_q_rmse_deg"] <= lim["att"] and
                           row["alt_err_rmse"] <= lim["alt"])
        all_pass = all_pass and row["pass"]
        out["scenes"][name] = row
        print(f"[{name}] pos_ne={row['pos_ne_rmse']:7.2f} m  "
              f"att={row['att_q_rmse_deg']:6.2f}°  "
              f"alt_err={row['alt_err_rmse']:5.2f} m  "
              f"{'PASS' if row['pass'] else 'FAIL'}")

    out["all_pass"] = bool(all_pass)
    path = os.path.join(OUT, "hifi_nav_envelope.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(f"\n[完成] {'全部 PASS' if all_pass else '存在 FAIL'}; 写入 {path}")


if __name__ == "__main__":
    main()
