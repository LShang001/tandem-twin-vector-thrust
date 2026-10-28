# -*- coding: utf-8 -*-
"""run_fw_ekf_parity.py — 固件 EKF（fwapi 直调）与 sim/ekf_core.py 的逐历元对拍。

@file    run_fw_ekf_parity.py
@module  analysis/固件算法直调（一致性核验）
@brief   证明"仿真里跑的算法 = 固件里烧的算法"：同一条 IMU/GNSS 序列分别喂
         固件 Ekf15State（经 fw_algo ABI）与 Python 复刻版 ekf_core.Ekf15，
         量化两者的位置/速度/姿态差异。
@details 输入：确定性静态+小幅正弦激励，GNSS 10 Hz；无磁/无气压/无静止辅助，
         只比较传播 + GNSS 6 维更新的差异。
         输出：sim-data/fw_ekf_parity_<date>.json（指标）与 figs/*.png。
@note    两者不可能逐位相同：Python 侧 Qd 用 Simpson（固件飞行构建为 ZOH）、
         地球模型/P0 有独立实现。本脚本的目的是把差异量化成可跟踪的基线，
         并作为后续"用 fwapi 替代 Python 实现"的验收依据。
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import fwapi
from ekf_core import D2R, Ekf15, HOME_LLA, cfg as PY_CFG, quat_to_dcm

plt.rcParams["font.family"] = "sans-serif"
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

DT = 0.005
GNSS_DIV = 20  # 10 Hz
R_STD = [3.0, 6.0, 0.5, 1.0]  # 与固件 Ekf15State 默认一致
ACCEL0 = np.array([0.0, 0.0, -9.79], dtype=np.float64)


def imu_sample(t: float):
    accel = np.array([
        0.02 * np.sin(1.3 * t),
        0.015 * np.cos(0.7 * t),
        -9.79 + 0.01 * np.sin(2.1 * t),
    ])
    gyro = np.array([
        2.0e-3 * np.sin(0.9 * t),
        1.5e-3 * np.cos(1.1 * t),
        1.0e-3 * np.sin(0.4 * t),
    ])
    return accel, gyro


def lla_to_ned(lla, home):
    """局部切平面近似：把 LLA 差转成 NED 米（用于两滤波器互比）。"""
    dlat = float(lla[0] - home[0])
    dlon = float(lla[1] - home[1])
    dalt = float(lla[2] - home[2])
    r_n = 6378137.0 / np.sqrt(1.0 - 0.00669438 * np.sin(home[0]) ** 2)
    r_m = r_n * (1.0 - 0.00669438) / (1.0 - 0.00669438 * np.sin(home[0]) ** 2)
    return np.array([dlat * r_m, dlon * r_n * np.cos(home[0]), -dalt])


def quat_angle_deg(q1, q2):
    q1 = np.asarray(q1, dtype=np.float64)
    q2 = np.asarray(q2, dtype=np.float64)
    q1 /= np.linalg.norm(q1)
    q2 /= np.linalg.norm(q2)
    d = abs(float(np.dot(q1, q2)))
    return float(np.degrees(2.0 * np.arccos(min(1.0, d))))


def _quat_to_ypr(q):
    w, x, y, z = [float(v) for v in q]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return yaw, pitch, roll


def run(duration_s: float):
    home = HOME_LLA.copy()
    fw = fwapi.Ekf()
    fw.initialize(ACCEL0, (0, 0, 0), lla=home)
    fw.set_velocity_subset(False)

    py = Ekf15(PY_CFG)
    py.home_lla = home.copy()
    py.initialize(ACCEL0, np.zeros(3), home)
    py.home_lla = home.copy()

    n = int(round(duration_s / DT))
    t_hist = []
    pos_h, pos_d, vel_d, att_d, tilt_d, yaw_d = [], [], [], [], [], []

    for k in range(n):
        t = k * DT
        accel, gyro = imu_sample(t)
        dth = gyro * (DT * 0.5)
        dv = accel * (DT * 0.5)
        fw.time_update_two_sample(dth, dth, dv, dv, DT)
        py.time_update(dth, dth, dv, dv, DT)

        if k % GNSS_DIV == 0:
            meas_lla = home + np.array([
                2.0e-7 * np.sin(0.05 * t),
                2.0e-7 * np.cos(0.05 * t),
                0.05 * np.sin(0.1 * t),
            ])
            z_vel = np.array([
                0.02 * np.sin(0.2 * t),
                0.02 * np.cos(0.2 * t),
                0.01 * np.sin(0.3 * t),
            ])
            fw.gnss_update(z_vel, meas_lla, DT, 0.0, False)
            z_pos_ned = py.ned_from_lla(meas_lla, home)
            py.measurement_update_gnss(z_pos_ned, z_vel, R_STD, age_s=0.0, hist=None)

        sf = fw.state()
        d_ned = lla_to_ned(sf["lla"], py.lla)
        t_hist.append(t)
        pos_h.append(float(np.hypot(d_ned[0], d_ned[1])))
        pos_d.append(float(abs(d_ned[2])))
        vel_d.append(float(np.linalg.norm(sf["vel"] - py.vel)))
        att_d.append(quat_angle_deg(sf["quat"], py.quat))
        # 无磁场景下 yaw 不可观，必须把倾角与航向差分开报，否则总姿态差会被 yaw 漂移主导。
        _, py_pitch, py_roll = [float(x) for x in _quat_to_ypr(py.quat)]
        tilt_d.append(float(np.degrees(np.hypot(sf["ypr"][1] - py_pitch,
                                               sf["ypr"][2] - py_roll))))
        dyaw = float(sf["ypr"][0] - _quat_to_ypr(py.quat)[0])
        dyaw = (dyaw + np.pi) % (2 * np.pi) - np.pi
        yaw_d.append(float(np.degrees(abs(dyaw))))

    return {
        "t": np.asarray(t_hist),
        "pos_h": np.asarray(pos_h),
        "pos_d": np.asarray(pos_d),
        "vel": np.asarray(vel_d),
        "att": np.asarray(att_d),
        "tilt": np.asarray(tilt_d),
        "yaw": np.asarray(yaw_d),
        "fw_final": fw.state(),
        "py_final": {
            "lla": py.lla.copy(), "vel": py.vel.copy(), "quat": py.quat.copy(),
        },
        "fw_build": fwapi.build_info(),
        "fw_lib": str(fwapi.library_path()),
    }


def summarize(r):
    def stat(a):
        return {
            "max": float(np.max(a)),
            "p95": float(np.percentile(a, 95)),
            "rms": float(np.sqrt(np.mean(a ** 2))),
        }

    return {
        "pos_h_m": stat(r["pos_h"]),
        "pos_d_m": stat(r["pos_d"]),
        "vel_mps": stat(r["vel"]),
        "att_deg": stat(r["att"]),
        "tilt_deg": stat(r["tilt"]),
        "yaw_deg": stat(r["yaw"]),
        "final_pos_h_m": float(r["pos_h"][-1]),
        "final_att_deg": float(r["att"][-1]),
        "final_tilt_deg": float(r["tilt"][-1]),
        "final_yaw_deg": float(r["yaw"][-1]),
    }


def plot(r, out_png: str):
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.2))

    ax = axes[0]
    ax.plot(r["t"], r["pos_h"], label="水平位置差 [m]")
    ax.plot(r["t"], r["vel"], label="速度差 [m/s]")
    ax.set_xlabel("时间 [s]")
    ax.set_ylabel("差异")
    ax.set_title("位置/速度差异")
    ax.grid(alpha=0.3)
    ax.legend()

    ax = axes[1]
    ax.plot(r["t"], r["tilt"], label="倾角差 [deg]")
    ax.plot(r["t"], r["yaw"], label="航向差 [deg]")
    ax.set_xlabel("时间 [s]")
    ax.set_ylabel("角度差 [deg]")
    ax.set_title("姿态差（无磁：航向不可观）")
    ax.grid(alpha=0.3)
    ax.legend()

    ax = axes[2]
    for key, label in (("pos_h", "水平位置 [m]"), ("vel", "速度 [m/s]"),
                       ("tilt", "倾角 [deg]"), ("yaw", "航向 [deg]")):
        a = np.sort(r[key])
        ax.plot(a, np.linspace(0, 1, a.size), label=label)
    ax.set_xscale("symlog", linthresh=1e-3)
    ax.set_xlabel("差异")
    ax.set_ylabel("CDF")
    ax.set_title("差异分布")
    ax.grid(alpha=0.3)
    ax.legend()

    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="固件 EKF 与 Python 复刻版对拍")
    ap.add_argument("--duration", type=float, default=20.0, help="时长 [s]")
    ap.add_argument("--out-dir", default=os.path.join(HERE, "..", "sim-data"))
    ap.add_argument("--fig-dir", default=os.path.join(HERE, "..", "figs"))
    args = ap.parse_args()

    r = run(args.duration)
    summary = summarize(r)
    os.makedirs(args.out_dir, exist_ok=True)
    os.makedirs(args.fig_dir, exist_ok=True)
    date = _dt.date.today().isoformat()
    out_json = os.path.abspath(os.path.join(args.out_dir, f"fw_ekf_parity_{date}.json"))
    out_png = os.path.abspath(os.path.join(args.fig_dir, f"fw_ekf_parity_{date}.png"))
    payload = {
        "date": date,
        "duration_s": args.duration,
        "dt_s": DT,
        "gnss_hz": 1.0 / (DT * GNSS_DIV),
        "r_std": R_STD,
        "fw_lib": r["fw_lib"],
        "fw_build": r["fw_build"],
        "summary": summary,
        "note": "两者实现不同（Qd Simpson vs ZOH、地球模型/P0 独立），差异是基线不是错误；"
                "后续研究应直接用 fwapi 调用固件实现。",
    }
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    plot(r, out_png)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print("out:", out_json)
    print("fig:", out_png)


if __name__ == "__main__":
    main()
