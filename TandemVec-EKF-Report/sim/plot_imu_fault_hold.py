# -*- coding: utf-8 -*-
"""IMU NaN/冻结故障（飞行武装）：默认 vs coast vs 组合 fallback。

@file    plot_imu_fault_hold.py
@brief   1 s / 2 s IMU 故障的位置、速度、姿态与拒绝次数对照。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 输出 figs/nav_imu_fault_fallback_2026-10-27.png。
@frame   NED
@units   m、m/s、°
"""
import json
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "sim-data")
FIG = os.path.join(HERE, "..", "figs")
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


def load(name):
    return json.load(open(os.path.join(DATA, name), encoding="utf-8"))["groups"]


BASE1 = load("missile-sil-2026-10-27_armed_main40.json")
BASE2 = load("missile-sil-2026-10-27_armed_base2s20.json")
COAST = load("missile-sil-2026-10-27_armed_coast20.json")
FB = load("missile-sil-2026-10-27_armed_fallback20.json")

CASES = [
    ("NaN 1 s", BASE1["imu_nan_icm_full"], COAST["imu_nan_icm_full"],
     FB["imu_nan_icm_full"]),
    ("NaN 2 s", BASE2["imu_nan2s_icm_full"], COAST["imu_nan2s_icm_full"],
     FB["imu_nan2s_icm_full"]),
    ("冻结 1 s", BASE1["imu_stuck_icm_full"], COAST["imu_stuck_icm_full"],
     FB["imu_stuck_icm_full"]),
    ("冻结 2 s", BASE2["imu_stuck2s_icm_full"],
     COAST["imu_stuck2s_icm_full"], FB["imu_stuck2s_icm_full"]),
]

C_BASE = "#e45756"
C_COAST = "#f58518"
C_FB = "#1f6fb4"
LABELS = ["默认整帧 return", "coast", "组合 fallback"]

fig, ax = plt.subplots(2, 2, figsize=(11.8, 8.4), dpi=160,
                       constrained_layout=True)
x = np.arange(len(CASES))
w = 0.26


def bars(a, key, title, ylabel, log=False):
    series = [[c[i][key]["med"] for c in CASES] for i in (1, 2, 3)]
    for i, (vals, c, lab) in enumerate(zip(series, (C_BASE, C_COAST, C_FB),
                                           LABELS)):
        a.bar(x + (i - 1) * w, vals, w, color=c, alpha=0.85, label=lab)
    if log:
        a.set_yscale("log")
    a.set_xticks(x)
    a.set_xticklabels([c[0] for c in CASES])
    a.set_ylabel(ylabel)
    a.set_title(title, fontsize=10)
    a.grid(axis="y", alpha=0.3)


bars(ax[0, 0], "pos_rmse", "(a) 位置 RMSE：1 s NaN 121.8→3.2 m", "位置 RMSE [m]",
     log=True)
bars(ax[0, 1], "pos_max", "(b) 位置峰值：1 s NaN 359→8.9 m", "位置峰值 [m]",
     log=True)
bars(ax[1, 0], "vel_rms", "(c) 速度 RMS：1 s NaN 14.1→0.58 m/s", "速度 RMS [m/s]",
     log=True)
bars(ax[1, 1], "att_rmse", "(d) 姿态 RMSE：1 s NaN 1.76→1.33°", "姿态 RMSE [°]")
ax[0, 0].legend(fontsize=8)
fig.suptitle("飞行武装 IMU NaN/冻结故障：组合 fallback 保留 GNSS 更新"
             "（L2，1 s 40 seeds、2 s 20 seeds；上限 2 s）", fontsize=11)
fig.savefig(os.path.join(FIG, "nav_imu_fault_fallback_2026-10-27.png"))
plt.close(fig)
print("saved nav_imu_fault_fallback_2026-10-27.png")
