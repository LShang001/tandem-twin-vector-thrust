#!/usr/bin/env python3
"""@file    plot_ins_trajectory.py
@module  工具
@brief   plot_ins_trajectory.py 模块说明。
捷联惯导真实轨迹可视化：真值 / 纯惯导 / GNSS 辅助 对比。

输入：tmp/ins_trajectory.csv（test_ins_trajectory 生成）
输出：tmp/ins_trajectory_v2.png
"""
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
CSV = os.path.join(REPO, "tmp", "ins_trajectory.csv")
OUT = os.path.join(REPO, "tmp", "ins_trajectory_v2.png")

if not os.path.exists(CSV):
    sys.exit(f"缺少 {CSV}，先运行 test_ins_trajectory")

raw = np.loadtxt(CSV, delimiter=",", skiprows=1)
t = raw[:, 0]
tn, te, td = raw[:, 1], raw[:, 2], raw[:, 3]
en, ee, ed = raw[:, 4], raw[:, 5], raw[:, 6]
gn, ge, gd = raw[:, 7], raw[:, 8], raw[:, 9]
att_t, att_i = raw[:, 10], raw[:, 11]
print(f"首行真值 ({tn[0]:.1f},{te[0]:.1f}) 末行真值 ({tn[-1]:.1f},{te[-1]:.1f})")
print(f"末行纯惯导 ({en[-1]:.1f},{ee[-1]:.1f}) GNSS ({gn[-1]:.1f},{ge[-1]:.1f})")

fig, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)

# 1) 水平轨迹（N-E）
ax = axes[0, 0]
ax.plot(te, tn, "k-", lw=2.4, label="真值")
ax.plot(ee, en, "r--", lw=1.4, label="纯惯导（无 GNSS）")
ax.plot(ge, gn, "b-.", lw=1.2, label="GNSS 10 Hz 辅助")
ax.set_xlabel("东向 [m]")
ax.set_ylabel("北向 [m]")
ax.set_title("水平轨迹（爬升→盘旋→加速→爬升）")
ax.grid(True, alpha=0.3)
ax.legend(loc="best", fontsize=9)
ax.set_aspect("equal", adjustable="datalim")

# 2) 高度剖面
ax = axes[0, 1]
ax.plot(t, td, "k-", lw=2.2, label="真值")
ax.plot(t, ed, "r--", lw=1.2, label="纯惯导")
ax.plot(t, gd, "b-.", lw=1.2, label="GNSS 辅助")
ax.set_xlabel("时间 [s]")
ax.set_ylabel("高度 [m]")
ax.set_title("高度剖面")
ax.grid(True, alpha=0.3)
ax.legend(loc="best", fontsize=9)

# 3) 水平位置误差
err_ins = np.hypot(en - tn, ee - te)
err_gnss = np.hypot(gn - tn, ge - te)
ax = axes[1, 0]
ax.plot(t, err_ins, "r-", lw=1.4, label=f"纯惯导（最大 {err_ins.max():.1f} m）")
ax.plot(t, err_gnss, "b-", lw=1.2,
        label=f"GNSS 辅助（最大 {err_gnss.max():.2f} m）")
ax.set_xlabel("时间 [s]")
ax.set_ylabel("水平位置误差 [m]")
ax.set_title("水平位置误差")
ax.set_yscale("log")
ax.grid(True, alpha=0.3)
ax.legend(loc="best", fontsize=9)

# 4) 航向
ax = axes[1, 1]
ax.plot(t, att_t, "k-", lw=2.0, label="真值航向")
ax.plot(t, att_i, "r--", lw=1.2, label="纯惯导航向")
ax.set_xlabel("时间 [s]")
ax.set_ylabel("航向 [deg]")
ax.set_title("航向角")
ax.grid(True, alpha=0.3)
ax.legend(loc="best", fontsize=9)

fig.suptitle("捷联惯导真实轨迹对比（MEMS 5 mg / 0.4 °/s，60 s，纯惯导 vs GNSS 10 Hz）",
             fontsize=12)
fig.savefig(OUT, dpi=150)
print(f"图已保存：{OUT}")
print(f"纯惯导最大水平误差 {err_ins.max():.1f} m，终值 {err_ins[-1]:.1f} m")
print(f"GNSS 辅助最大水平误差 {err_gnss.max():.2f} m，终值 {err_gnss[-1]:.2f} m")
print(f"纯惯导最大高度误差 {np.abs(ed - td).max():.1f} m")
