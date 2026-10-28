# -*- coding: utf-8 -*-
"""减震座 fn/ζ 容差热图：2 g/120 Hz/8.4 g 与 1 g/80 Hz/6.4 g。

@file    plot_vib_iso_tolerance.py
@brief   检查制造/温度引起的 fn、ζ 偏差下姿态误差是否仍可接受。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 输出 figs/nav_vib_isolator_tolerance_2026-10-27.png。
@frame   N/A
@units   Hz、无量纲、°
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

D = json.load(open(os.path.join(DATA, "vib-iso-tol-2026-10-27.json"),
                   encoding="utf-8"))["summary"]
FNS = ["15", "20", "25", "30", "35"]
ZETAS = ["0p05", "0p10", "0p20"]
ZL = ["0.05", "0.10", "0.20"]

fig, ax = plt.subplots(1, 2, figsize=(11.8, 4.6), dpi=160,
                       constrained_layout=True)
for k, (vib, title) in enumerate((
        ("2g120", "(a) 2 g / 120 Hz / 8.4 g：fn 15–35、ζ 0.05–0.20 均 <0.9°"),
        ("1g80", "(b) 1 g / 80 Hz / 6.4 g：fn≤25 可用；fn≥30 双峰 2°+"))):
    mat = np.zeros((len(ZETAS), len(FNS)))
    mx = np.zeros_like(mat)
    for i, z in enumerate(ZETAS):
        for j, fn in enumerate(FNS):
            g = D[f"{vib}_fn{fn}_z{z}"]
            mat[i, j] = g["att_med"]
            mx[i, j] = g["att_max"]
    im = ax[k].imshow(mat, aspect="auto", cmap="viridis", vmin=0.6, vmax=2.2)
    ax[k].set_xticks(np.arange(len(FNS)))
    ax[k].set_xticklabels(FNS)
    ax[k].set_yticks(np.arange(len(ZETAS)))
    ax[k].set_yticklabels(ZL)
    ax[k].set_xlabel("固有频率 fn [Hz]")
    ax[k].set_ylabel("阻尼比 ζ")
    ax[k].set_title(title, fontsize=9)
    for i in range(len(ZETAS)):
        for j in range(len(FNS)):
            ax[k].text(j, i, f"{mat[i, j]:.2f}\n({mx[i, j]:.2f})",
                       ha="center", va="center", fontsize=7,
                       color="white" if mat[i, j] < 1.3 else "black")
fig.colorbar(im, ax=ax, shrink=0.85, label="姿态 RMSE 中位 [°]")
fig.suptitle("减震座参数容差（5 seeds/格；括号内为最坏 seed）："
             "80 Hz 主谱需 fn≤25 Hz，120 Hz 主谱可放宽", fontsize=11)
fig.savefig(os.path.join(FIG, "nav_vib_isolator_tolerance_2026-10-27.png"))
plt.close(fig)
print("saved nav_vib_isolator_tolerance_2026-10-27.png")
