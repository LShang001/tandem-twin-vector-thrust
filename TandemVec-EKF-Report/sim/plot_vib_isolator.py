# -*- coding: utf-8 -*-
"""弹载飞控物理减震座：传递率、扫频、2 g 杆图、阻尼与静挠度。

@file    plot_vib_isolator.py
@brief   读取 vib-iso-*.json 与 8.4 g 减震座 A/B，输出视觉审查图。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 输出 figs/nav_vib_isolator_2026-10-27.png。
@frame   N/A（标量传递率/姿态误差）
@units   Hz、无量纲、°、m/s、mm
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

C_NONE = "#888888"
C_20 = "#1f6fb4"
C_30 = "#54a24b"
C_50 = "#f58518"
C_80 = "#e45756"


def trans(f, fn, zeta=0.10):
    r = np.asarray(f, float) / fn
    c = 2.0 * zeta * r
    return np.sqrt((1.0 + c * c) / ((1.0 - r * r) ** 2 + c * c))


def load(name):
    return json.load(open(os.path.join(DATA, name), encoding="utf-8"))


D = load("vib-iso-2026-10-27.json")["groups"]
N84 = load("missile-sil-2026-10-27_vib84_nomount_fresh20.json")["groups"]
I20 = load("missile-sil-2026-10-27_vib84_iso020.json")["groups"]
I30 = load("missile-sil-2026-10-27_vib84_iso030.json")["groups"]

fig, ax = plt.subplots(2, 3, figsize=(14.2, 9.2), dpi=160,
                       constrained_layout=True)

# (a) 传递率曲线
a = ax[0, 0]
f = np.linspace(5, 200, 800)
for fn, c, lab in ((20, C_20, "fn=20 Hz"), (30, C_30, "fn=30 Hz"),
                   (50, C_50, "fn=50 Hz"), (80, C_80, "fn=80 Hz")):
    a.semilogx(f, trans(f, fn), color=c, lw=1.6, label=lab + "，ζ=0.10")
a.axhline(1.0, color="black", lw=0.7, ls=":")
for fx in (80, 120):
    a.axvline(fx, color="#f2c14e", ls="--", lw=0.8)
a.set_xlabel("激励频率 [Hz]")
a.set_ylabel("传递率 T")
a.set_title("(a) 单自由度减震座传递率：80/120 Hz 需 fn≤40 Hz", fontsize=10)
a.grid(alpha=0.3)
a.legend(fontsize=7)

# (b) 1 g 扫频热图：fn × 激励频率（5 seeds）
a = ax[0, 1]
freqs = [20, 30, 40, 50, 60, 80, 100, 120, 150]
fns = [0, 15, 20, 25, 30, 40, 50, 60, 80]
mat = np.zeros((len(fns), len(freqs)))
for i, fn in enumerate(fns):
    for j, fx in enumerate(freqs):
        mat[i, j] = D[f"f{fx:03d}_fn{fn:03d}_z015"]["att"]["med"]
im = a.imshow(mat, aspect="auto", cmap="viridis", origin="lower")
a.set_xticks(np.arange(len(freqs)))
a.set_xticklabels([str(x) for x in freqs], fontsize=7)
a.set_yticks(np.arange(len(fns)))
a.set_yticklabels(["无座"] + [str(x) for x in fns[1:]], fontsize=7)
a.set_xlabel("激励频率 [Hz]")
a.set_ylabel("减震座 fn [Hz]")
for i in range(len(fns)):
    for j in range(len(freqs)):
        a.text(j, i, f"{mat[i, j]:.1f}", ha="center", va="center", fontsize=6,
               color="white" if mat[i, j] < 1.6 else "black")
a.set_title("(b) 1 g 扫频（5 seeds）：软座避 80–120 Hz，fn≈激励共振放大",
            fontsize=10)
fig.colorbar(im, ax=a, shrink=0.85, label="姿态 RMSE [°]")

# (c) 2 g/120/8.4 g 杆图（同 fresh 二进制）
a = ax[0, 2]
labels = ["无座", "fn=20", "fn=30"]
vals = [N84["vib2g_8p4g_icm_full"]["att_rmse"]["med"],
        I20["vib2g_8p4g_icm_full"]["att_rmse"]["med"],
        I30["vib2g_8p4g_icm_full"]["att_rmse"]["med"]]
a.bar(np.arange(3), vals, color=[C_NONE, C_20, C_30], alpha=0.85)
for i, v in enumerate(vals):
    a.text(i, v + 0.15, f"{v:.2f}°", ha="center", fontsize=8)
a.set_xticks(np.arange(3))
a.set_xticklabels(labels)
a.set_ylabel("姿态 RMSE 中位 [°]")
a.set_title("(c) 2 g/120 Hz/8.4 g：7.97°→0.64–0.66°（20 seeds）", fontsize=10)
a.grid(axis="y", alpha=0.3)

# (d) 阻尼比（fn=25、1 g/80 Hz）
a = ax[1, 0]
zs = ["0p05", "0p10", "0p15", "0p25", "0p40"]
xl = ["0.05", "0.10", "0.15", "0.25", "0.40"]
vals = [D[f"f080_fn025_z{z}"]["att"]["med"] for z in zs]
a.bar(np.arange(len(zs)), vals,
      color=[C_30, C_20, C_20, C_50, C_80], alpha=0.85)
a.set_xticks(np.arange(len(zs)))
a.set_xticklabels(xl)
a.set_xlabel("阻尼比 ζ")
a.set_ylabel("姿态 RMSE 中位 [°]")
a.set_title("(d) fn=25、1 g/80 Hz：ζ 0.05–0.15 好，过大反噬", fontsize=10)
a.grid(axis="y", alpha=0.3)

# (e) 2 g/8.4 g 位置/速度交换
a = ax[1, 1]
x = np.arange(3)
pos = [N84["vib2g_8p4g_icm_full"]["pos_rmse"]["med"],
       I20["vib2g_8p4g_icm_full"]["pos_rmse"]["med"],
       I30["vib2g_8p4g_icm_full"]["pos_rmse"]["med"]]
vel = [N84["vib2g_8p4g_icm_full"]["vel_rms"]["med"],
       I20["vib2g_8p4g_icm_full"]["vel_rms"]["med"],
       I30["vib2g_8p4g_icm_full"]["vel_rms"]["med"]]
a.bar(x - 0.19, pos, width=0.38, color=C_20, alpha=0.85, label="位置 RMSE [m]")
a2 = a.twinx()
a2.bar(x + 0.19, vel, width=0.38, color=C_50, alpha=0.85,
       label="速度 RMS [m/s]")
a.set_xticks(x)
a.set_xticklabels(labels)
a.set_ylabel("位置 RMSE [m]")
a2.set_ylabel("速度 RMS [m/s]")
a.set_title("(e) 2 g/8.4 g：速度 0.63→0.24 m/s，位置略增", fontsize=10)
h1, l1 = a.get_legend_handles_labels()
h2, l2 = a2.get_legend_handles_labels()
a.legend(h1 + h2, l1 + l2, fontsize=7, loc="upper left")
a.grid(axis="y", alpha=0.3)

# (f) 静挠度 vs 120 Hz 传递率
a = ax[1, 2]
fns = np.linspace(10, 100, 300)
stat = 9.80665 / (2.0 * np.pi * fns) ** 2 * 1000.0  # mm
a.semilogy(fns, stat, color=C_20, lw=1.8, label="静挠度 [mm]")
a2 = a.twinx()
a2.semilogy(fns, trans(120.0, fns), color=C_50, lw=1.8,
            label="120 Hz 传递率")
a.axvspan(20, 30, color=C_30, alpha=0.18)
a.text(25, 1.2, "推荐 20–30 Hz", ha="center", fontsize=8, color=C_30)
a.set_xlabel("固有频率 fn [Hz]")
a.set_ylabel("静挠度 [mm]")
a2.set_ylabel("120 Hz 传递率")
a.set_title("(f) 软座代价：fn=20/30 Hz 静挠度 0.6/0.3 mm", fontsize=10)
h1, l1 = a.get_legend_handles_labels()
h2, l2 = a2.get_legend_handles_labels()
a.legend(h1 + h2, l1 + l2, fontsize=7, loc="upper right")
a.grid(alpha=0.3)

fig.suptitle("弹载飞控物理减震座：传递率模型与 SIL 评估（L2，生产双路 + fresh gate）",
             fontsize=12)
fig.savefig(os.path.join(FIG, "nav_vib_isolator_2026-10-27.png"))
plt.close(fig)
print("saved nav_vib_isolator_2026-10-27.png")
