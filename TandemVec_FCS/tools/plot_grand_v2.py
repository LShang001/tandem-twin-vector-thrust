#!/usr/bin/env python3
"""@file    plot_grand_v2.py
@module  工具
@brief   GRAND v2 可视化：2 IMU（42688P/45686）× 6 GNSS（含 F10N）× 场景 × 振动 × 延迟
@details 读矩阵 CSV（脚本内 CSV 常量）后出 IMU×GNSS 位置 RMSE 热图，以及 F10N/M8N/低端
         接收机在振动/延迟下的位置 RMSE 曲线两图。
@frame   N/A（只画误差统计，不做坐标变换）
@units   m（位置 RMSE）、g（振动幅值）、ms（GNSS 延迟）

输入：tmp/grand_matrix_v2.csv（imu,gnss,scene,vib,delay,seed,pos,att,alt）
输出：tmp/grand_v2_imu.png、tmp/grand_v2_f10n.png
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
CSV = os.path.join(REPO, "tmp", "grand_matrix_v3.csv")
OUT1 = os.path.join(REPO, "tmp", "grand_v2_imu.png")
OUT2 = os.path.join(REPO, "tmp", "grand_v2_f10n.png")

rows = []
started = False
with open(CSV, encoding="utf-8") as f:
    for line in f:
        if not started:
            if line.startswith("== GRAND matrix =="):
                started = True
            continue
        p = line.strip().split(",")
        if len(p) != 9:
            continue
        try:
            _ = float(p[3])
        except ValueError:
            continue
        rows.append((p[0], p[1], p[2], float(p[3]), float(p[4]), int(p[5]),
                     float(p[6]), float(p[7]), float(p[8])))
print("rows:", len(rows))

imus = sorted({r[0] for r in rows})
gnss = sorted({r[1] for r in rows})
scenes = sorted({r[2] for r in rows})
print("IMU:", imus)
print("GNSS:", gnss)

# 1) 2 IMU × GNSS 位置 RMSE（static、vib 0.5、delay 0）
mat = np.full((len(imus), len(gnss)), np.nan)
for i, im in enumerate(imus):
    for j, g in enumerate(gnss):
        vals = [r[6] for r in rows
                if r[0] == im and r[1] == g and r[2] == "static"
                and abs(r[3] - 0.5) < 1e-9 and r[4] == 0.0]
        if vals:
            mat[i, j] = float(np.mean(vals))
fig, ax = plt.subplots(figsize=(11, 3.2), constrained_layout=True)
im = ax.imshow(mat, aspect="auto", cmap="viridis",
               norm=matplotlib.colors.LogNorm())
ax.set_xticks(range(len(gnss)), gnss, rotation=25)
ax.set_yticks(range(len(imus)), imus)
ax.set_title("位置 RMSE [m]：IMU × GNSS（static、vib 0.5 g、无延迟）")
fig.colorbar(im, ax=ax, label="m")
for i in range(len(imus)):
    for j in range(len(gnss)):
        if np.isfinite(mat[i, j]):
            ax.text(j, i, f"{mat[i, j]:.3f}", ha="center", va="center",
                    color="w", fontsize=8)
fig.savefig(OUT1, dpi=150)
print("图已保存：", OUT1)

# 2) F10N vs M8N：振动/延迟曲线（ICM45686）
fig, axes = plt.subplots(1, 2, figsize=(12, 4.2), constrained_layout=True)
for ax, key, xlabel in [(axes[0], "vib", "振动幅值 [g]"),
                        (axes[1], "delay", "GNSS 延迟 [ms]")]:
    for g in ["M8N_10Hz", "F10N_10Hz", "LowEnd_5Hz"]:
        xs, ys = [], []
        for v in ([0.2, 0.5, 1.0] if key == "vib" else [0.0, 0.03, 0.10]):
            vals = [r[6] for r in rows
                    if r[0] == "ICM45686_3k2" and r[1] == g
                    and r[2] == "vibration"
                    and (abs(r[3] - v) < 1e-9 if key == "vib" else abs(r[3] - 0.5) < 1e-9)
                    and (r[4] == 0.0 if key == "vib" else abs(r[4] - v) < 1e-9)]
            if vals:
                xs.append(v if key == "vib" else v * 1000.0)
                ys.append(float(np.mean(vals)))
        if xs:
            ax.plot(xs, ys, "o-", label=g)
    ax.set_xlabel(xlabel)
    ax.set_ylabel("位置 RMSE [m]")
    ax.set_yscale("log")
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_title("ICM45686：" + ("振动" if key == "vib" else "GNSS 延迟"))
fig.savefig(OUT2, dpi=150)
print("图已保存：", OUT2)
