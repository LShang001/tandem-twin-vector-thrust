#!/usr/bin/env python3
"""GRAND 超广覆盖矩阵可视化（GNSS × 场景 × 振动 × 延迟）。

输入：tmp/grand_matrix.csv（gnss,scene,vib,delay,seed,pos,att,alt）
输出：tmp/grand_heatmap.png、tmp/grand_curves.png
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
CSV = os.path.join(REPO, "tmp", "grand_matrix.csv")
OUT1 = os.path.join(REPO, "tmp", "grand_heatmap.png")
OUT2 = os.path.join(REPO, "tmp", "grand_curves.png")

if not os.path.exists(CSV):
    raise SystemExit(f"缺少 {CSV}，先运行 IMUPERF_GRAND=1")

rows = []
with open(CSV, encoding="utf-8") as f:
    started = False
    for line in f:
        if not started:
            if line.startswith("== GRAND matrix =="):
                started = True
            continue
        if line.startswith("gnss,scene"):
            continue
        p = line.strip().split(",")
        if len(p) != 8:
            continue
        try:
            _ = float(p[2])
        except ValueError:
            continue
        rows.append((p[0], p[1], float(p[2]), float(p[3]), int(p[4]),
                     float(p[5]), float(p[6]), float(p[7])))
print("rows:", len(rows))

gnss = sorted({r[0] for r in rows})
scenes = ["static", "maneuver", "cruise", "vibration", "vib_outage", "accel"]
vibs = sorted({r[2] for r in rows})
delays = sorted({r[3] for r in rows})

# 1) GNSS × 场景 位置误差热图（取 vib=0.5、delay=0 的均值）
mat = np.full((len(gnss), len(scenes)), np.nan)
for i, g in enumerate(gnss):
    for j, s in enumerate(scenes):
        vals = [r[5] for r in rows
                if r[0] == g and r[1] == s and abs(r[2] - 0.5) < 1e-9
                and r[3] == 0.0]
        if vals:
            mat[i, j] = float(np.mean(vals))
fig, ax = plt.subplots(figsize=(10, 4.5), constrained_layout=True)
im = ax.imshow(mat, aspect="auto", cmap="viridis", norm=matplotlib.colors.LogNorm())
ax.set_xticks(range(len(scenes)), scenes, rotation=20)
ax.set_yticks(range(len(gnss)), gnss)
ax.set_title("位置 RMSE [m]：GNSS × 场景（vib 0.5 g、无延迟）")
fig.colorbar(im, ax=ax, label="m")
for i in range(len(gnss)):
    for j in range(len(scenes)):
        if np.isfinite(mat[i, j]):
            ax.text(j, i, f"{mat[i, j]:.2f}", ha="center", va="center",
                    color="w", fontsize=8)
fig.savefig(OUT1, dpi=150)
print("图已保存：", OUT1)

# 2) 振动/延迟影响（M8N_10Hz × maneuver/vibration）
fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), constrained_layout=True)
for ax, key, xlabel in [
        (axes[0], "vib", "振动幅值 [g]"),
        (axes[1], "delay", "GNSS 延迟 [ms]")]:
    for scene in ["static", "vibration", "vib_outage"]:
        xs, ys = [], []
        for v in (vibs if key == "vib" else delays):
            vals = [r[5] for r in rows
                    if r[1] == scene and r[0] == "M8N_10Hz"
                    and (abs(r[2] - v) < 1e-9 if key == "vib" else abs(r[2] - 0.5) < 1e-9)
                    and (r[3] == 0.0 if key == "vib" else abs(r[3] - v) < 1e-9)]
            if vals:
                xs.append(v * (1.0 if key == "vib" else 1000.0))
                ys.append(float(np.mean(vals)))
        if xs:
            ax.plot(xs, ys, "o-", label=scene)
    ax.set_xlabel(xlabel)
    ax.set_ylabel("位置 RMSE [m]")
    ax.set_yscale("log")
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_title("M8N_10Hz：" + ("振动幅值" if key == "vib" else "GNSS 延迟"))
fig.savefig(OUT2, dpi=150)
print("图已保存：", OUT2)
