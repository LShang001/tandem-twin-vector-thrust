#!/usr/bin/env python3
"""@file    compare_grand_ab.py
@module  工具
@brief   GRAND 矩阵 A/B 对比：默认参数 vs CMA-ES 优化参数（位置/姿态改善率）
@details 按 (gnss, scene) 聚合 pos/att 均值，逐格打印改善率，并输出位置改善率热图。
@frame   N/A（只比对误差指标，不做坐标变换）
@units   m（位置 RMSE）、deg（姿态 RMSE）、%（改善率）

输入：tmp/grand_default.csv、tmp/grand_opt.csv（IMUPERF_GRAND=1 输出）
输出：tmp/grand_ab.png + 控制台汇总表
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
OUT = os.path.join(REPO, "tmp", "grand_ab.png")


def load(path):
    """读 GRAND 矩阵 CSV：从 `== GRAND matrix ==` 行起、跳过表头与列数≠8 的行，只保留第 3 列可转 float 的行，返回七元组列表；文件缺失返回空列表。"""
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path, encoding="utf-8") as f:
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
                         float(p[5]), float(p[6])))
    return rows


def agg(rows):
    """按 (gnss, scene) 分组对 pos/att 求均值，返回 {(gnss, scene): (pos 均值, att 均值)}。"""
    d = {}
    for g, s, vb, dl, seed, pos, att in rows:
        key = (g, s)
        d.setdefault(key, []).append((pos, att))
    return {k: (float(np.mean([x[0] for x in v])),
                float(np.mean([x[1] for x in v]))) for k, v in d.items()}


base = agg(load(os.path.join(REPO, "tmp", "grand_default.csv")))
opt = agg(load(os.path.join(REPO, "tmp", "grand_opt.csv")))
if not base or not opt:
    raise SystemExit("缺少 GRAND A/B CSV")

keys = sorted(set(base) & set(opt))
print(f"{'gnss':<16}{'scene':<12}{'pos(默认)':>10}{'pos(优化)':>10}"
      f"{'改善':>8}{'att(默认)':>10}{'att(优化)':>10}{'改善':>8}")
pos_gains, att_gains = [], []
for k in keys:
    p0, a0 = base[k]
    p1, a1 = opt[k]
    gp = (p0 - p1) / p0 * 100.0 if p0 > 0 else 0.0
    ga = (a0 - a1) / a0 * 100.0 if a0 > 0 else 0.0
    pos_gains.append(gp)
    att_gains.append(ga)
    print(f"{k[0]:<16}{k[1]:<12}{p0:>10.3f}{p1:>10.3f}{gp:>7.1f}%"
          f"{a0:>10.3f}{a1:>10.3f}{ga:>7.1f}%")
print(f"\n平均改善：位置 {np.mean(pos_gains):.1f}%、姿态 {np.mean(att_gains):.1f}%")
print(f"位置改善为正的比例：{np.mean([g > 0 for g in pos_gains]) * 100:.0f}%")

# 图：pos 改善率热图
gnss = sorted({k[0] for k in keys})
scenes = sorted({k[1] for k in keys})
mat = np.full((len(gnss), len(scenes)), np.nan)
for i, g in enumerate(gnss):
    for j, s in enumerate(scenes):
        k = (g, s)
        if k in base and k in opt and base[k][0] > 0:
            mat[i, j] = (base[k][0] - opt[k][0]) / base[k][0] * 100.0
fig, ax = plt.subplots(figsize=(10, 4.5), constrained_layout=True)
im = ax.imshow(mat, aspect="auto", cmap="RdYlGn", vmin=-50, vmax=50)
ax.set_xticks(range(len(scenes)), scenes, rotation=20)
ax.set_yticks(range(len(gnss)), gnss)
ax.set_title("位置 RMSE 改善率 [%]（正=优化后更好）")
fig.colorbar(im, ax=ax, label="%")
for i in range(len(gnss)):
    for j in range(len(scenes)):
        if np.isfinite(mat[i, j]):
            ax.text(j, i, f"{mat[i, j]:.0f}", ha="center", va="center",
                    fontsize=8)
fig.savefig(OUT, dpi=150)
print("图已保存：", OUT)
