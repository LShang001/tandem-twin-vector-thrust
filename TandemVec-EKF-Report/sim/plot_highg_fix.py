# -*- coding: utf-8 -*-
"""EKF 12.2 g 硬门限修复前后对比图。
@file    plot_highg_fix.py
@brief   16/32 g 量程在 12–25 g 协调转弯下的 pos/att，修复前 vs 修复后。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 读修复前 highg_fs_2026-10-27.jsonl（逐 seed 行）与修复后
         highg_fix_2026-10-27.json（按组），按 (过载, 量程 16/32 g) 求位置 RMSE 与
         姿态 RMSE 的中位，出双栏折线图（纵轴 symlog）到
         figs/nav_highg_fix_2026-10-27.png。
@frame   NED（误差量取自 SIL 汇总的 NED 口径，本脚本只做归组与作图）
@units   m（位置 RMSE）、°（姿态 RMSE）、g（过载）
"""
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Arial"]
plt.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "sim-data")
FIGS = os.path.join(HERE, "..", "figs")


def summary(rows):
    """把逐条结果按 (过载 load, 量程 fs_g∈{16,32}) 归组求中位。

    参数：rows 为记录列表（需含 load / fs_g / pos_rmse / att_rmse_deg）。
    返回：{(load, 16.0|32.0): (位置 RMSE 中位 [m], 姿态 RMSE 中位 [°])}，
    该组合无样本时不出键（作图时按缺失跳过）。
    """
    out = {}
    for load in sorted({r["load"] for r in rows}):
        for fs in (16.0, 32.0):
            rs = [r for r in rows if r["load"] == load and r["fs_g"] == fs]
            if rs:
                out[(load, fs)] = (
                    float(np.median([r["pos_rmse"] for r in rs])),
                    float(np.median([r["att_rmse_deg"] for r in rs])),
                )
    return out


def main():
    """命令行入口：读修复前/修复后两份结果 → 各自 summary() 归组 → 画双栏折线
    （左位置 RMSE、右姿态 RMSE，4 条线=量程 16/32 g × 修复前/后，纵轴 symlog），
    存 figs/nav_highg_fix_2026-10-27.png 并打印路径。"""
    old = [json.loads(l) for l in
           open(os.path.join(DATA, "highg_fs_2026-10-27.jsonl"),
                encoding="utf-8") if l.strip()]
    fixed = json.load(open(os.path.join(DATA, "highg_fix_2026-10-27.json"),
                           encoding="utf-8"))
    o, f = summary(old), summary(fixed)
    loads = sorted({k[0] for k in o})
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8))
    for ax, idx, title in ((axes[0], 0, "位置 RMSE"),
                           (axes[1], 1, "姿态 RMSE")):
        for key, color, ls, label in (
                ((16.0, "old"), "#d95f5f", "--", "16 g 修复前"),
                ((32.0, "old"), "#8b3a3a", "--", "32 g 修复前"),
                ((16.0, "fix"), "#4f8fc0", "-", "16 g 修复后"),
                ((32.0, "fix"), "#2e8b57", "-", "32 g 修复后")):
            fs, which = key
            src = o if which == "old" else f
            xs = [l for l in loads if (l, fs) in src]
            ys = [src[(l, fs)][idx] for l in xs]
            if xs:
                ax.plot(xs, ys, marker="o", color=color, linestyle=ls,
                        label=label)
        ax.set_xlabel("过载 (g)")
        ax.set_yscale("symlog", linthresh=1.0)
        ax.set_title(f"协调转弯：{title}")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle("EKF 120 m/s² 硬门限修复前后（3 seeds，300 m/s）")
    fig.tight_layout()
    out = os.path.join(FIGS, "nav_highg_fix_2026-10-27.png")
    fig.savefig(out, dpi=150)
    print(out)


if __name__ == "__main__":
    main()
