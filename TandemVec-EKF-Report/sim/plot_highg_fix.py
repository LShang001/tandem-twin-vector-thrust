# -*- coding: utf-8 -*-
"""EKF 12.2 g 硬门限修复前后对比图。
@file    plot_highg_fix.py
@brief   16/32 g 量程在 12–25 g 协调转弯下的 pos/att，修复前 vs 修复后。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
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
