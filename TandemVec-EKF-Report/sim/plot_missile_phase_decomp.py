#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24am 配图：失败/通过 seed 的误差分窗对比。

@file    plot_missile_phase_decomp.py
@brief   读 `missile_phase_decomp_*.json`，画 4 面板：
          (a) 各窗姿态 RMS（逐 seed 线，按簇着色）
          (b) 各窗位置 RMS（对数轴）
          (c) 真值俯仰/高度剖面 + 窗边界（标定飞行阶段）
          (d) 两条代表 seed 的 200 Hz 姿态误差时间线
@module  TandemVec-EKF-Report/绘图（L2 研究设施）
@note    图内文字用中文（项目约定）；生成后必须逐张目视审查（NaN/空图/尺度）。
"""
import argparse
import json
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams["font.family"] = "sans-serif"
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
FIG = os.path.abspath(os.path.join(HERE, "..", "figs"))
KEYS = ["0-10", "10-20", "20-30", "30-45"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", default=os.path.join(
        SD, "missile_phase_decomp_2026-10-28_am.json"))
    ap.add_argument("--out", default=os.path.join(
        FIG, "nav_missile_phase_decomp_2026-10-28.png"))
    args = ap.parse_args()
    data = json.load(open(args.json, encoding="utf-8"))
    rows = data["rows"]
    summ = data["summary"]
    fail = [r for r in rows if r["pos_rms"] > 20.0]
    ok = [r for r in rows if r["pos_rms"] <= 20.0]
    x = np.arange(len(KEYS))
    fig, axes = plt.subplots(2, 2, figsize=(13.5, 8.6))
    ax = axes[0][0]
    for r in ok:
        ax.plot(x, [r["windows"][k]["att_rms"] for k in KEYS], "-o", ms=3,
                color="#4C78A8", alpha=0.45, lw=1.0)
    for r in fail:
        ax.plot(x, [r["windows"][k]["att_rms"] for k in KEYS], "-o", ms=3,
                color="#D62728", alpha=0.75, lw=1.2)
    if fail and ok:
        ax.plot(x, [np.median([r["windows"][k]["att_rms"] for r in fail])
                    for k in KEYS], "-s", color="#D62728", lw=2.6,
                label="失效簇中位 (n=%d)" % len(fail))
        ax.plot(x, [np.median([r["windows"][k]["att_rms"] for r in ok])
                    for k in KEYS], "-s", color="#4C78A8", lw=2.6,
                label="通过簇中位 (n=%d)" % len(ok))
        ax.legend(fontsize=9)
    ax.set_xticks(x)
    ax.set_xticklabels([k.replace("-", "–") + " s" for k in KEYS])
    ax.set_ylabel("姿态 RMS (deg)  √(roll²+pitch²)")
    ax.set_title("(a) 姿态误差按时间窗")
    ax.grid(alpha=0.3)

    ax = axes[0][1]
    for r in ok:
        ax.plot(x, [max(r["windows"][k]["pos_rms"], 1e-3) for k in KEYS],
                "-o", ms=3, color="#4C78A8", alpha=0.45, lw=1.0)
    for r in fail:
        ax.plot(x, [max(r["windows"][k]["pos_rms"], 1e-3) for k in KEYS],
                "-o", ms=3, color="#D62728", alpha=0.75, lw=1.2)
    ax.set_yscale("log")
    ax.set_xticks(x)
    ax.set_xticklabels([k.replace("-", "–") + " s" for k in KEYS])
    ax.set_ylabel("水平位置 RMS (m, 对数)")
    ax.set_title("(b) 位置误差按时间窗（簇划分用全程 pos_rms>20 m）")
    ax.grid(alpha=0.3, which="both")

    ax = axes[1][0]
    rep = rows[0] if rows else None
    if rep:
        tl = rep["timeline"]
        ax.plot(tl["t"], tl["truth_pitch"], color="#333", lw=1.4,
                label="真值俯仰 (deg)")
        ax2 = ax.twinx()
        ax2.plot(tl["t"], tl["truth_alt"], color="#59A14F", lw=1.4,
                 label="真值高度 (m)")
        ax2.set_ylabel("高度 (m)", color="#59A14F")
        for b in [10, 20, 30]:
            ax.axvline(b, color="#999", ls="--", lw=0.9)
        ax.set_xlabel("t (s)")
        ax.set_ylabel("俯仰 (deg)")
        ax.set_title("(c) 飞行剖面（seed %d）与窗边界" % rep["seed"])
        ax.grid(alpha=0.3)

    ax = axes[1][1]
    show_f = fail[:2]
    show_p = ok[:2]
    for r in show_f:
        tl = r["timeline"]
        ax.plot(tl["t"], tl["att_err"], color="#D62728", lw=1.0,
                label="失效 s%d (%.1f m)" % (r["seed"], r["pos_rms"]))
    for r in show_p:
        tl = r["timeline"]
        ax.plot(tl["t"], tl["att_err"], color="#4C78A8", lw=1.0,
                label="通过 s%d (%.1f m)" % (r["seed"], r["pos_rms"]))
    for b in [10, 20, 30]:
        ax.axvline(b, color="#999", ls="--", lw=0.9)
    ax.set_xlabel("t (s)")
    ax.set_ylabel("姿态误差 (deg)")
    ax.set_title("(d) 代表 seed 的姿态误差时间线（2 Hz 抽样）")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    win = summ.get("windows", {})
    txt = []
    for k in KEYS:
        e = win.get(k)
        if e:
            txt.append("%s: att ×%.2f pos ×%.1f" %
                       (k, e["att_ratio"] or 0, e["pos_ratio"] or 0))
    fig.suptitle("30.06 g / 10 Hz 失效的时间窗分解（%d 失效 / %d 通过；%s）"
                 % (len(fail), len(ok), "；".join(txt)), fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    os.makedirs(FIG, exist_ok=True)
    fig.savefig(args.out, dpi=135)
    print("FIG:", args.out)


if __name__ == "__main__":
    main()
