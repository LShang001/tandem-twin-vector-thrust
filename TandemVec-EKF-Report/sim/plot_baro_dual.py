# -*- coding: utf-8 -*-
"""气压双故障/失联 A/B 可视化（v4 水平-only 重锚）。
@file    plot_baro_dual.py
@brief   落地计数、pos_max、final_alt 三联图。
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

TAGS = ["2026-10-27_v4dual", "2026-10-27_v4reg"]
ARMS = [("drift+spoof", "漂移+spoof"), ("outage+drift", "失联+漂移"),
        ("drift0.5", "仅漂移"), ("spoofz100", "仅spoof"),
        ("nominal", "标称")]


def rows_by(tag):
    rows = []
    with open(os.path.join(DATA, f"baro_health_ab_{tag}.jsonl"), encoding="utf-8") as f:
        for line in f:
            if line.strip():
                rows.append(json.loads(line))
    return rows


def main():
    rows = []
    for t in TAGS:
        rows += rows_by(t)
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8))
    x = np.arange(len(ARMS))
    w = 0.36

    ax = axes[0]
    for gi, (gate, color) in enumerate(((0, "#d95f5f"), (1, "#4f8fc0"))):
        vals = []
        for arm, _ in ARMS:
            rs = [r for r in rows if r["arm"] == arm and r["gate"] == gate]
            vals.append(sum(1 for r in rs
                            if isinstance(r.get("final_alt"), (int, float))
                            and r["final_alt"] < 0.5))
        ax.bar(x + (gi - 0.5) * w, vals, w, color=color,
               label=f"gate{gate}")
    ax.set_xticks(x)
    ax.set_xticklabels([a[1] for a in ARMS], fontsize=9)
    ax.set_ylabel("落地次数 / n")
    ax.set_title("落地计数（越低越好）")
    ax.legend()
    ax.grid(True, alpha=0.3, axis="y")

    ax = axes[1]
    data = []
    for arm, _ in ARMS:
        for gate in (0, 1):
            v = [r["pos_max"] for r in rows
                 if r["arm"] == arm and r["gate"] == gate
                 and np.isfinite(r.get("pos_max", np.nan))]
            data.append(np.array(v, float) if v else np.array([np.nan]))
    bp = ax.boxplot(data, positions=np.repeat(x, 2) + np.tile([-0.18, 0.18], len(x)),
                    widths=0.3, patch_artist=True, showfliers=True)
    for i, b in enumerate(bp["boxes"]):
        b.set_facecolor("#d95f5f" if i % 2 == 0 else "#4f8fc0")
    ax.set_yscale("symlog", linthresh=1.0)
    ax.set_xticks(x)
    ax.set_xticklabels([a[1] for a in ARMS], fontsize=9)
    ax.set_ylabel("pos_max (m)")
    ax.set_title("水平位置峰值（红=关, 蓝=开）")
    ax.grid(True, alpha=0.3)

    ax = axes[2]
    data = []
    for arm, _ in ARMS:
        for gate in (0, 1):
            v = [r["final_alt"] for r in rows
                 if r["arm"] == arm and r["gate"] == gate
                 and np.isfinite(r.get("final_alt", np.nan))]
            data.append(np.array(v, float) if v else np.array([np.nan]))
    bp = ax.boxplot(data, positions=np.repeat(x, 2) + np.tile([-0.18, 0.18], len(x)),
                    widths=0.3, patch_artist=True, showfliers=True)
    for i, b in enumerate(bp["boxes"]):
        b.set_facecolor("#d95f5f" if i % 2 == 0 else "#4f8fc0")
    ax.set_xticks(x)
    ax.set_xticklabels([a[1] for a in ARMS], fontsize=9)
    ax.set_ylabel("final_alt (m)")
    ax.set_title("末端真高（越低=落地）")
    ax.grid(True, alpha=0.3)

    fig.suptitle("气压健康 + 水平-only 重锚：双故障/失联 A/B（v4，core2）")
    fig.tight_layout()
    out = os.path.join(FIGS, "nav_baro_dual_2026-10-27.png")
    fig.savefig(out, dpi=150)
    print(out)


if __name__ == "__main__":
    main()
