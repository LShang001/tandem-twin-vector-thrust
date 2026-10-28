# -*- coding: utf-8 -*-
"""导弹导航在环：EKF 姿态反馈自驾时的弹道偏差。

@file    plot_navloop.py
@brief   6.4 g/10.1 g 真值闭环 vs 导航在环的末端位置偏差与 CDF。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 数据：missile-sil-2026-10-27_nl_*.jsonl；输出 figs/nav_navloop_2026-10-27.png。
@frame   NED
@units   m
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


def load(tag):
    rows = {}
    path = os.path.join(DATA, f"missile-sil-2026-10-27_{tag}.jsonl")
    for line in open(path, encoding="utf-8"):
        r = json.loads(line)
        rows[(r["config"], r["seed"])] = r
    return rows


def main():
    arms = [
        ("6.4 g 基线", "nl_clean_truth", "nl_clean_nav", "clean_icm_full",
         "#2c3e50"),
        ("10.1 g / 8 g 无补偿", "nl_off_truth", "nl_off_nav",
         "thrust10g_icm_full", "#c0392b"),
        ("10.1 g / 8 g + Q", "nl_q_truth", "nl_q_nav",
         "thrust10g_icm_full", "#e67e22"),
        ("10.1 g / 16 g", "nl_16g_truth", "nl_16g_nav",
         "thrust10g_icm_full", "#27ae60"),
    ]
    devs = []
    for _, t, n, cfg, _c in arms:
        T, N = load(t), load(n)
        d = []
        for s in range(1, 21):
            a, b = T[(cfg, s)], N[(cfg, s)]
            d.append(np.linalg.norm([a["ms_final_n"] - b["ms_final_n"],
                                     a["ms_final_e"] - b["ms_final_e"],
                                     a["ms_final_d"] - b["ms_final_d"]]))
        devs.append(d)

    fig, axes = plt.subplots(1, 2, figsize=(12.8, 4.5))
    ax = axes[0]
    x = np.arange(len(arms))
    med = [np.median(d) for d in devs]
    p95 = [np.percentile(d, 95) for d in devs]
    mx = [np.max(d) for d in devs]
    ax.bar(x, med, 0.55, color=[a[4] for a in arms])
    ax.errorbar(x, med, yerr=[np.zeros(len(med)), np.array(mx) - np.array(med)],
                fmt="none", ecolor="k", capsize=4)
    for i, (m, p, M) in enumerate(zip(med, p95, mx)):
        ax.text(i, M * 1.05,
                "med {:.0f} / p95 {:.0f} / max {:.0f}".format(m, p, M),
                ha="center", va="bottom", fontsize=8)
    ax.set_xticks(x)
    ax.set_xticklabels([a[0] for a in arms], rotation=12, ha="right",
                       fontsize=8)
    ax.set_yscale("log")
    ax.set_ylabel("导航在环 vs 真值闭环末端偏差 [m]（对数）")
    ax.set_title("(a) 同 seed 末端弹道偏差")
    ax.grid(True, axis="y", which="both", alpha=0.3)

    ax = axes[1]
    for (label, _t, _n, _cfg, color), d in zip(arms, devs):
        v = np.sort(d)
        ax.plot(v, np.arange(1, len(v) + 1) / len(v), "-o", ms=3,
                color=color, label=label)
    ax.set_xscale("log")
    ax.set_xlabel("末端偏差 [m]（对数）")
    ax.set_ylabel("CDF")
    ax.set_title("(b) 偏差分布")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    out = os.path.join(FIG, "nav_navloop_2026-10-27.png")
    fig.savefig(out, dpi=150)
    print("wrote", out)
    for a, d in zip(arms, devs):
        print(a[0], "med", round(float(np.median(d)), 0),
              "p95", round(float(np.percentile(d, 95)), 0),
              "max", round(float(np.max(d)), 0))


if __name__ == "__main__":
    main()
