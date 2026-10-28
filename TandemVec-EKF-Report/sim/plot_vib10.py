# -*- coding: utf-8 -*-
"""10.1 g 削顶 + 飞行段振动：off/on 与导航在环。

@file    plot_vib10.py
@brief   1 g/80 Hz、2 g/120 Hz 下位置/姿态的 Q 补偿效果。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 数据：missile-sil-2026-10-27_vib10_*.json；输出 figs/nav_vib10_2026-10-27.png。
@frame   NED
@units   m、°
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

CASES = [
    ("1 g/80 无补偿", "vib10_off_1g", "#c0392b"),
    ("1 g/80 +Q", "vib10_on_1g", "#e67e22"),
    ("2 g/120 无补偿", "vib10_off_2g", "#8e44ad"),
    ("2 g/120 +Q", "vib10_on_2g", "#2c3e50"),
    ("2 g/120 +Q 在环", "vib10_on_2g_nav", "#27ae60"),
]


def main():
    groups = {}
    for _, tag, _c in CASES:
        path = os.path.join(DATA, f"missile-sil-2026-10-27_{tag}.json")
        groups[tag] = json.load(open(path, encoding="utf-8"))["groups"][
            "thrust10g_icm_full"]
    fig, axes = plt.subplots(1, 2, figsize=(13.0, 4.6))
    x = np.arange(len(CASES))
    med = [groups[t]["pos_rmse"]["med"] for _, t, _c in CASES]
    mx = [groups[t]["pos_rmse"]["mx"] for _, t, _c in CASES]
    axes[0].bar(x, med, 0.55, color=[c for _, _, c in CASES])
    axes[0].errorbar(x, med, yerr=[np.zeros(len(med)),
                                   np.array(mx) - np.array(med)],
                     fmt="none", ecolor="k", capsize=4)
    for i, (m, M) in enumerate(zip(med, mx)):
        axes[0].text(i, M * 1.05, f"{m:.2f} / {M:.2f}", ha="center",
                     va="bottom", fontsize=8)
    axes[0].set_xticks(x)
    axes[0].set_xticklabels([a[0] for a in CASES], rotation=12, ha="right",
                            fontsize=8)
    axes[0].set_yscale("log")
    axes[0].set_ylabel("位置 RMSE 中位/max [m]（对数）")
    axes[0].set_title("(a) 10.1 g + 振动：位置")
    axes[0].grid(True, axis="y", which="both", alpha=0.3)

    amed = [groups[t]["att_rmse"]["med"] for _, t, _c in CASES]
    amx = [groups[t]["att_max"]["mx"] for _, t, _c in CASES]
    axes[1].bar(x, amed, 0.55, color=[c for _, _, c in CASES])
    axes[1].errorbar(x, amed, yerr=[np.zeros(len(amed)),
                                    np.array(amx) - np.array(amed)],
                     fmt="none", ecolor="k", capsize=4)
    for i, (m, M) in enumerate(zip(amed, amx)):
        axes[1].text(i, M * 1.02, f"{m:.2f} / {M:.2f}", ha="center",
                     va="bottom", fontsize=8)
    axes[1].set_xticks(x)
    axes[1].set_xticklabels([a[0] for a in CASES], rotation=12, ha="right",
                            fontsize=8)
    axes[1].set_ylabel("姿态 RMSE 中位/max [°]")
    axes[1].set_title("(b) 10.1 g + 振动：姿态")
    axes[1].grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    out = os.path.join(FIG, "nav_vib10_2026-10-27.png")
    fig.savefig(out, dpi=150)
    print("wrote", out)


if __name__ == "__main__":
    main()
