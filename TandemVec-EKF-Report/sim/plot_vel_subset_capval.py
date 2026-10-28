#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24r 图：15 m 触发上限在独立 seeds 51–100 上的泛化与阈值平坦性。

@file    plot_vel_subset_capval.py
@brief   出两张图：(1) 七臂发生率/阈值敏感性/箱线；(2) 逐 seed 配对散点/CDF/直方分布
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 这张图回答两个问题，缺一不可：
         ① 阈值 15 m 是不是只在选出它的那 50 个 seed 上有效（看 20 Hz 组在 seeds 51–100
            上的发生率与 nocap 臂的差）；
         ② 它是不是落在一个平坦平台上（cap 12/15/21 三档几乎同结果），
            还是悬崖上的一点（12 与 21 分处两侧）。
         分布类面板必须同时出现：双峰格只看聚合数会把"失效簇被消掉"读成"精度提升"，
         所以直方图与 CDF 用来确认改善发生在哪一簇。
@frame   NED；pos_rms [m]、nav_att_rmse [deg]
@units   米 / 个 seed / 比例
@note    图内文字避免使用 U+21D2（微软雅黑缺字，会渲染成方框）
@see     sim/check_vel_subset_capval.py（Y1–Y4 裁决）、sim/run_vel_subset_capval_matrix.sh
"""
import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.family"] = "sans-serif"
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
FIG = os.path.abspath(os.path.join(HERE, "..", "figs"))
SEEDS = list(range(51, 101))
FAIL_M = 25.0
ARMS = [("20 Hz 子集关", "y20_off", "#7f7f7f"),
        ("20 Hz 无上限", "y20_nocap", "#d62728"),
        ("20 Hz cap 12", "y20_c12", "#1f77b4"),
        ("20 Hz cap 15", "y20_c15", "#2ca02c"),
        ("20 Hz cap 21", "y20_c21", "#ff7f0e"),
        ("10 Hz 无上限", "y10_nocap", "#9467bd"),
        ("10 Hz cap 15", "y10_c15", "#8c564b")]


def load(tag):
    path = os.path.join(SD, "attrib305_2026-10-28_%s.jsonl" % tag)
    if not os.path.exists(path):
        raise SystemExit("缺证据文件：%s" % path)
    out = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                r = json.loads(line)
                out[int(r["seed"])] = r
    missing = sorted(set(SEEDS) - set(out))
    if missing:
        raise SystemExit("%s 缺 seed：%s" % (tag, missing[:5]))
    return out


def vec(d, key="pos_rms"):
    return np.array([float(d[s][key]) for s in SEEDS])


def main():
    arms = {tag: load(tag) for _, tag, _ in ARMS}

    # 图 1：发生率条形 + 阈值敏感性 + 箱线
    fig, ax = plt.subplots(1, 3, figsize=(16.5, 5.2))
    names = [n for n, _, _ in ARMS]
    colors = [c for _, _, c in ARMS]
    rates = [(vec(arms[t]) > FAIL_M).sum() for _, t, _ in ARMS]
    ax[0].bar(range(len(ARMS)), rates, color=colors)
    for i, v in enumerate(rates):
        ax[0].text(i, v + 0.25, "%d/50" % v, ha="center", fontsize=9)
    ax[0].set_xticks(range(len(ARMS)))
    ax[0].set_xticklabels(names, rotation=30, ha="right")
    ax[0].set_ylabel("pos_rms > %.0f m 的 seeds" % FAIL_M)
    ax[0].set_title("独立 seeds 51–100，30.5 g / 32 g 二进制 tv_32g_q1（core5）")
    ax[0].grid(axis="y", alpha=0.3)

    cap_x = [("off", "y20_nocap"), ("12", "y20_c12"), ("15", "y20_c15"), ("21", "y20_c21")]
    r2 = [(vec(arms[t]) > FAIL_M).sum() for _, t in cap_x]
    m2 = [float(vec(arms[t]).max()) for _, t in cap_x]
    a = ax[1]
    a.plot(range(len(cap_x)), r2, "o-", color="#1f77b4", label=">25 m 发生率 (seeds)")
    a.set_xlabel("触发时位置残差上限 [m]")
    a.set_ylabel("发生率", color="#1f77b4")
    a.set_xticks(range(len(cap_x)))
    a.set_xticklabels(["无上限"] + ["%s m" % s for s, _ in cap_x[1:]])
    b = a.twinx()
    b.plot(range(len(cap_x)), m2, "s--", color="#d62728", label="max pos_rms [m]")
    b.set_ylabel("max [m]", color="#d62728")
    a.set_title("阈值敏感性（20 Hz 臂）：收紧到 12 m 反而更差 => 15 贴着下界")
    a.grid(alpha=0.3)

    data = [vec(arms[t]) for _, t, _ in ARMS]
    ax[2].boxplot(data, tick_labels=names, patch_artist=True,
                  boxprops=dict(facecolor="#cfd8dc"), flierprops=dict(marker=".", ms=4))
    ax[2].axhline(FAIL_M, color="#d62728", ls=":", lw=1.2)
    ax[2].text(len(ARMS) - 0.4, FAIL_M + 0.6, "%.0f m 门" % FAIL_M, color="#d62728",
               fontsize=8, ha="right")
    plt.setp(ax[2].get_xticklabels(), rotation=30, ha="right")
    ax[2].set_ylabel("pos_rms [m]")
    ax[2].set_title("七臂分布（双峰时箱线只作形状参考，不当中位指标）")
    ax[2].grid(axis="y", alpha=0.3)
    fig.suptitle("§4.24r 速度子集 15 m 触发上限：独立 seeds 泛化与阈值扫描", y=1.0)
    fig.tight_layout()
    p1 = os.path.join(FIG, "nav_velsubset_capval_matrix_2026-10-28.png")
    fig.savefig(p1, dpi=140, bbox_inches="tight")
    plt.close(fig)

    # 图 2：逐 seed 配对 + CDF + 直方
    fig, ax = plt.subplots(1, 3, figsize=(16.5, 5.2))
    off = vec(arms["y20_nocap"])
    cap = vec(arms["y20_c15"])
    lim = max(float(np.maximum(off, cap).max()) * 1.05, FAIL_M * 1.4)
    ax[0].plot([0, lim], [0, lim], color="k", lw=0.8, ls="--")
    ax[0].scatter(off, cap, s=18, c="#2ca02c", alpha=0.8, label="20 Hz：无上限 vs cap 15")
    off10 = vec(arms["y10_nocap"])
    cap10 = vec(arms["y10_c15"])
    ax[0].scatter(off10, cap10, s=18, c="#9467bd", marker="^", alpha=0.7,
                  label="10 Hz：无上限 vs cap 15")
    ax[0].axvline(FAIL_M, color="#7f7f7f", ls=":", lw=1)
    ax[0].axhline(FAIL_M, color="#7f7f7f", ls=":", lw=1)
    ax[0].set_xlabel("无上限臂 pos_rms [m]")
    ax[0].set_ylabel("cap 15 m 臂 pos_rms [m]")
    ax[0].set_title("逐 seed 配对：对角线下方 = 上限带来改善")
    ax[0].legend(fontsize=8)
    ax[0].grid(alpha=0.3)

    for lbl, tag, col in ARMS:
        v = np.sort(vec(arms[tag]))
        ax[1].plot(v, np.arange(1, len(v) + 1) / len(v), color=col, lw=1.6, label=lbl)
    ax[1].axvline(FAIL_M, color="k", ls=":", lw=1)
    ax[1].set_xscale("log")
    ax[1].set_xlabel("pos_rms [m]（对数）")
    ax[1].set_ylabel("累积比例")
    ax[1].set_title("CDF：上限是否只改右尾")
    ax[1].legend(fontsize=7, ncol=2)
    ax[1].grid(alpha=0.3)

    bins = np.linspace(0, float(max(off.max(), cap.max(), 40.0)), 41)
    ax[2].hist(off, bins=bins, alpha=0.55, color="#d62728", label="20 Hz 无上限")
    ax[2].hist(cap, bins=bins, alpha=0.55, color="#2ca02c", label="20 Hz cap 15")
    ax[2].axvline(FAIL_M, color="k", ls=":", lw=1)
    ax[2].set_xlabel("pos_rms [m]")
    ax[2].set_ylabel("seeds")
    ax[2].set_title("直方：改善发生在哪一簇（双峰不得看中位）")
    ax[2].legend(fontsize=8)
    ax[2].grid(alpha=0.3)
    fig.suptitle("§4.24r 上限的两端效应：有害端被拦、有益端不被误杀", y=1.0)
    fig.tight_layout()
    p2 = os.path.join(FIG, "nav_velsubset_capval_dist_2026-10-28.png")
    fig.savefig(p2, dpi=140, bbox_inches="tight")
    plt.close(fig)

    print(json.dumps({"rate_gt25": dict(zip([t for _, t, _ in ARMS], [int(x) for x in rates])),
                      "cap_max": dict(zip([t for _, t in cap_x], [round(x, 3) for x in m2]))},
                     ensure_ascii=False, indent=1))
    print("written", p1)
    print("written", p2)


if __name__ == "__main__":
    main()
