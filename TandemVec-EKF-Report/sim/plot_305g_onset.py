#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24o 图：30.5 g 条件性失稳的 onset 细扫（22→30.5 g）可视化。

@file    plot_305g_onset.py
@brief   把 `run_305g_onset_scan.py` 的 jsonl 画成发生率曲线、逐档直方图、
         簇中位随过载、散点（按是否应用 τ̂ 着色）与拒帧/重锚对照五张图
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 口径纪律（§4.24 已登记）：30.5 g 是**双峰**分布，中位数是两簇插值、
         没有物理含义 ⇒ 本脚本一律按"发生率 + 分簇"报告，
         中位只在"簇内"使用；判分叉看发生率是否在某档从 ≈0 跳到 ≈0.5 以上。
         第二张图（逐档直方图）是判双峰的**主证据**，发生率曲线只是它的汇总。
@frame   NED；pos_rms 为全程位置 RMSE [m]
@units   长度 m、推力 N、过载 g（按 781.8 N/g 线性换算，仅作横轴标记）
@note    若某档直方图出现 15–25 m 的中间填充，说明不是双吸引子而是连续过渡，
         §4.24 的"双吸引子"表述需修正（判据 O2 的反面）
@see     sim/run_305g_onset_scan.py（数据与预登记判据 O1–O3）
"""
import argparse
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
N_PER_G = 23900.0 / 30.5
BAD_M = 25.0
GOOD_M = 15.0


def load(path):
    rows = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pc", default="2026-10-28_onset")
    args = ap.parse_args()
    src = os.path.join(SD, "attrib305_%s.jsonl" % args.pc)
    rows = load(src)
    assert rows, "空数据 %s" % src
    os.makedirs(FIG, exist_ok=True)
    thrusts = sorted({int(r["thrust_n"]) for r in rows})
    by_t = {t: [r for r in rows if int(r["thrust_n"]) == t] for t in thrusts}
    for t in thrusts:
        assert len(by_t[t]) == len(by_t[thrusts[0]]), "各档 seeds 数不等，O1 不成立"
    n_seed = len(by_t[thrusts[0]])

    g_ax = [round(t / N_PER_G, 2) for t in thrusts]
    rate = [float(np.mean([float(r["pos_rms"]) > BAD_M for r in by_t[t]])) for t in thrusts]
    good_med = [float(np.median([float(r["pos_rms"]) for r in by_t[t]
                                 if float(r["pos_rms"]) <= GOOD_M])) if any(
        float(r["pos_rms"]) <= GOOD_M for r in by_t[t]) else np.nan for t in thrusts]
    bad_med = [float(np.median([float(r["pos_rms"]) for r in by_t[t]
                                if float(r["pos_rms"]) > BAD_M])) if any(
        float(r["pos_rms"]) > BAD_M for r in by_t[t]) else np.nan for t in thrusts]
    rej_med = [float(np.median([float(r.get("outlier_rejects", 0.0)) for r in by_t[t]]))
               for t in thrusts]
    reasn = [int(sum(1 for r in by_t[t] if float(r.get("reacquire_resets", 0.0)) > 0))
             for t in thrusts]
    appn = [int(sum(1 for r in by_t[t] if float(r.get("ad_app", 0.0)) > 0)) for t in thrusts]

    # 图 1：发生率 + 簇中位（双轴）
    fig, ax = plt.subplots(figsize=(8.6, 4.8))
    ax.step(g_ax, rate, where="mid", color="crimson", lw=1.8, marker="o",
            label=">25 m 发生率")
    ax.set_xlabel("峰值轴向过载 [g]（按 781.8 N/g 换算）")
    ax.set_ylabel("发生率（%d seeds/档）" % n_seed, color="crimson")
    ax.set_ylim(-0.05, 1.05)
    ax.grid(ls="dotted", alpha=0.4)
    ax2 = ax.twinx()
    ax2.plot(g_ax, good_med, "o-", color="tab:green", label="正常簇中位 (<=15 m)")
    ax2.plot(g_ax, bad_med, "s--", color="tab:orange", label="失效簇中位 (>25 m)")
    ax2.set_ylabel("簇内 pos_rms 中位 [m]")
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, fontsize=8, loc="center left")
    ax.set_title("§4.24o 分叉位置：发生率起跳档 vs 两簇中位（不报整体中位）")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_305g_onset_rate_%s.png" % args.pc), dpi=150)
    plt.close(fig)

    # 图 2：逐档直方图（双峰判定的主证据）
    ncols = 5
    nrows = (len(thrusts) + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, figsize=(15.0, 3.0 * nrows))
    allv = np.array([float(r["pos_rms"]) for r in rows])
    bins = np.linspace(0.0, max(allv.max() * 1.02, 40.0), 41)
    for i, (t, ax) in enumerate(zip(thrusts, np.atleast_1d(axes).ravel())):
        v = np.array([float(r["pos_rms"]) for r in by_t[t]])
        ax.hist(v, bins=bins, color="steelblue")
        ax.axvline(BAD_M, color="crimson", ls="--", lw=1.0)
        ax.set_title("%.2f g / %d N  失效 %d/%d" % (t / N_PER_G, t,
                                                    int((v > BAD_M).sum()), len(v)),
                     fontsize=9)
        ax.set_yticks([])
    for ax in np.atleast_1d(axes).ravel()[len(thrusts):]:
        ax.axis("off")
    fig.suptitle("逐档 pos_rms 直方图（双峰=两吸引子；中间填充=连续过渡，需修正表述）")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_305g_onset_hist_%s.png" % args.pc), dpi=130)
    plt.close(fig)

    # 图 3：散点 pos_rms vs 推力，按 ad_app>0 着色（τ̂ 是否真被应用）
    fig, ax = plt.subplots(figsize=(8.6, 4.8))
    for flag, lab, col in ((True, "ad_app>0（τ̂ 被应用过）", "tab:red"),
                           (False, "ad_app=0（从未应用）", "tab:blue")):
        xs, ys = [], []
        for t in thrusts:
            for r in by_t[t]:
                if (float(r.get("ad_app", 0.0)) > 0) == flag:
                    xs.append(t / N_PER_G)
                    ys.append(float(r["pos_rms"]))
        ax.scatter(xs, ys, s=16, alpha=0.7, color=col, label=lab)
    ax.axhline(BAD_M, color="crimson", ls="--", lw=1.0)
    ax.set_xlabel("峰值轴向过载 [g]")
    ax.set_ylabel("pos_rms [m]")
    ax.set_yscale("symlog", linthresh=1.0)
    ax.legend(fontsize=8)
    ax.grid(ls="dotted", alpha=0.4)
    ax.set_title("失效与「τ̂ 是否被应用」的关系（若 ad_app=0 的点也失效，则应用不是必要条件）")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_305g_onset_scatter_%s.png" % args.pc), dpi=150)
    plt.close(fig)

    # 图 4：拒帧 / 重锚 / 应用计数随过载（判据 O3 的时间顺序）
    fig, ax = plt.subplots(figsize=(8.6, 4.6))
    ax.plot(g_ax, rej_med, "o-", color="tab:purple", label="outlier_rejects 中位")
    ax.plot(g_ax, reasn, "s-", color="tab:olive", label="发生硬重锚的 seed 数")
    ax.plot(g_ax, appn, "^-", color="tab:blue", label="ad_app>0 的 seed 数")
    ax2 = ax.twinx()
    ax2.plot(g_ax, rate, "--", color="crimson", lw=1.4, label=">25 m 发生率（右轴）")
    ax.set_xlabel("峰值轴向过载 [g]")
    ax.set_ylabel("计数 / seed 数")
    ax2.set_ylabel("发生率", color="crimson")
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, fontsize=8, loc="upper left")
    ax.grid(ls="dotted", alpha=0.4)
    ax.set_title("判据 O3：拒帧首次非零的档是否晚于发生率起跳档")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_305g_onset_gates_%s.png" % args.pc), dpi=150)
    plt.close(fig)

    # 图 5：箱线（每档分布形状，含双峰时的"哑铃"形态）
    fig, ax = plt.subplots(figsize=(9.2, 4.6))
    data = [np.array([float(r["pos_rms"]) for r in by_t[t]]) for t in thrusts]
    ax.boxplot(data, tick_labels=["%.1f" % g for g in g_ax], whis=2.0, showfliers=True)
    ax.axhline(BAD_M, color="crimson", ls="--", lw=1.0)
    ax.set_ylabel("pos_rms [m]")
    ax.set_xlabel("峰值轴向过载 [g]")
    ax.set_title("逐档箱线（与直方图互为汇总/明细，双峰时箱线会误导，以直方图为准）")
    ax.grid(axis="y", ls="dotted", alpha=0.4)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_305g_onset_box_%s.png" % args.pc), dpi=150)
    plt.close(fig)

    print(json.dumps({"thrust_N": thrusts, "g": g_ax, "rate_gt25": rate,
                      "good_med": [round(x, 2) if x == x else None for x in good_med],
                      "bad_med": [round(x, 2) if x == x else None for x in bad_med],
                      "rej_med": rej_med, "reanchor_seeds": reasn,
                      "ad_app_seeds": appn, "seeds_per_cell": n_seed},
                     ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
