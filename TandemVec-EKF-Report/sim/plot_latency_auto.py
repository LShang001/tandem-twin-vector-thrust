# -*- coding: utf-8 -*-
"""GNSS 接收机延迟 × 自动延时估计网格 A/B 可视化。
@file    plot_latency_auto.py
@brief   默认 0~150 ms 网格与放宽 0~300 ms 网格在 0~350 ms 注入延迟下的
         误差、τ 估计分布、逐 seed 对照与理论欠补偿线。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 图型 (a) 折线+理论线 (b) τ 估计箱线 (c) 姿态 RMSE 折线
         (d) 逐 seed 成对散点 (e) 延迟×指标热图 (f) 拒帧计数柱状。
@frame   NED
@units   m / deg / ms / 次

用法：py -3.12 sim/plot_latency_auto.py --tag 2026-10-27
"""
import argparse
import json
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DATA = os.path.join(ROOT, "sim-data")
FIGS = os.path.join(ROOT, "figs")
ARMS = [("默认150ms", "#c0392b"), ("放宽300ms", "#2b7fc8")]
# 默认先验 15 ms + 100 ms 偏差门限 ⇒ 默认配置实际最多只能应用 115 ms
DEFAULT_APPLY_CEILING_MS = 115.0
PRIOR_MS = 15.0


def setup_font():
    for name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC"):
        try:
            font_manager.findfont(name, fallback_to_default=False)
            plt.rcParams["font.family"] = name
            break
        except Exception:
            continue
    plt.rcParams["axes.unicode_minus"] = False


def load(tag):
    rows = []
    with open(os.path.join(DATA, f"latency_auto_{tag}.jsonl"),
              encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def sel(rows, arm, lat, key):
    out = []
    for r in rows:
        if r["arm"] != arm or r["lat"] != lat:
            continue
        v = r.get(key)
        if isinstance(v, (int, float)) and math.isfinite(v):
            out.append(float(v))
    return np.array(out, float)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="2026-10-27")
    args = ap.parse_args()
    setup_font()
    rows = load(args.tag)
    lats = sorted({r["lat"] for r in rows})

    fig, ax = plt.subplots(2, 3, figsize=(16.5, 9.2))
    fig.suptitle("GNSS 接收机延迟 × 自动延时估计网格 A/B"
                 "（导弹 6DOF SIL，20 seeds/格，同源码只差编译期网格上界）",
                 fontsize=13, fontweight="bold")

    # (a) 位置 RMSE vs 注入延迟：两臂逐格差值直接标注
    a = ax[0, 0]
    md = [np.median(sel(rows, "默认150ms", l, "pos_rms")) for l in lats]
    mw = [np.median(sel(rows, "放宽300ms", l, "pos_rms")) for l in lats]
    a.plot(lats, md, "o-", color=ARMS[0][1], lw=1.8, ms=6, label=ARMS[0][0])
    a.plot(lats, mw, "s--", color=ARMS[1][1], lw=1.4, ms=5,
           label=f"{ARMS[1][0]}（与默认重合）")
    dmax = max(abs(x - y) for x, y in zip(md, mw))
    a.annotate(f"逐格中位最大差 {dmax:.2f} m\n⇒ 放宽网格无收益",
               xy=(0.45, 0.18), xycoords="axes fraction", fontsize=8,
               color="#333333")
    a.set_yscale("symlog", linthresh=1.0)
    a.set_xlabel("注入接收机延迟 [ms]")
    a.set_ylabel("位置 RMSE 中位 [m]")
    a.set_title("(a) 位置误差 vs 延迟（两臂重合）")
    a.legend(fontsize=7)
    a.grid(alpha=.3)

    # (b) τ 估计分布
    b = ax[0, 1]
    pos = np.arange(len(lats))
    data_def = [sel(rows, "默认150ms", l, "ad_tau_ms") for l in lats]
    data_wid = [sel(rows, "放宽300ms", l, "ad_tau_ms") for l in lats]
    box = [d for d in data_def + data_wid if d.size]
    labels = [str(l) for l in lats]
    if box:
        bp = b.boxplot(box, positions=list(pos - .18) + list(pos + .18),
                       widths=.32, patch_artist=True)
        for i, patch in enumerate(bp["boxes"]):
            patch.set_facecolor(ARMS[0][1] if i < len(lats) else ARMS[1][1])
            patch.set_alpha(.5)
    b.axhline(DEFAULT_APPLY_CEILING_MS, color="#4a4a4a", ls=":",
              label="默认可应用上限 115 ms")
    b.axhline(150, color=ARMS[0][1], ls="--", lw=1, label="默认网格上界 150 ms")
    b.axhline(300, color=ARMS[1][1], ls="--", lw=1, label="放宽网格上界 300 ms")
    b.set_xticks(pos)
    b.set_xticklabels(labels, fontsize=8)
    b.set_xlabel("注入延迟 [ms]")
    b.set_ylabel("估计 τ 分布 [ms]")
    b.set_title("(b) 估计 τ：SIL 中位≈0（左默认/右放宽）")
    b.legend(fontsize=6.5)
    b.grid(alpha=.3, axis="y")

    # (c) 姿态 RMSE
    c = ax[0, 2]
    for arm, col in ARMS:
        c.plot(lats, [np.median(sel(rows, arm, l, "nav_att_rmse"))
                      for l in lats], "o-", color=col, lw=1.8, ms=5, label=arm)
    c.set_xlabel("注入接收机延迟 [ms]")
    c.set_ylabel("姿态 RMSE 中位 [°]")
    c.set_title("(c) 姿态误差随延迟恶化（与补偿无关）")
    c.legend(fontsize=7)
    c.grid(alpha=.3)

    # (d) 逐 seed 成对散点（取 200 ms 与最大档）
    d = ax[1, 0]
    for lat, mk in ((lats[-5] if len(lats) >= 5 else lats[0], "o"),
                    (lats[-1], "s")):
        dd = sel(rows, "默认150ms", lat, "pos_rms")
        dw = sel(rows, "放宽300ms", lat, "pos_rms")
        if dd.size and dw.size and dd.shape == dw.shape:
            d.scatter(dd, dw, marker=mk, s=34, alpha=.75,
                      label=f"{lat} ms")
    lim = max([d.get_xlim()[1], d.get_ylim()[1]]) if d.collections else 1
    d.plot([0, lim], [0, lim], color="k", lw=.8, alpha=.4)
    d.set_xlabel("默认 150 ms 网格 位置 RMSE [m]")
    d.set_ylabel("放宽 300 ms 网格 位置 RMSE [m]")
    d.set_title("(d) 逐 seed：两臂落在对角线上=无差异")
    d.legend(fontsize=7)
    d.grid(alpha=.3)

    # (e) 延迟×臂 位置 RMSE 热图
    e = ax[1, 1]
    mat = np.array([[np.median(sel(rows, arm, l, "pos_rms"))
                     for l in lats] for arm, _ in ARMS])
    im = e.imshow(np.log10(np.maximum(mat, 1e-6)), aspect="auto", cmap="viridis")
    e.set_xticks(range(len(lats)))
    e.set_xticklabels(lats, fontsize=8)
    e.set_yticks(range(len(ARMS)))
    e.set_yticklabels([a for a, _ in ARMS], fontsize=8)
    for i in range(mat.shape[0]):
        for j in range(mat.shape[1]):
            e.text(j, i, f"{mat[i, j]:.1f}", ha="center", va="center",
                   fontsize=7.5, color="w")
    e.set_xlabel("注入延迟 [ms]")
    e.set_title("(e) 位置 RMSE 中位热图 [m]（log 着色）")
    fig.colorbar(im, ax=e, fraction=.046, label="log10(m)")

    # (f) 拒帧计数
    f = ax[1, 2]
    w = .38
    for i, (arm, col) in enumerate(ARMS):
        tot = [sum(sel(rows, arm, l, "outlier_rejects")) for l in lats]
        f.bar(np.arange(len(lats)) + (i - .5) * w, tot, w, color=col, label=arm)
    f.set_xticks(range(len(lats)))
    f.set_xticklabels(lats, fontsize=8)
    f.set_xlabel("注入延迟 [ms]")
    f.set_ylabel("整帧拒绝次数合计（20 seeds）")
    f.set_title("(f) 野值门拒绝：≥200 ms 才大量触发")
    f.legend(fontsize=7)
    f.grid(alpha=.3, axis="y")

    fig.tight_layout(rect=(0, 0, 1, 0.965))
    os.makedirs(FIGS, exist_ok=True)
    out = os.path.join(FIGS, f"nav_latency_auto_{args.tag}.png")
    fig.savefig(out, dpi=140)
    print("saved", out)


if __name__ == "__main__":
    main()
