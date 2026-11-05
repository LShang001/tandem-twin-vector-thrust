# -*- coding: utf-8 -*-
"""气压健康门控 A/B 可视化。
@file    plot_baro_health.py
@brief   箱线/CDF/散点/条形四联图，读 baro_health_ab_<tag>.json/jsonl。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 输入 sim-data/baro_health_ab_<tag>.jsonl 的逐 seed 结果（arm × gate 两臂），
         输出 figs/nav_baro_health_ab_<tag>.png 四联图：位置峰值箱线、CDF、
         baro 健康跳帧次数散点、滚转 p95 与末端真高对比。只读数据，不改固件。
@frame   NED 水平（pos_max 为水平位置峰值）；姿态取 roll 欧拉角
@units   m（pos_max/final_alt）、deg（max_roll）、次（bh_skips）
"""
import argparse
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


def load(tag):
    """读 sim-data/baro_health_ab_<tag>.jsonl，返回逐 seed 记录列表。

    每行一个字典，键含 arm/gate/pos_max/max_roll/final_alt/bh_skips 等指标；
    文件缺失时抛 OSError（说明该 tag 的 A/B 还没跑过）。
    """
    jl = os.path.join(DATA, f"baro_health_ab_{tag}.jsonl")
    rows = []
    with open(jl, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def finite(vals):
    """剔除 None、非数值项与 NaN/Inf，返回可直接进箱线图/分位数的 float 数组。"""
    return np.array([v for v in vals
                     if isinstance(v, (int, float)) and np.isfinite(v)], float)


def main():
    """命令行入口：读指定 tag 的 A/B 结果，画四联图并保存到 figs/。

    四联图依次为 pos_max 箱线（按 arm × gate 区分红蓝）、pos_max CDF、
    baro 健康跳帧次数与 pos_max 的散点、max_roll p95 与 final_alt 中位数对比；
    输出 PNG 的路径打印到 stdout，供报告引用。
    """
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="2026-10-27")
    args = ap.parse_args()
    rows = load(args.tag)
    arms = []
    for r in rows:
        if r["arm"] not in arms:
            arms.append(r["arm"])

    fig, axes = plt.subplots(2, 2, figsize=(13, 8.5))
    ax = axes[0, 0]
    data0, data1 = [], []
    for arm in arms:
        data0.append(finite([r.get("pos_max") for r in rows
                             if r["arm"] == arm and r["gate"] == 0]))
        data1.append(finite([r.get("pos_max") for r in rows
                             if r["arm"] == arm and r["gate"] == 1]))
    pos = np.arange(len(arms))
    bp0 = ax.boxplot(data0, positions=pos - 0.18, widths=0.3,
                     patch_artist=True, showfliers=True)
    bp1 = ax.boxplot(data1, positions=pos + 0.18, widths=0.3,
                     patch_artist=True, showfliers=True)
    for b in bp0["boxes"]:
        b.set_facecolor("#d95f5f")
    for b in bp1["boxes"]:
        b.set_facecolor("#4f8fc0")
    ax.set_yscale("symlog", linthresh=1.0)
    ax.set_xticks(pos)
    ax.set_xticklabels(arms, fontsize=9)
    ax.set_ylabel("pos_max (m, symlog)")
    ax.set_title("水平位置峰值（红=门控关, 蓝=门控开）")
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    for arm in arms:
        for gate, color in ((0, "#d95f5f"), (1, "#4f8fc0")):
            v = np.sort(finite([r.get("pos_max") for r in rows
                                if r["arm"] == arm and r["gate"] == gate]))
            if len(v):
                ax.plot(v, np.arange(1, len(v) + 1) / len(v),
                        color=color, alpha=0.75,
                        label=f"{arm} g{gate}")
    ax.set_xscale("symlog", linthresh=1.0)
    ax.set_xlabel("pos_max (m)")
    ax.set_ylabel("CDF")
    ax.set_title("位置峰值 CDF")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=6, ncol=2)

    ax = axes[1, 0]
    for arm in arms:
        for gate, color in ((0, "#d95f5f"), (1, "#4f8fc0")):
            rs = [r for r in rows if r["arm"] == arm and r["gate"] == gate]
            ax.scatter(finite([r.get("bh_skips") for r in rs]),
                       finite([r.get("pos_max") for r in rs]),
                       s=22, color=color, alpha=0.7,
                       label=f"{arm} g{gate}")
    ax.set_xlabel("baro health skips")
    ax.set_ylabel("pos_max (m)")
    ax.set_yscale("symlog", linthresh=1.0)
    ax.set_title("跳气压更新次数 vs 位置峰值")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=6, ncol=2)

    ax = axes[1, 1]
    w = 0.38
    roll0 = [np.percentile(finite([r.get("max_roll") for r in rows
                                   if r["arm"] == arm and r["gate"] == 0]), 95)
             if len(finite([r.get("max_roll") for r in rows
                            if r["arm"] == arm and r["gate"] == 0])) else np.nan
             for arm in arms]
    roll1 = [np.percentile(finite([r.get("max_roll") for r in rows
                                   if r["arm"] == arm and r["gate"] == 1]), 95)
             if len(finite([r.get("max_roll") for r in rows
                            if r["arm"] == arm and r["gate"] == 1])) else np.nan
             for arm in arms]
    alt0 = [np.median(finite([r.get("final_alt") for r in rows
                              if r["arm"] == arm and r["gate"] == 0]))
            if len(finite([r.get("final_alt") for r in rows
                           if r["arm"] == arm and r["gate"] == 0])) else np.nan
            for arm in arms]
    alt1 = [np.median(finite([r.get("final_alt") for r in rows
                              if r["arm"] == arm and r["gate"] == 1]))
            if len(finite([r.get("final_alt") for r in rows
                           if r["arm"] == arm and r["gate"] == 1])) else np.nan
            for arm in arms]
    ax.bar(pos - w / 2, roll0, w, color="#d95f5f", label="roll p95 g0")
    ax.bar(pos + w / 2, roll1, w, color="#4f8fc0", label="roll p95 g1")
    ax2 = ax.twinx()
    ax2.plot(pos, alt0, "o--", color="#8b3a3a", label="final_alt med g0")
    ax2.plot(pos, alt1, "o-", color="#1f4e79", label="final_alt med g1")
    ax.set_xticks(pos)
    ax.set_xticklabels(arms, fontsize=9)
    ax.set_ylabel("max_roll p95 (deg)")
    ax2.set_ylabel("final_alt median (m)")
    ax.set_title("滚转 p95 与末端真高")
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, fontsize=7)
    ax.grid(True, alpha=0.3)

    fig.suptitle(f"气压健康门控 A/B（{args.tag}）")
    fig.tight_layout()
    out = os.path.join(FIGS, f"nav_baro_health_ab_{args.tag}.png")
    fig.savefig(out, dpi=150)
    print(out)


if __name__ == "__main__":
    main()
