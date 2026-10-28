#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24p 图：速度子集软恢复对 30.5 g onset 台阶的作用（同二进制 runtime A/B）。

@file    plot_vel_subset_onset.py
@brief   画两臂发生率曲线、逐 seed 配对散点（含 45° 线）、簇迁移前后对照、
         以及拒帧/重锚自证据计数四张图
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 判据 V2/V4 要求"消除失效簇"与"残余误差"同时出现，不允许只报前者：
         图 1 发生率、图 2 逐 seed 配对（点全在 45° 线下方或线上 ⇒ 无一变差）、
         图 3 簇迁移（下簇不动、上簇从 ~27 m 压到 ~17 m）、图 4 自证据计数。
         双峰格的中位数无物理含义（§4.24），所以图 3 画的是"按基线簇分组后的各自中位"，
         不是整体中位。
@frame   NED；pos_rms [m]
@units   米 / 次 / g（781.8 N/g 名义换算）
@note    数据由 run_305g_onset_scan.py 产出；两臂只差 --gnss-vel-subset 1
@see     sim/check_vel_subset_onset.py（V1–V4 裁决）、报告 §4.24p
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
N_PER_G = 23900.0 / 30.5
BASE = {"onset": 20, "onsetfine": 20, "onset50": 50}
VEL = {"vel6sub": 20, "vel50sub": 50, "vellazy": 20}
CELLS = [(22500, 20), (23000, 20), (23500, 20), (23700, 20), (23800, 20), (23900, 20)]


def load(tags):
    out = {}
    for t in tags:
        path = os.path.join(SD, "attrib305_2026-10-28_%s.jsonl" % t)
        with open(path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line:
                    r = json.loads(line)
                    out[(int(r["thrust_n"]), int(r["seed"]))] = r
    return out


def main():
    base, vel = load(BASE.keys()), load(VEL.keys())
    os.makedirs(FIG, exist_ok=True)

    # 图 1：发生率两臂
    xs = [round(c[0] / N_PER_G, 2) for c in CELLS]
    rb, rv = [], []
    for thr, n in CELLS:
        b = [base[(thr, s)]["pos_rms"] for s in range(1, n + 1) if (thr, s) in base]
        v = [vel[(thr, s)]["pos_rms"] for s in range(1, n + 1) if (thr, s) in vel]
        assert len(b) == n and len(v) == n, "配对不足 %d: %d/%d" % (thr, len(b), len(v))
        rb.append(float(np.mean([float(x) > 25 for x in b])))
        rv.append(float(np.mean([float(x) > 25 for x in v])))
    fig, ax = plt.subplots(figsize=(8.4, 4.6))
    ax.step(xs, rb, where="mid", marker="o", color="crimson", label="基线（子集关）")
    ax.step(xs, rv, where="mid", marker="s", color="tab:blue", label="--gnss-vel-subset 1")
    ax.axvline(23900 / N_PER_G, color="gray", ls=":", lw=1)
    ax.text(23900 / N_PER_G - 0.05, 0.55, "50 seeds 档：基线 36% → 子集 0%", fontsize=8,
            rotation=90, va="center")
    ax.set_xlabel("峰值轴向过载 [g]")
    ax.set_ylabel("`>25 m` 发生率（20 seeds/档）")
    ax.set_ylim(-0.05, 0.65)
    ax.legend(fontsize=8)
    ax.grid(ls="dotted", alpha=0.4)
    ax.set_title("§4.24p V2/V4：速度子集压平 onset 台阶（发生率）")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_velsubset_onset_rate_2026-10-28.png"), dpi=150)
    plt.close(fig)

    # 图 2：30.5 g / 50 seeds 逐 seed 配对
    b50 = {s: float(base[(23900, s)]["pos_rms"]) for s in range(1, 51) if (23900, s) in base}
    v50 = {s: float(vel[(23900, s)]["pos_rms"]) for s in range(1, 51) if (23900, s) in vel}
    assert len(b50) == 50 and len(v50) == 50
    fig, ax = plt.subplots(figsize=(6.4, 6.0))
    ax.scatter(list(b50.values()), list(v50.values()), s=26, alpha=0.8)
    lim = max(max(b50.values()), max(v50.values())) * 1.05
    ax.plot([0, lim], [0, lim], color="crimson", ls="--", lw=1.0, label="y=x（不变）")
    ax.axhline(25, color="gray", ls=":", lw=1)
    ax.axvline(25, color="gray", ls=":", lw=1)
    ax.set_xlabel("基线 pos_rms [m]")
    ax.set_ylabel("速度子集 pos_rms [m]")
    ax.legend(fontsize=8)
    ax.grid(ls="dotted", alpha=0.4)
    worse = sum(1 for s in b50 if v50[s] > b50[s] + 1e-9)
    ax.set_title("30.5 g / 50 seeds 逐 seed 配对：变差 %d 个、变好 %d 个、不变 %d 个"
                 % (worse, sum(1 for s in b50 if v50[s] < b50[s] - 1e-9),
                    sum(1 for s in b50 if abs(v50[s] - b50[s]) <= 1e-9)))
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_velsubset_onset_paired_2026-10-28.png"), dpi=150)
    plt.close(fig)

    # 图 3：簇迁移（按基线簇分组）
    bad = [s for s in b50 if b50[s] > 25]
    good = [s for s in b50 if b50[s] <= 15]
    fig, ax = plt.subplots(figsize=(7.6, 4.4))
    data = [[b50[s] for s in good], [v50[s] for s in good],
            [b50[s] for s in bad], [v50[s] for s in bad]]
    ax.boxplot(data, tick_labels=["下簇 基线", "下簇 子集", "上簇 基线", "上簇 子集"],
               whis=2.0, showfliers=False)
    for i, d in enumerate(data, 1):
        ax.scatter(np.full(len(d), i) + np.random.default_rng(i).normal(0, 0.05, len(d)),
                   d, s=10, alpha=0.5)
    ax.axhline(25, color="crimson", ls="--", lw=1.0)
    ax.set_ylabel("pos_rms [m]")
    ax.set_title("簇迁移：下簇逐位不变，上簇从 ~27 m 压到 ~17 m（未回到 7 m）")
    ax.grid(axis="y", ls="dotted", alpha=0.4)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_velsubset_onset_clusters_2026-10-28.png"), dpi=150)
    plt.close(fig)

    # 图 4：自证据计数两臂
    keys = ("outlier_rejects", "reacquire_resets", "vel_sub_rec")
    labels = ["整帧拒帧", "硬重锚", "速度子集生效次数"]
    fig, ax = plt.subplots(figsize=(8.0, 4.4))
    med_b = {k: float(np.median([float(base[(23900, s)].get(k, 0.0)) for s in range(1, 51)
                                 if (23900, s) in base])) for k in keys}
    med_v = {k: float(np.median([float(vel[(23900, s)].get(k, 0.0)) for s in range(1, 51)
                                 if (23900, s) in vel])) for k in keys}
    w = 0.36
    for j, (k, lab) in enumerate(zip(keys, labels)):
        ax.bar([j - w / 2], [med_b[k]], w, color="crimson", alpha=0.85,
               label="基线（子集关）" if j == 0 else None)
        ax.bar([j + w / 2], [med_v[k]], w, color="tab:blue", alpha=0.85,
               label="速度子集开" if j == 0 else None)
        for x, val in ((j - w / 2, med_b[k]), (j + w / 2, med_v[k])):
            ax.text(x, max(val, 0.05), "%.0f" % val, ha="center", va="bottom", fontsize=8)
    ax.set_xticks(range(len(keys)))
    ax.set_xticklabels(labels)
    ax.set_yscale("symlog", linthresh=1.0)
    ax.set_ylabel("中位计数（对数轴，0 画在下限）")
    med_b = {k: float(np.median([float(base[(23900, s)].get(k, 0.0)) for s in range(1, 51)
                                 if (23900, s) in base])) for k in keys}
    med_v = {k: float(np.median([float(vel[(23900, s)].get(k, 0.0)) for s in range(1, 51)
                                 if (23900, s) in vel])) for k in keys}
    ax.set_title("自证据（中位）：拒帧 %.0f→%.0f，硬重锚 %.0f→%.0f，"
                 "子集生效 %.0f→%.0f（基线恒 0，说明通道确实被走到）"
                 % (med_b[keys[0]], med_v[keys[0]], med_b[keys[1]], med_v[keys[1]],
                    med_b[keys[2]], med_v[keys[2]]))
    ax.legend(fontsize=8)
    ax.grid(axis="y", ls="dotted", alpha=0.4)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_velsubset_onset_evidence_2026-10-28.png"), dpi=150)
    plt.close(fig)
    print(json.dumps({"cells": xs, "rate_base": rb, "rate_vel": rv,
                      "n_better": sum(1 for s in b50 if v50[s] < b50[s] - 1e-9),
                      "n_worse": worse, "n_same": sum(1 for s in b50 if abs(v50[s]-b50[s]) <= 1e-9)},
                     ensure_ascii=False))


if __name__ == "__main__":
    main()
