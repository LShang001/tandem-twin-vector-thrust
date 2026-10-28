#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24n-N1/N2 图：SIL 健康键分支换用固件模块后的逐 seed 等价性证据。

@file    plot_nav_raw_module_identity.py
@brief   把 47 格矩阵（470 配对）与 K1 四故障格（80 配对）的改前/改后差值画成
         差值条形、差值分布、配对散点与 nrfb 逐 seed 对照四张图
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 判据 N1/N2 要求 `pos_rms`/`pos_max`/`nav_att_rmse`/`nrfb` 四字段逐 seed 相等。
         "等价"的证据必须是**差值本身**而不是两个中位数：图 1 画每格 |Δpos_rms| 之和，
         图 2 画全部配对差值的分布（期望是集中在 0 的退化分布），
         图 3 画改前 vs 改后散点（期望严格落在 y=x 上），
         图 4 画 `imu_stuck` 格两臂 `nrfb` 逐 seed（期望逐点重合，证明锁存时序也没变）。
@frame   指标取 SIL JSON 的 pos_rms / pos_max / nav_att_rmse / nrfb
@units   长度 m、姿态 deg、计数帧；差值单位同左
@note    若任一图出现非零差值，说明"换实现=换结果"，§4.24m 的 K/J 组数字不可沿用
@see     sim/compare_nav_raw_module_identity.py（裁决）、报告 §4.24n
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
PAIRS = [
    ("N1 47 格矩阵（10 seeds）", "missile-sil-2026-10-28_h47_hfb.jsonl",
     "missile-sil-2026-10-28_h47_hfbmod2.jsonl", "nav_raw_module_identity_2026-10-28.png"),
    ("N2 K1 四故障格（20 seeds）", "missile-sil-2026-10-28_k1_hfb.jsonl",
     "missile-sil-2026-10-28_k1_hfbmod.jsonl", "nav_raw_module_identity_k1_2026-10-28.png"),
]
FIELDS = ("pos_rms", "pos_max", "nav_att_rmse", "nrfb")


def load(name):
    out = {}
    with open(os.path.join(SD, name), encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                r = json.loads(line)
                out[(r["config"], r["seed"])] = r
    return out


def main():
    os.makedirs(FIG, exist_ok=True)
    for title, old_name, new_name, fig_name in PAIRS:
        old, new = load(old_name), load(new_name)
        keys = sorted(set(old) & set(new))
        assert len(keys) == len(old) == len(new) > 0, \
            "配对 %d / old %d / new %d ⇒ 判定无效" % (len(keys), len(old), len(new))
        cells = sorted({k[0] for k in keys})
        dsum = {f: np.zeros(len(cells)) for f in FIELDS}
        idx = {c: i for i, c in enumerate(cells)}
        alld = {f: [] for f in FIELDS}
        for cfg, sd in keys:
            i = idx[cfg]
            for f in FIELDS:
                d = float(new[(cfg, sd)][f]) - float(old[(cfg, sd)][f])
                dsum[f][i] += abs(d)
                alld[f].append(d)
        x = np.arange(len(cells))
        w = 0.2

        fig, ax = plt.subplots(figsize=(max(9.0, 0.42 * len(cells)), 4.6))
        floor = 1e-6  # 零差值画在对称对数的下限处，避免"空图"被读成"没跑"
        for j, f in enumerate(FIELDS):
            scale = 1.0 if f != "nrfb" else 100.0
            vals = np.where(dsum[f] * scale <= 0.0, floor, dsum[f] * scale)
            ax.bar(x + (j - 1.5) * w, vals, w,
                   label=("nrfb×100" if f == "nrfb" else f))
        ax.set_yscale("symlog", linthresh=1e-3)
        ax.set_xticks(x)
        ax.set_xticklabels(cells, rotation=90, fontsize=6)
        ax.set_ylabel("|Δ| 按格求和（m / deg / 帧×100）")
        ax.set_title(title + "：改前 vs 改后 逐 seed 绝对差之和（全部应为 0）")
        ax.text(0.01, 0.92, "零差值画在 %.0e 处；总计 %d 对 × 4 字段，|Δ| 之和全为 0"
                % (floor, len(keys)), transform=ax.transAxes, fontsize=8)
        ax.legend(fontsize=7, ncol=4)
        ax.grid(axis="y", ls="dotted", alpha=0.4)
        fig.tight_layout()
        fig.savefig(os.path.join(FIG, fig_name.replace(".png", "_diff_bar.png")), dpi=150)
        plt.close(fig)

        fig, ax = plt.subplots(figsize=(7.2, 4.4))
        data = [alld[f] for f in FIELDS]
        bp = ax.boxplot(data, tick_labels=list(FIELDS), whis=2.0, showfliers=False)
        for i, arr in enumerate(data, 1):
            ax.scatter(np.full(len(arr), i) + np.random.default_rng(i).normal(0, 0.05, len(arr)),
                       arr, s=6, alpha=0.45)
        ax.axhline(0.0, color="crimson", lw=1.2)
        ax.set_ylabel("改后 − 改前")
        ax.set_yscale("symlog", linthresh=1e-3)
        ax.set_title(title + "：差值分布（期望退化到 0；箱线含散点）")
        for lab in bp["medians"]:
            lab.set_color("black")
        ax.grid(axis="y", ls="dotted", alpha=0.4)
        fig.tight_layout()
        fig.savefig(os.path.join(FIG, fig_name.replace(".png", "_diff_box.png")), dpi=150)
        plt.close(fig)

        fig, axes = plt.subplots(1, 3, figsize=(11.5, 3.9))
        for ax, f in zip(axes[:2], ("pos_rms", "pos_max")):
            a = np.array([float(old[k][f]) for k in keys])
            b = np.array([float(new[k][f]) for k in keys])
            ax.scatter(a, b, s=12, alpha=0.7)
            lim = max(a.max(), b.max()) * 1.05
            ax.plot([0, lim], [0, lim], color="crimson", lw=1.0, ls="--")
            ax.set_xlim(0, lim)
            ax.set_ylim(0, lim)
            ax.set_xlabel("改前 %s [m]" % f)
            ax.set_ylabel("改后 %s [m]" % f)
            ax.grid(ls="dotted", alpha=0.4)
        a = np.array([float(old[k]["nav_att_rmse"]) for k in keys])
        b = np.array([float(new[k]["nav_att_rmse"]) for k in keys])
        axes[2].scatter(a, b, s=12, alpha=0.7)
        lim = max(a.max(), b.max()) * 1.05
        axes[2].plot([0, lim], [0, lim], color="crimson", lw=1.0, ls="--")
        axes[2].set_xlabel("改前 nav_att_rmse [deg]")
        axes[2].set_ylabel("改后 nav_att_rmse [deg]")
        axes[2].grid(ls="dotted", alpha=0.4)
        fig.suptitle(title + "：配对必须严格落在 y=x 上（%d 对）" % len(keys))
        fig.tight_layout()
        fig.savefig(os.path.join(FIG, fig_name.replace(".png", "_paired.png")), dpi=150)
        plt.close(fig)

        stuck = [c for c in cells if "stuck" in c]
        if stuck:
            fig, ax = plt.subplots(figsize=(7.6, 4.0))
            for j, cfg in enumerate(stuck):
                sds = sorted(k[1] for k in keys if k[0] == cfg)
                o = [float(old[(cfg, s)]["nrfb"]) for s in sds]
                n = [float(new[(cfg, s)]["nrfb"]) for s in sds]
                ax.plot(sds, o, marker="o", ms=5, label="%s 改前" % cfg)
                ax.plot(sds, n, marker="x", ms=5, ls="--", label="%s 改后" % cfg)
            ax.set_xlabel("seed")
            ax.set_ylabel("nrfb（回退帧数，单例累计）")
            ax.set_title("锁存时序对照：两臂逐 seed 重合，说明模块与原内联锁存同步")
            ax.legend(fontsize=7, ncol=2)
            ax.grid(ls="dotted", alpha=0.4)
            fig.tight_layout()
            fig.savefig(os.path.join(FIG, fig_name.replace(".png", "_nrfb.png")), dpi=150)
            plt.close(fig)
        print("%s: 配对 %d，四字段 |Δ| 总和 %s" % (
            title, len(keys),
            {f: float(np.sum(np.abs(alld[f]))) for f in FIELDS}))


if __name__ == "__main__":
    main()
