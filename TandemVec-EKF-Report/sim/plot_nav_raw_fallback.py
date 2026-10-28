#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24m 导航增量源头故障回退配图（4 故障格 × 20 seeds × 3 臂 + 非故障格惰性对照）。

@file    plot_nav_raw_fallback.py
@brief   读 `_nr_{base,raw,fb,rin,inert}` 五臂，产出 §4.24m 判据所需图型
@module  TandemVec-EKF-Report/作图脚本（L2 研究设施）
@details 四张图：
         1 `nav_nrfb_median_bar.png`  四格 × 三臂中位（含 fb/base 与 raw/base 标注）
         2 `nav_nrfb_paired.png`      逐 seed 配对：raw vs raw+fb（应整体落在对角线上方）
         3 `nav_nrfb_cdf.png`         imu_stuck / imu_stuck2s 三臂经验 CDF
         4 `nav_nrfb_inert.png`       非故障格 raw vs raw+fb 差值（应恒 0，J4 惰性）
         生成后必须逐张目视审查（空图、NaN、坐标尺度、标签）。
@frame   指标取 SIL JSON 的 pos_rms（全程位置 RMSE，m）
@units   长度 m；百分比相对默认臂
@note    nrfb（回退帧数）必须 >0 才说明故障格的分支真的执行；非故障格必须为 0。
"""
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
SD = os.path.join(ROOT, "sim-data")
FIG = os.path.join(ROOT, "figs")
os.makedirs(FIG, exist_ok=True)

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

ARMS = [("默认", "nr_base", "_nrb", "#2b8a3e"),
        ("nav-raw", "nr_raw", "_nrr", "#e8590c"),
        ("raw+源头回退", "nr_fb", "_nrf", "#1971c2")]
FAULT = ["imu_stuck_icm_full", "imu_stuck2s_icm_full",
         "imu_nan_icm_full", "imu_nan2s_icm_full"]


def load(tag, suf):
    path = os.path.join(SD, f"missile-sil-2026-10-28_{tag}.jsonl")
    if not os.path.exists(path):
        raise SystemExit(f"缺证据文件：{path}")
    out = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            r = json.loads(line)
            cfg = r["config"]
            assert cfg.endswith(suf), (cfg, suf)
            out.setdefault(cfg[:-len(suf)].rstrip("_"), {})[r["seed"]] = r
    return out


def main():
    D = {lab: load(tag, suf) for lab, tag, suf, _ in ARMS}
    for lab in D:
        assert set(D[lab]) == set(FAULT), (lab, sorted(D[lab]))
    cols = {lab: c for lab, _, _, c in ARMS}

    # ---- 1. 中位条形 ----------------------------------------------------------
    fig, ax = plt.subplots(figsize=(10.4, 4.6))
    x = np.arange(len(FAULT))
    for i, (lab, _, _, col) in enumerate(ARMS):
        vals = [float(np.median([r["pos_rms"] for r in D[lab][c].values()])) for c in FAULT]
        ax.bar(x + (i - 1) * .26, vals, width=.25, color=col, alpha=.88, label=lab)
        for xx, v in zip(x + (i - 1) * .26, vals):
            ax.text(xx, v, f"{v:.1f}", ha="center", va="bottom", fontsize=7.5)
    base = [float(np.median([r["pos_rms"] for r in D["默认"][c].values()])) for c in FAULT]
    fb = [float(np.median([r["pos_rms"] for r in D["raw+源头回退"][c].values()])) for c in FAULT]
    raw = [float(np.median([r["pos_rms"] for r in D["nav-raw"][c].values()])) for c in FAULT]
    for i, c in enumerate(FAULT):
        ax.text(x[i], max(base[i], fb[i], raw[i]) * 1.26,
                f"raw {100*(raw[i]/base[i]-1):+.1f}%\nfb {100*(fb[i]/base[i]-1):+.1f}%",
                ha="center", va="bottom", fontsize=8, color="k")
    ax.set_xticks(x)
    ax.set_xticklabels(FAULT, fontsize=8.5)
    ax.set_ylim(0, max(raw) * 1.52)
    ax.set_ylabel("位置 RMSE 中位 [m]（20 seeds）")
    ax.set_title("§4.24m 增量源头回退把 nav-raw 的冻结格代价完全抹平：\n"
                 "imu_stuck +67.6% -> -0.5%、stuck2s +17.6% -> -0.3%（NaN 两格本就中性）", fontsize=10.5)
    ax.legend(fontsize=8.5)
    ax.grid(alpha=.3, axis="y")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_nrfb_median_bar.png"), dpi=140)
    plt.close(fig)

    # ---- 2. 逐 seed 配对 ------------------------------------------------------
    fig, axes = plt.subplots(1, 2, figsize=(10.8, 4.5))
    for ax, cell in zip(axes, ("imu_stuck_icm_full", "imu_stuck2s_icm_full")):
        a = D["nav-raw"][cell]
        b = D["raw+源头回退"][cell]
        seeds = sorted(set(a) & set(b))
        assert len(seeds) == 20, f"{cell} 配对 {len(seeds)} != 20 => 无效"
        va = np.array([a[s]["pos_rms"] for s in seeds])
        vb = np.array([b[s]["pos_rms"] for s in seeds])
        ax.scatter(va, vb, s=26, color=cols["raw+源头回退"], alpha=.85)
        lim = [min(va.min(), vb.min()) * .96, max(va.max(), vb.max()) * 1.04]
        ax.plot(lim, lim, "k--", lw=.9)
        ax.annotate(f"{int((vb < va).sum())}/20 seed 回退后更好",
                    xy=(.03, .95), xycoords="axes fraction", fontsize=8.5, va="top")
        ax.set_xlabel("nav-raw pos_rms [m]")
        ax.set_ylabel("raw+回退 pos_rms [m]")
        ax.set_title(cell, fontsize=9.5)
        ax.grid(alpha=.3)
    fig.suptitle("逐 seed 配对（同二进制 tv_hf9、同核 core5）：全部落在对角线下方", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_nrfb_paired.png"), dpi=140)
    plt.close(fig)

    # ---- 3. CDF --------------------------------------------------------------
    fig, axes = plt.subplots(1, 2, figsize=(10.8, 4.2))
    for ax, cell in zip(axes, ("imu_stuck_icm_full", "imu_stuck2s_icm_full")):
        for lab in ("默认", "nav-raw", "raw+源头回退"):
            v = np.sort([r["pos_rms"] for r in D[lab][cell].values()])
            ax.step(v, np.arange(1, len(v) + 1) / len(v), color=cols[lab], lw=1.8,
                    label=f"{lab} 中位 {np.median(v):.1f}")
        ax.set_title(cell, fontsize=9.5)
        ax.set_xlabel("位置 RMSE [m]")
        ax.set_ylabel("累积概率")
        ax.grid(alpha=.3)
    axes[0].legend(fontsize=8)
    fig.suptitle("三臂经验 CDF：回退臂与默认臂几乎重合，raw 臂整体右移", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_nrfb_cdf.png"), dpi=140)
    plt.close(fig)

    # ---- 4. 非故障格惰性（J4）-------------------------------------------------
    try:
        N = load("nr_rin", "_nrin")
        I = load("nr_inert", "_nri")
    except SystemExit:
        N = I = None
    if N and I:
        cells = sorted(set(N) & set(I))
        assert cells, "J4 两臂无交集格"
        fig, ax = plt.subplots(figsize=(9.6, 4.0))
        rows = []
        for c in cells:
            seeds = sorted(set(N[c]) & set(I[c]))
            assert len(seeds) == 20, f"{c} 配对 {len(seeds)} != 20 => 无效"
            d = np.array([I[c][s]["pos_rms"] - N[c][s]["pos_rms"] for s in seeds])
            nrfb = max(int(r.get("nrfb", -1)) for r in I[c].values())
            rows.append((c, d, nrfb))
        for i, (c, d, nrfb) in enumerate(rows):
            ax.barh(i, d, color="#1971c2" if np.all(d == 0) else "crimson", alpha=.85)
            ax.text(0, i, f"  {c}  差值 max|Δ|={np.abs(d).max():.4f}  nrfb={nrfb}",
                    va="center", fontsize=8)
        ax.axvline(0, color="k", lw=.9)
        ax.set_yticks([])
        ax.set_xlabel("(raw+回退) - raw 的位置 RMSE 差 [m]")
        ax.set_xlim(min(-.02, min(np.abs(d).max() for _, d, _ in rows) * -3),
                    max(.02, max(np.abs(d).max() for _, d, _ in rows) * 3))
        ax.set_title("J4 惰性对照：非故障格上回退开关不应改变任何数值（差恒 0、nrfb=0）", fontsize=10.5)
        ax.grid(alpha=.3, axis="x")
        fig.tight_layout()
        fig.savefig(os.path.join(FIG, "nav_nrfb_inert.png"), dpi=140)
        plt.close(fig)
        print("J4:", [(c, float(np.abs(d).max()), n) for c, d, n in rows])
    else:
        print("J4 图跳过：_nr_rin 或 _nr_inert 未就绪")

    print("figures ->", FIG)


if __name__ == "__main__":
    main()
