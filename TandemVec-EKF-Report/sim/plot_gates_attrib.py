#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24l 两门三臂归因配图（默认 / 仅 HF 否决 / HF+互锁，同二进制同核）。

@file    plot_gates_attrib.py
@brief   读 `sim-data/missile-sil-2026-10-28_g6{base,veto}.jsonl` 与 `_gates2.jsonl`，
         产出 §4.24l 归因与双峰复核所需图型
@module  TandemVec-EKF-Report/作图脚本（L2 研究设施）
@details 四张图：
         1 `nav_gates_attrib_bar.png`   三臂 × 10 格相对默认的百分比（横向条形）
         2 `nav_gates_vib2g_hist.png`   vib2g 三臂直方图（证明"失效簇 12/20→0/20"）
         3 `nav_gates_paired.png`       逐 seed 配对散点（veto/base、both/veto）
         4 `nav_gates_cdf.png`          vib2g 与 imu_stuck 的经验 CDF
         生成后必须逐张目视审查（空图、NaN、坐标尺度、标签）。
@frame   指标取 SIL JSON 的 pos_rms（全程位置 RMSE）
@units   长度 m；百分比相对默认臂
@note    双峰格（vib2g 默认臂）不得用中位差当"精度提升"，本图用直方图给出簇结构。
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

ARMS = [("默认", "g6base", "bs0", "#2b8a3e"),
        ("仅 HF 否决", "g6veto", "vf0", "#1971c2"),
        ("HF+互锁(废)", "gates2", "both", "#868e96")]


def load(tag, suf):
    out = {}
    with open(os.path.join(SD, f"missile-sil-2026-10-28_{tag}.jsonl"), encoding="utf-8") as f:
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
    data = {lab: load(tag, suf) for lab, tag, suf, _ in ARMS}
    cells = sorted(data["默认"])
    assert len(cells) == 10, f"格数 {len(cells)} != 10 => 键映射错，比对无效"
    for lab in data:
        assert set(data[lab]) == set(cells), lab
    cols = {lab: col for lab, _, _, col in ARMS}

    med = {lab: {c: float(np.median([r["pos_rms"] for r in data[lab][c].values()]))
                 for c in cells} for lab in data}

    # ---- 1. 相对默认的百分比条形 --------------------------------------------
    order = sorted(cells, key=lambda c: med["仅 HF 否决"][c] / med["默认"][c])
    y = np.arange(len(order))
    fig, ax = plt.subplots(figsize=(9.2, 5.4))
    for lab in ("仅 HF 否决", "HF+互锁(废)"):
        pct = [(med[lab][c] / med["默认"][c] - 1) * 100 for c in order]
        ax.barh(y + (0.2 if lab.startswith("HF+") else -0.2), pct, height=.38,
                color=cols[lab], alpha=.85,
                label=lab + ("（已撤回）" if lab.startswith("HF+") else ""))
    ax.axvline(0, color="k", lw=.9)
    ax.axvline(-50, color="crimson", ls=":", lw=1, label="P2 门限 −50%")
    ax.axvline(10, color="darkorange", ls=":", lw=1, label="P1/P4 门限 +10%/+5%")
    ax.set_yticks(y)
    ax.set_yticklabels(order, fontsize=8.5)
    ax.set_xlabel("位置 RMSE 中位相对默认 [%]（20 seeds）")
    ax.set_title("§4.24l 三臂归因：vib2g 的 −86% 全部来自 HF 否决；\n"
                 "互锁只在 2 个 IMU 故障格动数且方向为变差（8/10 格逐位不变）", fontsize=10.5)
    ax.legend(fontsize=8, loc="lower right")
    ax.grid(alpha=.3, axis="x")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_gates_attrib_bar.png"), dpi=140)
    plt.close(fig)

    # ---- 2. vib2g 三臂直方图（双峰 → 单峰）----------------------------------
    fig, axes = plt.subplots(1, 3, figsize=(12.6, 3.9))
    for ax, cell in zip(axes, ("vib2g_8p4g_icm_full", "imu_stuck_icm_full",
                               "latency100_icm_full")):
        for lab in ("默认", "仅 HF 否决", "HF+互锁(废)"):
            v = np.array([r["pos_rms"] for r in data[lab][cell].values()], float)
            ax.hist(v, bins=12, alpha=.55, color=cols[lab], label=f"{lab} 中位 {np.median(v):.2f}")
        ax.set_title(cell, fontsize=9)
        ax.set_xlabel("pos_rms [m]")
        ax.set_ylabel("seed 数")
        ax.grid(alpha=.3)
    axes[0].legend(fontsize=7.5)
    fig.suptitle("双峰复核：vib2g 默认臂是 8 个 2.9–3.5 m + 12 个 20.4–35.9 m 的空档双峰 => "
                 "「−86%」的正确读法是失效簇 12/20 → 0/20", fontsize=10.5)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_gates_vib2g_hist.png"), dpi=140)
    plt.close(fig)

    nfail = {lab: int(sum(1 for r in data[lab]["vib2g_8p4g_icm_full"].values()
                          if r["pos_rms"] > 10)) for lab in data}
    print("vib2g >10 m 的 seed 数:", nfail)

    # ---- 3. 逐 seed 配对散点 --------------------------------------------------
    fig, axes = plt.subplots(1, 2, figsize=(10.6, 4.4))
    for ax, (xa, xb, cell, ttl) in ((axes[0], ("默认", "仅 HF 否决", "vib2g_8p4g_icm_full",
                                               "HF 否决的作用（vib2g）")),
                                    (axes[1], ("仅 HF 否决", "HF+互锁(废)", "imu_stuck_icm_full",
                                               "再加互锁的作用（imu_stuck）"))):
        sa = data[xa][cell]
        sb = data[xb][cell]
        seeds = sorted(set(sa) & set(sb))
        assert len(seeds) == 20, f"{ttl} 配对 {len(seeds)} != 20 => 无效"
        a = np.array([sa[s]["pos_rms"] for s in seeds])
        b = np.array([sb[s]["pos_rms"] for s in seeds])
        ax.scatter(a, b, s=22, color=cols[xb], alpha=.8)
        lim = [min(a.min(), b.min()) * .85, max(a.max(), b.max()) * 1.15]
        ax.plot(lim, lim, "k--", lw=.9)
        better = int((b < a).sum())
        ax.set_xlabel(f"{xa} pos_rms [m]")
        ax.set_ylabel(f"{xb} pos_rms [m]")
        ax.set_title(f"{ttl}：{better}/20 seed 变好、{int((b > a).sum())}/20 变差", fontsize=9.5)
        ax.grid(alpha=.3)
    fig.suptitle("逐 seed 配对（同二进制 tv_hf6、同核 core6、20 seeds）", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_gates_paired.png"), dpi=140)
    plt.close(fig)

    # ---- 4. CDF --------------------------------------------------------------
    fig, axes = plt.subplots(1, 2, figsize=(10.6, 4.2))
    for ax, cell in zip(axes, ("vib2g_8p4g_icm_full", "imu_stuck_icm_full")):
        for lab in ("默认", "仅 HF 否决", "HF+互锁(废)"):
            v = np.sort([r["pos_rms"] for r in data[lab][cell].values()])
            ax.step(v, np.arange(1, len(v) + 1) / len(v), color=cols[lab], lw=1.8, label=lab)
        ax.set_title(cell, fontsize=9.5)
        ax.set_xlabel("位置 RMSE [m]")
        ax.set_ylabel("累积概率")
        ax.grid(alpha=.3)
    axes[0].legend(fontsize=8)
    fig.suptitle("三臂经验 CDF：左=HF 否决把整条分布搬回 3 m 级；右=互锁只把分布右移 ~0.6 m", fontsize=10.5)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_gates_cdf.png"), dpi=140)
    plt.close(fig)

    summary = {"cells": cells, "median": med,
               "vib2g_gt10m_seeds": nfail,
               "same_value3dp_veto_vs_both": {
                   c: int(sum(1 for s in data["仅 HF 否决"][c]
                              if data["仅 HF 否决"][c][s]["pos_rms"] ==
                              data["HF+互锁(废)"][c][s]["pos_rms"])) for c in cells}}
    with open(os.path.join(SD, "gates_attribution_figs_2026-10-28.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=1)
    print("JSON 值相同 seed 数（veto vs both，3 位小数口径）:", summary["same_value3dp_veto_vs_both"])
    print("figures ->", FIG)


if __name__ == "__main__":
    main()
