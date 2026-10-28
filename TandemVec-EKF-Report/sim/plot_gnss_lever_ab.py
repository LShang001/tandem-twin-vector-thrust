# -*- coding: utf-8 -*-
"""GNSS 天线杆臂端到端注入/补偿 A/B 可视化。
@file    plot_gnss_lever_ab.py
@brief   无注入 / 注入不补偿 / 正确补偿 / 反号补偿 四臂 × 多工况多类型图。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 图型 (a) 分组条形 (b) 箱线 (c) CDF (d) 逐 seed 成对散点
         (e) 工况×臂中位热图 (f) 速度误差条形。
@frame   NED
@units   m / m/s

用法：py -3.12 sim/plot_gnss_lever_ab.py --jsonl gnss_lever_2026-10-27.jsonl
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

CASES = [("clean", "标称 6.4 g"), ("8p4g", "8.4 g 高过载"),
         ("out5", "5 s 失联"), ("out20", "20 s 失联"),
         ("hz5", "GNSS 5 Hz"), ("lat200", "200 ms 延迟"),
         ("vib1g", "1 g/80 Hz 振动"),
         ("hover", "VTOL 悬停定点"), ("wind", "VTOL 侧风")]
VARIANTS = [("none", "无注入(基线)", "#4a4a4a"),
            ("inj_x2", "注入 2 m 不补偿", "#c0392b"),
            ("cmp_x2", "注入 2 m + 补偿", "#2b7fc8"),
            ("neg_x2", "注入 2 m + 反号", "#c8792b")]


def setup_font():
    for name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC"):
        try:
            font_manager.findfont(name, fallback_to_default=False)
            plt.rcParams["font.family"] = name
            break
        except Exception:
            continue
    plt.rcParams["axes.unicode_minus"] = False


def load(path):
    rows = []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def sel(rows, case, variant, key="pos_rms"):
    arm = f"{case}_{variant}"
    out, seeds = [], []
    for r in rows:
        if r["arm"] != arm:
            continue
        v = r.get(key)
        if isinstance(v, (int, float)) and math.isfinite(v):
            out.append(float(v))
            seeds.append(r.get("seed", 0))
    order = np.argsort(seeds)
    return np.array(out, float), np.array(seeds, int)[order]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--jsonl", default="gnss_lever_2026-10-27.jsonl")
    ap.add_argument("--tag", default="2026-10-27")
    args = ap.parse_args()
    setup_font()
    rows = load(os.path.join(DATA, args.jsonl))
    cases = [(c, lab) for c, lab in CASES
             if any(r["arm"].startswith(c + "_") for r in rows)]

    fig, ax = plt.subplots(2, 3, figsize=(16.5, 9.2))
    fig.suptitle("GNSS 天线杆臂端到端注入与固件补偿 A/B（天线在 IMU 前方 2 m，"
                 "同一二进制 runtime 开关）",
                 fontsize=13, fontweight="bold")

    # (a) 分组条形：各工况 pos_rms 中位
    a = ax[0, 0]
    w = 0.2
    x = np.arange(len(cases))
    for i, (var, lab, col) in enumerate(VARIANTS):
        med = [np.median(sel(rows, c, var)[0]) if sel(rows, c, var)[0].size
               else np.nan for c, _ in cases]
        a.bar(x + (i - 1.5) * w, med, w, label=lab, color=col)
    a.set_xticks(x)
    a.set_xticklabels([lab for _, lab in cases], fontsize=8, rotation=18)
    a.set_ylabel("位置 RMSE 中位 [m]")
    a.set_yscale("log")
    a.set_title("(a) 各工况：注入/补偿四臂对照（log 轴，200 ms 延迟臂量级不同）")
    a.legend(fontsize=7)
    a.grid(alpha=.3, axis="y")

    # (b) 箱线：参考工况分布
    b = ax[0, 1]
    ref = cases[0][0] if cases else "clean"
    box = [sel(rows, ref, var)[0] for var, _, _ in VARIANTS]
    box = [z for z in box if z.size]
    if box:
        bp = b.boxplot(box, tick_labels=[v for v, _, _ in VARIANTS],
                       patch_artist=True)
        for patch, (_, _, col) in zip(bp["boxes"], VARIANTS):
            patch.set_facecolor(col)
            patch.set_alpha(.55)
    b.set_ylabel("位置 RMSE [m]")
    b.set_title(f"(b) {cases[0][1] if cases else ref}：多 seed 分布")
    b.grid(alpha=.3, axis="y")
    b.tick_params(labelsize=8)

    # (c) CDF：标称 + 高过载
    c = ax[0, 2]
    for ci, (case, clab) in enumerate(cases[:2]):
        for var, vlab, col in VARIANTS:
            v, _ = sel(rows, case, var)
            if v.size:
                c.plot(np.sort(v), np.arange(1, v.size + 1) / v.size,
                       color=col, ls="-" if ci == 0 else "--",
                       lw=1.6, label=f"{clab}·{vlab}")
    c.set_xscale("log")
    c.set_xlabel("位置 RMSE [m]")
    c.set_ylabel("累积概率")
    c.set_title("(c) CDF：实线=参考工况 / 虚线=次工况")
    c.legend(fontsize=5.5, ncol=2)
    c.grid(alpha=.3)

    # (d) 逐 seed 成对散点：不补偿 vs 补偿（用第一个可用工况，不硬编码 clean）
    d = ax[1, 0]
    lim = 0.0
    ref_case = cases[0][0] if cases else "clean"
    inj, s1 = sel(rows, ref_case, "inj_x2")
    base_v, _ = sel(rows, ref_case, "none")
    for var, vlab, col in VARIANTS[2:]:
        cmp_, s2 = sel(rows, ref_case, var)
        if inj.size and cmp_.size and inj.shape == cmp_.shape:
            d.scatter(inj, cmp_, s=22, color=col, alpha=.8, label=vlab)
            lim = max(lim, float(np.max(np.concatenate([inj, cmp_]))))
    if base_v.size:
        d.axhline(np.median(base_v), color="#4a4a4a", ls=":",
                  label="基线中位")
    if lim > 0:
        d.plot([0, lim], [0, lim], color="k", lw=.8, alpha=.4)
    d.set_xlabel(f"{cases[0][1] if cases else ref_case}：注入不补偿 位置 RMSE [m]")
    d.set_ylabel("补偿后 位置 RMSE [m]")
    d.set_title("(d) 逐 seed：补偿能否回到基线（对角=同值）")
    d.legend(fontsize=7)
    d.grid(alpha=.3)

    # (e) 热图：工况×臂 中位（log10 着色）
    e = ax[1, 1]
    mat = np.full((len(cases), len(VARIANTS)), np.nan)
    for i, (case, _) in enumerate(cases):
        for j, (var, _, _) in enumerate(VARIANTS):
            v, _ = sel(rows, case, var)
            if v.size:
                mat[i, j] = np.median(v)
    with np.errstate(divide="ignore"):
        logm = np.log10(np.where(np.isfinite(mat), np.maximum(mat, 1e-6),
                                 np.nan))
    im = e.imshow(logm, aspect="auto", cmap="viridis")
    e.set_xticks(range(len(VARIANTS)))
    e.set_xticklabels([v for v, _, _ in VARIANTS], fontsize=7, rotation=20)
    e.set_yticks(range(len(cases)))
    e.set_yticklabels([lab for _, lab in cases], fontsize=8)
    for i in range(mat.shape[0]):
        for j in range(mat.shape[1]):
            if np.isfinite(mat[i, j]):
                e.text(j, i, f"{mat[i, j]:.2f}", ha="center", va="center",
                       fontsize=7,
                       color="w" if logm[i, j] < np.nanmax(logm) * .6 else "k")
    e.set_title("(e) 位置 RMSE 中位热图 [m]（log 着色）")
    fig.colorbar(im, ax=e, fraction=.046, label="log10(m)")

    # (f) 速度误差条形：ω×r 项
    f = ax[1, 2]
    for i, (var, lab, col) in enumerate(VARIANTS):
        med = []
        for c_, _ in cases:
            v, _ = sel(rows, c_, var, "nav_vel_rms")
            med.append(np.median(v) if v.size else np.nan)
        f.plot(range(len(cases)), med, "o-", color=col, lw=1.6, ms=5,
               label=lab)
    f.set_xticks(range(len(cases)))
    f.set_xticklabels([lab for _, lab in cases], fontsize=8, rotation=18)
    f.set_ylabel("速度 RMSE 中位 [m/s]")
    f.set_title("(f) 速度误差：天线运动项 ω×r")
    f.legend(fontsize=7)
    f.grid(alpha=.3)

    fig.tight_layout(rect=(0, 0, 1, 0.965))
    os.makedirs(FIGS, exist_ok=True)
    out = os.path.join(FIGS, f"nav_gnss_lever_{args.tag}.png")
    fig.savefig(out, dpi=140)
    print("saved", out)


if __name__ == "__main__":
    main()
