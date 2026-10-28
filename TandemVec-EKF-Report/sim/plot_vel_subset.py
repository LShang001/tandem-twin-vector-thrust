# -*- coding: utf-8 -*-
"""GNSS 速度子集软恢复 A/B 可视化。
@file    plot_vel_subset.py
@brief   同二进制 off/on 双臂对比图：条形、成对散点、箱线、CDF。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 读 sim-data/sil32g_velrec_*.jsonl（每行一次 SIL），按 arm 分组配对。
@frame   NED
@units   m / 次 / deg

用法：py -3.12 sim/plot_vel_subset.py --tag 2026-10-27
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

PAIRS = [("tail", "tail_off", "tail_on"),
         ("hz20", "hz20_off", "hz20_on"),
         ("spoof20", "spoof20_off", "spoof20_on"),
         ("spoof30", "spoof30_off", "spoof30_on"),
         ("nominal", "nominal_off", "nominal_on"),
         ("pop50", "pop50_off", "pop50_on")]
C_OFF = "#c8792b"
C_ON = "#2b7fc8"


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


def finite(rows, key):
    out = []
    for r in rows:
        v = r.get(key)
        if isinstance(v, (int, float)) and math.isfinite(v):
            out.append(v)
    return np.array(out, float)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="2026-10-27")
    ap.add_argument("--extra-jsonl", nargs="*", default=[],
                    help="额外的 jsonl 证据文件（如 pop50 群体复核），合并进同一分组")
    args = ap.parse_args()
    setup_font()
    rows = load(os.path.join(DATA, f"sil32g_velrec_{args.tag}.jsonl"))
    for extra in args.extra_jsonl:
        path = extra if os.path.isabs(extra) else os.path.join(DATA, extra)
        rows += load(path)
    by_arm = {}
    for r in rows:
        by_arm.setdefault(r["arm"], {})[r["seed"]] = r

    fig, ax = plt.subplots(2, 2, figsize=(13.5, 9.0))
    fig.suptitle("GNSS 速度子集软恢复 A/B（32 g 导弹 SIL，30.5 g / GNSS 10 Hz）",
                 fontsize=13, fontweight="bold")

    # (a) 中位位置 RMSE：off vs on
    a = ax[0, 0]
    x = np.arange(len(PAIRS))
    med_off, med_on, err_on = [], [], []
    for _, off, on in PAIRS:
        ro = list(by_arm.get(off, {}).values())
        rn = list(by_arm.get(on, {}).values())
        vo, vn = finite(ro, "pos_rms"), finite(rn, "pos_rms")
        med_off.append(np.median(vo) if len(vo) else np.nan)
        med_on.append(np.median(vn) if len(vn) else np.nan)
        err_on.append((np.percentile(vn, 90) - np.median(vn)) if len(vn) else 0)
    a.bar(x - 0.18, med_off, 0.34, color=C_OFF, label="vel-subset OFF")
    a.bar(x + 0.18, med_on, 0.34, color=C_ON, label="vel-subset ON",
          yerr=err_on, capsize=4)
    a.set_xticks(x)
    a.set_xticklabels([p[0] for p in PAIRS])
    a.set_ylabel("位置 RMSE 中位 (m)")
    a.set_title("(a) 各工况中位位置误差（误差棒=ON 臂 p90−中位）")
    a.legend()
    a.grid(axis="y", alpha=0.3)

    # (b) 成对散点：每个 seed 的 pos_max off vs on
    b = ax[0, 1]
    lim = 1.0
    pairs_pts = []
    for label, off, on in PAIRS:
        for s in sorted(set(by_arm.get(off, {})) & set(by_arm.get(on, {}))):
            vo = by_arm[off][s].get("pos_max", float("nan"))
            vn = by_arm[on][s].get("pos_max", float("nan"))
            if not (math.isfinite(vo) and math.isfinite(vn)):
                continue
            pairs_pts.append((label, s, vo, vn))
            lim = max(lim, vo, vn)
    marker = {"tail": "o", "hz20": "P", "spoof20": "s", "spoof30": "^",
              "nominal": "D", "pop50": "x"}
    color = {"tail": "#111111", "hz20": "#1f7a70", "spoof20": "#7b3fa0",
             "spoof30": "#2f8f5b", "nominal": "#8c8c8c", "pop50": "#b03030"}
    seen = set()
    for label, s, vo, vn in pairs_pts:
        b.scatter(vo, vn, s=42, marker=marker[label], color=color[label],
                  edgecolor="white", linewidth=0.5,
                  label=label if label not in seen else None)
        seen.add(label)
    b.plot([0, lim], [0, lim], color="k", ls="--", lw=0.8)
    b.fill_between([0, lim], [0, lim], [lim, lim], alpha=0.07, color="green")
    b.fill_between([0, lim], [0, 0], [lim, lim], alpha=0.07, color="red")
    b.set_xlim(0, lim * 1.05)
    b.set_ylim(0, lim * 1.05)
    b.set_xlabel("pos_max OFF (m)")
    b.set_ylabel("pos_max ON (m)")
    b.set_title("(b) 逐 seed 配对：线下方=ON 更好")
    b.legend(fontsize=8)
    b.grid(alpha=0.3)

    # (c) 野值拒帧数箱线
    c = ax[1, 0]
    box_data, box_lab, box_col = [], [], []
    for _, off, on in PAIRS:
        for arm, col in ((off, C_OFF), (on, C_ON)):
            v = finite(list(by_arm.get(arm, {}).values()), "outlier_rejects")
            if len(v):
                box_data.append(v)
                box_lab.append(arm)
                box_col.append(col)
    bp = c.boxplot(box_data, labels=box_lab, patch_artist=True, widths=0.6)
    for patch, col in zip(bp["boxes"], box_col):
        patch.set_facecolor(col)
        patch.set_alpha(0.55)
    c.set_ylabel("outlier_rejects (次)")
    c.set_title("(c) 物理野值门拒绝次数分布")
    c.tick_params(axis="x", rotation=35, labelsize=8)
    c.grid(axis="y", alpha=0.3)

    # (d) 尾部 seed 的 pos_rms CDF
    d = ax[1, 1]
    for _, off, on in PAIRS:
        for arm, col, ls in ((off, C_OFF, "--"), (on, C_ON, "-")):
            v = np.sort(finite(list(by_arm.get(arm, {}).values()), "pos_rms"))
            if not len(v):
                continue
            cdf = (np.arange(1, len(v) + 1)) / len(v)
            d.step(np.concatenate([v, v[-1:]]),
                   np.concatenate([cdf, [1.0]]), color=col, ls=ls,
                   where="post", lw=1.8,
                   label=f"{arm} (n={len(v)})")
    d.set_xlabel("位置 RMSE (m)")
    d.set_ylabel("累积概率")
    d.set_title("(d) 位置误差 CDF（实线=ON）")
    d.legend(fontsize=8)
    d.grid(alpha=0.3)

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    out = os.path.join(FIGS, f"nav_vel_subset_{args.tag}.png")
    fig.savefig(out, dpi=150)
    print("saved:", out)
    for label, off, on in PAIRS:
        ro = list(by_arm.get(off, {}).values())
        rn = list(by_arm.get(on, {}).values())
        print(f"{label:8s} n={len(ro)}/{len(rn)} "
              f"pos_rms med {np.median(finite(ro,'pos_rms')):.3f}->"
              f"{np.median(finite(rn,'pos_rms')):.3f} "
              f"pos_max max {finite(ro,'pos_max').max() if len(finite(ro,'pos_max')) else float('nan'):.2f}->"
              f"{finite(rn,'pos_max').max() if len(finite(rn,'pos_max')) else float('nan'):.2f} "
              f"rej med {np.median(finite(ro,'outlier_rejects')):.0f}->"
              f"{np.median(finite(rn,'outlier_rejects')):.0f} "
              f"velsub max {finite(rn,'vel_sub_rec').max() if len(finite(rn,'vel_sub_rec')) else 0:.0f} "
              f"roll max {finite(rn,'max_roll').max() if len(finite(rn,'max_roll')) else float('nan'):.2f}")


if __name__ == "__main__":
    main()
