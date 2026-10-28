# -*- coding: utf-8 -*-
"""Summarize and plot the step-maneuver navigation matrix.
@file    plot_maneuver_matrix.py
@brief   速度/机动矩阵可视化。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 绘制方向/条件失败率与位置/姿态分布。
@frame   N/A
@units   N/A
"""
import json
import os
import statistics as st

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

SRC = r"C:\Users\12631\AppData\Local\Temp\man2\results.jsonl"
OUT = r"D:\纵列双发矢量推力飞行器\TandemVec-EKF-Report\figs\nav_maneuver_matrix.png"
CONDS = ["nominal", "outage", "wind", "both"]
DIRS = ["N", "E", "S", "W", "NE"]
DISTS = [0, 3, 10, 25, 50]
COL = {"nominal": "tab:blue", "outage": "tab:red",
       "wind": "tab:orange", "both": "tab:purple"}


def nav_fail(r):
    p = r.get("pos_rms")
    return bool(p is None or p != p or p > 3.0 or (r.get("pos_max") or 0) > 10.0)


def stab_fail(r):
    return bool(abs((r.get("final_alt") or 0) - 10.0) > 3.0
                or (r.get("max_roll") or 0) > 45.0)


def load():
    rows = []
    if not os.path.exists(SRC):
        return rows
    for line in open(SRC, encoding="utf-8"):
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    return rows


def finite(v):
    return [x for x in v if x is not None and x == x]


def median_of(v):
    v = finite(v)
    return st.median(v) if v else float("nan")


def pct(v, q):
    v = sorted(finite(v))
    if not v:
        return float("nan")
    i = min(len(v) - 1, max(0, int(round(q * (len(v) - 1)))))
    return v[i]


def group(rows, cond, dist):
    return [r for r in rows if r["cond"] == cond and int(r["dist"]) == dist]


def main():
    rows = load()
    print(f"rows={len(rows)}")
    print("cond     n  navfail stabfail  pos_rms_med p95 max  roll_p95 max  peak_med")
    for c in CONDS:
        g = [r for r in rows if r["cond"] == c]
        ok = [r for r in g if r.get("pos_rms") is not None]
        if not ok:
            continue
        print(f"{c:8s} {len(g):5d} {sum(nav_fail(r) for r in g):7d} "
              f"{sum(stab_fail(r) for r in g):8d} "
              f"{median_of([r['pos_rms'] for r in ok]):9.3f} "
              f"{pct([r['pos_rms'] for r in ok], .95):6.3f} "
              f"{max(r['pos_rms'] for r in ok):6.3f} "
              f"{pct([r['max_roll'] for r in ok], .95):7.1f} "
              f"{max(r['max_roll'] for r in ok):6.1f} "
              f"{median_of([r['peak_speed'] for r in ok]):7.2f}")
    print("\nnominal end_target_err (median/p95/max) by distance")
    for d in DISTS:
        g = [r for r in group(rows, "nominal", d) if r.get("end_target_err") is not None]
        if g:
            e = [r["end_target_err"] for r in g]
            print(f"  {d:3d} m  {median_of(e):7.2f} {pct(e, .95):7.2f} {max(e):7.2f}")

    fig, ax = plt.subplots(2, 3, figsize=(18, 9.5))
    x = np.arange(len(DISTS))
    w = 0.19

    # A peak speed achieved
    for j, c in enumerate(CONDS):
        med = [median_of([r["peak_speed"] for r in group(rows, c, d)])
               for d in DISTS]
        lo = [pct([r["peak_speed"] for r in group(rows, c, d)], .25) for d in DISTS]
        hi = [pct([r["peak_speed"] for r in group(rows, c, d)], .75) for d in DISTS]
        ax[0, 0].bar(x + (j - 1.5) * w, med, w, color=COL[c], label=c)
        ax[0, 0].errorbar(x + (j - 1.5) * w, med,
                          yerr=[np.maximum(0, np.array(med) - np.array(lo)),
                                np.maximum(0, np.array(hi) - np.array(med))],
                          fmt="none", ecolor="k", lw=.7, capsize=2)
    ax[0, 0].set_xticks(x, [str(d) for d in DISTS])
    ax[0, 0].set_xlabel("commanded leg [m]")
    ax[0, 0].set_ylabel("peak speed [m/s] (1 s truth)")
    ax[0, 0].set_title("Achieved speed per leg (median, IQR whisker)")
    ax[0, 0].legend(fontsize=8)
    ax[0, 0].grid(alpha=.25)

    # B navigation error
    for j, c in enumerate(CONDS):
        med = [median_of([r["pos_rms"] for r in group(rows, c, d)])
               for d in DISTS]
        hi = [pct([r["pos_rms"] for r in group(rows, c, d)], .95) for d in DISTS]
        ax[0, 1].plot(x, med, "o-", color=COL[c], label=c)
        ax[0, 1].fill_between(x, med, hi, color=COL[c], alpha=.12)
    ax[0, 1].set_yscale("log")
    ax[0, 1].axhline(3.0, color="r", ls="--", lw=1, label="3 m fail gate")
    ax[0, 1].set_xticks(x, [str(d) for d in DISTS])
    ax[0, 1].set_xlabel("commanded leg [m]")
    ax[0, 1].set_ylabel("EKF-vs-truth pos RMS [m] (log)")
    ax[0, 1].set_title("Navigation error vs maneuver size")
    ax[0, 1].legend(fontsize=8)
    ax[0, 1].grid(alpha=.25)

    # C roll tail
    for j, c in enumerate(CONDS):
        p95 = [pct([r["max_roll"] for r in group(rows, c, d)], .95) for d in DISTS]
        mx = [max([r["max_roll"] for r in group(rows, c, d)] or [0]) for d in DISTS]
        ax[0, 2].plot(x, p95, "o-", color=COL[c], label=f"{c} p95")
        ax[0, 2].plot(x, mx, "x--", color=COL[c], alpha=.6)
    ax[0, 2].axhline(45.0, color="r", ls="--", lw=1, label="45 deg fail gate")
    ax[0, 2].set_xticks(x, [str(d) for d in DISTS])
    ax[0, 2].set_xlabel("commanded leg [m]")
    ax[0, 2].set_ylabel("max |roll| [deg]")
    ax[0, 2].set_title("Attitude tail (solid p95, dashed max)")
    ax[0, 2].legend(fontsize=7, ncol=2)
    ax[0, 2].grid(alpha=.25)

    # D failure counts by condition
    navf = [sum(nav_fail(r) for r in rows if r["cond"] == c) for c in CONDS]
    stabf = [sum(stab_fail(r) for r in rows if r["cond"] == c) for c in CONDS]
    xx = np.arange(len(CONDS))
    ax[1, 0].bar(xx - .18, navf, .36, label="nav fail", color="indianred")
    ax[1, 0].bar(xx + .18, stabf, .36, label="stability fail", color="steelblue")
    ax[1, 0].set_xticks(xx, CONDS)
    ax[1, 0].set_ylabel("runs")
    ax[1, 0].yaxis.get_major_locator().set_params(integer=True)
    ax[1, 0].set_title(f"Failures / {len(rows)//len(CONDS)} per condition")
    ax[1, 0].legend(fontsize=8)
    for i, (a, b) in enumerate(zip(navf, stabf)):
        ax[1, 0].text(i - .18, a + .3, str(a), ha="center", fontsize=8)
        ax[1, 0].text(i + .18, b + .3, str(b), ha="center", fontsize=8)

    # E heatmap: failure rate by direction x distance, worst condition "both"
    mat = np.zeros((len(DIRS), len(DISTS)))
    for i, dv in enumerate(DIRS):
        for j, d in enumerate(DISTS):
            g = [r for r in group(rows, "both", d) if r["dir"] == dv]
            mat[i, j] = (100.0 * sum(nav_fail(r) for r in g) / len(g)
                         if g else np.nan)
    im = ax[1, 1].imshow(mat, cmap="Reds", vmin=0, vmax=max(10, np.nanmax(mat)))
    ax[1, 1].set_xticks(range(len(DISTS)), [str(d) for d in DISTS])
    ax[1, 1].set_yticks(range(len(DIRS)), DIRS)
    ax[1, 1].set_xlabel("commanded leg [m]")
    ax[1, 1].set_title("Nav-fail rate [%], outage+wind")
    for i in range(len(DIRS)):
        for j in range(len(DISTS)):
            if not np.isnan(mat[i, j]):
                ax[1, 1].text(j, i, f"{mat[i, j]:.0f}", ha="center", va="center",
                              fontsize=8,
                              color="white" if mat[i, j] > np.nanmax(mat) * .6 else "k")
    fig.colorbar(im, ax=ax[1, 1], fraction=.046)

    # F worst seeds
    ok = [r for r in rows if r.get("pos_rms") is not None]
    for c in CONDS:
        g = [r for r in ok if r["cond"] == c]
        ax[1, 2].scatter([r["pos_max"] for r in g], [r["max_roll"] for r in g],
                         s=12, alpha=.55, color=COL[c], label=c)
    worst = sorted([r for r in ok if (r.get("max_roll") or 0) > 90
                    or (r.get("pos_max") or 0) > 1e6],
                   key=lambda r: ((r.get("pos_max") or 0) / 10.0
                                  + (r.get("max_roll") or 0) / 45.0),
                   reverse=True)[:3]
    for i, r in enumerate(worst):
        ax[1, 2].annotate(f"{r['tag']}", (r["pos_max"], r["max_roll"]),
                          fontsize=6, xytext=(4, (-14 if i % 2 else 6)),
                          textcoords="offset points")
    ax[1, 2].text(2e6, 95, "EKF numerical blow-up", fontsize=7, color="darkred")
    ax[1, 2].axvline(10.0, color="r", ls="--", lw=1)
    ax[1, 2].axhline(45.0, color="r", ls="--", lw=1)
    ax[1, 2].set_xscale("log")
    ax[1, 2].set_xlabel("pos_max [m] (log)")
    ax[1, 2].set_ylabel("max |roll| [deg]")
    ax[1, 2].set_title("Failure plane (red = gates)")
    ax[1, 2].legend(fontsize=7)
    ax[1, 2].grid(alpha=.25)

    fig.suptitle("Step maneuver matrix: 5 directions x 5 legs x 4 conditions x 40 seeds "
                 "(mag_clean + flow, step at 20 s, fault 25-30 s, 45 s)",
                 fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, .96))
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.savefig(OUT, dpi=150)
    print("saved", OUT)


if __name__ == "__main__":
    main()
