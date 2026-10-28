# -*- coding: utf-8 -*-
"""速度子集软恢复的可靠性交互矩阵可视化。
@file    plot_vel_reliability.py
@brief   故障组合（失联/振动/IMU NaN/无故障群体）off/on 双臂多类型图。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 图型：(a) 成对散点 (b) 分位条形 (c) 箱线 (d) CDF (e) seed×工况热图
         (f) 事件计数。读单一 jsonl（同二进制、含全部臂）。
@frame   NED
@units   m / 次

用法：py -3.12 sim/plot_vel_reliability.py --jsonl sil32g_velrec_rel8_2026-10-27.jsonl
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

CONDS = [("pop50", "无故障 50 seeds"),
         ("out5", "5 s 失联"),
         ("out20", "20 s 失联"),
         ("vib", "1 g/80 Hz 振动"),
         ("nan", "1 s IMU NaN")]
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


def vals(rows, arm, key):
    out = []
    for r in rows:
        if r["arm"] != arm:
            continue
        v = r.get(key)
        if isinstance(v, (int, float)) and math.isfinite(v):
            out.append(float(v))
    return np.array(out, float)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--jsonl", default="sil32g_velrec_rel8_2026-10-27.jsonl")
    ap.add_argument("--tag", default="2026-10-27")
    args = ap.parse_args()
    setup_font()
    rows = load(os.path.join(DATA, args.jsonl))

    fig, ax = plt.subplots(2, 3, figsize=(17.0, 9.2))
    fig.suptitle("GNSS 速度子集软恢复 × 故障组合：同二进制 off/on 可靠性矩阵",
                 fontsize=13.5, fontweight="bold")

    # (a) 逐 seed 配对散点
    a = ax[0, 0]
    lim = 1.0
    pts = []
    for key, label in CONDS:
        off = {r["seed"]: r["pos_rms"] for r in rows if r["arm"] == key + "_off"}
        on = {r["seed"]: r["pos_rms"] for r in rows if r["arm"] == key + "_on"}
        for s in sorted(set(off) & set(on)):
            pts.append((key, label, s, off[s], on[s]))
            lim = max(lim, off[s], on[s])
    mk = {"pop50": "o", "out5": "s", "out20": "^", "vib": "D", "nan": "x"}
    cl = {"pop50": "#111111", "out5": "#2f8f5b", "out20": "#7b3fa0",
          "vib": "#1f7a70", "nan": "#b03030"}
    seen = set()
    for key, label, s, vo, vn in pts:
        a.scatter(vo, vn, s=34, marker=mk[key], color=cl[key],
                  edgecolor="white", linewidth=0.4,
                  label=label if key not in seen else None)
        seen.add(key)
    a.plot([0, lim], [0, lim], "k--", lw=0.8)
    a.set_xlim(0, lim * 1.05)
    a.set_ylim(0, lim * 1.05)
    a.set_xlabel("位置 RMSE OFF (m)")
    a.set_ylabel("位置 RMSE ON (m)")
    a.set_title("(a) 逐 seed 配对（线下方=ON 更好）")
    a.legend(fontsize=7.5)
    a.grid(alpha=0.3)

    # (b) p95 / max 条形
    b = ax[0, 1]
    x = np.arange(len(CONDS))
    p95o, p95n, mxo, mxn = [], [], [], []
    for key, _ in CONDS:
        vo, vn = vals(rows, key + "_off", "pos_rms"), vals(rows, key + "_on", "pos_rms")
        f = lambda v, q: float(np.percentile(v, q)) if len(v) else np.nan
        p95o.append(f(vo, 95)); p95n.append(f(vn, 95))
        mxo.append(f(vo, 100)); mxn.append(f(vn, 100))
    b.bar(x - 0.2, p95o, 0.38, color=C_OFF, label="OFF p95")
    b.bar(x + 0.2, p95n, 0.38, color=C_ON, label="ON p95")
    b.plot(x - 0.2, mxo, "k_", ms=14, color=C_OFF, label="OFF max")
    b.plot(x + 0.2, mxn, "k_", ms=14, color=C_ON, label="ON max")
    b.set_xticks(x)
    b.set_xticklabels([l for _, l in CONDS], rotation=20, fontsize=8)
    b.set_yscale("log")
    b.set_ylabel("位置 RMSE (m, log)")
    b.set_title("(b) 分位与最坏值")
    b.legend(fontsize=7.5, ncol=2)
    b.grid(axis="y", alpha=0.3)

    # (c) 箱线
    c = ax[0, 2]
    data, labs, cols = [], [], []
    for key, label in CONDS:
        for arm, col in ((key + "_off", C_OFF), (key + "_on", C_ON)):
            v = vals(rows, arm, "pos_rms")
            if len(v):
                data.append(v)
                labs.append(label + (" off" if arm.endswith("off") else " on"))
                cols.append(col)
    bp = c.boxplot(data, tick_labels=labs, patch_artist=True, widths=0.62)
    for patch, col in zip(bp["boxes"], cols):
        patch.set_facecolor(col)
        patch.set_alpha(0.55)
    c.set_yscale("log")
    c.set_ylabel("位置 RMSE (m, log)")
    c.set_title("(c) 分布箱线")
    c.tick_params(axis="x", rotation=55, labelsize=6.5)
    c.grid(axis="y", alpha=0.3)

    # (d) CDF
    d = ax[1, 0]
    for key, label in CONDS:
        for arm, col, ls in ((key + "_off", C_OFF, "--"), (key + "_on", C_ON, "-")):
            v = np.sort(vals(rows, arm, "pos_rms"))
            if not len(v):
                continue
            cdf = np.arange(1, len(v) + 1) / len(v)
            d.step(np.concatenate([v, v[-1:]]), np.concatenate([cdf, [1.0]]),
                   color=col, ls=ls, where="post", lw=1.6,
                   label=f"{label} {'off' if arm.endswith('off') else 'on'}")
    d.set_xscale("log")
    d.set_xlabel("位置 RMSE (m, log)")
    d.set_ylabel("累积概率")
    d.set_title("(d) 累积分布")
    d.legend(fontsize=6.5, ncol=2)
    d.grid(alpha=0.3)

    # (e) seed × 工况 热图（log 位置误差）
    e = ax[1, 1]
    seeds = sorted({r["seed"] for r in rows})
    grid = np.full((len(CONDS) * 2, len(seeds)), np.nan)
    for i, (key, _) in enumerate(CONDS):
        for j, arm in enumerate((key + "_off", key + "_on")):
            m = {r["seed"]: r["pos_rms"] for r in rows if r["arm"] == arm}
            for k, s in enumerate(seeds):
                if s in m and m[s] > 0:
                    grid[i * 2 + j, k] = m[s]
    im = e.imshow(np.log10(np.where(grid > 0, grid, np.nan)), aspect="auto",
                  cmap="magma", vmin=0.0, vmax=2.8)
    e.set_yticks(np.arange(len(CONDS) * 2))
    e.set_yticklabels([f"{l} {'off' if j == 0 else 'on'}"
                       for l in [lab for _, lab in CONDS] for j in (0, 1)],
                      fontsize=6.5)
    e.set_xticks(np.arange(0, len(seeds), 5))
    e.set_xticklabels([str(seeds[i]) for i in range(0, len(seeds), 5)], fontsize=7)
    e.set_xlabel("seed")
    e.set_title("(e) log10 位置 RMSE 热图（空=无数据）")
    plt.colorbar(im, ax=e, fraction=0.03)

    # (f) 事件计数
    f = ax[1, 2]
    rej_o = [vals(rows, k + "_off", "outlier_rejects").sum() for k, _ in CONDS]
    rej_n = [vals(rows, k + "_on", "outlier_rejects").sum() for k, _ in CONDS]
    rec_n = [int((vals(rows, k + "_on", "vel_sub_rec") > 0).sum()) for k, _ in CONDS]
    rq_o = [int((vals(rows, k + "_off", "reacquire_resets") > 0).sum())
            for k, _ in CONDS]
    rq_n = [int((vals(rows, k + "_on", "reacquire_resets") > 0).sum())
            for k, _ in CONDS]
    x = np.arange(len(CONDS))
    f.bar(x - 0.3, rej_o, 0.28, color=C_OFF, label="拒帧总数 off")
    f.bar(x - 0.05, rej_n, 0.28, color=C_ON, label="拒帧总数 on")
    f.bar(x + 0.2, rq_o, 0.28, color="#8c8c8c", label="硬重锚 seeds off")
    f.bar(x + 0.45, rq_n, 0.28, color="#4a74a7", label="硬重锚 seeds on")
    for i, v in enumerate(rec_n):
        f.annotate(f"rec {v}", (i, max(rej_o[i], 1) * 1.05), fontsize=7.5,
                   ha="center", color="#2b7fc8")
    f.set_xticks(x)
    f.set_xticklabels([l for _, l in CONDS], rotation=20, fontsize=8)
    f.set_yscale("symlog")
    f.set_ylabel("计数（symlog）")
    f.set_title("(f) 门控/恢复事件计数")
    f.legend(fontsize=7)
    f.grid(axis="y", alpha=0.3)

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    out = os.path.join(FIGS, f"nav_vel_reliability_{args.tag}.png")
    fig.savefig(out, dpi=150)
    print("saved:", out)
    for key, label in CONDS:
        vo, vn = vals(rows, key + "_off", "pos_rms"), vals(rows, key + "_on", "pos_rms")
        if not len(vo) or not len(vn):
            print(f"{label:16s} 数据不全 off={len(vo)} on={len(vn)}")
            continue
        worse = sum(1 for r in rows if r["arm"] == key + "_on")
        print(f"{label:16s} n={len(vo)}/{len(vn)} off med {np.median(vo):7.2f} "
              f"p95 {np.percentile(vo,95):7.2f} max {vo.max():7.2f} | "
              f"on med {np.median(vn):7.2f} p95 {np.percentile(vn,95):7.2f} "
              f"max {vn.max():7.2f}")


if __name__ == "__main__":
    main()
