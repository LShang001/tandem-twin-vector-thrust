#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24h 冻结格代价面配图（40 seeds × 4 格 × 3 臂 + seed1 故障窗口分解）。

@file    plot_fz_cost.py
@brief   读 `sim-data/missile-sil-2026-10-28_fz{base,raw0,raw1}.jsonl` 与
         `stuck_tr_{base,raw}.csv`，产出 §4.24h 代价确认与窗口分解所需图型
@module  TandemVec-EKF-Report/作图脚本（L2 研究设施）
@details 六张图：paired 逐 seed 配对散点（位置/峰值/姿态）、box 三臂×四格箱线、
         cdf imu_stuck 累积分布 + raw/默认比值、window 故障前/窗内/窗后 RMS、
         timeseries seed1 误差时序 + ad_app 累计、heat 窗口×臂相对变化。
         生成后必须逐张目视审查（空图、NaN、坐标尺度、标签）。
@frame   NED 水平面：误差取 hypot(ekf_n-true_n, ekf_e-true_e)
@units   长度 m、姿态 deg、时间 s

输入（TandemVec-EKF-Report/sim-data/）：
  missile-sil-2026-10-28_fz{base,raw0,raw1}.jsonl  4 格 × 40 seeds
  stuck_tr_{base,raw}.csv                          seed1 200 Hz 追踪
输出 figs/nav_fz_cost_{paired,box,cdf,window,timeseries,heat}.png

误差口径：不用 ctl_alt-truth_alt 作垂向项（两者相差约 50 m 定义量，
会把洁净段伪造成 51 m 误差）。
"""
import csv
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

ARMS = [("默认", "fzbase", "#2b8a3e"), ("nav-raw", "fzraw0", "#e8590c"),
        ("raw+互锁(废)", "fzraw1", "#868e96")]
KEYS = {"pos_rms": "位置 RMSE [m]", "pos_max": "位置峰值 [m]",
        "nav_att_rmse": "姿态 RMSE [deg]"}


def load(path, arm):
    out = []
    with open(os.path.join(SD, path), encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            r = json.loads(line)
            r["_arm"] = arm
            out.append(r)
    return out


def main():
    recs = {}
    for label, tag, col in ARMS:
        recs[label] = load(f"missile-sil-2026-10-28_{tag}.jsonl", label)
    cells = sorted({r["config"].rsplit("_fz", 1)[0] for r in recs["默认"]})
    if len(cells) != 4:
        raise SystemExit(f"格数异常：{cells}")
    print("格:", cells)

    def pick(recs, cell, key):
        v = [r.get(key) for r in recs
             if r["config"].rsplit("_fz", 1)[0] == cell and r.get("_seed_ok", True)]
        return np.array([x for x in v if isinstance(x, (int, float)) and np.isfinite(x)], float)

    # ---- 1. 配对散点：每 seed 的 raw/默认 比值 -------------------------------
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.1))
    for ax, key in zip(axes, ["pos_rms", "pos_max", "nav_att_rmse"]):
        for cell in cells:
            a = pick(recs["默认"], cell, key)
            b = pick(recs["nav-raw"], cell, key)
            if len(a) != len(b) or not len(a):
                continue
            ax.scatter(a, b, s=13, alpha=.72, label=cell)
        lim = [min(1.0, a.min() * .9), max(a.max(), b.max()) * 1.1]
        ax.plot(lim, lim, "k--", lw=.9)
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.set_xlabel("默认 " + KEYS[key])
        ax.set_ylabel("nav-raw " + KEYS[key])
        ax.set_title(f"逐 seed 配对（40 seeds）：{KEYS[key]}")
        ax.grid(alpha=.3, which="both")
    axes[0].legend(fontsize=7, ncol=2)
    fig.suptitle("§4.24h 冻结/NaN 格：nav-raw 相对默认是「坏 vs 更坏」还是真回归？", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_fz_cost_paired.png"), dpi=140)
    plt.close(fig)

    # ---- 2. 箱线：三臂 × 四格 -------------------------------------------------
    fig, axes = plt.subplots(1, 4, figsize=(15, 4.2))  # 各面板独立量纲：四格差 4 倍，sharey 会把 imu_stuck 压扁
    for ax, cell in zip(axes, cells):
        data = [pick(recs[lab], cell, "pos_rms") for lab, _, _ in ARMS]
        bp = ax.boxplot(data, labels=[l for l, _, _ in ARMS], patch_artist=True,
                        showfliers=False)
        for patch, (_, _, col) in zip(bp["boxes"], ARMS):
            patch.set_facecolor(col)
            patch.set_alpha(.45)
        for m in bp["medians"]:
            m.set_color("k")
        vmax = max((float(np.max(d)) for d in data if len(d)), default=1.0)
        ax.set_ylim(0, vmax * 1.28)  # 先留头部空间，中位数标注才不会跑出坐标区
        for xx, dd in enumerate(data):
            if len(dd):
                ax.text(xx, float(np.max(dd)) * 1.04, f"{np.median(dd):.1f}",
                        ha="center", va="bottom", fontsize=7.5)
        ax.set_title(cell, fontsize=9)
        ax.grid(alpha=.3, axis="y")
        ax.tick_params(axis="y", labelsize=7.5)
    axes[0].set_ylabel("位置 RMSE [m]")
    fig.suptitle("40 seeds 三臂分布：imu_stuck +67.5% / stuck2s +17.6% / NaN 两格中性", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_fz_cost_box.png"), dpi=140)
    plt.close(fig)

    # ---- 3. CDF：imu_stuck 三臂 + 全格 raw/默认 比值分布 ----------------------
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.2))
    for lab, _, col in ARMS:
        v = np.sort(pick(recs[lab], "imu_stuck_icm_full", "pos_rms"))
        axes[0].step(v, np.arange(1, len(v) + 1) / len(v), color=col, lw=1.8, label=lab)
    axes[0].set_xlabel("位置 RMSE [m]")
    axes[0].set_ylabel("累积概率")
    axes[0].set_title("imu_stuck 1 s：累积分布（40 seeds）")
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=.3)
    ratios, labels = [], []
    for cell in cells:
        a = pick(recs["默认"], cell, "pos_rms")
        b = pick(recs["nav-raw"], cell, "pos_rms")
        if len(a) == len(b) and len(a):
            ratios.append(b / a)
            labels.append(cell)
    axes[1].boxplot(ratios, labels=labels, showfliers=False)
    axes[1].axhline(1.0, color="k", ls="--", lw=.9)
    axes[1].set_ylabel("raw / 默认 位置 RMSE 比")
    axes[1].set_title("比值分布：代价只集中在冻结形态")
    axes[1].grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_fz_cost_cdf.png"), dpi=140)
    plt.close(fig)

    # ---- 4. 故障窗口分解（seed1 trace）--------------------------------------
    def trace(path):
        with open(os.path.join(SD, path), encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        arr = {k: np.array([float(r[k]) for r in rows])
               for k in ("t", "ekf_n", "ekf_e", "true_n", "true_e", "ad_app")}
        arr["err"] = np.hypot(arr["ekf_n"] - arr["true_n"], arr["ekf_e"] - arr["true_e"])
        return arr

    tb, tr = trace("stuck_tr_base.csv"), trace("stuck_tr_raw.csv")
    win = [("故障前 t<12 s", tb["t"] < 12), ("故障窗 12-13 s", (tb["t"] >= 12) & (tb["t"] < 13)),
           ("故障后 13-45 s", tb["t"] >= 13)]
    fig, ax = plt.subplots(figsize=(8.2, 4.3))
    for i, (name, m) in enumerate(win):
        vals = [np.sqrt(np.mean(tb["err"][m] ** 2)), np.sqrt(np.mean(tr["err"][m] ** 2))]
        x = np.arange(2) + i * 3
        ax.bar(x, vals, color=["#2b8a3e", "#e8590c"], alpha=.8)
        for xx, vv in zip(x, vals):
            ax.text(xx, vv, f"{vv:.1f}", ha="center", va="bottom", fontsize=8)
    ax.set_xticks([0.5, 3.5, 6.5])
    ax.set_xticklabels([n for n, _ in win], fontsize=8.5)
    ax.set_ylabel("水平位置 RMSE [m]")
    ax.set_title("imu_stuck seed1 按时间窗分解：全程 +67.9% 全部继承自 1 s 故障窗\n"
                 "（故障前 raw 反而更好 −18.7%；45 s 末帧两臂均 <1 m）", fontsize=10)
    ax.grid(alpha=.3, axis="y")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_fz_cost_window.png"), dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9.5, 4.4))
    ax.semilogy(tb["t"], tb["err"], color="#2b8a3e", lw=1.1, label="默认（滤波增量）")
    ax.semilogy(tr["t"], tr["err"], color="#e8590c", lw=1.1, alpha=.85, label="nav-raw（绕过控制 LPF）")
    ax.axvspan(12, 13, color="crimson", alpha=.14, label="IMU 冻结窗 12-13 s")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("水平位置误差 [m]（对数轴）")
    ax.set_title("冻结故障期间/之后的误差演化（seed1，200 Hz 追踪）")
    ax.legend(fontsize=8)
    ax.grid(alpha=.3, which="both")
    ax2 = ax.twinx()
    ax2.step(tr["t"], tr["ad_app"], color="#e8590c", ls=":", lw=1.0, alpha=.6)
    ax2.step(tb["t"], tb["ad_app"], color="#2b8a3e", ls=":", lw=1.0, alpha=.6)
    ax2.set_ylabel("自动延时累计应用次数 ad_app（点线）", fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_fz_cost_timeseries.png"), dpi=140)
    plt.close(fig)

    # ---- 5. 热图：窗口 × 臂 的相对变化 --------------------------------------
    mat, rowl, coll = [], [n for n, _ in win] + ["全程", "末帧 t=45 s"], ["默认", "nav-raw"]
    for name, m in win:
        row = [np.sqrt(np.mean(tb["err"][m] ** 2)), np.sqrt(np.mean(tr["err"][m] ** 2))]
        mat.append(row)
    mat.append([np.sqrt(np.mean(tb["err"] ** 2)), np.sqrt(np.mean(tr["err"] ** 2))])
    mat.append([tb["err"][-1], tr["err"][-1]])
    mat = np.array(mat)
    fig, ax = plt.subplots(figsize=(6.6, 3.9))
    rel = mat / mat[:, :1]
    im = ax.imshow(rel, cmap="RdYlGn_r", vmin=.5, vmax=2.0, aspect="auto")
    ax.set_xticks(range(2))
    ax.set_xticklabels(coll)
    ax.set_yticks(range(len(rowl)))
    ax.set_yticklabels(rowl, fontsize=8.5)
    for i in range(mat.shape[0]):
        for j in range(2):
            txt = f"{mat[i, j]:.2f} m" + ("\n" + f"{(rel[i, j] - 1) * 100:+.0f}%" if j else "")
            ax.text(j, i, txt, ha="center", va="center", fontsize=8,
                    color="white" if rel[i, j] > 1.55 else "black")
    ax.set_title("误差按窗口分解：默认 vs nav-raw（%为相对默认）", fontsize=10)
    fig.colorbar(im, ax=ax, label="raw / 默认", fraction=.046)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_fz_cost_heat.png"), dpi=140)
    plt.close(fig)

    summary = {"cells": cells,
               "pos_rms_med": {lab: {c: float(np.median(pick(recs[lab], c, "pos_rms")))
                                     for c in cells} for lab, _, _ in ARMS},
               "window_rms_seed1": {n: {"base": float(np.sqrt(np.mean(tb["err"][m] ** 2))),
                                        "raw": float(np.sqrt(np.mean(tr["err"][m] ** 2)))}
                                    for n, m in win},
               "end_err_seed1": {"base": float(tb["err"][-1]), "raw": float(tr["err"][-1])},
               "ad_app_seed1": {"base": float(tb["ad_app"][-1]), "raw": float(tr["ad_app"][-1])}}
    with open(os.path.join(SD, "fz_cost_summary_2026-10-28.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=1)
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    print("figures ->", FIG)


if __name__ == "__main__":
    main()
