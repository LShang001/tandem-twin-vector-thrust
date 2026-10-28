#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24m K 组配图：窗键 vs 健康键回退（含四臂误差时序与误触发占空比）。

@file    plot_nav_raw_healthkey.py
@brief   读 `_k1_hfb` / `_nr_{base,raw,fb}` / `_k2_risk` 与 `stuck_tr_{base,raw,win,health}.csv`，
         产出 K1/K2 判据所需图型
@module  TandemVec-EKF-Report/作图脚本（L2 研究设施）
@details 四张图：
         1 `nav_hk_timeseries.png`  seed1 四臂水平误差时序，标出冻结解除拍的台阶
         2 `nav_hk_median_bar.png`  四故障格 × 四臂中位（健康键 vs 窗键 vs raw vs 默认）
         3 `nav_hk_cdf.png`         imu_stuck / imu_stuck2s 四臂经验 CDF
         4 `nav_hk_duty.png`        K2 非故障风险格的 nrfb 占空比（应全 0）
         生成后必须逐张目视审查（空图、NaN、坐标尺度、标签、缺字字形）。
@frame   误差 = hypot(ekf_n-true_n, ekf_e-true_e)（NED 水平面）
@units   长度 m、时间 s；占空比单位=帧（2 kHz）
@note    不用 ctl_alt-truth_alt 作垂向项（相差约 50 m 定义量，会伪造误差）。
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

ARMS = [("默认", "nr_base", "_nrb", "#2b8a3e"),
        ("nav-raw", "nr_raw", "_nrr", "#e8590c"),
        ("raw+窗键回退", "nr_fb", "_nrf", "#868e96"),
        ("raw+健康键回退", "k1_hfb", "_k1h", "#1971c2")]


def load(tag, suf):
    path = os.path.join(SD, f"missile-sil-2026-10-28_{tag}.jsonl")
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


def trace(name):
    with open(os.path.join(SD, f"stuck_tr_{name}.csv"), encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    a = {k: np.array([float(r[k]) for r in rows])
         for k in ("t", "ekf_n", "ekf_e", "true_n", "true_e")}
    a["err"] = np.hypot(a["ekf_n"] - a["true_n"], a["ekf_e"] - a["true_e"])
    return a


def main():
    D = {lab: load(tag, suf) for lab, tag, suf, _ in ARMS}
    fault = ["imu_stuck_icm_full", "imu_stuck2s_icm_full",
             "imu_nan_icm_full", "imu_nan2s_icm_full"]
    for lab in D:
        assert set(D[lab]) == set(fault), (lab, sorted(D[lab]))
    cols = {lab: c for lab, _, _, c in ARMS}

    # ---- 1. seed1 四臂时序 ----------------------------------------------------
    key = {"默认": "base", "nav-raw": "raw", "raw+窗键回退": "win", "raw+健康键回退": "health"}
    fig, ax = plt.subplots(figsize=(10.6, 4.8))
    for lab, k in key.items():
        a = trace(k)
        ax.semilogy(a["t"], np.maximum(a["err"], 1e-3), color=cols[lab], lw=1.2, label=lab)
    ax.axvspan(12, 13, color="crimson", alpha=.12)
    ax.axvline(13.0, color="k", ls=":", lw=1.1)
    ax.annotate("冻结解除拍：默认/窗键/raw 在此跳到 100-172 m\n健康键臂全程 <7 m",
                xy=(13.2, 60), fontsize=8.5)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("水平位置误差 [m]（对数轴）")
    ax.set_title("§4.24m K 组 seed1：四臂误差演化（IMU 冻结 12-13 s）", fontsize=11)
    ax.legend(fontsize=8.5, loc="lower left")
    ax.grid(alpha=.3, which="both")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_hk_timeseries.png"), dpi=140)
    plt.close(fig)

    # ---- 2. 四格 × 四臂中位 ---------------------------------------------------
    fig, ax = plt.subplots(figsize=(11.2, 4.7))
    x = np.arange(len(fault))
    for i, (lab, _, _, col) in enumerate(ARMS):
        v = [float(np.median([r["pos_rms"] for r in D[lab][c].values()])) for c in fault]
        ax.bar(x + (i - 1.5) * .21, v, width=.2, color=col, alpha=.88, label=lab)
        for xx, vv in zip(x + (i - 1.5) * .21, v):
            ax.text(xx, vv, f"{vv:.1f}", ha="center", va="bottom", fontsize=7)
    ax.set_xticks(x)
    ax.set_xticklabels(fault, fontsize=8.5)
    ax.set_ylabel("位置 RMSE 中位 [m]（20 seeds）")
    ax.set_ylim(0, 265)
    ax.set_title("K1：健康键回退在两个冻结格上优于窗键与默认（机制未证，仅登记实测排序）",
                 fontsize=10.5)
    ax.legend(fontsize=8, ncol=2)
    ax.grid(alpha=.3, axis="y")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_hk_median_bar.png"), dpi=140)
    plt.close(fig)

    # ---- 3. CDF --------------------------------------------------------------
    fig, axes = plt.subplots(1, 2, figsize=(11.2, 4.2))
    for ax2, cell in zip(axes, ("imu_stuck_icm_full", "imu_stuck2s_icm_full")):
        for lab in ("默认", "nav-raw", "raw+窗键回退", "raw+健康键回退"):
            v = np.sort([r["pos_rms"] for r in D[lab][cell].values()])
            ax2.step(v, np.arange(1, len(v) + 1) / len(v), color=cols[lab], lw=1.8,
                     label=f"{lab} 中位 {np.median(v):.1f}")
        ax2.set_title(cell, fontsize=9.5)
        ax2.set_xlabel("位置 RMSE [m]")
        ax2.set_ylabel("累积概率")
        ax2.grid(alpha=.3)
    axes[0].legend(fontsize=7.5)
    fig.suptitle("四臂经验 CDF：健康键臂整体左移到个位数米级", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_hk_cdf.png"), dpi=140)
    plt.close(fig)

    # ---- 4. K2 误触发占空比 ---------------------------------------------------
    K2 = load("k2_risk", "_k2r")
    fig, ax = plt.subplots(figsize=(9.6, 3.9))
    cells = sorted(K2)
    duty = [float(np.mean([r.get("nrfb", 0) / 90000.0 for r in K2[c].values()])) * 100
            for c in cells]
    mx = [int(max(r.get("nrfb", 0) for r in K2[c].values())) for c in cells]
    ax.bar(np.arange(len(cells)), duty, color="#1971c2", alpha=.85)
    for i, (d, m) in enumerate(zip(duty, mx)):
        ax.text(i, d, f"峰值 {m} 帧", ha="center", va="bottom", fontsize=8)
    ax.set_xticks(np.arange(len(cells)))
    ax.set_xticklabels(cells, fontsize=8.5)
    ax.set_ylabel("回退占空比 [% of 45 s @2 kHz]")
    ax.set_ylim(0, max(duty) * 1.6 + .5)
    ax.set_title("K2 误触发检查：削顶(10.1 g@8 g)与强振动格上占空比恒 0 ⇒ 饱和/振动只进 Warning，"
                 "不会静默关掉收益", fontsize=10)
    ax.grid(alpha=.3, axis="y")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_hk_duty.png"), dpi=140)
    plt.close(fig)

    out = {"median": {lab: {c: float(np.median([r["pos_rms"] for r in D[lab][c].values()]))
                            for c in fault} for lab in D},
           "k2_duty_pct": dict(zip(cells, [round(d, 4) for d in duty])),
           "k2_nrfb_max": dict(zip(cells, mx))}
    with open(os.path.join(SD, "nav_raw_healthkey_summary_2026-10-28.json"), "w",
              encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    print(json.dumps(out["k2_duty_pct"], ensure_ascii=False))
    print("figures ->", FIG)


if __name__ == "__main__":
    main()
