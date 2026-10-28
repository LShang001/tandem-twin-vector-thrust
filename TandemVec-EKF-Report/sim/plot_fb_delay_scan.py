#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24m-W 组配图：窗键回退起点延迟 D 的代价曲线（非单调）+ 后持时长反证。

@file    plot_fb_delay_scan.py
@brief   读 `_w_d*` 与 `_w_h*` 臂，产出 W 组判据所需图型
@module  TandemVec-EKF-Report/作图脚本（L2 研究设施）
@details 三张图：
         1 `nav_fb_delay_curve.png`  pos_rms 中位随 D 的曲线（含健康键参考线）
         2 `nav_fb_delay_box.png`    每档 D 的逐 seed 箱线（看是否双峰）
         3 `nav_fb_hold.png`         后持时长 H=5.0 vs 0.1 的反证对比
         生成后必须逐张目视审查（空图、NaN、坐标尺度、标签、缺字字形）。
@frame   指标取 SIL JSON 的 pos_rms（全程位置 RMSE，m）
@units   长度 m；D/H 单位 s
@note    曲线非单调：结论只能是"切换落点决定收益"，不能写成"越晚越好"。
"""
import glob
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
FIG = os.path.join(ROOT := os.path.abspath(os.path.join(HERE, "..")), "figs")
os.makedirs(FIG, exist_ok=True)

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
CELLS = ["imu_stuck_icm_full", "imu_stuck2s_icm_full"]


def load(path):
    d = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            r = json.loads(line)
            base = r["config"].split("_wd")[0].split("_wh")[0]
            d.setdefault(base, []).append(r)
    return {k: sorted(v, key=lambda x: x["seed"]) for k, v in d.items()}


def arms(pattern, strip):
    out = {}
    for p in sorted(glob.glob(os.path.join(SD, pattern))):
        tag = os.path.basename(p)[len("missile-sil-2026-10-28_"):-len(".jsonl")]
        out[tag.replace(strip, "")] = load(p)
    return out


def main():
    D = arms("missile-sil-2026-10-28_w_d*.jsonl", "w_d")
    H = arms("missile-sil-2026-10-28_w_h*.jsonl", "w_h")
    assert D, "没有 _w_d* 证据"
    for k in D:
        assert set(D[k]) >= set(CELLS), (k, sorted(D[k]))
    ds = sorted(D, key=lambda x: float(x))

    # ---- 1. 曲线 -------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(8.6, 4.6))
    for i, cell in enumerate(CELLS):
        xs = [float(x) for x in ds]
        ys = [float(np.median([r["pos_rms"] for r in D[d][cell]])) for d in ds]
        ax.plot(xs, ys, "o-", lw=1.9, ms=7, label=cell)
    ax.axhline(2.792, color="#1971c2", ls="--", lw=1.4, label="健康键参照 2.79 m")
    ax.axhline(57.623, color="#e8590c", ls=":", lw=1.4, label="纯 raw 参照 57.62 m")
    ax.axhline(34.371, color="#2b8a3e", ls="dotted", lw=1.4, label="默认参照 34.37 m")
    ax.set_xlabel("窗键回退起点推迟 D [s]")
    ax.set_ylabel("位置 RMSE 中位 [m]（20 seeds）")
    ax.set_title("W 组：收益取决于起点落点——D=0.5 跳过故障前沿最好（2.5 m），"
                 "D=1.0 对 1 s 故障等于零覆盖 ⇒ 退回 raw（57.8 m）", fontsize=10.5)
    ax.legend(fontsize=8)
    ax.grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_fb_delay_curve.png"), dpi=140)
    plt.close(fig)

    # ---- 2. 每档箱线 ---------------------------------------------------------
    fig, axes = plt.subplots(1, 2, figsize=(11.6, 4.3), sharey=False)
    for ax, cell in zip(axes, CELLS):
        data = [[r["pos_rms"] for r in D[d][cell]] for d in ds]
        bp = ax.boxplot(data, tick_labels=[f"D={x}" for x in ds], showfliers=False)
        for m in bp["medians"]:
            m.set_color("k")
        ax.set_ylabel("位置 RMSE [m]")
        ax.set_title(cell, fontsize=9.5)
        ax.grid(alpha=.3, axis="y")
    fig.suptitle("每档 20 seeds 分布：单峰为主，D=0.5 整簇左移", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_fb_delay_box.png"), dpi=140)
    plt.close(fig)

    # ---- 3. 后持时长反证 -----------------------------------------------------
    if len(H) >= 2:
        hs = sorted(H, key=lambda x: float(x))
        fig, ax = plt.subplots(figsize=(8.2, 4.2))
        x = np.arange(len(CELLS))
        for j, h in enumerate(hs):
            v = [float(np.median([r["pos_rms"] for r in H[h][c]])) for c in CELLS]
            ax.bar(x + (j - 0.5) * .35, v, width=.33, alpha=.85,
                   color=["#1971c2", "crimson"][j % 2], label=f"hold={h}s")
            for xx, vv in zip(x + (j - 0.5) * .35, v):
                ax.text(xx, vv, f"{vv:.1f}", ha="center", va="bottom", fontsize=8)
        ax.set_xticks(x)
        ax.set_xticklabels(CELLS, fontsize=9)
        ax.set_ylabel("位置 RMSE 中位 [m]")
        ax.set_title("反证：固定 D=0.5，把后持从 5 s 缩到 0.1 s"
                     "（释放点落在 LPF 仍带冻结值处）是否让台阶重现", fontsize=10.5)
        ax.legend(fontsize=8.5)
        ax.grid(alpha=.3, axis="y")
        fig.tight_layout()
        fig.savefig(os.path.join(FIG, "nav_fb_hold.png"), dpi=140)
        plt.close(fig)
        print("hold 档:", {h: {c: round(float(np.median([r["pos_rms"] for r in H[h][c]])), 3)
                              for c in CELLS} for h in hs})
    else:
        print("hold 反证臂不足 2 档，跳过第 3 图")

    summary = {"delay_curve": {c: {d: round(float(np.median([r["pos_rms"] for r in D[d][c]])), 3)
                                   for d in ds} for c in CELLS},
               "hold_curve": {c: {h: round(float(np.median([r["pos_rms"] for r in H[h][c]])), 3)
                                  for h in H} for c in CELLS} if H else {}}
    with open(os.path.join(SD, "fb_delay_scan_2026-10-28.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=1)
    print(json.dumps(summary, ensure_ascii=False))
    print("figures ->", FIG)


if __name__ == "__main__":
    main()
