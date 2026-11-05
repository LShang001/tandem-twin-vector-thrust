# -*- coding: utf-8 -*-
"""无 GNSS 姿态辅助量测源 MC 成图（AHRS vs 重力直连 vs 重力+回退）。

@file    plot_att_aid_mc.py
@brief   读 `sim-data/att-aid-mc-2026-11-01/mc.json`，产出
         §AHRS姿态辅助与无GNSS重力融合 需要的图型；生成后逐张目视审查
         （空图、NaN、坐标尺度、标签）。
@module  TandemVec-EKF-Report/作图脚本（L2 研究设施）
@details 五张图：
         1 `nav_att_aid_att_box.png`    各条线 × 模式 姿态 RMSE 箱线 + 散点
         2 `nav_att_aid_cdf.png`        out10 / out20 经验 CDF（尾部可读）
         3 `nav_att_aid_pitch_pos.png`  pitch RMSE / 位置 RMS 两面板箱线
         4 `nav_att_aid_paired.png`     逐 seed 配对散点（mode0 vs mode1/2）
         5 `nav_att_aid_yaw.png`        yaw RMSE 箱线（展示小幅回归）
@frame   N/A（统计图）
@units   deg、m
"""
import json
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.normpath(os.path.join(HERE, "..", "sim-data", "att-aid-mc-2026-11-01", "mc2.json"))
FIG = os.path.normpath(os.path.join(HERE, "..", "figs"))

COND_ORDER = ["clean", "out5", "out10", "out10_noph", "out10_wind",
              "out10_vib1g", "out20"]
COND_LABEL = {
    "clean": "无失联",
    "out5": "失联 5 s",
    "out10": "失联 10 s（定点保持）",
    "out10_noph": "失联 10 s（无位置环）",
    "out10_wind": "失联 10 s + 风",
    "out10_vib1g": "失联 10 s + 1 g 振动",
    "out20": "失联 20 s",
}
MODE_ORDER = [0, 1, 2]
MODE_LABEL = {0: "mode 0：AHRS 量测", 1: "mode 1：重力直连", 2: "mode 2：重力+回退"}
MODE_COLOR = {0: "#c44e52", 1: "#4c72b0", 2: "#55a868"}


def setup_font():
    """按优先级挑选可用的中文字体（雅黑/黑体/Noto CJK）设为 matplotlib 全局字体，并关闭 unicode 负号变体。"""
    for name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC"):
        try:
            font_manager.findfont(name, fallback_to_default=False)
            plt.rcParams["font.family"] = name
            break
        except Exception:
            continue
    plt.rcParams["axes.unicode_minus"] = False


def load():
    """读 mc2.json，丢弃含 _error 的记录后返回列表；文件缺失直接退出。"""
    if not os.path.exists(DATA):
        sys.exit(f"missing {DATA}")
    with open(DATA, encoding="utf-8") as fh:
        recs = json.load(fh)
    return [r for r in recs if "_error" not in r]


def vals(recs, cond, mode, key):
    """按条件与模式筛出某一指标的数值数组（非数值/缺失跳过），供箱线与 CDF 复用。"""
    xs = [r[key] for r in recs
          if r["_cond"] == cond and r["_mode"] == mode
          and isinstance(r.get(key), (int, float))]
    return np.asarray(xs, dtype=float)


def box_by_condition(recs, key, ylabel, title, fname, log=False):
    """画「条件 × 模式」分组箱线并叠加逐 seed 散点：x 为条件、按模式错位配色；log=True 时 y 轴取对数，结果存 fname。"""
    fig, ax = plt.subplots(figsize=(11, 5.2))
    width = 0.26
    for mi, mode in enumerate(MODE_ORDER):
        data, pos = [], []
        for ci, cond in enumerate(COND_ORDER):
            xs = vals(recs, cond, mode, key)
            if xs.size:
                data.append(xs)
                pos.append(ci + (mi - 1) * width)
        bp = ax.boxplot(data, positions=pos, widths=width * 0.85,
                        patch_artist=True, showfliers=True,
                        medianprops=dict(color="black", lw=1.2))
        for patch in bp["boxes"]:
            patch.set_facecolor(MODE_COLOR[mode])
            patch.set_alpha(0.55)
        rng = np.random.default_rng(7)
        for x, xs in zip(pos, data):
            ax.scatter(x + rng.uniform(-0.05, 0.05, xs.size), xs, s=9,
                       color=MODE_COLOR[mode], alpha=0.55, zorder=3)
        ax.plot([], [], color=MODE_COLOR[mode], lw=6, alpha=0.55,
                label=MODE_LABEL[mode])
    ax.set_xticks(range(len(COND_ORDER)))
    ax.set_xticklabels([COND_LABEL[c] for c in COND_ORDER])
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    if log:
        ax.set_yscale("log")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend(loc="upper left", fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, fname), dpi=150)
    plt.close(fig)
    print(f"[fig] {fname}")


def plot_cdf(recs):
    """画失联 10 s / 20 s 的姿态 RMSE 经验 CDF（尾部可读），输出 nav_att_aid_cdf.png。"""
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.6))
    for ax, cond in zip(axes, ["out10", "out20"]):
        for mode in MODE_ORDER:
            xs = np.sort(vals(recs, cond, mode, "nav_att_rmse"))
            if not xs.size:
                continue
            ax.plot(xs, np.arange(1, xs.size + 1) / xs.size, marker=".",
                    color=MODE_COLOR[mode], label=MODE_LABEL[mode])
        ax.set_title(f"{COND_LABEL[cond]}：姿态 RMSE 经验 CDF")
        ax.set_xlabel("姿态 RMSE [deg]")
        ax.set_ylabel("累计比例")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_att_aid_cdf.png"), dpi=150)
    plt.close(fig)
    print("[fig] nav_att_aid_cdf.png")


def plot_pitch_pos(recs):
    """画失联 10 s 下俯仰 RMSE 与水平位置 RMS 的模式对比箱线，输出 nav_att_aid_pitch_pos.png。"""
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.6))
    for ax, key, ylabel, title in (
            (axes[0], "nav_pitch_rmse", "俯仰 RMSE [deg]", "俯仰误差"),
            (axes[1], "pos_rms", "水平位置 RMS [m]", "水平位置误差")):
        for mode in MODE_ORDER:
            xs = vals(recs, "out10", mode, key)
            if xs.size:
                ax.boxplot([xs], positions=[mode], widths=0.55,
                           patch_artist=True, showfliers=True,
                           medianprops=dict(color="black", lw=1.2),
                           boxprops=dict(facecolor=MODE_COLOR[mode], alpha=0.55))
                rng = np.random.default_rng(11 + mode)
                ax.scatter(mode + rng.uniform(-0.12, 0.12, xs.size), xs,
                           s=12, color=MODE_COLOR[mode], alpha=0.6, zorder=3)
        ax.set_xticks(MODE_ORDER)
        ax.set_xticklabels([MODE_LABEL[m].split("：")[0] for m in MODE_ORDER])
        ax.set_ylabel(ylabel)
        ax.set_title(f"{title}（失联 10 s，30 seeds）")
        ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_att_aid_pitch_pos.png"), dpi=150)
    plt.close(fig)
    print("[fig] nav_att_aid_pitch_pos.png")


def plot_paired(recs):
    """画 mode0 与 mode1/2 的逐 seed 配对散点（对角线以下=新模式更低），标题给出更低的 seed 数。"""
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.8), sharey=False)
    for ax, base in zip(axes, [1, 2]):
        d0 = {r["_seed"]: r for r in recs if r["_cond"] == "out10" and r["_mode"] == 0}
        dX = {r["_seed"]: r for r in recs if r["_cond"] == "out10" and r["_mode"] == base}
        seeds = sorted(set(d0) & set(dX))
        x = np.array([d0[s]["nav_att_rmse"] for s in seeds])
        y = np.array([dX[s]["nav_att_rmse"] for s in seeds])
        ax.scatter(x, y, s=22, color=MODE_COLOR[base], alpha=0.75,
                   edgecolor="black", linewidth=0.3)
        lim = max(x.max(), y.max()) * 1.08
        ax.plot([0, lim], [0, lim], "--", color="gray", lw=1)
        ax.set_xlim(0, lim)
        ax.set_ylim(0, lim)
        ax.set_xlabel("mode 0（AHRS）姿态 RMSE [deg]")
        ax.set_ylabel(f"mode {base} 姿态 RMSE [deg]")
        better = int(np.sum(y < x))
        ax.set_title(f"逐 seed 配对：{better}/{len(seeds)} 个 seed 更低（失联 10 s）")
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_att_aid_paired.png"), dpi=150)
    plt.close(fig)
    print("[fig] nav_att_aid_paired.png")


def plot_fail_rate(recs):
    """失败率：阈值超限样本占比（尾部风险比中位数更能区分方案）。"""
    criteria = [
        ("nav_att_rmse", 5.0, "姿态 RMSE > 5°"),
        ("pos_rms", 10.0, "位置 RMS > 10 m"),
    ]
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8))
    for ax, (key, thr, title) in zip(axes, criteria):
        for mi, mode in enumerate(MODE_ORDER):
            rates = []
            for cond in COND_ORDER:
                xs = vals(recs, cond, mode, key)
                rates.append(100.0 * float(np.sum(xs > thr)) / xs.size if xs.size else 0.0)
            xs = np.arange(len(COND_ORDER)) + (mi - 1) * 0.26
            ax.bar(xs, rates, width=0.25, color=MODE_COLOR[mode], alpha=0.78,
                   label=MODE_LABEL[mode])
        ax.set_xticks(range(len(COND_ORDER)))
        ax.set_xticklabels([COND_LABEL[c] for c in COND_ORDER],
                           rotation=15, ha="right")
        ax.set_ylabel("失败率 [%]")
        ax.set_title(f"{title}（样本占比）")
        ax.grid(True, axis="y", alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_att_aid_fail_rate.png"), dpi=150)
    plt.close(fig)
    print("[fig] nav_att_aid_fail_rate.png")


def main():
    """命令行入口：设字体、建目录、载入记录，依次出姿态箱线、CDF、俯仰/位置、配对散点、失败率与航向箱线六张图。"""
    setup_font()
    os.makedirs(FIG, exist_ok=True)
    recs = load()
    print(f"[data] {len(recs)} records")
    box_by_condition(recs, "nav_att_rmse", "姿态 RMSE [deg]",
                     "各条线姿态 RMSE：AHRS 量测 vs 重力直连（点=逐 seed）",
                     "nav_att_aid_att_box.png", log=False)
    plot_cdf(recs)
    plot_pitch_pos(recs)
    plot_paired(recs)
    plot_fail_rate(recs)
    box_by_condition(recs, "nav_yaw_rmse", "航向 RMSE [deg]",
                     "各条线航向 RMSE（重力直连不观测航向，作为对照）",
                     "nav_att_aid_yaw.png", log=False)


if __name__ == "__main__":
    main()
