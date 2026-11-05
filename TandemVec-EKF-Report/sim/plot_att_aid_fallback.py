# -*- coding: utf-8 -*-
"""有界兜底（mode 5）与纯陀螺、AHRS 的失联时长扫描成图。

@file    plot_att_aid_fallback.py
@brief   读 sim-data/att-aid-mc-2026-11-01/mc_fb_{matrix,final}.json，产出
         “姿态中位/max 随失联时长”与“干扰条线”两图，生成后逐张目视审查。
@module  TandemVec-EKF-Report/作图脚本（L2 研究设施）
@details 两张图：
         1 `nav_att_aid_fallback_vs_duration.png`  5/10/20/30 s 失联 × 三种模式
         2 `nav_att_aid_fallback_disturb.png`       30 s 失联 × 风/振动 × 三种模式
@frame   N/A（统计姿态角误差，不涉坐标变换）
@units   deg（nav_att_rmse）、s（失联时长）、m/s²（风/振动幅值）
"""
import json
import os
import statistics

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.normpath(os.path.join(HERE, "..", "sim-data", "att-aid-mc-2026-11-01"))
FIG = os.path.normpath(os.path.join(HERE, "..", "figs"))
MODE_LABEL = {0: "mode 0：AHRS", 1: "mode 1：纯陀螺", 5: "mode 5：有界兜底"}
MODE_COLOR = {0: "#c44e52", 1: "#4c72b0", 5: "#55a868"}


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


def load_all():
    """读 matrix/final 两份 MC JSON（跳过 _error 记录），按 condition/mode/seed 去重取最后一条，返回记录列表。"""
    recs = []
    for fn in ("mc_fb_matrix.json", "mc_fb_final.json"):
        p = os.path.join(DATA, fn)
        if not os.path.exists(p):
            continue
        with open(p, encoding="utf-8") as fh:
            recs.extend(r for r in json.load(fh) if "_error" not in r)
    # 去重（同 condition/mode/seed 取最后一条）
    key = {}
    for r in recs:
        key[(r["_cond"], r["_mode"], r["_seed"])] = r
    return list(key.values())


def stats(recs, cond, mode):
    """取指定条件与模式的姿态 RMSE 列表，返回（中位数, 最大值, 样本数）；无样本返回 None。"""
    xs = [r["nav_att_rmse"] for r in recs
          if r["_cond"] == cond and r["_mode"] == mode
          and isinstance(r.get("nav_att_rmse"), (int, float))]
    if not xs:
        return None
    return statistics.median(xs), max(xs), len(xs)


def plot_duration(recs):
    """画失联时长扫描（5/10/20/30 s × 三种模式）：左图姿态 RMSE 中位、右图最大，输出 nav_att_aid_fallback_vs_duration.png。"""
    conds = ["out5", "out10", "out20", "out30"]
    labels = ["5 s", "10 s", "20 s", "30 s"]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.6))
    width = 0.26
    for mi, mode in enumerate((0, 1, 5)):
        med, mx = [], []
        for cond in conds:
            st = stats(recs, cond, mode)
            med.append(st[0] if st else np.nan)
            mx.append(st[1] if st else np.nan)
        x = np.arange(len(conds)) + (mi - 1) * width
        axes[0].bar(x, med, width=width * 0.9, color=MODE_COLOR[mode],
                    alpha=0.85, label=MODE_LABEL[mode])
        axes[1].bar(x, mx, width=width * 0.9, color=MODE_COLOR[mode],
                    alpha=0.85, label=MODE_LABEL[mode])
    for ax, title in zip(axes, ("姿态 RMSE 中位", "姿态 RMSE 最大")):
        ax.set_xticks(range(len(conds)))
        ax.set_xticklabels(labels)
        ax.set_xlabel("GNSS 失联时长")
        ax.set_ylabel("姿态 RMSE [deg]")
        ax.set_title(title)
        ax.grid(True, axis="y", alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle("失联时长扫描：纯陀螺 vs AHRS vs 有界兜底")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_att_aid_fallback_vs_duration.png"), dpi=150)
    plt.close(fig)
    print("[fig] nav_att_aid_fallback_vs_duration.png")


def plot_disturb(recs):
    """画 30 s 失联 + 风/振动干扰的对比图（柱=中位、须=最大），输出 nav_att_aid_fallback_disturb.png。"""
    conds = ["out30", "out30_wind", "out30_vib1g"]
    labels = ["30 s 失联", "+ 风 1.5 m/s²", "+ 1 g 振动"]
    fig, ax = plt.subplots(figsize=(9, 4.6))
    width = 0.26
    for mi, mode in enumerate((0, 1, 5)):
        med, mx = [], []
        for cond in conds:
            st = stats(recs, cond, mode)
            med.append(st[0] if st else np.nan)
            mx.append(st[1] if st else np.nan)
        x = np.arange(len(conds)) + (mi - 1) * width
        ax.bar(x, med, width=width * 0.9, color=MODE_COLOR[mode], alpha=0.85,
               label=MODE_LABEL[mode])
        ax.errorbar(x, med, yerr=np.asarray(mx) - np.asarray(med), fmt="none",
                    ecolor="black", elinewidth=1, capsize=3)
    ax.set_xticks(range(len(conds)))
    ax.set_xticklabels(labels)
    ax.set_ylabel("姿态 RMSE [deg]（柱=中位，须=最大）")
    ax.set_title("30 s 失联 + 干扰：三种模式")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_att_aid_fallback_disturb.png"), dpi=150)
    plt.close(fig)
    print("[fig] nav_att_aid_fallback_disturb.png")


def main():
    """命令行入口：设字体、建输出目录、载入并去重记录，依次生成时长与干扰两张图。"""
    setup_font()
    os.makedirs(FIG, exist_ok=True)
    recs = load_all()
    print(f"[data] {len(recs)} records")
    plot_duration(recs)
    plot_disturb(recs)


if __name__ == "__main__":
    main()
