# -*- coding: utf-8 -*-
"""大过载饱和自适应 Q 膨胀：off/on A/B 与 Q 扫描。

@file    plot_satq.py
@brief   10.1 g 超量程、8.4 g、6.4 g 的位置/姿态/拒帧对照，以及 Q 取值权衡。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 数据：missile-sil-2026-10-27_satq_{off,on}.json(l)、satq_qsweep_2026-10-27.json(l)。
         输出 figs/nav_sat_q_ab_2026-10-27.png、figs/nav_sat_q_sweep_2026-10-27.png。
@frame   NED
@units   m、m/s、°
"""
import argparse
import json
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "sim-data")
FIG = os.path.join(HERE, "..", "figs")
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

CASES = [
    ("10.1 g", "thrust10g_icm_full"),
    ("10.1 g+spoof30", "thrust10g_spoof30_icm_full"),
    ("10.1 g+磁失联", "thrust10g_magdrop_icm_full"),
    ("10.1 g+失联5s", "thrust10g_outage5_icm_full"),
    ("8.4 g", "thrust8g_icm_full"),
    ("6.4 g", "clean_icm_full"),
]


def load_json(name):
    return json.load(open(os.path.join(DATA, name), encoding="utf-8"))["groups"]


def load_rows(name):
    rows = {}
    with open(os.path.join(DATA, name), encoding="utf-8") as f:
        for line in f:
            r = json.loads(line)
            rows.setdefault(r["config"], {})[r["seed"]] = r
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sweep", default="satq_qsweep_2026-10-27")
    args = ap.parse_args()

    off = load_json("missile-sil-2026-10-27_satq_off.json")
    on = load_json("missile-sil-2026-10-27_satq_on.json")
    rows_off = load_rows("missile-sil-2026-10-27_satq_off.jsonl")
    rows_on = load_rows("missile-sil-2026-10-27_satq_on.jsonl")

    fig, axes = plt.subplots(2, 2, figsize=(13.5, 8.6))
    ax = axes[0, 0]
    y = np.arange(len(CASES))
    h = 0.36
    med_off = [off[c]["pos_rmse"]["med"] for _, c in CASES]
    med_on = [on[c]["pos_rmse"]["med"] for _, c in CASES]
    ax.barh(y + h / 2, med_off, h, label="默认（无 Q 膨胀）", color="#888888")
    ax.barh(y - h / 2, med_on, h, label="饱和 Q 膨胀", color="#c0392b")
    ax.set_yticks(y)
    ax.set_yticklabels([n for n, _ in CASES])
    ax.set_xscale("log")
    ax.set_xlabel("位置 RMSE 中位 [m]（对数）")
    ax.set_title("(a) 位置 RMSE：10.1 g 由 km 级回到米级")
    ax.grid(True, axis="x", alpha=0.3)
    ax.legend(fontsize=8)
    for yi, (vo, vn) in enumerate(zip(med_off, med_on)):
        ax.text(vo * 1.08, yi + h / 2, f"{vo:.1f}", va="center", fontsize=8)
        ax.text(vn * 1.08, yi - h / 2, f"{vn:.1f}", va="center", fontsize=8,
                color="#c0392b")

    ax = axes[0, 1]
    for seed in range(1, 21):
        a = rows_off["thrust10g_icm_full"][seed]["pos_rms"]
        b = rows_on["thrust10g_icm_full"][seed]["pos_rms"]
        ax.plot([0, 1], [a, b], "-o", color="#c0392b", alpha=0.55, ms=3.5)
    ax.set_xticks([0, 1])
    ax.set_xticklabels(["默认", "Q 膨胀"])
    ax.set_yscale("log")
    ax.set_ylabel("10.1 g 单 seed 位置 RMSE [m]")
    ax.set_title("(b) 20/20 seed 一致改善（对数）")
    ax.grid(True, axis="y", alpha=0.3)

    ax = axes[1, 0]
    labels = ["10.1 g 姿态", "10.1 g 航向", "8.4 g 姿态", "8.4 g 航向",
              "6.4 g 姿态", "6.4 g 航向"]
    keys = [("thrust10g_icm_full", "att_rmse"),
            ("thrust10g_icm_full", "yaw_rmse"),
            ("thrust8g_icm_full", "att_rmse"),
            ("thrust8g_icm_full", "yaw_rmse"),
            ("clean_icm_full", "att_rmse"),
            ("clean_icm_full", "yaw_rmse")]
    vo = [off[c][k]["med"] for c, k in keys]
    vn = [on[c][k]["med"] for c, k in keys]
    x = np.arange(len(labels))
    ax.bar(x - h / 2, vo, h, color="#888888", label="默认")
    ax.bar(x + h / 2, vn, h, color="#c0392b", label="Q 膨胀")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=20, ha="right", fontsize=8)
    ax.set_ylabel("误差中位 [°]")
    ax.set_title("(c) 姿态/航向代价：8.4 g 约 +0.1°/+0.3°")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes[1, 1]
    names = ["10.1 g", "10.1 g+spoof30", "10.1 g+失联5s"]
    cfg = ["thrust10g_icm_full", "thrust10g_spoof30_icm_full",
           "thrust10g_outage5_icm_full"]
    x = np.arange(len(names))
    rej_o = [off[c]["outliers"]["med"] for c in cfg]
    rej_n = [on[c]["outliers"]["med"] for c in cfg]
    rst_o = [off[c]["reanchors"]["med"] for c in cfg]
    rst_n = [on[c]["reanchors"]["med"] for c in cfg]
    ax.bar(x - h / 2, rej_o, h, color="#888888", label="默认 拒帧")
    ax.bar(x + h / 2, rej_n, h, color="#c0392b", label="Q 膨胀 拒帧")
    ax.plot(x - h / 2, rst_o, "s--", color="#2c3e50", ms=6, label="默认 重锚")
    ax.plot(x + h / 2, rst_n, "s-", color="#e67e22", ms=6, label="Q 膨胀 重锚")
    ax.set_xticks(x)
    ax.set_xticklabels(names, fontsize=9)
    ax.set_ylabel("次数（中位）")
    ax.set_title("(d) 拒帧/重锚：10.1 g 清零")
    ax.legend(fontsize=8)
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    out = os.path.join(FIG, "nav_sat_q_ab_2026-10-27.png")
    fig.savefig(out, dpi=150)
    print("wrote", out)

    qs = [0.5, 1.0, 2.0, 3.0, 5.0, 10.0, 15.0, 25.0, 40.0]
    qs = [q for q in qs if os.path.exists(os.path.join(
        DATA, f"missile-sil-2026-10-27_satq_q{q:g}.json"))]
    fig, axes = plt.subplots(1, 2, figsize=(12.5, 4.4))
    for label, cfg, color in [("10.1 g", "thrust10g_icm_full", "#c0392b"),
                              ("8.4 g", "thrust8g_icm_full", "#2c3e50")]:
        pos = []
        att = []
        for q in qs:
            d = load_json(f"missile-sil-2026-10-27_satq_q{q:g}.json")[cfg]
            pos.append(d["pos_rmse"]["med"])
            att.append(d["att_rmse"]["med"])
        axes[0].plot(qs, pos, "-o", color=color, label=label)
        axes[1].plot(qs, att, "-o", color=color, label=label)
    axes[0].set_xscale("log")
    axes[0].set_yscale("log")
    axes[0].set_xlabel("锁存期间加计噪声标准差 Q [m/s²]（对数）")
    axes[0].set_ylabel("位置 RMSE 中位 [m]（对数）")
    axes[0].set_title("(a) Q 对位置的影响（10 seeds）")
    axes[0].grid(True, which="both", alpha=0.3)
    axes[0].legend()
    axes[1].set_xscale("log")
    axes[1].set_xlabel("锁存期间加计噪声标准差 Q [m/s²]（对数）")
    axes[1].set_ylabel("姿态 RMSE 中位 [°]")
    axes[1].set_title("(b) Q 对姿态的影响（10 seeds）")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    fig.tight_layout()
    out = os.path.join(FIG, "nav_sat_q_sweep_2026-10-27.png")
    fig.savefig(out, dpi=150)
    print("wrote", out)


    opt_path = os.path.join(DATA, "satq_opt_2026-10-27.json")
    if not os.path.exists(opt_path):
        return
    opt = json.load(open(opt_path, encoding="utf-8"))
    hist_all = sorted(opt["cache"].values(), key=lambda r: r["at"])
    fig, axes = plt.subplots(2, 2, figsize=(13.0, 8.2))
    ax = axes[0, 0]
    cum = np.minimum.accumulate([r["J"] for r in hist_all])
    ax.plot(np.arange(1, len(cum) + 1), cum, "-", color="#c0392b",
            label="累计最优")
    for name, color in [("cma", "#8e44ad"), ("tpe", "#2c3e50"),
                        ("de", "#27ae60"), ("nm", "#e67e22")]:
        if name in opt["results"]:
            J = opt["results"][name]["J"]
            ax.axhline(J, ls="--", lw=0.9, color=color,
                       label=f"{name.upper()} 最优 {J:.3f}")
    ax.set_xlabel("评估次数（按落盘顺序）")
    ax.set_ylabel("目标 J")
    ax.set_title("(a) 累计最优与各优化器结果")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7)

    ax = axes[0, 1]
    qs = [r["params"][0] for r in hist_all]
    js = [r["J"] for r in hist_all]
    ens = [r["params"][1] for r in hist_all]
    sc = ax.scatter(qs, js, c=ens, cmap="viridis", s=18)
    ax.set_xscale("log")
    ax.set_xlabel("Q [m/s²]（对数）")
    ax.set_ylabel("目标 J")
    ax.set_title("(b) 全部评估点：Q–J（颜色=enter_s）")
    ax.grid(True, which="both", alpha=0.3)
    n_hi = sum(1 for j in js if j > 3.0)
    ax.set_ylim(1.2, 3.0)
    ax.text(0.02, 0.95, f"{n_hi} 点 J>3 未显示", transform=ax.transAxes,
            fontsize=8, va="top")
    fig.colorbar(sc, ax=ax, label="enter_s")
    best = opt["best"]
    ax.scatter([best["params"][0]], [best["J"]], marker="*", s=200,
               color="#e74c3c", edgecolor="k", zorder=5, label="最优")
    ax.legend()

    ax = axes[1, 0]
    val = opt["validation"]
    x = np.arange(len(val))
    tr10 = [v["train_metrics"]["thrust10g_icm_full"]["pos"] for v in val]
    te10 = [v["test_metrics"]["thrust10g_icm_full"]["pos"] for v in val]
    ax.bar(x - 0.2, tr10, 0.4, color="#888888", label="train seeds")
    ax.bar(x + 0.2, te10, 0.4, color="#c0392b", label="test seeds")
    ax.set_xticks(x)
    ax.set_xticklabels([f"Q{v['params'][0]:.1f} e{v['params'][1]:.2f}"
                        for v in val], fontsize=7)
    ax.set_ylabel("10.1 g 位置 RMSE 中位 [m]")
    ax.set_title("(c) 候选 vs DE 最优：训练/独立 seeds")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes[1, 1]
    rel = np.array([r["params"][2] for r in hist_all])
    ent = np.array([r["params"][1] for r in hist_all])
    qq = np.array([r["params"][0] for r in hist_all])
    sc = ax.scatter(ent, rel, c=np.log10(np.maximum(qq, 1e-3)),
                    cmap="plasma", s=20)
    ax.set_xlabel("enter_s [s]")
    ax.set_ylabel("release_s [s]")
    ax.set_title("(d) 时间参数覆盖（颜色=log10 Q）")
    ax.grid(True, alpha=0.3)
    fig.colorbar(sc, ax=ax, label="log10 Q")
    fig.tight_layout()
    out = os.path.join(FIG, "nav_sat_q_opt_2026-10-27.png")
    fig.savefig(out, dpi=150)
    print("wrote", out)


    tail_path = os.path.join(DATA, "missile-sil-2026-10-27_satq_final100.jsonl")
    if os.path.exists(tail_path):
        rows = {}
        for line in open(tail_path, encoding="utf-8"):
            r = json.loads(line)
            rows.setdefault(r["config"], []).append(r)
        fig, axes = plt.subplots(1, 2, figsize=(12.5, 4.4))
        colors = {"thrust10g_icm_full": "#c0392b",
                  "thrust8g_icm_full": "#2c3e50",
                  "thrust10g_spoof30_icm_full": "#e67e22"}
        names = {"thrust10g_icm_full": "10.1 g",
                 "thrust8g_icm_full": "8.4 g",
                 "thrust10g_spoof30_icm_full": "10.1 g+spoof30"}
        for cfg, color in colors.items():
            rr = rows.get(cfg, [])
            if not rr:
                continue
            v = np.sort([r["pos_rms"] for r in rr])
            axes[0].plot(v, np.arange(1, len(v) + 1) / len(v), "-",
                         color=color, label=f"{names[cfg]}（max {v.max():.1f}）")
            a = np.sort([r["nav_att_rmse"] for r in rr])
            axes[1].plot(a, np.arange(1, len(a) + 1) / len(a), "-",
                         color=color, label=names[cfg])
        axes[0].set_xscale("log")
        axes[0].set_xlabel("位置 RMSE [m]（对数）")
        axes[0].set_ylabel("CDF")
        axes[0].set_title("(a) 100 seeds 位置误差 CDF")
        axes[0].grid(True, which="both", alpha=0.3)
        axes[0].legend(fontsize=8)
        axes[1].set_xlabel("姿态 RMSE [°]")
        axes[1].set_ylabel("CDF")
        axes[1].set_title("(b) 100 seeds 姿态误差 CDF")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend(fontsize=8)
        fig.tight_layout()
        out = os.path.join(FIG, "nav_sat_q_tail_2026-10-27.png")
        fig.savefig(out, dpi=150)
        print("wrote", out)


    range_data = [
        ("8 g 无补偿", "#c0392b", [
            ("satq_off", "clean_icm_full"), ("satq_off", "thrust8g_icm_full"),
            ("satq_off", "thrust10g_icm_full")]),
        ("8 g + 饱和 Q", "#e67e22", [
            ("satq_cand1_20", "clean_icm_full"),
            ("satq_cand1_20", "thrust8g_icm_full"),
            ("satq_cand1_20", "thrust10g_icm_full")]),
        ("16 g 无 Q", "#2c3e50", [
            ("satq_fs16_off", "thrust8g_icm_full"),
            ("satq_fs16_off", "thrust10g_icm_full"),
            ("fs16_g11000", "thrust10g_icm_full"),
            ("fs16_g12000", "thrust10g_icm_full"),
            ("fs16_g13000", "thrust10g_icm_full")]),
    ]
    fig, axes = plt.subplots(1, 2, figsize=(12.5, 4.4))
    for label, color, items in range_data:
        gs, ps, ats = [], [], []
        for tag, cfg in items:
            path = os.path.join(DATA, f"missile-sil-2026-10-27_{tag}.jsonl")
            if not os.path.exists(path):
                continue
            rr = [json.loads(line) for line in open(path, encoding="utf-8")]
            rr = [r for r in rr if r["config"] == cfg]
            if not rr:
                continue
            gs.append(float(np.median([r["ms_gmax"] for r in rr])))
            ps.append(float(np.median([r["pos_rms"] for r in rr])))
            ats.append(float(np.median([r["nav_att_rmse"] for r in rr])))
        axes[0].plot(gs, ps, "-o", color=color, label=label)
        axes[1].plot(gs, ats, "-o", color=color, label=label)
    axes[0].set_yscale("log")
    axes[0].set_xlabel("真值过载 [g]")
    axes[0].set_ylabel("位置 RMSE 中位 [m]（对数）")
    axes[0].set_title("(a) 量程/补偿 vs 过载：位置")
    axes[0].grid(True, which="both", alpha=0.3)
    axes[0].legend(fontsize=8)
    axes[1].set_xlabel("真值过载 [g]")
    axes[1].set_ylabel("姿态 RMSE 中位 [°]")
    axes[1].set_title("(b) 量程/补偿 vs 过载：姿态")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(fontsize=8)
    fig.tight_layout()
    out = os.path.join(FIG, "nav_sat_q_range_2026-10-27.png")
    fig.savefig(out, dpi=150)
    print("wrote", out)


if __name__ == "__main__":
    main()
