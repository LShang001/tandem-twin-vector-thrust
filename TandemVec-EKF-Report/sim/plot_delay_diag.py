# -*- coding: utf-8 -*-
"""自动 GNSS 延时估计的病因分解配图（宿主合成弹道探针 + 导弹 SIL 三臂 A/B）。

@file    plot_delay_diag.py
@brief   把 τ̂ 时间序列、代价曲线两端、粗差剔除计数、A/B 位置误差分布画成可复核面板
@module  TandemVec-EKF-Report/作图脚本（L2 研究设施）
@details 面板：(a) τ̂ vs 时间（按 fdi×网格×真延时）；(b) cost(0) 与 cost(τ_true) 对照，
         判"信息在不在数据里"；(c) GNSS 缓存样本数与被剔除数 vs 时间（M3 证据）；
         (d) 锐度 vs 时间并叠加 0.003 应用门限（假置信证据）；
         (e) 有 A/B JSON 时：位置 RMSE 随注入延迟的分组条形 + (f) 200 ms 档 CDF。
@frame   NED；时间 s；延时 ms；位置 m
@units   见各面板轴标签

用法：
  py -3.12 sim/plot_delay_diag.py --probe-dir "$TMP" --out figs/nav_delay_diag_2026-10-27.png
"""
import argparse
import glob
import json
import os
import re

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager


def setup_font():
    for name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC"):
        try:
            font_manager.findfont(name, fallback_to_default=False)
            plt.rcParams["font.family"] = name
            break
        except Exception:
            continue
    plt.rcParams["axes.unicode_minus"] = False


LINE = re.compile(
    r"t=\s*([\d.]+)\s+v=\s*([\d.-]+)\s+ok=(\d)\s+rej=(\S+)\s+nref=\s*(\d+)\s+"
    r"ng=\s*(\d+)\s+er=\s*(\d+)\s+span=\s*([\d.-]+)\s+smax=\s*([\d.-]+)\s+"
    r"tau_hat=\s*([\d.-]+)\s+ms\s+\(true\s+([\d.]+)\)\s+res=\s*([\d.eE+-]+)\s+"
    r"sharp=\s*([\d.]+)\s+cost\(0\)=\s*([\d.eE+-]+)\s+cost\(true\)=\s*([\d.eE+-]+)"
)
FIELDS = ["t", "v", "ok", "rej", "nref", "ng", "er", "span", "smax",
          "tau_hat", "tau_true", "res", "sharp", "cost0", "costtrue"]


def load_probe(path):
    rows = []
    with open(path, encoding="utf-8", errors="replace") as fh:
        for ln in fh:
            m = LINE.match(ln.strip())
            if not m:
                continue
            d = dict(zip(FIELDS, m.groups()))
            for k in FIELDS:
                if k not in ("rej",):
                    d[k] = float(d[k])
            rows.append(d)
    return rows


def series(rows, key):
    return np.array([r[key] for r in rows], dtype=float)


def main():
    setup_font()
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe-dir", required=True)
    ap.add_argument("--ab-json", default=None)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.probe_dir, "p_fdi*_t*.txt")))
    if not files:
        raise SystemExit(f"没有探针文件：{args.probe_dir}/p_fdi*_t*.txt")
    data = {}
    for f in files:
        base = os.path.basename(f)[2:-4]          # fdi0_grid0150_t0200
        data[base] = load_probe(f)
    for k, v in data.items():
        print(f"{k}: {len(v)} 行")

    fig, ax = plt.subplots(3, 2, figsize=(14, 13))
    colors = plt.cm.tab10(np.linspace(0, 1, max(len(data), 3)))

    for i, (k, rows) in enumerate(sorted(data.items())):
        c = colors[i % 10]
        ax[0, 0].plot(series(rows, "t"), series(rows, "tau_hat"),
                      label=k, color=c, lw=1.6)
        ax[0, 1].plot(series(rows, "t"), series(rows, "cost0"),
                      label=f"{k} cost(0)", color=c, ls="-", lw=1.2)
        ax[0, 1].plot(series(rows, "t"), series(rows, "costtrue"),
                      label=f"{k} cost(τ_true)", color=c, ls="--", lw=1.2)
        ax[1, 0].plot(series(rows, "t"), series(rows, "ng"),
                      label=f"{k} n_gnss", color=c, lw=1.6)
        ax[1, 1].plot(series(rows, "t"), series(rows, "sharp"),
                      label=k, color=c, lw=1.6)

    ax[0, 0].axhline(150, color="k", ls=":", lw=1)
    ax[0, 0].text(45.5, 152, "默认网格上界 150 ms", fontsize=8, ha="right")
    for key, rows in data.items():
        if key.startswith("fdi0_grid0150"):
            ax[0, 0].axhline(rows[0]["tau_true"], color="crimson", ls="-.", lw=1)
    ax[0, 0].set_xlabel("时间 [s]")
    ax[0, 0].set_ylabel("τ̂ [ms]")
    ax[0, 0].set_title("(a) τ̂ 时间序列：端点钉位 / 随机游走 / 归零")
    ax[0, 0].legend(fontsize=6, ncol=2)

    ax[0, 1].set_yscale("log")
    ax[0, 1].set_xlabel("时间 [s]")
    ax[0, 1].set_ylabel("代价（SVD 拟合残差范数）")
    ax[0, 1].set_title("(b) cost(0) vs cost(τ_true)：信息在不在数据里")
    ax[0, 1].legend(fontsize=5, ncol=2)

    ax[1, 0].axhline(20, color="crimson", ls=":", lw=1)
    ax[1, 0].text(0.5, 24, "min_gnss=20（低于即 OutlierStarved）", fontsize=7)
    ax[1, 0].set_xlabel("时间 [s]")
    ax[1, 0].set_ylabel("参与拟合的 GNSS 样本数")
    ax[1, 0].set_title("(c) M3：固定 50 m 粗差门把缓存删空（200→42）")

    ax[1, 1].axhline(0.003, color="k", ls=":", lw=1)
    ax[1, 1].text(0.5, 0.0036, "应用门限 sharp≥0.003", fontsize=7)
    ax[1, 1].set_yscale("symlog")
    ax[1, 1].set_xlabel("时间 [s]")
    ax[1, 1].set_ylabel("锐度")
    ax[1, 1].set_title("(d) 锐度：真极小值 vs 端点假置信")

    # (e) τ̂ 误差 vs 速度分箱（散点）
    for k, rows in sorted(data.items()):
        if not k.endswith("t0200"):
            continue
        ax[2, 0].scatter(series(rows, "v"), series(rows, "tau_hat"),
                         label=k, s=14, alpha=0.8)
    ax[2, 0].axhline(200, color="crimson", ls="-.", lw=1)
    ax[2, 0].set_xlabel("窗口最大参考速度 [m/s]")
    ax[2, 0].set_ylabel("τ̂ [ms]")
    ax[2, 0].set_title("(e) 真延时 200 ms：τ̂ 随速度的失效轨迹")
    ax[2, 0].legend(fontsize=6)

    # (f) A/B 位置 RMSE 或 探针速度-代价热图
    if args.ab_json and os.path.exists(args.ab_json):
        with open(args.ab_json, encoding="utf-8") as fh:
            ab = json.load(fh)
        rows = ab.get("rows", [])
        arms = sorted({r["arm"] for r in rows})
        lats = sorted({r["lat"] for r in rows})
        x = np.arange(len(lats))
        w = 0.8 / max(len(arms), 1)
        for j, arm in enumerate(arms):
            vals = [next((r["pos_rms_med"] for r in rows
                          if r["arm"] == arm and r["lat"] == lat), np.nan)
                    for lat in lats]
            ax[2, 1].bar(x + (j - (len(arms) - 1) / 2.0) * w, vals, width=w,
                         label=arm)
        ax[2, 1].set_xticks(x)
        ax[2, 1].set_xticklabels([str(l) for l in lats])
        ax[2, 1].set_xlabel("注入接收机延迟 [ms]")
        ax[2, 1].set_ylabel("位置 RMSE 中位 [m]")
        ax[2, 1].set_title("(f) SIL 三臂端到端 A/B")
        ax[2, 1].legend(fontsize=6)
    else:
        rows = [r for k, v in data.items() if k.endswith("t0200") for r in v]
        smax = np.array([r["smax"] for r in rows])
        ratio = np.array([r["costtrue"] / max(r["cost0"], 1e-9) for r in rows])
        sc = ax[2, 1].scatter(smax, ratio, c=np.array([r["sharp"] for r in rows]),
                              s=16, norm=matplotlib.colors.Normalize(vmin=0, vmax=2))
        fig.colorbar(sc, ax=ax[2, 1], label="锐度")
        ax[2, 1].axhline(1.0, color="k", ls=":", lw=1)
        ax[2, 1].set_xlabel("窗口最大速度 [m/s]")
        ax[2, 1].set_ylabel("cost(τ_true) / cost(0)")
        ax[2, 1].set_title("(f) 比值 <1 ⇒ 真延时代价更低（信息存在）")

    fig.suptitle("无 PPS 自动 GNSS 延时估计的失效分解（宿主合成弹道，L2）")
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    fig.savefig(args.out, dpi=125)
    print("figure:", args.out)


if __name__ == "__main__":
    main()
