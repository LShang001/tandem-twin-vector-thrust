# -*- coding: utf-8 -*-
"""§4.23 延时网格放宽的成图：曲面、机制、逐 seed 分布。

@file    plot_delay_grid.py
@brief   读 5×5 扫描、逐历元 tau 取证与三臂 A/B 三份产物，出四张图
@module  TandemVec-EKF-Report/作图脚本（L2 研究设施）
@details 图型：
         1 `nav_delay_grid_surface.png`  真延迟 × 网格上界的 pos_rms 热图 + 折线
           （决策面：哪里开关、哪里饱和）；
         2 `nav_delay_tau_mechanism.png` 应用历元 tau 中位 vs 真延迟（y=x 参考线），
           并叠同一格的"末帧 tau"点 ⇒ 一图说清"为什么不能用末帧判命中"；
         3 `nav_delay_grid_cdf.png`      延 150/200/250 ms 三格 G0/G1 的 pos_rms 经验 CDF；
         4 `nav_delay_grid_apps.png`     应用历元数 `ad_app` 热图（开关位置的另一半证据）。
         生成后逐张目视审查（空图、NaN、刻度重叠、被裁的柱/线）。
"""
import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402
from matplotlib.ticker import ScalarFormatter  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_DATA = os.path.normpath(os.path.join(HERE, "..", "sim-data"))
FIGS = os.path.normpath(os.path.join(HERE, "..", "figs"))
SWEEP = os.path.join(SIM_DATA, "delay_grid_sweep_2026-10-28.json")
TAU = os.path.join(SIM_DATA, "delay_traces", "tau_by_epoch_2026-10-28.json")
AB = os.path.join(SIM_DATA, "delay_grid_widen_ab_2026-10-28.json")
GRIDS = [150, 200, 250, 300, 330]
LATS = [150, 200, 250, 300, 330]


def setup_font():
    for name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC"):
        try:
            font_manager.findfont(name, fallback_to_default=False)
            plt.rcParams["font.family"] = name
            break
        except Exception:
            continue
    plt.rcParams["axes.unicode_minus"] = False


def load(p):
    return json.load(open(p, encoding="utf-8")) if os.path.exists(p) else None


def mat(sweep, key):
    m = np.full((len(LATS), len(GRIDS)), np.nan)
    for i, lat in enumerate(LATS):
        for j, g in enumerate(GRIDS):
            k = f"{lat}_{g}"   # 扫描 JSON 的键是 f"{lat}_{grid}"
            if k in sweep["med"]:
                m[i, j] = sweep["med"][k][key]
    return m


def fig_surface(sweep):
    pos = mat(sweep, "pos_rms")
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13.0, 5.0))
    im = ax1.imshow(pos, cmap="viridis_r", aspect="auto",
                    vmin=3.0, vmax=np.nanmax(pos))
    ax1.set_xticks(range(len(GRIDS)))
    ax1.set_xticklabels([f"{g}" for g in GRIDS])
    ax1.set_yticks(range(len(LATS)))
    ax1.set_yticklabels([f"{l}" for l in LATS])
    ax1.set_xlabel("搜索网格上界 [ms]（CAP 同步）")
    ax1.set_ylabel("真实接收机延迟 [ms]")
    for i in range(len(LATS)):
        for j in range(len(GRIDS)):
            if pos[i, j] == pos[i, j]:
                ax1.text(j, i, f"{pos[i, j]:.1f}", ha="center", va="center",
                         fontsize=8,
                         color="w" if pos[i, j] > 20 else "#222222")
    ax1.set_title("pos_rms 中位 [m]（20 seeds）", fontsize=10)
    fig.colorbar(im, ax=ax1, shrink=0.85)
    cols = ["#4C72B0", "#DD8452", "#55A868", "#C44E52", "#8172B3"]
    for j, g in enumerate(GRIDS):
        ax2.plot(LATS, pos[:, j], "o-", color=cols[j], lw=1.5, ms=5,
                 label=f"网格 {g} ms")
    ax2.set_xlabel("真实接收机延迟 [ms]")
    ax2.set_ylabel("pos_rms 中位 [m]")
    ax2.set_title("收益只在『网格 ≥ 约 1.2×真延迟』后出现（端点判无效的必要代价）",
                  fontsize=9.5)
    ax2.grid(alpha=0.25)
    ax2.legend(fontsize=8)
    fig.suptitle("GNSS 延时搜索网格上界 × 真延迟：放宽的开关位置与饱和（tv_hf3 9ef342ca）")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    out = os.path.join(FIGS, "nav_delay_grid_surface.png")
    fig.savefig(out, dpi=145)
    plt.close(fig)
    print("wrote", out)


def fig_mechanism(sweep, tau):
    fig, ax = plt.subplots(figsize=(7.4, 6.4))
    ax.plot([0, 360], [0, 360], ls="--", color="k", lw=1.0, label="tau = 真延迟")
    xs, ys, ls_ = [], [], []
    for key, rows in tau.items():
        g, lat = [int(v) for v in key.split("_")]
        good = [r for r in rows if r["med"] == r["med"]]
        if not good:
            continue
        med_app = st_median([r["med"] for r in good])
        last = st_median([r["last_tau"] for r in good])
        xs.append(lat)
        ys.append(med_app)
        ls_.append(g)
        ax.plot([lat], [med_app], "o", ms=10,
                color="#C44E52" if g > 150 else "#4C72B0")
        ax.annotate(f"末帧 {last:.0f}", (lat, med_app), xytext=(8, -4),
                    textcoords="offset points", fontsize=7.5, color="#555555")
        ax.annotate(f"g{g}", (lat, med_app), xytext=(-20, 6),
                    textcoords="offset points", fontsize=7.5,
                    color="#C44E52" if g > 150 else "#4C72B0")
    # 同一真延迟下"默认网格 vs 放宽网格"用竖线连起来（把两条混在一条折线上会误导）
    by_lat = {}
    for key, rows in tau.items():
        g, lat = [int(v) for v in key.split("_")]
        good = [r for r in rows if r["med"] == r["med"]]
        if good:
            by_lat.setdefault(lat, {})[g] = st_median([r["med"] for r in good])
    for lat, d in by_lat.items():
        if 150 in d and 300 in d:
            ax.plot([lat, lat], [d[150], d[300]], "-", color="#999999", lw=1.0,
                    zorder=1)
    ax.set_xlabel("真实接收机延迟 [ms]")
    ax.set_ylabel("被应用历元上的 tau 中位 [ms]")
    ax.set_xlim(130, 360)
    ax.set_ylim(0, 340)
    ax.grid(alpha=0.25)
    from matplotlib.patches import Patch
    ax.legend(handles=[plt.Line2D([], [], ls="--", color="k", label="tau = 真延迟"),
                       Patch(facecolor="#4C72B0", label="默认网格 150 ms"),
                       Patch(facecolor="#C44E52", label="放宽网格 300 ms")],
              fontsize=8, loc="upper left")
    fig.suptitle("机制：放宽后 tau 跟住真延迟；末帧值差 50–100 ms（第 9 次末帧坑）")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    out = os.path.join(FIGS, "nav_delay_tau_mechanism.png")
    fig.savefig(out, dpi=145)
    plt.close(fig)
    print("wrote", out)


def st_median(v):
    import statistics
    return statistics.median(v)


def fig_apps(sweep):
    app = mat(sweep, "ad_app")
    rej = mat(sweep, "outlier_rejects")
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13.0, 4.8))
    for name, ax, data, fmt in (("tau 被应用历元数（中位）", ax1, app, "{:.0f}"),
                                ("GNSS 整帧拒绝次数（中位）", ax2, rej, "{:.0f}")):
        im = ax.imshow(data, cmap="magma_r", aspect="auto", vmin=0,
                       vmax=np.nanmax(data) if np.isfinite(np.nanmax(data)) else 1)
        ax.set_xticks(range(len(GRIDS)))
        ax.set_xticklabels([str(g) for g in GRIDS])
        ax.set_yticks(range(len(LATS)))
        ax.set_yticklabels([str(l) for l in LATS])
        ax.set_xlabel("搜索网格上界 [ms]")
        ax.set_ylabel("真延迟 [ms]")
        ax.set_title(name, fontsize=10)
        for i in range(data.shape[0]):
            for j in range(data.shape[1]):
                if data[i, j] == data[i, j]:
                    ax.text(j, i, fmt.format(data[i, j]), ha="center", va="center",
                            fontsize=7.5,
                            color="w" if data[i, j] > 0.55 * np.nanmax(data) else "#222")
        fig.colorbar(im, ax=ax, shrink=0.85)
    fig.suptitle("开关的另一半证据：应用次数随网格放宽跳变，整帧拒绝多数档反而下降")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    out = os.path.join(FIGS, "nav_delay_grid_apps.png")
    fig.savefig(out, dpi=145)
    plt.close(fig)
    print("wrote", out)


def fig_cdf(ab):
    cells = [c for c in ("延150ms", "延200ms", "延250ms") if c in ab["rows"]]
    fig, axes = plt.subplots(1, len(cells), figsize=(4.6 * len(cells), 4.2))
    if len(cells) == 1:
        axes = [axes]
    for ax, c in zip(axes, cells):
        for arm, col, lab in (("G0 默认", "#4C72B0", "网格150/CAP100(默认)"),
                              ("G1 网格300+CAP300", "#C44E52", "网格300+CAP300")):
            v = np.sort([r["pos_rms"] for r in ab["rows"][c]["rows"][arm]])
            if v.size == 0:
                continue
            ax.step(v, np.arange(1, v.size + 1) / v.size, where="post", color=col,
                    lw=1.8, label=lab)
        ax.set_xscale("log")
        ax.xaxis.set_major_formatter(ScalarFormatter())
        ax.xaxis.set_minor_locator(plt.NullLocator())
        ax.set_xlabel("pos_rms [m]")
        ax.set_title(c, fontsize=10)
        ax.grid(alpha=0.25)
        ax.set_ylim(0, 1.02)
    axes[0].set_ylabel("经验 CDF")
    axes[0].legend(fontsize=8, loc="lower right")
    fig.suptitle("放宽网格/CAP 的位置误差分布（同二进制 tv_hf3，20 seeds）")
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    out = os.path.join(FIGS, "nav_delay_grid_cdf.png")
    fig.savefig(out, dpi=145)
    plt.close(fig)
    print("wrote", out)


def main():
    setup_font()
    os.makedirs(FIGS, exist_ok=True)
    sweep, tau, ab = load(SWEEP), load(TAU), load(AB)
    if sweep:
        fig_surface(sweep)
        fig_apps(sweep)
    else:
        print("skip sweep figs:", SWEEP)
    if tau:
        fig_mechanism(sweep, tau)
    else:
        print("skip mechanism fig:", TAU)
    if ab:
        fig_cdf(ab)
    else:
        print("skip cdf fig:", AB)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
