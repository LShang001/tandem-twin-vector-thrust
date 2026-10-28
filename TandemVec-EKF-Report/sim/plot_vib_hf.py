# -*- coding: utf-8 -*-
"""高频振动通道取证 + 三臂 A/B 的成图（时序/箱线/CDF/散点/热图）。

@file    plot_vib_hf.py
@brief   读 `sim-data/vib_hf_channel_2026-10-27.json`、`vibhf_traces/*.csv` 与
         `sim-data/vib_hf_ab_2026-10-28.json`，产出 §4.22.4 需要的全部图
@module  TandemVec-EKF-Report/作图脚本（L2 研究设施）
@details 图型齐全是项目约定（时序/箱线/CDF/散点/热图），本脚本一张图各出一 PNG：
         1 `nav_vib_hf_timeseries.png`  洁净/振1g/振2g 三格，`vib_hf_g` 与旧
           `vib_rms_g` 时序 + 进入门限 0.15 g 水平线；
         2 `nav_vib_hf_separation.png`  两通道中位与占空比的分组条形（可分性一眼可见）；
         3 `nav_vib_hf_ab_box.png`      三臂 pos_rms 箱线（8 工况 × 3 臂）；
         4 `nav_vib_hf_ab_cdf.png`      关键三格（洁净/振1g/振2g）pos_rms 经验 CDF；
         5 `nav_vib_hf_ab_scatter.png`  ad_app vs pos_rms 散点，按工况着色
           （检验"挡住"与"误差"之间的关系是否真的存在）；
         6 `nav_vib_hf_ab_heat.png`     工况×臂 的 pos_rms / A 臂比值热图。
         生成后必须逐张目视审查（空图、NaN、坐标尺度、标签重叠）。
"""
import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402
import numpy as np  # noqa: E402


def setup_font():
    for name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC"):
        try:
            font_manager.findfont(name, fallback_to_default=False)
            plt.rcParams["font.family"] = name
            break
        except Exception:
            continue
    plt.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_DATA = os.path.join(HERE, "..", "sim-data")
FIGS = os.path.join(HERE, "..", "figs")
CHANNEL_JSON = os.path.join(SIM_DATA, "vib_hf_channel_2026-10-27.json")
AB_JSON = os.path.join(SIM_DATA, "vib_hf_ab_2026-10-28.json")
TRACES = os.path.join(SIM_DATA, "vibhf_traces")
ENTER_G, RELEASE_G = 0.15, 0.10
ARMS = ["A veto关(默认)", "B veto开+HF", "C veto开+中频"]
ARM_SHORT = ["A 否决关", "B 否决开+高频", "C 否决开+中频"]
COLORS = ["#4C72B0", "#DD8452", "#55A868"]


def load_json(path):
    if not os.path.exists(path):
        return None
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def read_trace(name):
    path = os.path.join(TRACES, name)
    if not os.path.exists(path):
        return None
    with open(path, encoding="utf-8") as fh:
        hdr = fh.readline().strip().split(",")
        rows = np.loadtxt(path, delimiter=",", skiprows=1)
    if rows.ndim == 1:
        rows = rows.reshape(1, -1)
    return {c: rows[:, i] for i, c in enumerate(hdr)}


def fig_timeseries():
    panels = [("洁净_1.csv", "洁净（无振动，6.4 g 机动）"),
              ("振1g_8.4g_1.csv", "振 1 g@80 Hz + 8.4 g"),
              ("振2g_8.4g_1.csv", "振 2 g@120 Hz + 8.4 g")]
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    any_data = False
    for ax, (fn, title) in zip(axes, panels):
        d = read_trace(fn)
        if d is None:
            ax.text(0.5, 0.5, f"缺迹线 {fn}", ha="center", va="center")
            ax.set_title(title)
            continue
        any_data = True
        t = d["t"]
        ax.plot(t, d["vib_hf_g"], lw=1.0, color="#C44E52",
                label="HF 通道 raw−LPF(raw)")
        ax.plot(t, d["vib_rms_g"], lw=1.0, color="#4C72B0",
                label="旧通道 0.5 Hz 高通(在 LPF 之后)")
        ax.axhline(ENTER_G, ls="--", lw=0.9, color="k", label="进入门限 0.15 g")
        ax.axhline(RELEASE_G, ls=":", lw=0.9, color="gray", label="释放 0.10 g")
        ax.set_xlim(0, t.max())
        ax.set_ylim(0, max(1.1 * np.nanmax(d["vib_hf_g"]), 0.35))
        ax.set_title(title, fontsize=10, loc="left")
        ax.grid(alpha=0.25)
    axes[0].legend(ncol=2, fontsize=8, loc="upper right")
    axes[-1].set_xlabel("时间 [s]")
    for ax in axes:
        ax.set_ylabel("RMS [g]")
    fig.suptitle("高频振动通道 vs 旧中频通道（导弹 6DOF SIL，seed1，200 Hz 迹线）")
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    out = os.path.join(FIGS, "nav_vib_hf_timeseries.png")
    fig.savefig(out, dpi=145)
    plt.close(fig)
    print("wrote", out, "data" if any_data else "NO-DATA")


def fig_separation(ch):
    conds = list(ch["rows"].keys())
    x = np.arange(len(conds))
    fig, (ax1, a2) = plt.subplots(1, 2, figsize=(11, 4.2))
    # 左图用点图：对数轴上画条形会拖出视觉长尾，误导量级对比。
    hf = [ch["rows"][c]["agg"]["hf_med"] for c in conds]
    mid = [ch["rows"][c]["agg"]["mid_med"] for c in conds]
    ax1.plot(hf, x, "o", ms=9, color="#C44E52", label="HF 通道中位", zorder=3)
    ax1.plot(mid, x, "s", ms=9, mfc="none", mew=1.6, color="#4C72B0",
             label="旧中频通道中位", zorder=3)
    for xi, (h, mm) in enumerate(zip(hf, mid)):
        ax1.annotate(f"{h:.3f}", (h, xi), xytext=(-6, 0), textcoords="offset points",
                     ha="right", va="center", fontsize=7.5, color="#C44E52")
        ax1.annotate(f"{mm:.3f}", (mm, xi), xytext=(7, 0), textcoords="offset points",
                     ha="left", va="center", fontsize=7.5, color="#4C72B0")
    ax1.axvline(ENTER_G, ls="--", color="k", lw=0.9)
    ax1.text(ENTER_G, len(conds) - 0.35, " 进入门限 0.15 g", fontsize=7.5)
    ax1.set_yticks(x)
    ax1.set_yticklabels(conds, fontsize=8)
    ax1.set_ylim(-0.6, len(conds) - 0.4)
    ax1.set_xscale("log")
    ax1.set_xlim(0.006, 3.0)
    ax1.set_xlabel("RMS 中位 [g]（对数轴）")
    ax1.set_title("中位幅值可分性：HF 68×，旧通道 1.7×", fontsize=10)
    ax1.legend(fontsize=8, loc="lower right")
    ax1.grid(alpha=0.25, axis="x", which="both")
    duty = [100 * ch["rows"][c]["agg"]["high_duty"] for c in conds]
    a2.bar(x, duty, color="#55A868")
    a2.set_xticks(x)
    a2.set_xticklabels(conds, fontsize=8, rotation=20, ha="right")
    a2.set_ylim(0, 110)
    a2.set_ylabel("vib_high 占空 [%]（HF 源）")
    a2.set_title("置位占空比：洁净 8% vs 振动 98%", fontsize=10)
    for xi, v in zip(x, duty):
        a2.text(xi, v + 2, f"{v:.0f}%", ha="center", fontsize=8)
    a2.grid(alpha=0.25, axis="y")
    fig.suptitle("高频振动通道的可分性取证（20/40 s 段，seeds 1–3 中位）")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    out = os.path.join(FIGS, "nav_vib_hf_separation.png")
    fig.savefig(out, dpi=145)
    plt.close(fig)
    print("wrote", out)


def fig_ab_box(ab):
    conds = list(ab["rows"].keys())
    n_arm = len(ARMS)
    fig, ax = plt.subplots(figsize=(13.0, 5.6))
    for i, c in enumerate(conds):
        for j, arm in enumerate(ARMS):
            vals = [r["pos_rms"] for r in ab["rows"][c]["rows"][arm]]
            if not vals:
                continue
            bp = ax.boxplot(vals, positions=[i * (n_arm + 1) + j], widths=0.82,
                            patch_artist=True, showfliers=False,
                            medianprops=dict(color="k", lw=1.2))
            for p in bp["boxes"]:
                p.set_facecolor(COLORS[j])
                p.set_alpha(0.8)
        if i:
            ax.axvline(i * (n_arm + 1) - 0.5, color="gray", lw=0.7, ls="--")
    ax.set_xticks([i * (n_arm + 1) + (n_arm - 1) / 2 for i in range(len(conds))])
    ax.set_xticklabels(conds, fontsize=8.5)
    ax.set_xlim(-0.7, len(conds) * (n_arm + 1) - n_arm + 0.2)
    ax.set_ylim(1.5, 60)
    ax.set_yscale("log")
    ax.set_ylabel("pos_rms [m]（对数轴）")
    ax.grid(alpha=0.25, axis="y", which="both")
    handles = [plt.Rectangle((0, 0), 1, 1, color=c, alpha=0.8) for c in COLORS]
    ax.legend(handles, ARM_SHORT, fontsize=8, ncol=3, loc="upper left")
    fig.suptitle("三臂 A/B：pos_rms 分布（同二进制 tv_hf a826c9a7，20 seeds/格，"
                 "绑 core2）")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    out = os.path.join(FIGS, "nav_vib_hf_ab_box.png")
    fig.savefig(out, dpi=145)
    plt.close(fig)
    print("wrote", out)


def fig_ab_cdf(ab):
    key_conds = [c for c in ("洁净", "振1g_8.4g", "振2g_8.4g", "延100ms")
                 if c in ab["rows"]]
    fig, axes = plt.subplots(1, len(key_conds), figsize=(4.1 * len(key_conds), 4.0),
                             sharey=False)
    if len(key_conds) == 1:
        axes = [axes]
    for ax, c in zip(axes, key_conds):
        allv = np.concatenate([np.array([r["pos_rms"] for r in ab["rows"][c]["rows"][a]])
                               for a in ARMS if ab["rows"][c]["rows"][a]])
        for arm, col, short in zip(ARMS, COLORS, ARM_SHORT):
            vals = np.sort([r["pos_rms"] for r in ab["rows"][c]["rows"][arm]])
            if vals.size == 0:
                continue
            ax.step(vals, np.arange(1, vals.size + 1) / vals.size, where="post",
                    color=col, lw=1.4, label=short)
        ax.set_xscale("log")
        # 对数轴默认次刻度会挤成一团（标签重叠不可读）⇒ 显式少数刻度 + 十进制标签。
        from matplotlib.ticker import ScalarFormatter
        cand = [v for v in (1, 2, 3, 5, 10, 20, 30, 50, 100)
                if allv.min() * 0.8 <= v <= allv.max() * 1.25]
        if len(cand) >= 2:
            ax.set_xticks(cand)
        ax.xaxis.set_major_formatter(ScalarFormatter())
        ax.xaxis.set_minor_locator(plt.NullLocator())
        ax.set_xlabel("pos_rms [m]")
        ax.set_title(c, fontsize=10)
        ax.grid(alpha=0.25)
        ax.set_ylim(0, 1.02)
    axes[0].set_ylabel("经验 CDF")
    axes[0].legend(fontsize=7.5, loc="lower right")
    fig.suptitle("三臂 pos_rms 经验 CDF（20 seeds）")
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    out = os.path.join(FIGS, "nav_vib_hf_ab_cdf.png")
    fig.savefig(out, dpi=145)
    plt.close(fig)
    print("wrote", out)


def fig_ab_scatter(ab):
    fig, ax = plt.subplots(figsize=(9, 6))
    cmap = plt.get_cmap("tab10")
    for i, c in enumerate(ab["rows"]):
        xs, ys = [], []
        for arm in ARMS:
            for r in ab["rows"][c]["rows"][arm]:
                xs.append(r.get("ad_app", 0))
                ys.append(r.get("pos_rms", np.nan))
        ax.scatter(xs, ys, s=16, alpha=0.6, color=cmap(i % 10), label=c)
    ax.axvline(0.5, color="k", lw=0.6, ls=":")
    ax.set_xlim(-6, max(xs) * 1.06)
    ax.set_yscale("log")
    ax.set_xlabel("延时估计被应用的历元数 ad_app（0=否决全程挡住）")
    ax.set_ylabel("pos_rms [m]")
    ax.grid(alpha=0.25, which="both")
    ax.legend(fontsize=7, ncol=2, loc="upper left")
    fig.suptitle("应用次数 vs 位置误差：振 2 g 被挡住(x=0)回到 3 m；"
                 "延 200 ms 无论挡不挡都停在 38 m ⇒ 否决只防'补错'，不补'欠补偿'")
    fig.tight_layout()
    out = os.path.join(FIGS, "nav_vib_hf_ab_scatter.png")
    fig.savefig(out, dpi=145)
    plt.close(fig)
    print("wrote", out)


def fig_ab_heat(ab):
    conds = list(ab["rows"].keys())
    mat = np.zeros((len(conds), len(ARMS)))
    for i, c in enumerate(conds):
        ref = ab["rows"][c]["med"][ARMS[0]]["pos_rms"]
        for j, arm in enumerate(ARMS):
            v = ab["rows"][c]["med"][arm]["pos_rms"]
            mat[i, j] = v / ref if ref > 1e-9 else np.nan
    fig, ax = plt.subplots(figsize=(6.2, 6.6))
    im = ax.imshow(mat, cmap="RdYlBu_r", vmin=0.2, vmax=3.0, aspect="auto")
    ax.set_xticks(range(len(ARMS)))
    ax.set_xticklabels(ARM_SHORT, fontsize=8)
    ax.set_yticks(range(len(conds)))
    ax.set_yticklabels(conds, fontsize=8)
    for i in range(len(conds)):
        for j in range(len(ARMS)):
            ax.text(j, i, f"{mat[i, j]:.2f}", ha="center", va="center", fontsize=7)
    ax.set_title("pos_rms / A 臂（1.0=与默认相同）")
    fig.colorbar(im, ax=ax, shrink=0.8, label="相对 A 臂倍数")
    fig.tight_layout()
    out = os.path.join(FIGS, "nav_vib_hf_ab_heat.png")
    fig.savefig(out, dpi=145)
    plt.close(fig)
    print("wrote", out)


def fig_enter_scan(scan):
    """门限扫描决策图：8 工况 × 5 臂的 pos_rms 中位（相对 A 臂倍数），标出选中臂。"""
    arms = list(next(iter(scan["rows"].values()))["med"].keys())
    conds = list(scan["rows"].keys())
    fig, ax = plt.subplots(figsize=(13.0, 5.6))
    n = len(arms)
    cmap = ["#4C72B0", "#B04C4C", "#DD8452", "#55A868", "#8172B3"]
    for i, c in enumerate(conds):
        for j, arm in enumerate(arms):
            v = scan["rows"][c]["med"][arm]["pos_rms"]
            ref = scan["rows"][c]["med"][arms[0]]["pos_rms"]
            r = v / ref if ref else np.nan
            ax.bar(i * (n + 1) + j, r, width=0.86, color=cmap[j % len(cmap)],
                   alpha=0.85)
            ax.text(i * (n + 1) + j, r, f"{r:.2f}", ha="center", va="bottom",
                    fontsize=6.5)
        if i:
            ax.axvline(i * (n + 1) - 0.5, color="gray", lw=0.7, ls="--")
    ax.axhline(1.0, color="k", lw=0.9)
    ax.text(len(conds) * (n + 1) - n + 0.3, 1.0, " 1.00 = A", fontsize=7, va="bottom")
    ax.set_xticks([i * (n + 1) + (n - 1) / 2 for i in range(len(conds))])
    ax.set_xticklabels(conds, fontsize=8.5)
    ax.set_xlim(-0.8, len(conds) * (n + 1) - n + 0.8)
    ax.set_ylim(0.05, 6.0)
    ax.set_yscale("log")
    from matplotlib.ticker import ScalarFormatter
    ax.set_yticks([0.1, 0.2, 0.5, 1.0, 2.0, 4.0])
    ax.yaxis.set_major_formatter(ScalarFormatter())
    ax.yaxis.set_minor_locator(plt.NullLocator())
    ax.axhline(1.05, color="k", lw=0.7, ls=":")
    ax.text(-0.7, 1.08, "S2 零代价上界 1.05×A", fontsize=7)
    ax.set_ylabel("pos_rms / A 臂（否决关=默认，对数轴）")
    ax.grid(alpha=0.25, axis="y")
    handles = [plt.Rectangle((0, 0), 1, 1, color=c, alpha=0.85) for c in cmap[:n]]
    ax.legend(handles, arms, fontsize=7.5, ncol=5, loc="upper left")
    ch = scan.get("chosen")
    fig.suptitle("高频振动否决的进入门限扫描（同二进制 tv_hf2 bcae4d1e，20 seeds/格）"
                 + f" —— 按 S1+S2+S3+S4 选中：{ch or '无（保持默认否决关）'}")
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    out = os.path.join(FIGS, "nav_vib_hf_enter_scan.png")
    fig.savefig(out, dpi=145)
    plt.close(fig)
    print("wrote", out)


def main():
    setup_font()
    os.makedirs(FIGS, exist_ok=True)
    ch = load_json(CHANNEL_JSON)
    ab = load_json(AB_JSON)
    scan = load_json(os.path.join(SIM_DATA, "vib_hf_enter_scan_2026-10-28.json"))
    if ch:
        fig_timeseries()
        fig_separation(ch)
    else:
        print("skip channel figs:", CHANNEL_JSON, "missing")
    if ab:
        fig_ab_box(ab)
        fig_ab_cdf(ab)
        fig_ab_scatter(ab)
        fig_ab_heat(ab)
    else:
        print("skip A/B figs:", AB_JSON, "missing")
    if scan:
        fig_enter_scan(scan)
    else:
        print("skip scan figs: vib_hf_enter_scan_2026-10-28.json missing")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
