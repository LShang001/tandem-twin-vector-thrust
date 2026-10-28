# -*- coding: utf-8 -*-
"""30.5 g 生产路径退化归因的成图（分布/时序/CDF/散点/热图/2×2 因子）。

@file    plot_305g_attrib.py
@brief   读 `sim-data/attrib305_*.jsonl` 与 `attrib305_traces/*.csv`，产出 §4.24
         需要的全部图型，生成后必须逐张目视审查（空图、NaN、坐标尺度、标签）
@module  TandemVec-EKF-Report/作图脚本（L2 研究设施）
@details 六张图：
         1 `nav_305g_attrib_box.png`    各臂 pos_rms 箱线（群体分布 + 尾部是否还在）
         2 `nav_305g_attrib_cdf.png`    各臂经验 CDF（"尾部 7/50" vs "整体平移"一眼可分）
         3 `nav_305g_paired.png`        逐 seed 配对散点（相对 A_prod），看有无 seed 变差
         4 `nav_305g_factor2x2.png`     LPF × AHRS 2×2 中位（位置 + 速度两面板）
         5 `nav_305g_metrics_heat.png`  臂 × 指标 归一化热图（拒帧/陈旧峰值/τ̂ 等）
         6 `nav_305g_timeseries.png`    seed1 追踪：位置误差与 GNSS 新息随时间
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
SIM = os.path.normpath(os.path.join(HERE, "..", "sim-data"))
FIG = os.path.normpath(os.path.join(HERE, "..", "figs"))
ARM_ORDER = ["A_prod", "B_nolpf", "C_noahrs", "D_neither", "E_hz20",
             "F_hz20_nolpf", "G_navraw", "H_navraw_hz20",
             "I_navraw_lat200", "J_prod_lat200", "K_prod_lat200_wide",
             "L_navraw_lat200_wide"]
ARM_LABEL = {
    "A_prod": "A 生产路径 10 Hz",
    "B_nolpf": "B 关 LPF 10 Hz",
    "C_noahrs": "C 关 AHRS 10 Hz",
    "D_neither": "D 关 LPF+AHRS（旧口径）",
    "E_hz20": "E 生产路径 20 Hz",
    "F_hz20_nolpf": "F 20 Hz 关 LPF",
    "G_navraw": "G 导航增量绕过 LPF",
    "H_navraw_hz20": "H 绕过 LPF + 20 Hz",
    "I_navraw_lat200": "I 绕过 LPF + 真延 200 ms",
    "J_prod_lat200": "J 生产路径 + 真延 200 ms",
    "K_prod_lat200_wide": "K J + 网格/CAP 300",
    "L_navraw_lat200_wide": "L K + 绕过 LPF",
}


def setup_font():
    for name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC"):
        try:
            font_manager.findfont(name, fallback_to_default=False)
            plt.rcParams["font.family"] = name
            break
        except Exception:
            continue
    plt.rcParams["axes.unicode_minus"] = False


def load(tags):
    rows = []
    for t in tags:
        p = os.path.join(SIM, f"attrib305_{t}.jsonl")
        if not os.path.exists(p):
            print(f"[skip] missing {p}")
            continue
        with open(p, encoding="utf-8") as fh:
            for ln in fh:
                ln = ln.strip()
                if ln:
                    rows.append(json.loads(ln))
    return rows


def series(rows, key):
    """按 (arm, seed) 收敛：同一臂在多个 tag 里重复出现时只留一条，
    否则 A_prod 会被 50-seed 主矩阵与 20-seed 附加矩阵双计，中位数被加权。"""
    out = {}
    for r in rows:
        v = r.get(key)
        if isinstance(v, (int, float)) and np.isfinite(v):
            out.setdefault(r["arm"], {})[r["seed"]] = float(v)
    return {a: sorted(d.items()) for a, d in out.items()}


def vals(d, arm):
    return [v for _, v in d.get(arm, [])]


def main(tags):
    setup_font()
    os.makedirs(FIG, exist_ok=True)
    rows = load(tags)
    if not rows:
        sys.exit("no rows")
    pr = series(rows, "pos_rms")
    arms = [a for a in ARM_ORDER if a in pr]
    print("arms:", {a: len(pr[a]) for a in arms})

    # 1 箱线：归因臂与"真延 200 ms"臂分面板（后者量级差 4 倍，同轴会把前者压平）
    import textwrap
    grp = [[a for a in arms if a in ("A_prod", "B_nolpf", "C_noahrs", "D_neither",
                                     "E_hz20", "F_hz20_nolpf", "G_navraw",
                                     "H_navraw_hz20")],
           [a for a in arms if a in ("I_navraw_lat200", "J_prod_lat200",
                                     "K_prod_lat200_wide", "L_navraw_lat200_wide")]]
    ttl = ["LPF×AHRS 归因臂 + 候选修复", "叠加真延 200 ms（估计器未参与）"]
    fig, axes = plt.subplots(1, 2, figsize=(11.4, 5.0))
    for ax, sel, t in zip(axes, grp, ttl):
        if not sel:
            ax.set_visible(False)
            continue
        data = [np.array(vals(pr, a)) for a in sel]
        ax.boxplot(data, tick_labels=[textwrap.fill(ARM_LABEL[a], 9) for a in sel],
                   showfliers=True, flierprops=dict(marker=".", ms=4, alpha=.6))
        ax.set_title(t, fontsize=10)
        ax.grid(axis="y", alpha=.3)
        ax.tick_params(axis="x", labelsize=7)
    axes[0].set_ylabel("位置 RMSE (m)")
    for ax in axes:
        ax.set_yscale("symlog", linthresh=1.0)
        ax.set_yticks([1, 2, 5, 10, 20, 40, 100])
        ax.get_yaxis().set_minor_formatter(matplotlib.ticker.NullFormatter())
    fig.suptitle("30.5 g / 32 g 量程：生产传感器路径退化的归因（各臂 pos_rms 分布）")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_305g_attrib_box.png"), dpi=140)
    plt.close(fig)

    # 2 CDF
    fig, ax = plt.subplots(figsize=(7.6, 5.4))
    for a in arms:
        v = np.sort(np.array(vals(pr, a)))
        ax.step(v, np.arange(1, len(v) + 1) / len(v), where="post",
                label=f"{ARM_LABEL[a]} (n={len(v)})")
    ax.axvline(25.0, ls="--", c="k", lw=.9, label="旧口径尾部判据 25 m")
    ax.set_xlabel("位置 RMSE (m)")
    ax.set_ylabel("累积概率")
    ax.set_xlim(0, max(45, max(max(vals(pr, a)) for a in arms)))
    ax.grid(alpha=.3)
    ax.legend(fontsize=8, loc="lower right")
    ax.set_title("CDF：尾部比例还是整体平移？")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_305g_attrib_cdf.png"), dpi=140)
    plt.close(fig)

    # 3 逐 seed 配对散点（相对 A_prod）
    others = [a for a in arms if a != "A_prod"]
    n = max(len(others), 1)
    fig, axes = plt.subplots(1, n, figsize=(3.5 * n, 3.4), sharey=True)
    axes = np.atleast_1d(axes)
    base = dict(pr.get("A_prod", []))
    for ax, a in zip(axes, others):
        d = dict(pr[a])
        common = sorted(set(base) & set(d))
        x = [base[s] for s in common]
        y = [d[s] for s in common]
        ax.scatter(x, y, s=14, alpha=.75)
        lim = max(max(x + [1]), max(y + [1]))
        ax.plot([0, lim], [0, lim], ls="--", c="k", lw=.8)
        ax.set_xlim(0, lim)
        ax.set_ylim(0, lim)
        ax.set_xlabel("A_prod pos_rms (m)")
        n_worse = sum(1 for xi, yi in zip(x, y) if yi - xi > 0.05)
        n_bett = sum(1 for xi, yi in zip(x, y) if xi - yi > 0.05)
        ax.set_title(f"{ARM_LABEL[a]}\nn={len(common)} 变好{n_bett}/变差{n_worse}",
                     fontsize=8)
        ax.grid(alpha=.3)
    axes[0].set_ylabel("候选臂 pos_rms (m)")
    fig.suptitle("逐 seed 配对（同二进制 runtime 开关，虚线为等值）", y=1.0)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_305g_paired.png"), dpi=140,
                bbox_inches="tight")
    plt.close(fig)

    # 4 2×2 因子（LPF × AHRS），位置 + 速度
    vel = series(rows, "nav_vel_rms")
    rej = series(rows, "outlier_rejects")
    grid = [("A_prod", "LPF 开 / AHRS 开"), ("B_nolpf", "LPF 关 / AHRS 开"),
            ("C_noahrs", "LPF 开 / AHRS 关"), ("D_neither", "LPF 关 / AHRS 关")]
    fig, axes = plt.subplots(1, 2, figsize=(9.2, 4.0))
    for ax, src, ttl in ((axes[0], pr, "位置 RMSE 中位 (m)"),
                         (axes[1], vel, "速度 RMSE 中位 (m/s)")):
        xs, ys, cs = [], [], []
        for i, (a, lab) in enumerate(grid):
            v = np.array(vals(src, a))
            if len(v) == 0:
                continue
            xs.append(i)
            ys.append(float(np.median(v)))
            cs.append(len(v))
        ax.bar(xs, ys, color=["#c0392b", "#27ae60", "#c0392b", "#27ae60"])
        for x, y, c in zip(xs, ys, cs):
            ax.text(x, y, f"{y:.2f}\nn={c}", ha="center",
                    va="bottom", fontsize=8)
        ax.set_xticks(xs)
        ax.set_xticklabels([grid[i][1] for i in xs], fontsize=8, rotation=12)
        ax.set_title(ttl)
        ax.grid(axis="y", alpha=.3)
    fig.suptitle("2×2 因子：驱动项是加计软件 LPF，AHRS 份额为 0")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_305g_factor2x2.png"), dpi=140)
    plt.close(fig)

    # 5 臂 × 指标 归一化热图
    metrics = ["pos_rms", "pos_max", "nav_vel_rms", "nav_att_rmse",
               "outlier_rejects", "reacquire_resets", "ad_tau_ms"]
    mat, rowlabs = [], []
    for a in arms:
        row = []
        for m in metrics:
            d = series(rows, m)
            v = np.array(vals(d, a))
            row.append(float(np.median(v)) if len(v) else np.nan)
        mat.append(row)
        rowlabs.append(ARM_LABEL[a])
    mat = np.array(mat, dtype=float)
    # 归一化基准只取归因臂（A–H）：I–L 的 86–89 m 会把整列压成"全黑 + 其余全白"，
    # 反而看不出 A/B 差 4.5 倍这件事。延迟臂单独用 * 号标注超界。
    base_rows = [i for i, a in enumerate(arms) if a in
                 ("A_prod", "B_nolpf", "C_noahrs", "D_neither", "E_hz20",
                  "F_hz20_nolpf", "G_navraw", "H_navraw_hz20")]
    denom = np.nanmax(np.abs(mat[base_rows]), axis=0)
    denom = np.where(denom == 0, 1, denom)
    norm = np.clip(mat / denom, 0, 1)
    fig, ax = plt.subplots(figsize=(8.6, 0.62 * len(arms) + 2.2))
    im = ax.imshow(norm, cmap="magma_r", vmin=0, vmax=1, aspect="auto")
    ax.set_xticks(range(len(metrics)))
    ax.set_xticklabels(metrics, rotation=30, ha="right", fontsize=8)
    ax.set_yticks(range(len(rowlabs)))
    ax.set_yticklabels(rowlabs, fontsize=8)
    for i in range(mat.shape[0]):
        for j in range(mat.shape[1]):
            if np.isfinite(mat[i, j]):
                clipped = mat[i, j] > denom[j] * 1.001
                ax.text(j, i, f"{mat[i, j]:.3g}" + ("↑" if clipped else ""),
                        ha="center", va="center", fontsize=7,
                        # magma_r：norm 越大越暗 ⇒ 暗底必须白字（上一版写反，
                        # 导致 A/C 两行的数字落在黑底上不可见）
                        color="w" if norm[i, j] > .45 else "k")
    fig.colorbar(im, ax=ax, fraction=.04, label="该列中位数 / 归因臂最大中位数（超界截断）")
    ax.set_title("臂 × 指标（格内为各臂中位数）")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_305g_metrics_heat.png"), dpi=140)
    plt.close(fig)

    # 6 seed1 时序（追踪 CSV）
    tr_dir = os.path.join(SIM, "attrib305_traces")
    files = {"A_prod": "A_prod_s1.csv", "B_nolpf": "B_nolpf_s1.csv",
             "E_hz20": "E_hz20_s1.csv", "G_navraw": "G_navraw_s1.csv"}
    have = {k: os.path.join(tr_dir, v) for k, v in files.items()
            if os.path.exists(os.path.join(tr_dir, v))}
    if have:
        fig, axes = plt.subplots(3, 1, figsize=(9.4, 8.2), sharex=True)
        with open(next(iter(have.values())), encoding="utf-8") as fh:
            cols = fh.readline().strip().split(",")
        need = ("t", "ekf_n", "ekf_e", "true_n", "true_e",
                "gnss_resid_pos_ne", "ad_tau_ms")
        miss = [c for c in need if c not in cols]
        if miss:
            sys.exit(f"追踪 CSV 缺列 {miss}，时序图不可生成（不画空图）")
        for a, p in have.items():
            d = np.genfromtxt(p, delimiter=",", names=True)
            t = d["t"]
            axes[0].plot(t, np.hypot(d["ekf_n"] - d["true_n"],
                                     d["ekf_e"] - d["true_e"]),
                         lw=1.1, label=ARM_LABEL[a])
            axes[1].plot(t, d["gnss_resid_pos_ne"], lw=1.1, label=ARM_LABEL[a])
            axes[2].plot(t, d["ad_tau_ms"], lw=1.1, label=ARM_LABEL[a])
        axes[1].axhline(30.0, ls="--", c="k", lw=.9)
        axes[1].text(0.5, 31, "30 m 物理野值门", fontsize=8)
        axes[0].set_ylabel("水平位置误差 (m)")
        axes[1].set_ylabel("GNSS 位置新息 (m)")
        axes[2].set_ylabel("逐历元 tau_ms")
        axes[2].set_xlabel("时间 (s)")
        axes[0].set_ylim(0, None)
        for ax in axes:
            ax.grid(alpha=.3)
            ax.legend(fontsize=7)
        fig.suptitle("seed 1 逐历元：估计延迟被加计群延迟撑到 ~64 ms → 新息越 30 m 门 → 位置误差发散\n"
                     "（B 关 LPF 与 G 导航增量绕过 LPF 按构造逐位重合，只在图中看到一条）")
        fig.tight_layout()
        fig.savefig(os.path.join(FIG, "nav_305g_timeseries.png"), dpi=140)
        plt.close(fig)
    # 7 双峰证据：30.5 g 生产路径的 pos_rms 直方图（两簇 + 空档）+ 失效率随过载
    fig, axes = plt.subplots(1, 2, figsize=(11.0, 4.4))
    a = np.array(vals(pr, "A_prod"))
    b = np.array(vals(pr, "B_nolpf"))
    bins = np.arange(0, 36, 1.5)
    axes[0].hist(a, bins=bins, color="#c0392b", alpha=.85,
                 label=f"A 生产路径 n={len(a)}")
    axes[0].hist(b, bins=bins, color="#27ae60", alpha=.6,
                 label=f"B 关 LPF n={len(b)}")
    if len(a):
        lo = a[a < 15]
        hi = a[a > 15]
        axes[0].axvspan(lo.max() if len(lo) else 0, hi.min() if len(hi) else 30,
                        color="grey", alpha=.25)
        axes[0].text(((lo.max() if len(lo) else 0) + (hi.min() if len(hi) else 30)) / 2,
                     axes[0].get_ylim()[1] * .55, "空档", ha="center", fontsize=9)
        axes[0].text(8, axes[0].get_ylim()[1] * .93, f"正常簇 {len(lo)}/{len(a)}",
                     ha="center", fontsize=9, color="#7f1d1d")
        axes[0].text(26, axes[0].get_ylim()[1] * .93, f"失效簇 {len(hi)}/{len(a)}",
                     ha="center", fontsize=9, color="#7f1d1d")
    axes[0].set_xlabel("位置 RMSE (m)")
    axes[0].set_ylabel("seed 数")
    axes[0].set_title("30.5 g：双峰，不是整体下移（中位数无效）")
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=.3)

    pts = []
    for tag, thr in (("16g_11000", 11000), ("16g_13000", 13000),
                     ("16g_14500", 14500), ("onset_22500", 22500),
                     ("onset_23000", 23000), ("onset_23500", 23500),
                     ("32g_17200", 17200), ("32g_20000", 20000),
                     ("32g_22000", 22000), ("2026-10-28", 23900)):
        p = os.path.join(SIM, f"attrib305_{tag}.json")
        if not os.path.exists(p):
            continue
        d = json.load(open(p, encoding="utf-8"))
        arm = d["arms"]["A_prod"]
        pts.append((thr, arm["n_tail_gt25m"] / arm["n"], arm["pos_rms_med"],
                    arm.get("ad_tau_ms_med")))
    if pts:
        pts.sort()
        x = [p[0] for p in pts]
        axes[1].plot(x, [p[1] for p in pts], "o-", color="#c0392b",
                     label="失效发生率 (pos_rms>25 m)")
        axes[1].set_ylabel("失效发生率", color="#c0392b")
        ax2 = axes[1].twinx()
        ax2.plot(x, [p[2] for p in pts], "s--", color="#1f4e79", label="位置 RMSE 中位")
        ax2.set_ylabel("位置 RMSE 中位 (m)", color="#1f4e79")
        for i, p in enumerate(pts):
            if p[3] and i % 2 == 0:  # 只隔点标注，避免高推力段标签互相压字
                axes[1].annotate(f"tau {p[3]:.0f} ms", (p[0], p[1]),
                                 textcoords="offset points",
                                 xytext=(0, 10 + 9 * (i % 2)),
                                 fontsize=7, ha="center")
        axes[1].set_xlabel("推力 [N]（≈22 g=17200、28.1 g=22000、30.5 g=23900）")
        axes[1].set_title("过载 → 失效率：τ 估计早已撑大，失效率却后发")
        axes[1].grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_305g_bimodal.png"), dpi=140)
    plt.close(fig)
    print("done")


if __name__ == "__main__":
    ap = [a for a in sys.argv[1:]]
    main(ap or ["2026-10-28", "fixcand", "wide200"])
