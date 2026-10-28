# -*- coding: utf-8 -*-
"""NEES/NIS 分布级一致性与 Q_pos 扫描作图（研究模型）。
@file    plot_consistency.py
@brief   EKF 一致性可视化。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 绘制 NEES/NIS 与理论区间。
@frame   N/A
@units   N/A
"""
import json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.stats import chi2

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "sim-data")
FIG = os.path.join(HERE, "..", "figs")
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

chk = json.load(open(os.path.join(DATA, "consistency_check_2026-10-26.json"),
                     encoding="utf-8"))
sweep = json.load(open(os.path.join(DATA, "q_pos_sweep_2026-10-26.json"),
                       encoding="utf-8"))
z = np.load(os.path.join(DATA, "consistency_check_2026-10-26.npz"))

scs = ["nominal", "highnoise", "delay", "outage5"]
labels = {"nominal": "标称", "highnoise": "高噪声", "delay": "15 ms 延迟",
          "outage5": "5 s 失联"}
colors = {"nominal": "#1f6fb4", "highnoise": "#c0392b",
          "delay": "#2e8b57", "outage5": "#8e44ad"}

fig, ax = plt.subplots(2, 2, figsize=(12.8, 8.6), dpi=160,
                       constrained_layout=True)

# (a) 经验 CDF vs χ²(3)/3
a = ax[0, 0]
grid = np.linspace(0.01, 12, 500)
a.plot(grid, chi2.cdf(3 * grid, 3), "k--", lw=1.6, label="理论 χ²(3)/3")
for sc in scs:
    v = z[f"{sc}_main_nees_pos"]
    v = v[np.isfinite(v)]
    xs = np.sort(v)
    a.plot(xs, np.arange(1, len(xs) + 1) / len(xs), lw=1.3,
           color=colors[sc], label=f"{labels[sc]}（median {np.median(v):.2f}）")
a.axvline(0.789, color="gray", ls=":", lw=1)
a.axvline(2.60, color="gray", ls=":", lw=1)
a.text(0.82, 0.12, "χ²(3)/3 中位 0.79", fontsize=8, color="gray", rotation=90)
a.text(2.65, 0.12, "95% 2.60", fontsize=8, color="gray", rotation=90)
a.set_xlim(0, 8)
a.set_ylim(0, 1.0)
a.set_xlabel("位置 NEES（按 3 自由度归一化）")
a.set_ylabel("经验 CDF")
a.set_title("(a) 位置协方差一致性：10 seeds × 60 s（12 万历元）", fontsize=10)
a.grid(alpha=0.3)
a.legend(fontsize=7.5, loc="lower right")

# (b) 各场景分位数与同样本量下最大值的理论分布
b = ax[0, 1]
x = np.arange(len(scs))
med = [chk[f"{s}_main"]["nees_pos_med"] for s in scs]
p95 = [chk[f"{s}_main"]["nees_pos_p95"] for s in scs]
mx = [chk[f"{s}_main"]["nees_pos_max"] for s in scs]
b.bar(x - 0.22, med, 0.2, color="#4c78a8", label="中位")
b.bar(x, p95, 0.2, color="#f58518", label="p95")
b.bar(x + 0.22, mx, 0.2, color="#b0413e", label="单步最大")
b.axhline(0.789, color="#4c78a8", ls="--", lw=1)
b.axhline(2.60, color="#f58518", ls="--", lw=1)
qmax = chk["nominal_main"]["chi2_df3_max_quantiles"]
b.axhspan(qmax["0.5"], qmax["0.95"], color="gray", alpha=0.12)
b.text(3.45, qmax["0.5"] + 0.15, "同样本量 max\n理论 50–95%", fontsize=7,
       ha="right", color="dimgray")
b.set_xticks(x)
b.set_xticklabels([labels[s] for s in scs], fontsize=8.5)
b.set_ylabel("NEES（3 自由度归一化）")
b.set_title("(b) 中位/p95 接近理论，最大单步远低于理论 max", fontsize=10)
b.legend(fontsize=7.5, ncol=3, loc="upper left")
b.grid(axis="y", alpha=0.3)

# (c) 速度 NEES 与 NIS 相对理论值
c = ax[1, 0]
nv = [chk[f"{s}_main"]["nees_vel_med"] for s in scs]
nis = [chk[f"{s}_main"]["nis_med"] for s in scs]
c.bar(x - 0.18, nv, 0.34, color="#54a24b", label="速度 NEES 中位")
c.bar(x + 0.18, [v / 5.348 for v in nis], 0.34, color="#e45756",
      label="GNSS NIS 中位 / χ²(6) 中位")
c.axhline(1.0, color="k", ls="--", lw=1)
c.set_xticks(x)
c.set_xticklabels([labels[s] for s in scs], fontsize=8.5)
c.set_ylim(0, 1.15)
c.set_ylabel("观测 / 理论中位")
c.set_title("(c) 速度协方差与 NIS 偏保守（约 2.5×）", fontsize=10)
c.legend(fontsize=7.5)
c.grid(axis="y", alpha=0.3)
for i, (a1, a2) in enumerate(zip(nv, nis)):
    c.text(i - 0.18, a1 + 0.02, f"{a1:.2f}", ha="center", fontsize=7)
    c.text(i + 0.18, a2 / 5.348 + 0.02, f"{a2:.2f}", ha="center", fontsize=7)

# (d) Q_pos 扫描：RMSE 相对变化 vs NEES p95
d = ax[1, 1]
qvals = [0.0, 1e-4, 1e-3, 1e-2, 3e-2]
for sc in scs:
    base = sweep[f"q{0:g}_p01_{sc}"]["pos_rmse_seedmed"]
    rel = [100 * (sweep[f"q{q:g}_p01_{sc}"]["pos_rmse_seedmed"] / base - 1)
           for q in qvals]
    d.plot(range(len(qvals)), rel, "o-", color=colors[sc], lw=1.4, ms=4,
           label=labels[sc])
d.axhline(0, color="k", lw=0.8)
d.set_xticks(range(len(qvals)))
d.set_xticklabels([f"{q:g}" if q else "0\n(基线)" for q in qvals], fontsize=8)
d.set_xlabel("位置随机游走 Q_pos [m²/s]")
d.set_ylabel("位置 RMSE 相对基线变化 [%]")
d.set_title("(d) 加 Q_pos 只换精度：q≥1e-3 尾部略降、RMSE 上升", fontsize=10)
d.grid(alpha=0.3)
d.legend(fontsize=7.5, ncol=2)
for q, lab in ((1e-3, "+0.8%"), (1e-2, "+5~7%"), (3e-2, "+10~15%")):
    i = qvals.index(q)
    d.text(i, -3.0, lab, fontsize=7, color="dimgray", ha="center")

fig.suptitle("EKF 协方差一致性核验（Python 同构 15 态 ESKF，非固件 SIL）",
             fontsize=12)
out = os.path.join(FIG, "nav_ekf_consistency_2026-10-26.png")
fig.savefig(out, bbox_inches="tight")
print("wrote", out)
