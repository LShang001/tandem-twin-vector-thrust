# -*- coding: utf-8 -*-
"""SIL 在飞 NEES：有色 vs 白 GNSS 误差（同二进制 50 seeds）。
@file    plot_nees_sil.py
@brief   SIL NEES 结果可视化。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 绘制 NEES/NIS 分布与门控对照。
@frame   N/A
@units   N/A
"""
import json, os, re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "sim-data")
FIG = os.path.join(HERE, "..", "figs")
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

rows = [json.loads(l) for l in open(
    os.path.join(DATA, "nees_sil_ab_2026-10-26_50seed.jsonl"), encoding="utf-8")]
C = {r["seed"]: r for r in rows if r["arm"] == "colored"}
W = {r["seed"]: r for r in rows if r["arm"] == "white"}
seeds = sorted(C)

NEES_RE = re.compile(r"\[nees\] t=\s*([0-9.]+).*?nees=\s*([-0-9.]+)")


def parse_log(path):
    t, ne = [], []
    for line in open(path, encoding="utf-8", errors="ignore"):
        m = NEES_RE.search(line)
        if m:
            t.append(float(m.group(1)))
            ne.append(float(m.group(2)))
    return np.array(t), np.array([]), np.array([]), np.array(ne)


tc, ec, sc, nc = parse_log(os.path.join(
    DATA, "trace-nees-seed1-colored-2026-10-26.log"))
tw, ew, sw, nw = parse_log(os.path.join(
    DATA, "trace-nees-seed1-white-2026-10-26.log"))

fig, ax = plt.subplots(2, 2, figsize=(12.8, 8.6), dpi=160,
                       constrained_layout=True)
col_c, col_w = "#c0392b", "#1f6fb4"

# (a) 配对散点
a = ax[0, 0]
a.scatter([W[s]["nees_pos_med"] for s in seeds],
          [C[s]["nees_pos_med"] for s in seeds], s=26, c=col_c,
          alpha=0.8, label="NEES 中位")
a.plot([0.01, 30], [0.01, 30], "k--", lw=1, label="1:1")
a.set_xscale("log")
a.set_yscale("log")
a.set_xlabel("白噪声臂 NEES_pos 中位")
a.set_ylabel("有色误差臂 NEES_pos 中位")
a.set_title("(a) 同一 seed：有色 GNSS 误差抬高 NEES（38/50 更高）", fontsize=10)
a.grid(alpha=0.3, which="both")
a.legend(fontsize=8)

# (b) 分布条
b = ax[0, 1]
keys = [("nees_pos_med", "中位"), ("nees_pos_p95", "p95"),
        ("nees_pos_max", "单步最大")]
x = np.arange(len(keys))
for i, arm in enumerate(("white", "colored")):
    rs = [r for r in rows if r["arm"] == arm]
    vals = [np.median([r[k] for r in rs]) for k, _ in keys]
    b.bar(x + (i - 0.5) * 0.34, vals, 0.32,
          color=(col_w if arm == "white" else col_c),
          label=("白噪声" if arm == "white" else "有色 +0.5 m/20 s"))
    for xx, v in zip(x + (i - 0.5) * 0.34, vals):
        b.text(xx, v + 0.1, f"{v:.2f}", ha="center", fontsize=8)
b.set_xticks(x)
b.set_xticklabels([k[1] for k in keys])
b.set_ylabel("NEES_pos（3 自由度归一化）")
b.set_title("(b) 50 seeds 中位：2.22 vs 1.05，尾部 3.84 vs 1.33", fontsize=10)
b.grid(axis="y", alpha=0.3)
b.legend(fontsize=8)

# (c) seed1 时序
c = ax[1, 0]
mask = tc > 10
c.semilogy(tc[mask], np.maximum(nc[mask], 1e-2), color=col_c, lw=1.4,
           label="有色：NEES")
c.semilogy(tw[mask], np.maximum(nw[mask], 1e-2), color=col_w, lw=1.4,
           label="白噪声：NEES")
c.axhline(1.0, color="gray", ls="--", lw=1)
c.axhline(2.6, color="gray", ls=":", lw=1)
c.text(11.2, 2.7, "χ²(3)/3 的 95%", fontsize=7, color="gray")
c.set_xlabel("时间 [s]")
c.set_ylabel("NEES_pos（对数）")
c.set_title("(c) seed1：有色误差下 NEES 持续 >1 并长尾", fontsize=10)
c.grid(alpha=0.3, which="both")
c.legend(fontsize=8)

# (d) 真值误差 vs NEES
d = ax[1, 1]
for arm, col, mk in (("white", col_w, "o"), ("colored", col_c, "s")):
    rs = [r for r in rows if r["arm"] == arm]
    d.scatter([r["pos_rms"] for r in rs], [r["nees_pos_med"] for r in rs],
              s=26, c=col, marker=mk, alpha=0.8,
              label=("白噪声" if arm == "white" else "有色"))
d.set_yscale("log")
d.set_xlabel("EKF 位置 RMSE vs 真值 [m]")
d.set_ylabel("NEES_pos 中位")
d.set_title("(d) 有色臂误差更大、协方差更不诚实；白噪声臂贴近 1", fontsize=10)
d.grid(alpha=0.3, which="both")
d.legend(fontsize=8)
d.axhline(1.0, color="gray", ls="--", lw=1)

fig.suptitle("SIL 在飞 EKF 协方差一致性（真实固件 + 6DOF plant，mag_clean + flow，40 s）",
             fontsize=12)
out = os.path.join(FIG, "nav_nees_sil_gnss_colored_2026-10-26.png")
fig.savefig(out, bbox_inches="tight")
print("wrote", out)
