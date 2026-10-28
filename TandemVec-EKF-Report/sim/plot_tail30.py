# -*- coding: utf-8 -*-
"""30.5 g 尾部 seed21 瞬态图：水平误差 / 真高与垂速 / 气压新息。
@file    plot_tail30.py
@brief   展示 19.5–21.4 s 有界瞬态与 21.5 s 重锚回落。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
"""
import csv
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Arial"]
plt.rcParams["axes.unicode_minus"] = False
HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "sim-data")
FIGS = os.path.join(HERE, "..", "figs")

rows = list(csv.DictReader(open(os.path.join(
    DATA, "trace-sil32g-30g-seed21-2026-10-27.csv"), encoding="utf-8")))
t = np.array([float(r["t"]) for r in rows])
err = np.array([np.hypot(float(r["ekf_n"]) - float(r["true_n"]),
                         float(r["ekf_e"]) - float(r["true_e"])) for r in rows])
alt = np.array([float(r["truth_alt"]) for r in rows])
vz = np.array([float(r["truth_vz"]) for r in rows])
innov = np.array([float(r["baro_innov_m"]) for r in rows])
fused = np.array([int(r["baro_fused"]) for r in rows])

fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
ax = axes[0]
ax.plot(t, err, color="#c0392b")
ax.axhline(50, color="gray", ls="--", lw=0.8, label="50 m 野值门")
ax.set_ylabel("水平位置误差 (m)")
ax.set_title("30.5 g 尾部 seed21：19.5–21.4 s 水平误差升至 148 m，21.5 s 重锚回 2.4 m")
ax.legend()
ax.grid(alpha=0.3)
ax = axes[1]
ax.plot(t, alt, color="#2e8b57", label="真高 (m)")
ax2 = ax.twinx()
ax2.plot(t, vz, color="#4f8fc0", label="真垂速 (m/s)")
ax.set_ylabel("真高 (m)")
ax2.set_ylabel("真垂速 (m/s)")
ax.legend(loc="upper left")
ax2.legend(loc="upper right")
ax.grid(alpha=0.3)
ax = axes[2]
ax.plot(t, innov, color="#8e44ad", label="气压新息 (m)")
ax.plot(t, fused * 2.0, color="#7f8c8d", alpha=0.5, label="baro_fused×2")
ax.set_ylabel("m")
ax.set_xlabel("t (s)")
ax.set_xlim(15, 26)
ax.legend()
ax.grid(alpha=0.3)
fig.tight_layout()
out = os.path.join(FIGS, "nav_tail30_2026-10-27.png")
fig.savefig(out, dpi=150)
print(out)
