# -*- coding: utf-8 -*-
"""大过载/大机动卫惯组合导航矩阵可视化。
@file    plot_high_dynamic.py
@brief   大过载/大机动矩阵可视化。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 绘制位置/姿态/NEES 与杆臂 A/B 图。
@frame   N/A
@units   N/A
"""
import json, os
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
    os.path.join(DATA, "high-dyn-mc-2026-10-26-v2-scalar.jsonl"), encoding="utf-8")]
lev = json.load(open(os.path.join(DATA, "high-dyn-lever-ab-2026-10-26.json"),
                     encoding="utf-8"))

fig, ax = plt.subplots(2, 2, figsize=(13.2, 9.2), dpi=160,
                       constrained_layout=True)
C = {"turn": "#1f6fb4", "sturn": "#e45756", "pull": "#54a24b",
     "roll": "#8e44ad", "coning": "#f58518", "negg": "#7f7f7f"}

# (a) 基线：负载/角速率 → pos/att RMSE 中位
a = ax[0, 0]
base = [r for r in rows if not r["outage"] and r["a_scale"] == 1 and
        r["misalign"] == 0 and r["lever"] == 0 and r["vib_g"] == 0 and
        r["latency"] == 20 and r["load"] <= 8]
series = []
for prof in ("turn", "sturn", "pull"):
    for load in (2, 4, 6, 8):
        rs = [r for r in base if r["profile"] == prof and r["load"] == load and
              r["speed"] == 80]
        if rs:
            series.append((f"{prof[:1].upper()}{load:g}", np.median([r["pos_rmse"] for r in rs]),
                           np.median([r["att_rmse_deg"] for r in rs]), C[prof]))
for rate in (180, 360, 720, 1080):
    rs = [r for r in base if r["profile"] == "roll" and r["rate"] == rate and
          r["load"] == 2]
    if rs:
        series.append((f"R{rate}", np.median([r["pos_rmse"] for r in rs]),
                       np.median([r["att_rmse_deg"] for r in rs]), C["roll"]))
for rate in (120, 300, 600, 900):
    rs = [r for r in base if r["profile"] == "coning" and r["rate"] == rate]
    if rs:
        series.append((f"C{rate}", np.median([r["pos_rmse"] for r in rs]),
                       np.median([r["att_rmse_deg"] for r in rs]), C["coning"]))
x = np.arange(len(series))
a.bar(x, [s[1] for s in series], color=[s[3] for s in series], alpha=0.85)
a.set_xticks(x)
a.set_xticklabels([s[0] for s in series], fontsize=7, rotation=45)
a.set_ylabel("位置 RMSE 中位 [m]")
a2 = a.twinx()
a2.plot(x, [s[2] for s in series], "k.-", lw=1, ms=5, label="姿态 RMSE")
a2.set_ylabel("姿态 RMSE 中位 [°]")
a.set_title("(a) 正过载/高角速率基线：2–8 g 位置 0.2–0.3 m，姿态 0.3–2.2°",
            fontsize=10)
a.grid(axis="y", alpha=0.3)

# (b) 延迟扫描：320 ms 缓冲边界
b = ax[0, 1]
lat = [20, 100, 200, 300, 330, 500]
for prof in ("turn", "sturn", "pull"):
    med, mx = [], []
    for L in lat:
        rs = [r for r in rows if r["profile"] == prof and r["load"] == 6 and
              r["speed"] == 80 and r["latency"] == L and not r["outage"] and
              r["a_scale"] == 1 and r["lever"] == 0 and r["vib_g"] == 0]
        med.append(np.median([r["pos_rmse"] for r in rs]) if rs else np.nan)
        mx.append(max([r["pos_rmse"] for r in rs]) if rs else np.nan)
    b.semilogy(lat, med, "o-", color=C[prof], lw=1.4, ms=4, label={"turn":"协调转弯","sturn":"S 转弯","pull":"拉杆"}[prof])
    b.semilogy(lat, mx, ":", color=C[prof], lw=1, alpha=0.7)
b.axvline(320, color="k", ls="--", lw=1)
b.text(323, 1.2, "64 格 × 5 ms\n= 320 ms 回放上限", fontsize=7)
b.set_xlabel("GNSS 链路延迟 [ms]")
b.set_ylabel("位置 RMSE [m，对数]")
b.set_title("(b) 延迟 ≤300 ms 正常；≥330 ms 超出回放缓冲，GNSS 全拒",
            fontsize=10)
b.grid(alpha=0.3, which="both")
b.legend(fontsize=8)

# (c) 20 s 失联 × IMU 尺度/失准
c = ax[1, 0]
grades = [(1.0, 0.0, "理想"), (1.002, 0.1, "0.2%/0.1°"),
          (1.005, 0.2, "0.5%/0.2°"), (1.01, 0.5, "1%/0.5°")]
med, p90, mx = [], [], []
for a_s, mis, _ in grades:
    rs = [r for r in rows if r["outage"] and
          r["outage"][1] - r["outage"][0] == 20 and
          abs(r["a_scale"] - a_s) < 1e-9 and abs(r["misalign"] - mis) < 1e-9]
    v = np.array([r["pos_rmse"] for r in rs])
    med.append(np.median(v)); p90.append(np.percentile(v, 90)); mx.append(v.max())
x = np.arange(len(grades))
c.bar(x - 0.22, med, 0.2, color="#1f6fb4", label="中位")
c.bar(x, p90, 0.2, color="#f58518", label="p90")
c.bar(x + 0.22, mx, 0.2, color="#b0413e", label="最大")
for i, v in enumerate(mx):
    c.text(i + 0.22, v + 0.8, f"{v:.1f}", ha="center", fontsize=7)
c.set_xticks(x)
c.set_xticklabels([g[2] for g in grades])
c.set_ylabel("20 s 失联位置 RMSE [m]")
c.set_title("(c) 20 s 失联：1% 尺度/0.5° 失准最大 29 m（sturn 6 g）",
            fontsize=10)
c.grid(axis="y", alpha=0.3)
c.legend(fontsize=8)

# (d) 杆臂 A/B
d = ax[1, 1]
labels = list(lev.keys())
modes = ["mode0", "mode1", "mode2"]
colors = ["#b0413e", "#f58518", "#1f6fb4"]
names = {"mode0": "不补偿", "mode1": "仅位置预补偿", "mode2": "EKF 完整杆臂"}
x = np.arange(len(labels))
for i, m in enumerate(modes):
    vals = [lev[k][m]["pos_rmse"]["med"] for k in labels]
    d.bar(x + (i - 1) * 0.26, vals, 0.25, color=colors[i], label=names[m])
d.set_yscale("log")
d.set_xticks(x)
d.set_xticklabels([k.replace("_L", "\nL").replace("_R", " R").replace("_lev", "\nlev ")
                   for k in labels], fontsize=6.5)
d.set_ylabel("位置 RMSE 中位 [m，对数]")
d.set_title("(d) GNSS 杆臂：仅位置补偿不够；EKF 位置+速度补偿恢复 0.2 m",
            fontsize=10)
d.grid(axis="y", alpha=0.3, which="both")
d.legend(fontsize=8, ncol=3)

fig.suptitle("大过载/大机动卫惯组合导航（固件 Ekf15State 宿主台架，L1，标量 Eigen；探索 10 + 定向 50 seeds）",
             fontsize=12)
out = os.path.join(FIG, "nav_high_dynamic_2026-10-26.png")
fig.savefig(out, bbox_inches="tight")
print("wrote", out)
