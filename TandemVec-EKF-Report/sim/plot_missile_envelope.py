# -*- coding: utf-8 -*-
"""导弹 SIL 过载/失联/故障包线可视化。
@file    plot_missile_envelope.py
@brief   绘制 6.4/8.4/10.1 g、故障注入、GNSS 失联时长与 20-seed 位置尾部。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 读取 missile-sil-2026-10-27_{fix,highg,fault,outrate}.jsonl。
@frame   N/A
@units   g、m、m/s、s
"""
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


def load(name):
    return [json.loads(l) for l in open(os.path.join(DATA, name), encoding="utf-8")
            if not json.loads(l).get("no_json")]


def vals(rows, cfg, key):
    return np.array([r[key] for r in rows if r["config"] == cfg and
                     isinstance(r.get(key), (int, float)) and np.isfinite(r[key])],
                    float)


fix = load("missile-sil-2026-10-27_armed_main40.jsonl")
highg = fix
fault = fix
out = fix


fig, ax = plt.subplots(2, 2, figsize=(13.4, 9.0), dpi=160,
                       constrained_layout=True)

# (a) 过载包线
a = ax[0, 0]
cfgs = [("clean_icm_full", "6.4 g"),
        ("thrust8g_icm_full", "8.4 g"),
        ("thrust10g_icm_full", "10.1 g")]
att = [np.median(vals(highg, c, "nav_att_rmse")) for c, _ in cfgs]
pos = [np.median(vals(highg, c, "pos_rms")) for c, _ in cfgs]
x = np.arange(len(cfgs))
a.bar(x - 0.2, att, 0.4, color="#1f6fb4", label="姿态 RMSE")
a.set_ylabel("姿态 RMSE [°]")
a2 = a.twinx()
a2.bar(x + 0.2, pos, 0.4, color="#e45756", label="位置 RMSE")
a2.set_yscale("log")
a2.set_ylim(1.0, 2000.0)
a2.set_ylabel("位置 RMSE [m]（对数）")
a.set_xticks(x)
a.set_xticklabels([l for _, l in cfgs])
a.set_title("(a) 生产 ±8 g：8.4 g 0.68°/3.9 m；10.1 g 触顶 636 m",
            fontsize=10)
a.grid(axis="y", alpha=0.3)
a.legend(loc="upper left", fontsize=8)
a2.legend(loc="upper right", fontsize=8)

# (b) 过载位置尾部
b = ax[0, 1]
labels = ["6.4 g", "8.4 g", "10.1 g (8g FS)"]
data = [vals(fix, "clean_icm_full", "pos_max"),
        vals(highg, "thrust8g_icm_full", "pos_max"),
        vals(highg, "thrust10g_icm_full", "pos_max")]
b.boxplot(data, labels=labels, showfliers=True)
b.set_ylabel("位置误差峰值 [m]")
b.set_title("(b) 位置峰值：6.4/8.4 g 8.1/13.0 m；10.1 g 触顶 max 1562 m",
            fontsize=10)
b.grid(axis="y", alpha=0.3)

# (c) 故障注入
c = ax[1, 0]
fc = [("imu_nan_icm_full", "IMU NaN 1 s"), ("imu_stuck_icm_full", "IMU 冻结 1 s"),
      ("gnss_spoof20_icm_full", "spoof 20 m"), ("gnss_spoof30_icm_full", "spoof 30 m"),
      ("mag_dropout_icm_full", "磁失联 5 s"), ("baro_drift_icm_full", "气压 0.5 m/s")]
fc = [(q, l) for q, l in fc if vals(fault, q, "pos_rms").size]
c.bar(np.arange(len(fc)), [np.median(vals(fault, q, "pos_rms")) for q, _ in fc],
      color=["#e45756" if q == "imu_nan_icm_full" else "#1f6fb4" for q, _ in fc],
      alpha=0.85)
c.set_xticks(np.arange(len(fc)))
c.set_xticklabels([l for _, l in fc], fontsize=8, rotation=20)
c.set_ylabel("位置 RMSE 中位 [m]")
c.set_title("(c) 故障：1 s IMU NaN 122 m、冻结 35 m，其余有界",
            fontsize=10)
c.grid(axis="y", alpha=0.3)

# (d) 失联时长
d = ax[1, 1]
oc = [("clean_icm_full", "0", fix), ("outage5_icm_full", "5", fix),
      ("outage10_icm_full", "10", out), ("outage20_icm_full", "20", out),
      ("outage30_icm_full", "30", out)]
oc = [(q, l, src) for q, l, src in oc if vals(src, q, "pos_rms").size]
med = [np.median(vals(src, q, "pos_rms")) for q, _, src in oc]
p95 = [np.percentile(vals(src, q, "pos_rms"), 95) for q, _, src in oc]
mx = [np.max(vals(src, q, "pos_max")) for q, _, src in oc]
x = np.arange(len(oc))
d.plot(x, med, "o-", color="#1f6fb4", label="RMSE 中位")
d.plot(x, p95, "s--", color="#f58518", label="RMSE p95")
d.plot(x, mx, "^:", color="#e45756", label="峰值 max")
d.set_xticks(x)
d.set_xticklabels([l + " s" for _, l, _ in oc])
d.set_xlabel("GNSS 失联时长")
d.set_ylabel("位置误差 [m]")
d.set_title("(d) 失联：5/10/20/30 s 中位 2.6/8.9/13.1/20.5 m",
            fontsize=10)
d.grid(alpha=0.3)
d.legend(fontsize=8)

fig.suptitle("6DOF 导弹 SIL：过载包线（生产 ±8 g）、故障注入与 GNSS 失联（L2，武装 40 seeds/格）",
             fontsize=12)
outp = os.path.join(FIG, "nav_missile_envelope_2026-10-27.png")
fig.savefig(outp)
print("saved", outp)
