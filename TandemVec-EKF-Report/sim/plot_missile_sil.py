# -*- coding: utf-8 -*-
"""导弹 SIL（6DOF plant + 真实固件导航）结果可视化。
@file    plot_missile_sil.py
@brief   6DOF 导弹 SIL（真实固件导航）结果可视化。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 读取 missile-sil JSONL 与 trace CSV，输出姿态/速度/位置与弹道图。
@frame   N/A
@units   N/A
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

BASE = os.path.join(DATA, "missile-sil-2026-10-27_base.jsonl")
FIX = os.path.join(DATA, "missile-sil-2026-10-27_final2.jsonl")
TRACE_BASE = os.path.join(os.environ.get("TEMP", ""), "msl_icm.csv")
TRACE_FIX = os.path.join(os.environ.get("TEMP", ""), "msl_fixed_trace.csv")


def load(path):
    return [json.loads(l) for l in open(path, encoding="utf-8")]


base = [r for r in load(BASE) if not r.get("no_json")]
fix = [r for r in load(FIX) if not r.get("no_json")]
CONDS = ["clean_ideal", "clean_icm", "nomag_icm", "outage5_icm",
         "noise_icm", "latency200_icm"]
LABELS = ["理想 IMU", "ICM 标称", "无磁", "5 s 失联", "GNSS 2.5 m/0.3",
          "200 ms 延迟"]


def pick(rows, prefix):
    return [r for r in rows if r["config"].startswith(prefix)]


def vals(rows, prefix, key):
    return [r[key] for r in pick(rows, prefix) if np.isfinite(r.get(key, np.nan))]


def med(rows, prefix, key):
    v = vals(rows, prefix, key)
    return float(np.median(v)) if v else np.nan


fig, ax = plt.subplots(2, 2, figsize=(13.4, 9.0), dpi=160,
                       constrained_layout=True)

# (a) 姿态误差时序（滚转+俯仰+航向合成）
a = ax[0, 0]
for path, lab, col, ls in ((TRACE_BASE, "修复前（AHRS 无方向门）", "#e45756", "-"),
                           (TRACE_FIX, "修复后（方向门）", "#1f6fb4", "-")):
    d = np.genfromtxt(path, delimiter=",", names=True)
    er = d["ekf_roll"] - d["truth_roll"]
    ep = d["ekf_pitch"] - d["truth_pitch"]
    ey = (d["ekf_yaw"] - d["truth_yaw"] + 180) % 360 - 180
    err = np.sqrt(er * er + ep * ep + ey * ey)
    a.plot(d["t"], err, ls, color=col, lw=1.1, label=lab)
a.axvspan(22, 30, color="#f2c14e", alpha=0.15)
a.set_xlabel("t [s]")
a.set_ylabel("姿态误差合成 [°]")
a.set_title("(a) 同一轨迹/同一 ICM 档：姿态误差时序（22–30 s 为无坡度转弯）",
            fontsize=10)
a.grid(alpha=0.3)
a.legend(fontsize=8)

# (b) 20 seeds 姿态 RMSE
b = ax[0, 1]
x = np.arange(len(CONDS))
w = 0.38
bv = [med(base, c + "_full_base", "nav_att_rmse") for c in CONDS]
fv = [med(fix, c + "_full", "nav_att_rmse") for c in CONDS]
b.bar(x - w / 2, bv, w, color="#e45756", alpha=0.85, label="修复前")
b.bar(x + w / 2, fv, w, color="#1f6fb4", alpha=0.85, label="修复后")
for i, (u, v) in enumerate(zip(bv, fv)):
    b.text(i - w / 2, u + 0.15, f"{u:.1f}", ha="center", fontsize=7)
    b.text(i + w / 2, v + 0.15, f"{v:.1f}", ha="center", fontsize=7)
b.set_xticks(x)
b.set_xticklabels(LABELS, fontsize=8)
b.set_ylabel("姿态 RMSE 中位 [°]")
b.set_title("(b) 基线 20 / 修复 40 seeds：姿态 RMSE 6.6–7.6° → 0.57–1.31°", fontsize=10)
b.grid(axis="y", alpha=0.3)
b.legend(fontsize=8)

# (c) 20 seeds 速度 RMS
c = ax[1, 0]
bv = [med(base, q + "_full_base", "nav_vel_rms") for q in CONDS]
fv = [med(fix, q + "_full", "nav_vel_rms") for q in CONDS]
c.bar(x - w / 2, bv, w, color="#e45756", alpha=0.85, label="修复前")
c.bar(x + w / 2, fv, w, color="#1f6fb4", alpha=0.85, label="修复后")
c.set_xticks(x)
c.set_xticklabels(LABELS, fontsize=8)
c.set_ylabel("速度 RMS 中位 [m/s]")
c.set_title("(c) 基线 20 / 修复 40 seeds：速度误差同步下降（0.49→0.16 m/s，200 ms 延迟除外）",
            fontsize=10)
c.grid(axis="y", alpha=0.3)
c.legend(fontsize=8)

# (d) 位置 RMSE 与 AHRS 融合次数
d = ax[1, 1]
bv = [med(base, q + "_full_base", "pos_rms") for q in CONDS]
fv = [med(fix, q + "_full", "pos_rms") for q in CONDS]
d.bar(x - w / 2, bv, w, color="#e45756", alpha=0.85, label="位置 RMSE 修复前")
d.bar(x + w / 2, fv, w, color="#1f6fb4", alpha=0.85, label="位置 RMSE 修复后")
d.set_xticks(x)
d.set_xticklabels(LABELS, fontsize=8)
d.set_ylabel("位置 RMSE 中位 [m]")
d.set_title("(d) 位置精度不回归；200 ms 延迟仍是 38 m 边界", fontsize=10)
d.grid(axis="y", alpha=0.3)
d.legend(fontsize=8)
for i, (u, v) in enumerate(zip(bv, fv)):
    d.text(i - w / 2, u + 0.8, f"{u:.1f}", ha="center", fontsize=7)
    d.text(i + w / 2, v + 0.8, f"{v:.1f}", ha="center", fontsize=7)

fig.suptitle("6DOF 导弹弹体 + 真实固件导航 SIL：AHRS 重力方向门修复效果 "
             "（L2，基线 20 / 修复后 40 seeds，标量 Eigen）", fontsize=12)
out = os.path.join(FIG, "nav_missile_sil_2026-10-27.png")
fig.savefig(out)
print("saved", out)

# 第二张：轨迹与 AHRS 融合次数
fig2, ax2 = plt.subplots(2, 2, figsize=(13.0, 8.2), dpi=160,
                         constrained_layout=True)
d = np.genfromtxt(TRACE_FIX, delimiter=",", names=True)
a = ax2[0, 0]
a.plot(d["true_n"], d["true_e"], color="#1f6fb4", lw=1.4)
a.set_xlabel("北 [m]")
a.set_ylabel("东 [m]")
a.set_title("(a) 45 s 真实航迹（含 85°→−60° 转弯）", fontsize=10)
a.grid(alpha=0.3)
a.axis("equal")

a = ax2[0, 1]
a.plot(d["t"], d["truth_alt"], color="#54a24b", label="真值高度")
a.plot(d["t"], d["truth_pitch"], color="#8e44ad", label="真值俯仰")
a.plot(d["t"], d["truth_yaw"], color="#f58518", label="真值航向")
a.set_xlabel("t [s]")
a.set_ylabel("m / °")
a.set_title("(b) 爬升-转弯-俯冲剖面（高度最高 1987 m，最大 6.4 g）",
            fontsize=10)
a.grid(alpha=0.3)
a.legend(fontsize=8)

a = ax2[1, 0]
tb = np.genfromtxt(TRACE_BASE, delimiter=",", names=True)
tf = np.genfromtxt(TRACE_FIX, delimiter=",", names=True)
a.plot(tb["t"], tb["ekf_pitch"] - tb["truth_pitch"], color="#e45756",
       label="修复前")
a.plot(tf["t"], tf["ekf_pitch"] - tf["truth_pitch"], color="#1f6fb4",
       label="修复后")
a.set_xlabel("t [s]")
a.set_ylabel("俯仰误差 [°]")
a.set_title("(c) 俯仰误差：修复前在 24 s 跳变 −8°、峰值 −19°", fontsize=10)
a.grid(alpha=0.3)
a.legend(fontsize=8)

a = ax2[1, 1]
bv = [np.mean(vals(base, q + "_full_base", "att_fused")) for q in CONDS]
fv = [np.mean(vals(fix, q + "_full", "att_fused")) for q in CONDS]
a.bar(x - w / 2, bv, w, color="#e45756", alpha=0.85, label="修复前")
a.bar(x + w / 2, fv, w, color="#1f6fb4", alpha=0.85, label="修复后")
a.set_xticks(x)
a.set_xticklabels(LABELS, fontsize=8)
a.set_ylabel("AHRS 融合次数（45 s）")
a.set_title("(d) 方向门把 AHRS 融合次数降到 0（17/20 seeds；噪声档 3 例少量）",
            fontsize=10)
a.grid(axis="y", alpha=0.3)
a.legend(fontsize=8)

out2 = os.path.join(FIG, "nav_missile_sil_attitude_2026-10-27.png")
fig2.savefig(out2)
print("saved", out2)
