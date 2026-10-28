# -*- coding: utf-8 -*-
"""导弹 SIL 标准可视化套件：时序/3D/箱线/CDF/直方/散点/热图/条形。

@file    plot_missile_visuals.py
@brief   从 missile-sil JSONL 与 200 Hz trace 生成多类型视觉分析图。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 输出 nav_missile_overview / distributions / robustness / tail_heatmap。
@frame   NED（北/东/下）；姿态为 ZYX 欧拉角。
@units   m、m/s、°、s、g。
"""
import json
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.colors import LogNorm

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "sim-data")
FIG = os.path.join(HERE, "..", "figs")
TMP = os.environ.get("TEMP", "")
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

C_BASE = "#e45756"
C_FIX = "#1f6fb4"
C_ALT = "#54a24b"
C_ORANGE = "#f58518"
C_PURPLE = "#8e44ad"


def load(name):
    return [json.loads(l) for l in open(os.path.join(DATA, name), encoding="utf-8")
            if not json.loads(l).get("no_json")]


def vals(rows, cfg, key):
    return np.array([r[key] for r in rows if r["config"] == cfg and
                     isinstance(r.get(key), (int, float)) and np.isfinite(r[key])],
                    float)


def med(rows, cfg, key):
    v = vals(rows, cfg, key)
    return float(np.median(v)) if v.size else np.nan


def trace(name):
    return np.genfromtxt(os.path.join(TMP, name), delimiter=",", names=True)


base = load("missile-sil-2026-10-27_prod_gateoff20.jsonl")
fix = load("missile-sil-2026-10-27_armed_main40.jsonl")
highg = fix
vib = fix
fault = fix
out = fix
align = load("missile-sil-2026-10-27_align_prod20.jsonl")
viblaunch = load("missile-sil-2026-10-27_viblaunch_prod20.jsonl")
vib84 = load("missile-sil-2026-10-27_vibprod20.jsonl")

CONDS = ["clean_ideal", "clean_icm", "nomag_icm", "outage5_icm",
         "noise_icm", "latency200_icm"]
LABELS = ["理想 IMU", "ICM", "无磁", "5 s 失联", "GNSS 2.5 m", "200 ms 偏移"]

# ============================================================
# 图 1：时序 + 3D 轨迹
# ============================================================
d = trace("msl_fixed_trace.csv")
t = d["t"]
er = d["ekf_roll"] - d["truth_roll"]
ep = d["ekf_pitch"] - d["truth_pitch"]
ey = (d["ekf_yaw"] - d["truth_yaw"] + 180) % 360 - 180
pos_err = np.hypot(d["ekf_n"] - d["true_n"], d["ekf_e"] - d["true_e"])

fig = plt.figure(figsize=(14.2, 10.0), dpi=160, constrained_layout=True)
gs = fig.add_gridspec(2, 3, height_ratios=[1.0, 1.0])

a = fig.add_subplot(gs[0, 0], projection="3d")
a.plot(d["true_n"], d["true_e"], d["truth_alt"], color=C_FIX, lw=1.6)
a.scatter([d["true_n"][0]], [d["true_e"][0]], [d["truth_alt"][0]],
          color=C_ALT, s=18, label="发射")
a.scatter([d["true_n"][-1]], [d["true_e"][-1]], [d["truth_alt"][-1]],
          color=C_ORANGE, s=18, label="45 s")
a.set_xlabel("北 [m]")
a.set_ylabel("东 [m]")
a.set_zlabel("高度 [m]")
a.set_title("(a) 45 s 三维弹道（6.4 g 标称）", fontsize=10)
a.legend(fontsize=7, loc="upper left")

a = fig.add_subplot(gs[0, 1])
for k, lab, c in (("roll", "滚转", C_FIX), ("pitch", "俯仰", C_ALT),
                  ("yaw", "航向", C_ORANGE)):
    a.plot(t, d[f"truth_{k}"], color=c, lw=1.0, alpha=0.5)
    a.plot(t, d[f"ekf_{k}"], color=c, lw=0.9, ls="--")
a.set_xlabel("t [s]")
a.set_ylabel("角度 [°]")
a.set_title("(b) 真值（实线）vs EKF（虚线）：三轴姿态", fontsize=10)
a.grid(alpha=0.3)

a = fig.add_subplot(gs[0, 2])
a.plot(t, er, color=C_FIX, lw=1.0, label="滚转")
a.plot(t, ep, color=C_ALT, lw=1.0, label="俯仰")
a.plot(t, ey, color=C_ORANGE, lw=1.0, label="航向")
a.axvspan(22, 30, color="#f2c14e", alpha=0.15)
a.set_xlabel("t [s]")
a.set_ylabel("误差 [°]")
a.set_title("(c) 分轴姿态误差（22–30 s 无坡度转弯）", fontsize=10)
a.grid(alpha=0.3)
a.legend(fontsize=8)

a = fig.add_subplot(gs[1, 0])
win = 100
rms = np.sqrt(np.convolve(pos_err * pos_err, np.ones(win) / win, mode="same"))
a.semilogy(t, np.maximum(pos_err, 1e-3), color=C_FIX, lw=0.4, alpha=0.25,
           label="瞬时")
a.semilogy(t, np.maximum(rms, 1e-3), color=C_ORANGE, lw=1.6,
           label="0.5 s 滚动 RMS")
a.set_xlabel("t [s]")
a.set_ylabel("水平位置误差 [m]（对数）")
a.set_title("(d) 水平位置误差：瞬时+0.5 s 滚动 RMS（对数）", fontsize=10)
a.legend(fontsize=7)
a.grid(alpha=0.3)

a = fig.add_subplot(gs[1, 1])
a.plot(t, d["truth_alt"], color=C_ALT, lw=1.2, label="真值高度")
a.plot(t, d["truth_vz"], color=C_FIX, lw=0.9, label="真值垂速")
a.set_xlabel("t [s]")
a.set_ylabel("m / (m/s)")
a.set_title("(e) 高度/垂速时序（非导航精度图，剖面核对）", fontsize=10)
a.grid(alpha=0.3)
a.legend(fontsize=7)

a = fig.add_subplot(gs[1, 2])
a.plot(t, d["gyro_x"] * 57.29578, color=C_FIX, lw=0.9, label="p")
a.plot(t, d["gyro_y"] * 57.29578, color=C_ALT, lw=0.9, label="q")
a.plot(t, d["gyro_z"] * 57.29578, color=C_ORANGE, lw=0.9, label="r")
a.set_xlabel("t [s]")
a.set_ylabel("机体角速率 [°/s]")
a.set_title("(f) 机体角速率（最大 27.8°/s）", fontsize=10)
a.grid(alpha=0.3)
a.legend(fontsize=8)

fig.suptitle("6DOF 导弹 + 真实固件导航 SIL：标称弹道总览（L2，ICM，45 s）",
             fontsize=12)
fig.savefig(os.path.join(FIG, "nav_missile_overview_2026-10-27.png"))
plt.close(fig)

# ============================================================
# 图 2：20/40-seed 分布
# ============================================================
fig, ax = plt.subplots(2, 3, figsize=(14.2, 9.4), dpi=160,
                       constrained_layout=True)
x = np.arange(len(CONDS))
w = 0.38

a = ax[0, 0]
data = [vals(fix, q + "_full", "nav_att_rmse") for q in CONDS]
bp = a.boxplot(data, positions=x, widths=0.5, patch_artist=True,
               showfliers=True)
for patch in bp["boxes"]:
    patch.set_facecolor(C_FIX)
    patch.set_alpha(0.75)
bv = [med(base, q + "_full", "nav_att_rmse") for q in CONDS]
a.plot(x, bv, "r.", ms=8, label="unarmed 无方向门中位")
a.set_xticks(x)
a.set_xticklabels(LABELS, fontsize=8, rotation=15)
a.set_ylabel("姿态 RMSE [°]")
a.set_title("(a) 武装 40 seeds 箱线；红点=unarmed 无方向门中位", fontsize=10)
a.grid(axis="y", alpha=0.3)
a.legend(fontsize=8)

a = ax[0, 1]
data = [vals(fix, q + "_full", "pos_rms") for q in CONDS]
vp = a.violinplot(data, positions=x, widths=0.65, showmedians=True,
                  showextrema=True)
for body in vp["bodies"]:
    body.set_facecolor(C_FIX)
    body.set_alpha(0.55)
bv = [med(base, q + "_full", "pos_rms") for q in CONDS]
a.plot(x, bv, "r.", ms=8, label="unarmed 无方向门中位")
a.set_xticks(x)
a.set_xticklabels(LABELS, fontsize=8, rotation=15)
a.set_ylabel("位置 RMSE [m]")
a.set_title("(b) 武装 40 seeds 小提琴；红点=unarmed 无方向门中位", fontsize=10)
a.grid(axis="y", alpha=0.3)
a.legend(fontsize=8)

a = ax[0, 2]
for rows, q, lab, c, suffix in ((fix, "clean_icm_full", "6.4 g", C_FIX, ""),
                                (highg, "thrust8g_icm_full", "8.4 g", C_ALT, ""),
                                (highg, "thrust10g_icm_full", "10.1 g/8g FS", C_BASE, "")):
    v = np.sort(vals(rows, q + suffix, "pos_max"))
    if v.size:
        a.plot(v, np.arange(1, v.size + 1) / v.size, color=c, lw=1.6,
               label=lab)
a.set_xscale("log")
a.set_xlabel("位置峰值 [m]（对数）")
a.set_ylabel("CDF")
a.set_title("(c) 位置峰值 CDF：10.1 g 在 ±8 g 触顶", fontsize=10)
a.grid(alpha=0.3)
a.legend(fontsize=8)

a = ax[1, 0]
v = vals(fix, "clean_icm_full", "nav_att_rmse")
a.hist(v, bins=12, color=C_FIX, alpha=0.8, edgecolor="white")
a.set_xlabel("姿态 RMSE [°]")
a.set_ylabel("seed 数")
a.set_title("(d) ICM 40 seeds 姿态 RMSE 直方图", fontsize=10)
a.grid(axis="y", alpha=0.3)

a = ax[1, 1]
for rows, q, lab, c in ((fix, "clean_icm_full", "ICM", C_FIX),
                        (fix, "clean_ideal_full", "理想", C_ALT),
                        (fix, "nomag_icm_full", "无磁", C_ORANGE),
                        (fix, "noise_icm_full", "GNSS 2.5 m", C_PURPLE)):
    px = vals(rows, q, "pos_max")
    py = vals(rows, q, "nav_att_max")
    n = min(px.size, py.size)
    if n:
        a.scatter(px[:n], py[:n], s=14, color=c, alpha=0.7, label=lab)
a.set_xlabel("位置峰值 [m]")
a.set_ylabel("姿态峰值 [°]")
a.set_title("(e) 位置峰值-姿态峰值散点（40 seeds）", fontsize=10)
a.grid(alpha=0.3)
a.legend(fontsize=7)

a = ax[1, 2]
mat = np.array([[med(fix, q + "_full", "pos_rms") for q in CONDS],
                [med(fix, q + "_full", "nav_att_rmse") for q in CONDS],
                [med(fix, q + "_full", "nav_vel_rms") for q in CONDS]])
im = a.imshow(mat, aspect="auto", cmap="viridis")
a.set_xticks(np.arange(len(CONDS)))
a.set_xticklabels(LABELS, fontsize=7, rotation=20)
a.set_yticks([0, 1, 2])
a.set_yticklabels(["位置 m", "姿态 °", "速度 m/s"])
for i in range(3):
    for j in range(len(CONDS)):
        a.text(j, i, f"{mat[i, j]:.2g}", ha="center", va="center",
               color="white" if mat[i, j] < mat.max() * 0.6 else "black",
               fontsize=7)
a.set_title("(f) 武装 6 条件指标热图", fontsize=10)
fig.colorbar(im, ax=a, shrink=0.8)

fig.suptitle("6DOF 导弹 SIL：武装 40 seeds 分布与尾部（L2，±8 g）",
             fontsize=12)
fig.savefig(os.path.join(FIG, "nav_missile_distributions_2026-10-27.png"))
plt.close(fig)

# ============================================================
# 图 3：振动/故障/失联/刷新率鲁棒性
# ============================================================
fig, ax = plt.subplots(2, 3, figsize=(14.2, 9.2), dpi=160,
                       constrained_layout=True)

a = ax[0, 0]
vc = [("vib1g_6p4g_icm_full", "1 g 飞行/6.4 g", vib),
      ("vib1g_8p4g_icm_full", "1 g 飞行/8.4 g", vib84),
      ("vib2g_8p4g_icm_full", "2 g 飞行/8.4 g", vib),
      ("viblaunch1g_6p4g_icm_full", "1 g 发射/6.4 g", viblaunch),
      ("viblaunch2g_8p4g_icm_full", "2 g 发射/8.4 g", viblaunch)]
xx = np.arange(1, len(vc) + 1)
a.bar(xx, [med(src, q, "nav_att_rmse") for q, _, src in vc],
      color=[C_FIX, C_ALT, C_BASE, C_FIX, C_BASE], alpha=0.85)
for i, (q, _, src) in enumerate(vc, start=1):
    a.text(i, med(src, q, "nav_att_rmse") + 0.15,
           f"{med(src, q, 'nav_att_rmse'):.2f}", ha="center", fontsize=7)
a.set_xticks(np.arange(1, len(vc) + 1))
a.set_xticklabels([l for _, l, _ in vc], fontsize=7, rotation=20)
a.set_xlim(0.5, len(vc) + 0.5)
a.set_ylim(0, 10.5)
a.set_ylabel("姿态 RMSE 中位 [°]")
a.set_title("(a) 生产双路振动：1 g 6.95°、2 g 8.32°；发射 v2 20/20",
            fontsize=10)
a.grid(axis="y", alpha=0.3)

a = ax[0, 1]
fc = [("imu_nan_icm_full", "IMU NaN 1 s"), ("imu_stuck_icm_full", "IMU 冻结 1 s"),
      ("gnss_spoof20_icm_full", "spoof 20 m"), ("gnss_spoof30_icm_full", "spoof 30 m"),
      ("mag_dropout_icm_full", "磁失联 5 s"), ("baro_drift_icm_full", "气压 0.5 m/s")]
a.bar(np.arange(len(fc)), [med(fault, q, "pos_rms") for q, _ in fc],
      color=[C_BASE if q == "imu_nan_icm_full" else C_FIX for q, _ in fc],
      alpha=0.85)
a.set_xticks(np.arange(len(fc)))
a.set_xticklabels([l for _, l in fc], fontsize=7, rotation=20)
a.set_ylabel("位置 RMSE 中位 [m]")
a.set_title("(b) 故障注入：1 s IMU NaN 122 m，其余有界", fontsize=10)
a.grid(axis="y", alpha=0.3)

a = ax[0, 2]
oc = [("clean_icm_full", "0", fix), ("outage5_icm_full", "5", fix),
      ("outage10_icm_full", "10", out), ("outage20_icm_full", "20", out),
      ("outage30_icm_full", "30", out)]
medv = [med(src, q, "pos_rms") for q, _, src in oc]
p95 = [np.percentile(vals(src, q, "pos_rms"), 95) for q, _, src in oc]
mx = [np.max(vals(src, q, "pos_max")) for q, _, src in oc]
xx = np.arange(len(oc))
a.plot(xx, medv, "o-", color=C_FIX, label="中位")
a.plot(xx, p95, "s--", color=C_ORANGE, label="p95")
a.plot(xx, mx, "^:", color=C_BASE, label="峰值")
a.set_xticks(xx)
a.set_xticklabels([l + " s" for _, l, _ in oc])
a.set_ylabel("位置误差 [m]")
a.set_title("(c) GNSS 失联 0–30 s：中位 2.6→20.5 m", fontsize=10)
a.grid(alpha=0.3)
a.legend(fontsize=7)

a = ax[1, 0]
rc = [("hz5_icm_full", "5 Hz"), ("clean_icm_full", "10 Hz"),
      ("hz20_icm_full", "20 Hz")]
a.bar(np.arange(len(rc)), [med(out, q, "nav_att_rmse") if q != "clean_icm_full"
                            else med(fix, q, "nav_att_rmse") for q, _ in rc],
      color=[C_BASE, C_FIX, C_ALT], alpha=0.85)
a.set_xticks(np.arange(len(rc)))
a.set_xticklabels([l for _, l in rc], fontsize=8)
a.set_ylabel("姿态 RMSE 中位 [°]")
a.set_title("(d) 刷新率：5 Hz 0.66°、20 Hz 0.72°", fontsize=10)
a.grid(axis="y", alpha=0.3)

a = ax[1, 1]
hc = [("thrust8g_icm_full", "8.4 g"), ("thrust10g_icm_full", "10.1 g")]
hv = [med(highg, q, "pos_rms") for q, _ in hc]
a.bar(np.arange(len(hc)), hv, color=[C_ALT, C_BASE], alpha=0.85)
a.set_yscale("log")
a.set_xticks(np.arange(len(hc)))
a.set_xticklabels([l for _, l in hc])
a.set_ylabel("位置 RMSE 中位 [m]（对数）")
a.set_title("(e) ±8 g：8.4 g 3.7 m，10.1 g 623 m（触顶）", fontsize=10)
a.grid(axis="y", alpha=0.3)

a = ax[1, 2]
ac = [("low_align", "low ×2"), ("adverse_align", "adverse ×5")]
a.bar(np.arange(len(ac)), [med(align, q, "nav_att_rmse") for q, _ in ac],
      color=[C_FIX, C_ORANGE], alpha=0.85)
a.set_xticks(np.arange(len(ac)))
a.set_xticklabels([l for _, l in ac])
a.set_ylabel("姿态 RMSE 中位 [°]")
a.set_title("(f) 高零偏对准（opt-in，生产双路 20 seeds）", fontsize=10)
a.grid(axis="y", alpha=0.3)

fig.suptitle("6DOF 导弹 SIL：振动/故障/失联/刷新率/量程/对准鲁棒性（L2）",
             fontsize=12)
fig.savefig(os.path.join(FIG, "nav_missile_robustness_2026-10-27.png"))
plt.close(fig)

# ============================================================
# 图 4：seed × 条件尾部热图
# ============================================================
fig, ax = plt.subplots(1, 2, figsize=(14.0, 5.2), dpi=160,
                       constrained_layout=True)
for k, (rows, suffix, title) in enumerate((
        (base, "", "unarmed 无方向门（20 seeds）"),
        (fix, "", "武装生产双路（40 seeds）"))):
    seeds = sorted({int(r["seed"]) for r in rows})
    mat = np.full((len(CONDS), len(seeds)), np.nan)
    for i, q in enumerate(CONDS):
        for j, s in enumerate(seeds):
            r = [x for x in rows if x["config"] == q + "_full" + suffix and
                 int(x["seed"]) == s]
            if r:
                mat[i, j] = r[0]["pos_max"]
    im = ax[k].imshow(mat, aspect="auto", cmap="magma",
                      norm=LogNorm(vmin=1.0, vmax=100.0))
    ax[k].set_yticks(np.arange(len(CONDS)))
    ax[k].set_yticklabels(LABELS, fontsize=8)
    ax[k].set_xticks(np.arange(0, len(seeds), 4))
    ax[k].set_xticklabels([str(seeds[i]) for i in range(0, len(seeds), 4)],
                          fontsize=8)
    ax[k].set_xlabel("seed")
    ax[k].set_title(title, fontsize=10)
    fig.colorbar(im, ax=ax[k], shrink=0.8, label="位置峰值 [m]（log）")
fig.suptitle("位置峰值 seed×条件热图：unarmed 无方向门 vs 武装生产双路（200 ms 偏移另计）",
             fontsize=12)
fig.savefig(os.path.join(FIG, "nav_missile_tail_heatmap_2026-10-27.png"))
plt.close(fig)

print("saved overview/distributions/robustness/tail_heatmap")
