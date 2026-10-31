#!/usr/bin/env python3
"""IMU/GNSS 导航性能矩阵可视化（数据：tmp/imuperf/out.csv 与 trace_*.csv）。

输出 PNG 到同目录，供人工/视觉模型审查。全部英文标签，避免中文字体缺失。
"""
import csv
import math
import os
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

BASE = os.path.dirname(os.path.abspath(__file__))
# 数据目录由环境变量指定（默认 ../tmp/imuperf，即仓库内临时证据目录）。
DATA = os.environ.get("IMUPERF_DATA", os.path.join(BASE, "..", "tmp", "imuperf"))
OUT = os.environ.get("IMUPERF_FIGS", os.path.join(BASE, "..", "tmp", "imuperf"))

IMUS = ["ICM42688P_2k", "ICM45686_3k2"]
GNSS = ["RTK_10Hz", "RTK_10Hz_100ms", "M8N_10Hz", "M8N_5Hz", "M8N_1Hz", "LowEnd_5Hz"]
SCEN = ["static", "maneuver", "cruise"]
COLORS = {"ICM42688P_2k": "#3b6ea5", "ICM45686_3k2": "#c0504d"}


def load_matrix():
    rows = []
    with open(os.path.join(DATA, "out.csv"), encoding="utf-8") as f:
        for line in f:
            p = line.strip().split(",")
            if len(p) == 10 and p[0] in IMUS:
                rows.append({
                    "imu": p[0], "gnss": p[1], "scen": p[2], "seed": int(p[3]),
                    "roll": float(p[4]), "pitch": float(p[5]), "yaw": float(p[6]),
                    "vel": float(p[7]), "pos": float(p[8]),
                })
    return rows


def mean_by(rows, key, field):
    acc = defaultdict(list)
    for r in rows:
        acc[key(r)].append(r[field])
    return {k: float(np.mean(v)) for k, v in acc.items()}


def plot_heatmap(rows):
    fig, axes = plt.subplots(1, 2, figsize=(13, 3.6), constrained_layout=True)
    for ax, field, title, unit in zip(
            axes, ["pos", "vel"], ["Position RMSE", "Velocity RMSE"], ["m", "m/s"]):
        m = mean_by(rows, lambda r: (r["imu"], r["gnss"]), field)
        data = np.array([[m[(im, g)] for g in GNSS] for im in IMUS])
        im = ax.imshow(np.log10(np.maximum(data, 1e-4)), cmap="viridis", aspect="auto")
        ax.set_xticks(range(len(GNSS)))
        ax.set_xticklabels(GNSS, rotation=25, ha="right", fontsize=8)
        ax.set_yticks(range(len(IMUS)))
        ax.set_yticklabels(IMUS, fontsize=9)
        ax.set_title(f"{title} (mean over scenarios)")
        for i in range(len(IMUS)):
            for j in range(len(GNSS)):
                v = data[i, j]
                ax.text(j, i, f"{v:.3g}", ha="center", va="center",
                        color="white" if math.log10(max(v, 1e-4)) < -1.0 else "black",
                        fontsize=8)
        cbar = fig.colorbar(im, ax=ax, fraction=0.035, pad=0.02)
        cbar.set_label(f"log10({unit})", fontsize=8)
        cbar.ax.tick_params(labelsize=7)
    fig.suptitle("IMU x GNSS navigation performance: RMSE heatmap", fontsize=11)
    fig.savefig(os.path.join(OUT, "01_heatmap.png"), dpi=150)
    plt.close(fig)


def plot_grouped(rows):
    fig, axes = plt.subplots(1, 3, figsize=(14, 3.8), constrained_layout=True)
    metrics = [("att", lambda r: 0.5 * (r["roll"] + r["pitch"]), "Attitude RMSE [deg]"),
               ("vel", lambda r: r["vel"], "Velocity RMSE [m/s]"),
               ("pos", lambda r: r["pos"], "Position RMSE [m]")]
    for ax, (name, fn, label) in zip(axes, metrics):
        x = np.arange(len(GNSS))
        width = 0.38
        for k, imu in enumerate(IMUS):
            vals = [np.mean([fn(r) for r in rows if r["imu"] == imu and r["gnss"] == g])
                    for g in GNSS]
            ax.bar(x + (k - 0.5) * width, vals, width, label=imu,
                   color=COLORS[imu], alpha=0.85)
        ax.set_xticks(x)
        ax.set_xticklabels(GNSS, rotation=30, ha="right", fontsize=7)
        ax.set_yscale("log")
        ax.set_ylabel(label, fontsize=8)
        ax.grid(True, axis="y", alpha=0.3)
    axes[0].legend(fontsize=8)
    fig.suptitle("Performance by GNSS class (mean over scenarios/seeds)", fontsize=11)
    fig.savefig(os.path.join(OUT, "02_grouped_bar.png"), dpi=150)
    plt.close(fig)


def plot_cdf(rows):
    fig, ax = plt.subplots(figsize=(6.2, 4.0), constrained_layout=True)
    for g in GNSS:
        vals = sorted(r["pos"] for r in rows if r["gnss"] == g)
        if not vals:
            continue
        ax.plot(vals, np.linspace(0, 1, len(vals)), marker=".", label=g, lw=1.2)
    ax.set_xscale("log")
    ax.set_xlabel("Position RMSE [m]")
    ax.set_ylabel("CDF")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7)
    ax.set_title("Position error CDF by GNSS class (both IMUs, all scenarios)")
    fig.savefig(os.path.join(OUT, "03_cdf_pos.png"), dpi=150)
    plt.close(fig)


def plot_box(rows):
    fig, ax = plt.subplots(figsize=(9, 4.0), constrained_layout=True)
    data, labels, colors = [], [], []
    for g in GNSS:
        for imu in IMUS:
            vals = [r["pos"] for r in rows if r["gnss"] == g and r["imu"] == imu]
            if vals:
                data.append(vals)
                labels.append(f"{g}\n{imu.split('_')[0]}")
                colors.append(COLORS[imu])
    bp = ax.boxplot(data, patch_artist=True, widths=0.6, showfliers=True)
    for patch, c in zip(bp["boxes"], colors):
        patch.set_facecolor(c)
        patch.set_alpha(0.6)
    ax.set_yscale("log")
    ax.set_ylabel("Position RMSE [m]")
    ax.set_xticklabels(labels, fontsize=7, rotation=0)
    ax.grid(True, axis="y", alpha=0.3)
    ax.set_title("Position error distribution (log scale)")
    fig.savefig(os.path.join(OUT, "04_box_pos.png"), dpi=150)
    plt.close(fig)


def load_trace(name):
    """trace 文件为 15 列纯数字（t, roll_t, roll_e, pitch_t, pitch_e,
    pn_t, pn_e, pe_t, pe_e, pd_t, pd_e, vn_t, vn_e, ve_t, ve_e）。"""
    path = os.path.join(DATA, name)
    return np.loadtxt(path, delimiter=",")


def plot_trace_maneuver():
    a = load_trace("trace_maneuver.csv")
    t = a[:, 0]
    fig, axes = plt.subplots(3, 1, figsize=(9, 7.2), sharex=True,
                             constrained_layout=True)
    axes[0].plot(t, a[:, 1] * 180 / math.pi, "k--", lw=1.0, label="truth roll")
    axes[0].plot(t, a[:, 2] * 180 / math.pi, color=COLORS["ICM45686_3k2"],
                 lw=1.2, label="EKF roll")
    axes[0].set_ylabel("roll [deg]")
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)
    axes[1].plot(t, a[:, 3] * 180 / math.pi, "k--", lw=1.0, label="truth pitch")
    axes[1].plot(t, a[:, 4] * 180 / math.pi, color=COLORS["ICM45686_3k2"],
                 lw=1.2, label="EKF pitch")
    axes[1].set_ylabel("pitch [deg]")
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.3)
    axes[2].plot(t, (a[:, 2] - a[:, 1]) * 180 / math.pi,
                 color="#c0504d", lw=1.0, label="roll err")
    axes[2].plot(t, (a[:, 4] - a[:, 3]) * 180 / math.pi,
                 color="#3b6ea5", lw=1.0, label="pitch err")
    axes[2].set_ylabel("error [deg]")
    axes[2].set_xlabel("time [s]")
    axes[2].legend(fontsize=8)
    axes[2].grid(alpha=0.3)
    fig.suptitle("ICM45686 @3.2kHz, M8N_5Hz, maneuver: attitude tracking", fontsize=11)
    fig.savefig(os.path.join(OUT, "05_trace_maneuver.png"), dpi=150)
    plt.close(fig)


def plot_trace_cruise():
    a = load_trace("trace_cruise.csv")
    t = a[:, 0]
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.0), constrained_layout=True)
    axes[0].plot(a[:, 5], a[:, 7], "k--", lw=1.0, label="truth")
    axes[0].plot(a[:, 6], a[:, 8], color=COLORS["ICM45686_3k2"], lw=1.2, label="EKF")
    axes[0].set_xlabel("North [m]")
    axes[0].set_ylabel("East [m]")
    axes[0].set_title("Cruise trajectory (N-E plane)", fontsize=9)
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)
    err_n = a[:, 6] - a[:, 5]
    err_e = a[:, 8] - a[:, 7]
    err_d = a[:, 10] - a[:, 9]
    axes[1].plot(t, err_n, label="N err", lw=1.0)
    axes[1].plot(t, err_e, label="E err", lw=1.0)
    axes[1].plot(t, err_d, label="D err", lw=1.0)
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("position error [m]")
    axes[1].grid(alpha=0.3)
    axes[1].legend(fontsize=8)
    axes[1].set_title("EKF position error", fontsize=9)
    fig.suptitle("ICM45686 @3.2kHz, M8N_10Hz, cruise: trajectory and error", fontsize=11)
    fig.savefig(os.path.join(OUT, "06_trace_cruise.png"), dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    rows = load_matrix()
    print(f"loaded {len(rows)} rows")
    plot_heatmap(rows)
    plot_grouped(rows)
    plot_cdf(rows)
    plot_box(rows)
    plot_trace_maneuver()
    plot_trace_cruise()
    print("figures written to", OUT)
