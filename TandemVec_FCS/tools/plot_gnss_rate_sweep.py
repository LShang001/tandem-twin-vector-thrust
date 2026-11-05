#!/usr/bin/env python3
"""@file    plot_gnss_rate_sweep.py
@module  工具
@brief   GNSS 更新率 × 速度 × 加速度 扫描结果可视化（双图 + 汇总表）
GNSS 更新率 × 速度 × 加速度 扫描结果可视化。

输入：tmp/gnss_rate_sweep.csv（gnss_hz,v_amp,a_peak,seed,pos_rmse,att_rmse_deg,updates）
输出：tmp/gnss_rate_vs_speed.png、tmp/gnss_rate_vs_accel.png
@details 读 tmp/ 下的扫描 CSV（要求 7 列，解析失败的行跳过）；按
         (更新率, 速度, 加速度) 组合对 pos_rmse 取算术均值；输出两张 PNG 与
         控制台汇总表（行=更新率，列分别为速度 / 加速度）。
@frame   N/A（只对 SIL 汇总指标求均值并作图，不做坐标变换；pos_rmse 的坐标系
         口径由生成该 CSV 的 SIL 决定）
@units   Hz（更新率）、m/s（速度幅值）、m/s²（加速度峰值）、m（位置 RMSE）、
         deg（姿态 RMSE）
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
CSV = os.path.join(REPO, "tmp", "gnss_rate_sweep_v2.csv")
OUT1 = os.path.join(REPO, "tmp", "gnss_rate_vs_speed.png")
OUT2 = os.path.join(REPO, "tmp", "gnss_rate_vs_accel.png")

rows = []
with open(CSV, encoding="utf-8") as f:
    header = f.readline()
    for line in f:
        p = line.strip().split(",")
        if len(p) != 7:
            continue
        try:
            rows.append((float(p[0]), float(p[1]), float(p[2]), int(p[3]),
                         float(p[4]), float(p[5]), int(p[6])))
        except ValueError:
            continue
print("rows:", len(rows))

rates = sorted({r[0] for r in rows})
speeds = sorted({r[1] for r in rows})
accels = sorted({r[2] for r in rows})


def agg(rate, v=None, a=None):
    """对满足 (更新率, 速度, 加速度) 条件的行取位置 RMSE 的算术均值。

    参数：rate 为 GNSS 更新率（Hz）；v 为速度幅值（m/s，None=不筛）；
    a 为加速度峰值（m/s²，None=不筛）。
    返回：均值（m）；无匹配行返回 NaN。
    """
    vals = [r[4] for r in rows
            if r[0] == rate and (v is None or r[1] == v) and (a is None or r[2] == a)]
    return float(np.mean(vals)) if vals else float("nan")


# 1) 位置 RMSE vs 更新率，按速度分组（a_peak=2）
fig, ax = plt.subplots(figsize=(8.5, 5), constrained_layout=True)
for v in speeds:
    ys = [agg(r, v=v, a=2.0) for r in rates]
    ax.plot(rates, ys, "o-", label=f"v={v:g} m/s")
ax.set_xscale("log")
ax.set_xlabel("GNSS 更新率 [Hz]")
ax.set_ylabel("位置 RMSE [m]")
ax.set_title("位置误差 vs GNSS 更新率（加速度峰值 2 m/s²）")
ax.grid(True, alpha=0.3)
ax.legend()
fig.savefig(OUT1, dpi=150)
print("图已保存：", OUT1)

# 2) 位置 RMSE vs 更新率，按加速度分组（v=5）
fig, ax = plt.subplots(figsize=(8.5, 5), constrained_layout=True)
for a in accels:
    ys = [agg(r, v=5.0, a=a) for r in rates]
    ax.plot(rates, ys, "o-", label=f"a={a:g} m/s²")
ax.set_xscale("log")
ax.set_xlabel("GNSS 更新率 [Hz]")
ax.set_ylabel("位置 RMSE [m]")
ax.set_title("位置误差 vs GNSS 更新率（速度 5 m/s）")
ax.grid(True, alpha=0.3)
ax.legend()
fig.savefig(OUT2, dpi=150)
print("图已保存：", OUT2)

# 3) 控制台汇总表
print("\n位置 RMSE [m]（行=更新率，列=速度，a=2）")
print("rate\\v  " + "  ".join(f"{v:>6g}" for v in speeds))
for r in rates:
    print(f"{r:>4g}  " + "  ".join(f"{agg(r, v=v, a=2.0):6.2f}" for v in speeds))
print("\n位置 RMSE [m]（行=更新率，列=加速度，v=5）")
print("rate\\a  " + "  ".join(f"{a:>6g}" for a in accels))
for r in rates:
    print(f"{r:>4g}  " + "  ".join(f"{agg(r, v=5.0, a=a):6.2f}" for a in accels))
