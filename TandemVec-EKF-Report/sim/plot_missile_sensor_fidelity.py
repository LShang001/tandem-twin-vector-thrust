# -*- coding: utf-8 -*-
"""导弹 SIL 传感器路径保真度可视化：软件 LPF / 轴映射 / AHRS 门。

@file    plot_missile_sensor_fidelity.py
@brief   生产 IMU 软件低通与 Madgwick 接入 SIL 后的 A/B 视觉审查。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 输出 nav_missile_sensor_fidelity_2026-10-27.png：
          LPF 频响、LPF on/off 速度、轴映射错误、AHRS 门 A/B、姿态时序。
@frame   NED；姿态 ZYX 欧拉角。
@units   Hz、dB、°、m/s、s。
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

C_OFF = "#888888"
C_ON = "#1f6fb4"
C_WRONG = "#e45756"
C_GATE = "#54a24b"
C_NOGATE = "#e45756"


def load(name):
    path = os.path.join(DATA, name)
    return [json.loads(l) for l in open(path, encoding="utf-8")
            if not json.loads(l).get("no_json")]


def vals(rows, cfg, key):
    return np.array([r[key] for r in rows if r["config"] == cfg and
                     isinstance(r.get(key), (int, float)) and np.isfinite(r[key])],
                    float)


def med(rows, cfg, key):
    v = vals(rows, cfg, key)
    return float(np.median(v)) if v.size else np.nan


off = load("missile-sil-2026-10-27_lpfab_off20.jsonl")
on = load("missile-sil-2026-10-27_lpfab_on20.jsonl")
wrong = load("missile-sil-2026-10-27_lpfab_wrongaxis20.jsonl")
goff = load("missile-sil-2026-10-27_prod_gateoff20.jsonl")
gon = load("missile-sil-2026-10-27_prod_gateon20.jsonl")
def resolve_tmp(path):
    # Git Bash 的 /tmp 与 Windows Python 的 TEMP 是同一目录，但路径写法不同。
    if os.path.exists(path):
        return path
    if path.startswith("/tmp/"):
        alt = os.path.join(os.environ.get("TEMP", ""), path[5:])
        if os.path.exists(alt):
            return alt
    return path


tr_off = np.genfromtxt(resolve_tmp("/tmp/msl_gateoff.csv"), delimiter=",",
                       names=True)
tr_on = np.genfromtxt(resolve_tmp("/tmp/msl_gateon.csv"), delimiter=",",
                      names=True)

fig, ax = plt.subplots(2, 3, figsize=(14.2, 9.2), dpi=160,
                       constrained_layout=True)

# (a) LPF 频响：生产传感器系系数 αz=0.05 / αxy=0.02 / 陀螺 α=0.15
dt = 0.0005
f = np.linspace(0.1, 150.0, 4000)
w = 2.0 * np.pi * f
zeta = np.exp(-1j * w * dt)
a = ax[0, 0]
for alpha, lab, c in ((0.15, "陀螺 α=0.15（τ=2.8 ms）", C_GATE),
                      (0.05, "加计纵向 α=0.05（τ=9.5 ms）", C_ON),
                      (0.02, "加计横向 α=0.02（τ=24.5 ms）", C_NOGATE)):
    h = alpha / (1.0 - (1.0 - alpha) * zeta)
    a.semilogx(f, 20.0 * np.log10(np.abs(h)), color=c, lw=1.5, label=lab)
for fx in (80.0, 120.0):
    a.axvline(fx, color="#f2c14e", ls="--", lw=0.8)
a.set_xlabel("频率 [Hz]")
a.set_ylabel("幅值 [dB]")
a.set_title("(a) 生产软件 LPF 幅频：80/120 Hz 振动被压 14–26 dB", fontsize=10)
a.grid(alpha=0.3)
a.legend(fontsize=7)

a = ax[0, 1]
for alpha, lab, c in ((0.15, "陀螺", C_GATE), (0.05, "加计纵向", C_ON),
                      (0.02, "加计横向", C_NOGATE)):
    h = alpha / (1.0 - (1.0 - alpha) * zeta)
    a.semilogx(f, -np.unwrap(np.angle(h)) * 180.0 / np.pi, color=c, lw=1.5,
               label=lab)
a.set_xlabel("频率 [Hz]")
a.set_ylabel("相位滞后 [°]")
a.set_title("(b) 相频：低频群延迟 2.8/9.5/24.5 ms，转弯时比力方向滞后",
            fontsize=10)
a.grid(alpha=0.3)
a.legend(fontsize=7)

# (c) LPF on/off：速度 RMS（gate-on，20 seeds）
a = ax[0, 2]
cfgs = ["clean_icm_full", "outage10_icm_full", "outage20_icm_full",
        "vib1g_6p4g_icm_full", "vib2g_8p4g_icm_full"]
labs = ["ICM", "失联 10 s", "失联 20 s", "1 g 振动", "2 g 振动"]
x = np.arange(len(cfgs))
a.bar(x - 0.19, [med(off, q, "nav_vel_rms") for q in cfgs], width=0.38,
      color=C_OFF, alpha=0.85, label="未滤波旧口径")
a.bar(x + 0.19, [med(on, q, "nav_vel_rms") for q in cfgs], width=0.38,
      color=C_ON, alpha=0.85, label="生产 LPF")
a.set_xticks(x)
a.set_xticklabels(labs, fontsize=8, rotation=15)
a.set_ylabel("速度 RMS 中位 [m/s]")
a.set_title("(c) LPF 代价：失联段速度 0.88→1.27 m/s，振动段 0.45→0.63", fontsize=10)
a.grid(axis="y", alpha=0.3)
a.legend(fontsize=8)

# (d) 轴映射错误：机体系 x 应取传感器 z 的 α=0.05
a = ax[1, 0]
cfgs = ["clean_icm_full", "outage5_icm_full", "vib1g_6p4g_icm_full"]
labs = ["ICM", "失联 5 s", "1 g 振动"]
x = np.arange(len(cfgs))
w = 0.26
a.bar(x - w, [med(off, q, "nav_vel_rms") for q in cfgs], width=w,
      color=C_OFF, alpha=0.85, label="不用 LPF")
a.bar(x, [med(wrong, q, "nav_vel_rms") for q in cfgs], width=w,
      color=C_WRONG, alpha=0.85, label="轴系数错（z=0.05→机体系 z）")
a.bar(x + w, [med(on, q, "nav_vel_rms") for q in cfgs], width=w,
      color=C_ON, alpha=0.85, label="修正轴映射")
a.set_xticks(x)
a.set_xticklabels(labs, fontsize=8)
a.set_ylabel("速度 RMS 中位 [m/s]")
a.set_title("(d) 轴映射审计：错映射把失联 5 s 速度放大到 1.27 m/s", fontsize=10)
a.grid(axis="y", alpha=0.3)
a.legend(fontsize=7)

# (e) AHRS 重力方向门 A/B（真实 Madgwick + 生产 LPF）
a = ax[1, 1]
cfgs = ["clean_ideal_full", "clean_icm_full", "nomag_icm_full",
        "outage5_icm_full", "noise_icm_full", "latency200_icm_full"]
labs = ["理想", "ICM", "无磁", "失联 5 s", "GNSS 2.5 m", "200 ms"]
x = np.arange(len(cfgs))
a.bar(x - 0.19, [med(goff, q, "nav_att_rmse") for q in cfgs], width=0.38,
      color=C_NOGATE, alpha=0.85, label="无方向门")
a.bar(x + 0.19, [med(gon, q, "nav_att_rmse") for q in cfgs], width=0.38,
      color=C_GATE, alpha=0.85, label="方向门（默认）")
a.set_xticks(x)
a.set_xticklabels(labs, fontsize=8, rotation=15)
a.set_ylabel("姿态 RMSE 中位 [°]")
a.set_title("(e) 真实 Madgwick：无门 9.8–12.7°，方向门 0.7–0.9°", fontsize=10)
a.grid(axis="y", alpha=0.3)
a.legend(fontsize=8)

# (f) seed1 姿态误差时序（无门错误出现在 36 s 后俯冲段）
a = ax[1, 2]
t = tr_off["t"]
for tr, lab, c in ((tr_off, "无方向门", C_NOGATE),
                   (tr_on, "方向门", C_GATE)):
    ep = tr["ekf_pitch"] - tr["truth_pitch"]
    ey = (tr["ekf_yaw"] - tr["truth_yaw"] + 180.0) % 360.0 - 180.0
    a.plot(t, ep, color=c, lw=1.2, label=lab + " 俯仰")
    a.plot(t, ey, color=c, lw=0.8, ls=":", alpha=0.85, label=lab + " 航向")
a.axvspan(36, 45, color="#f2c14e", alpha=0.12)
a.set_xlabel("t [s]")
a.set_ylabel("姿态误差 [°]")
a.set_title("(f) seed1：无门 36 s 后俯冲段 6.5°/19.6°；有门 <0.8°", fontsize=10)
a.grid(alpha=0.3)
a.legend(fontsize=7, ncol=2)

fig.suptitle("6DOF 导弹 SIL：生产 IMU 软件 LPF / 轴映射 / AHRS 门保真度审计（L2）",
             fontsize=12)
fig.savefig(os.path.join(FIG, "nav_missile_sensor_fidelity_2026-10-27.png"))
plt.close(fig)
print("saved nav_missile_sensor_fidelity_2026-10-27.png")
