# -*- coding: utf-8 -*-
"""振动估计器下游消费点审计的可视化套件（占空比 / 频段响应 / τ̂ 分布）。

@file    plot_vib_audit.py
@brief   把 `check_vib_q_duty.py` 与 `check_vib_band_response.py` 的证据画成多类型图
@module  TandemVec-EKF-Report/可视化（L2 研究设施）
@details 输入：sim-data/vib_q_duty_2026-10-27.json、vib_band_response_2026-10-27.json、
         以及 Temp/vibq/<工况>_s<seed>.csv 原始 200 Hz trace；
         输出：figs/nav_vib_audit_2026-10-27.png（6 面板：时序/条形/频段响应/箱线/散点/CDF）。
         判读要点：零振动弹道上 accel_rms_g 随过载档位上升、vib_high 占空比 63–72%，
         而 1 g 机械振动只在 10–20 Hz 才有效响应（80/120 Hz 仅 +0.04/+0.01 g）。
"""
import glob
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager

ROOT = "D:/纵列双发矢量推力飞行器/TandemVec-EKF-Report"
TR = "C:/Users/12631/AppData/Local/Temp/vibq"
BAND = f"{ROOT}/sim-data/vib_band_response_2026-10-27.json"
DUTY = f"{ROOT}/sim-data/vib_q_duty_2026-10-27.json"
OUT = f"{ROOT}/figs/nav_vib_audit_2026-10-27.png"
GRADE = ["6.4g_洁净", "8.4g", "10.1g"]
VIB = "6.4g_振1g80"


def setup_font():
    for name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC"):
        try:
            font_manager.findfont(name, fallback_to_default=False)
            plt.rcParams["font.family"] = name
            break
        except Exception:
            continue
    plt.rcParams["axes.unicode_minus"] = False


def read_trace(tag, seed=1):
    path = f"{TR}/{tag}_s{seed}.csv"
    with open(path, encoding="utf-8") as fh:
        head = fh.readline().strip().split(",")
        rows = [dict(zip(head, ln.strip().split(",")))
                for ln in fh if ln.strip()]
    want = ("t", "gyro_x", "gyro_y", "gyro_z", "vib_rms_g", "vib_q", "vib_gq",
            "vib_high", "ad_tau_ms", "ad_valid")
    return {k: np.array([float(r[k]) for r in rows])
            for k in want if k in head}


def main():
    setup_font()
    duty = json.load(open(DUTY, encoding="utf-8"))
    band = json.load(open(BAND, encoding="utf-8"))
    fig, ax = plt.subplots(2, 3, figsize=(16.5, 9.2))

    # (a) 时序：高通加速度 RMS 与 |ω|、vib_high
    for i, tag in enumerate([GRADE[0], VIB]):
        d = read_trace(tag)
        omg = np.sqrt(d["gyro_x"] ** 2 + d["gyro_y"] ** 2 +
                      d["gyro_z"] ** 2) * 57.29578
        a = ax[0, 0]
        a.plot(d["t"], np.minimum(d["vib_rms_g"], 0.6), lw=1.1,
           label=f"accel_rms_g {tag}（>0.6 截顶）")
        a.plot(d["t"], omg / 40.0, lw=0.8, alpha=0.55,
               label=f"|ω|/40 [dps] {tag}")
        a.axhline(0.15, color="k", ls="--", lw=0.8)
        a.text(0.3, 0.163, "高振动门限 0.15 g", fontsize=8)
    a = ax[0, 0]
    a.set_xlabel("时间 [s]")
    a.set_ylim(0, 0.62)
    a.set_ylabel("高通加速度 RMS [g]")
    a.set_title("(a) 时序：两者都被机动台阶主导，末段 80 Hz 振动只多 ~0.1 g")
    a.legend(fontsize=7.5)
    a.grid(alpha=0.3)

    # (b) 占空比条形
    labels = ["q 卡 0.25", "q 到 0.15", "q 中间", "vib_high", "gyro Q 顶 0.010"]
    keys = ["q_cap025", "q_floor015", "q_mid", "hi_duty", "gs_cap"]
    x = np.arange(len(keys))
    for j, tag in enumerate(GRADE + [VIB]):
        vals = [np.median([r[k] for r in duty[tag]]) * 100 for k in keys]
        a = ax[0, 1]
        a.bar(x + j * 0.2 - 0.4, vals, width=0.18, label=tag)
    a = ax[0, 1]
    a.set_xticks(x)
    a.set_xticklabels(labels, rotation=20, ha="right", fontsize=8.5)
    a.set_ylabel("飞行段占空比 [%]")
    a.set_ylim(0, 108)
    a.set_title("(b) 下游消费点：加计 Q 100% 卡在保守端、vib_high 63–72%")
    a.legend(fontsize=7.5, ncol=2)
    a.grid(alpha=0.3, axis="y")

    # (c) 频段响应
    order = ["零振动", "10Hz", "20Hz", "40Hz", "80Hz", "120Hz"]
    freqs = [np.nan, 10, 20, 40, 80, 120]
    delta = [band[t]["delta"] for t in order]
    a = ax[0, 2]
    a.semilogx(freqs[1:], delta[1:], "o-", label="1 g 注入")
    a.semilogx([80], [band["80Hz_3g"]["delta"]], "r^", ms=10,
               label="3 g @80 Hz")
    a.axhline(0, color="k", lw=0.8)
    a.axhline(band["零振动"]["rms"], color="gray", ls="--", lw=0.9)
    a.annotate(f"零振动弹道本身 RMS {band['零振动']['rms']:.3f} g" "（虚线，绝对值）",
                 (11, 0.06), fontsize=8)
    a.set_xlabel("注入振动频率 [Hz]（对数轴）")
    a.set_ylabel("Δ accel_rms_g / 1 g")
    a.set_title("(c) 频段响应：80 Hz 灵敏度只有 10 Hz 的 8%，120 Hz 几乎为零")
    a.legend(fontsize=8)
    a.grid(alpha=0.3, which="both")

    # (d) 箱线：零振动下 rms_g 随过载档位
    tr = []
    names = []
    for tag in GRADE:
        for seed in (1, 2, 3):
            f = f"{TR}/{tag}_s{seed}.csv"
            if not os.path.exists(f):
                continue
            d = read_trace(tag, seed)
            fly = d["t"] > 1.2
            tr.append(d["vib_rms_g"][fly][::5])
            names.append(tag.split("_")[0])
    a = ax[1, 0]
    bp = a.boxplot(tr, labels=names, showfliers=False, patch_artist=True)
    for p, c in zip(bp["boxes"], ["#4c72b0", "#dd8452", "#55a868"] * 3):
        p.set_facecolor(c)
        p.set_alpha(0.6)
    a.set_ylabel("高通加速度 RMS [g]")
    a.set_title("(d) 零振动：RMS 随过载档位单调上升（6.4→8.4→10.1 g）")
    a.grid(alpha=0.3, axis="y")

    # (e) 散点：rms_g vs 过载水平
    a = ax[1, 1]
    gx = [6.4, 8.4, 10.1]
    med = [np.median([r["rms_med"] for r in duty[t]]) for t in GRADE]
    a.plot(gx, med, "o-", label="零振动弹道")
    a.plot([6.4], [np.median([r["rms_med"] for r in duty[VIB]])], "r*", ms=14,
           label="6.4 g + 1 g·80 Hz 振动")
    a.axhline(0.15, color="k", ls="--", lw=0.8)
    a.text(9.0, 0.157, "0.15 g 门限", fontsize=8)
    for xv, yv in zip(gx, med):
        a.annotate(f"{yv:.3f}", (xv, yv + 0.012), fontsize=8, ha="center")
    a.set_xlabel("弹道过载 [g]")
    a.set_ylabel("accel_rms_g 中位 [g]")
    a.set_title("(e) 机动把 RMS 推过门限，真振动却推不动")
    a.legend(fontsize=8)
    a.grid(alpha=0.3)

    # (f) CDF：τ̂ 分布
    a = ax[1, 2]
    for tag, c in [("6.4g_洁净", "#4c72b0"), ("6.4g_延100ms", "#dd8452"),
                   ("6.4g_延200ms", "#55a868")]:
        if tag not in duty:
            continue
        d = read_trace(tag, 1)
        v = d["ad_tau_ms"][(d["ad_valid"] > 0) & (d["t"] > 1.2)]
        if v.size == 0:
            continue
        a.plot(np.sort(v), np.arange(1, v.size + 1) / v.size, color=c,
               label=f"{tag}  n={v.size}")
    a.axvline(15, color="k", ls=":", lw=0.9)
    a.text(16, 0.05, "编译期先验 15 ms", fontsize=8)
    a.set_xlabel("估计延时 τ [ms]")
    a.set_ylabel("累计概率")
    a.set_title("(f) 估计延时分布：洁净档挤在先验附近（伪改善），200 ms 档饱和")
    a.legend(fontsize=8)
    a.grid(alpha=0.3)

    fig.suptitle("GNSS 延时/振动自适应链路的下游消费点审计（导弹 6DOF SIL，"
                 "二进制 831c89a9，core2，8759 帧/seed）", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(OUT, dpi=115)
    print("saved:", OUT)


if __name__ == "__main__":
    main()
