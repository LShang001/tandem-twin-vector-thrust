# -*- coding: utf-8 -*-
"""30.5 g 尾部 seed 失效时间线图（SIL trace CSV）。
@file    plot_tail_timeline.py
@brief   位置误差/垂速误差/高度三条时序 + 越门时刻标注。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 判读"速度残差是否先于位置残差越门"——决定速度子集软恢复通道能否触发。
@frame   NED
@units   m / m/s / s

用法：py -3.12 sim/plot_tail_timeline.py --tag 2026-10-27
"""
import argparse
import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DATA = os.path.join(ROOT, "sim-data")
FIGS = os.path.join(ROOT, "figs")


def setup_font():
    for name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC"):
        try:
            font_manager.findfont(name, fallback_to_default=False)
            plt.rcParams["font.family"] = name
            break
        except Exception:
            continue
    plt.rcParams["axes.unicode_minus"] = False


def load(path):
    with open(path, encoding="utf-8") as fh:
        header = fh.readline().strip().split(",")
        rows = [ln.strip().split(",") for ln in fh if ln.strip()]
    arr = np.array(rows, dtype=np.float64)
    return {n: arr[:, i] for i, n in enumerate(header)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="trace-sil32g-30g-seed21-2026-10-27.csv")
    ap.add_argument("--json", default="tail_timeline_2026-10-27.json")
    ap.add_argument("--tag", default="2026-10-27")
    args = ap.parse_args()
    setup_font()

    col = load(os.path.join(DATA, args.csv))
    tl = json.load(open(os.path.join(DATA, args.json), encoding="utf-8"))
    key = os.path.splitext(args.csv)[0]
    info = tl.get(key, {})

    t = col["t"]
    err_ne = np.hypot(col["ekf_n"] - col["true_n"], col["ekf_e"] - col["true_e"])
    vel_err_d = np.abs(col["ekf_vz"] - col["truth_vz"])

    fig, ax = plt.subplots(3, 1, figsize=(12.5, 9.5), sharex=True)
    fig.suptitle("30.5 g / GNSS 10 Hz 尾部 seed21 失效时间线（32 g 量程，SIL L2）",
                 fontsize=13, fontweight="bold")

    a0 = ax[0]
    a0.plot(t, err_ne, color="#111111", lw=1.1, label="水平位置误差 |EKF−真值|")
    a0.axhline(30.0, color="#c8792b", ls="--", lw=1.0,
               label="位置野值门 30 m")
    a0.axhline(10.0, color="#888888", ls=":", lw=1.0, label="10 m 参考")
    for k, c, lab in (("t_over_10m", "#2b7fc8", "越过 10 m"),
                      ("t_over_30m", "#c8792b", "越过 30 m 野值门")):
        v = info.get(k)
        if v:
            a0.axvline(v, color=c, alpha=0.5, lw=1.0)
            a0.annotate(f"{lab}\nt={v:.1f}s", (v, 60), fontsize=8, color=c,
                        xytext=(v + 0.6, 78))
    a0.set_ylabel("位置误差 (m)")
    a0.set_ylim(0, min(160, err_ne.max() * 1.1))
    a0.legend(fontsize=8, ncol=2)
    a0.grid(alpha=0.3)

    a1 = ax[1]
    a1.plot(t, vel_err_d, color="#7b3fa0", lw=1.1, label="垂速误差 |EKF−真值|")
    a1.axhline(10.0, color="#c8792b", ls="--", lw=1.0, label="速度野值门 10 m/s")
    v = info.get("t_over_vel10")
    if v:
        a1.axvline(v, color="#2b7fc8", alpha=0.6, lw=1.0)
        a1.annotate(f"速度先越门\nt={v:.1f}s", (v, 8), fontsize=8, color="#2b7fc8",
                    xytext=(v + 0.8, 9))
    a1.set_ylabel("垂速误差 (m/s)")
    a1.legend(fontsize=8)
    a1.grid(alpha=0.3)

    a2 = ax[2]
    a2.plot(t, col["truth_alt"], color="#111111", lw=1.0, label="真值高度")
    a2.plot(t, col["ctl_alt"], color="#2b7fc8", lw=1.0, ls="-",
            label="EKF/控制高度")
    a2.plot(t, col["target_alt"], color="#888888", lw=0.9, ls=":", label="目标高度")
    a2.set_ylabel("高度 (m)")
    a2.set_xlabel("时间 (s)")
    a2.legend(fontsize=8)
    a2.grid(alpha=0.3)

    note = (f"峰值位置误差 {info.get('err_ne_max', float('nan')):.1f} m，"
            f"末端 {info.get('err_ne_final', float('nan')):.2f} m；"
            f"越门前增长斜率 {info.get('growth_mps_to_30m', float('nan')):.1f} m/s；"
            f"垂速误差峰值 {info.get('vel_err_d_max', float('nan')):.1f} m/s"
            "（同 seed SIL JSON：整帧拒 20 次、重锚 1 次）")
    fig.text(0.5, 0.005, note, ha="center", fontsize=9, color="#444444")
    fig.tight_layout(rect=(0, 0.02, 1, 0.96))
    out = os.path.join(FIGS, f"nav_tail_timeline_{args.tag}.png")
    fig.savefig(out, dpi=150)
    print("saved:", out)


if __name__ == "__main__":
    main()
