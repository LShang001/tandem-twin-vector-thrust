#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24an 配图：10–20 s 窗的速度通道证据 + 10 Hz 子集 A/B。

@file    plot_missile_epoch_channels.py
@brief   4 面板：(a) 速度残差历元级时间线（失效 vs 通过，10 m/s 门线）；
          (b) 各窗越门历元数中位（速度门/位置门 × 两簇）；
          (c) 子集关/开逐 seed 位置 RMS 配对散点；(d) 两臂位置 RMS 的 CDF
@module  TandemVec-EKF-Report/绘图（L2 研究设施）
@note    图内文字中文；生成后逐面板目视审查（空图/尺度/图例遮挡）。
"""
import argparse
import csv
import glob
import json
import os
import re

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams["font.family"] = "sans-serif"
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
FIG = os.path.abspath(os.path.join(HERE, "..", "figs"))
WINDOWS = ((0, 10), (10, 20), (20, 30), (30, 45))


def load_epochs(path):
    with open(path, newline="") as f:
        r = csv.reader(f)
        h = next(r)
        ix = {k: i for i, k in enumerate(h)}
        rows = list(r)
    t = np.array([float(x[ix["t"]]) for x in rows])
    msr = np.array([[float(x[ix["msr_n"]]), float(x[ix["msr_e"]])]
                    for x in rows])
    cols = {k: np.array([float(x[ix[k]]) for x in rows])
            for k in ("gate_vel", "gate_pos", "gnss_resid_vel_ne",
                      "gnss_resid_pos_ne")}
    ch = np.where(np.any(np.diff(msr, axis=0) != 0, axis=1))[0] + 1
    return t[ch], {k: v[ch] for k, v in cols.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trace-dir",
                    default=os.path.join(os.environ.get("TEMP", "/tmp"),
                                         "phase_tr"))
    ap.add_argument("--off", default=os.path.join(
        SD, "attrib305_2026-10-28_an_hz10_off2.jsonl"))
    ap.add_argument("--on", default=os.path.join(
        SD, "attrib305_2026-10-28_an_hz10_on2.jsonl"))
    ap.add_argument("--level", type=float, default=None,
                    help="只画该推力档（缺省：文件内全部档位）")
    ap.add_argument("--out", default=os.path.join(
        FIG, "nav_missile_epoch_channels_2026-10-28.png"))
    args = ap.parse_args()

    def load_arm(path):
        d = {}
        for line in open(path, encoding="utf-8"):
            r = json.loads(line)
            d[(r["thrust_n"], r["seed"])] = r
        return d

    off, on = load_arm(args.off), load_arm(args.on)
    keys = sorted(set(off) & set(on))
    if args.level:
        keys = [k for k in keys if abs(k[0] - args.level) < 1.0]

    fig, axes = plt.subplots(2, 2, figsize=(13.5, 8.6))

    ax = axes[0][0]
    seeds_f, seeds_p = (59, 63, 73), (57, 76)
    for s in seeds_f:
        p = os.path.join(args.trace_dir, "s%d.csv" % s)
        if not os.path.exists(p):
            continue
        t, c = load_epochs(p)
        ax.plot(t, c["gnss_resid_vel_ne"], color="#D62728", lw=1.0,
                label="失效 s%d" % s)
    for s in seeds_p:
        p = os.path.join(args.trace_dir, "s%d.csv" % s)
        if not os.path.exists(p):
            continue
        t, c = load_epochs(p)
        ax.plot(t, c["gnss_resid_vel_ne"], color="#4C78A8", lw=1.0,
                label="通过 s%d" % s)
    ax.axhline(10.0, color="#333", ls="--", lw=1.0,
               label="速度物理门 10 m/s")
    for b in (10, 20, 30):
        ax.axvline(b, color="#999", ls=":", lw=0.9)
    ax.set_xlabel("t (s)")
    ax.set_ylabel("GNSS−INS 水平速度残差 (m/s)")
    ax.set_title("(a) 历元级速度残差：失效 seed 在 10–20 s 整窗偏高")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[0][1]
    chk = json.load(open(os.path.join(
        SD, "missile_epoch_channels_2026-10-28_an.json"), encoding="utf-8"))
    labels = [("%d–%d" % w) for w in WINDOWS]
    fv = [chk["summary"]["%d-%d" % w]["fail_gate_vel_med"] for w in WINDOWS]
    pv = [chk["summary"]["%d-%d" % w]["pass_gate_vel_med"] for w in WINDOWS]
    fp = [chk["summary"]["%d-%d" % w]["fail_gate_pos_med"] for w in WINDOWS]
    pp = [chk["summary"]["%d-%d" % w]["pass_gate_pos_med"] for w in WINDOWS]
    x = np.arange(len(WINDOWS))
    w = 0.2
    ax.bar(x - 1.5 * w, fv, w, color="#D62728", label="失效·速度门")
    ax.bar(x - 0.5 * w, fp, w, color="#F2A0A0", label="失效·位置门")
    ax.bar(x + 0.5 * w, pv, w, color="#4C78A8", label="通过·速度门")
    ax.bar(x + 1.5 * w, pp, w, color="#A8C4E0", label="通过·位置门")
    ax.set_xticks(x)
    ax.set_xticklabels([l + " s" for l in labels])
    ax.set_ylabel("越门历元数（中位，每窗 100–150 历元）")
    ax.set_title("(b) 越门事件按窗：速度门在 10–20 s 先起")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3, axis="y")

    ax = axes[1][0]
    xs = [off[k]["pos_rms"] for k in keys]
    ys = [on[k]["pos_rms"] for k in keys]
    ax.scatter(xs, ys, s=26, color="#555", alpha=0.75)
    lim = [0, max(max(xs), max(ys)) * 1.05]
    ax.plot(lim, lim, "--", color="#999", lw=1.0)
    ax.axhline(25, color="#D62728", ls=":", lw=1.0, label="25 m 失效线")
    ax.axvline(25, color="#D62728", ls=":", lw=1.0)
    ax.set_xlim(lim)
    ax.set_ylim(lim)
    ax.set_xlabel("子集关 pos_rms (m)")
    ax.set_ylabel("子集开 pos_rms (m)")
    ax.set_title("(c) 10 Hz 逐 seed 配对（%d 对：%d 变好 / 0 变差）"
                 % (len(keys), sum(1 for a, b in zip(xs, ys) if b < a - 0.05)))
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[1][1]
    for lab, v, c in (("子集关", xs, "#D62728"), ("子集开", ys, "#4C78A8")):
        v = np.sort(np.array(v))
        ax.step(v, np.arange(1, len(v) + 1) / len(v), where="post",
                color=c, lw=1.6, label=lab)
    ax.axvline(25, color="#333", ls="--", lw=1.0, label="25 m")
    ax.set_xlabel("pos_rms (m)")
    ax.set_ylabel("累积概率")
    ax.set_title("(d) 位置 RMS 的 CDF（10 Hz，同一 seed 集合）")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    fig.suptitle("30.06 g / 10 Hz：速度通道是失效入口，速度子集把 13/50 修到 0/50", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    os.makedirs(FIG, exist_ok=True)
    fig.savefig(args.out, dpi=135)
    print("FIG:", args.out)


if __name__ == "__main__":
    main()
