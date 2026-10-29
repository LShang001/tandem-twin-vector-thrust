#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""越门锁死注入矩阵作图（§4.24bg）。

@file    plot_gate_lockout_inject.py
@brief   A) 代表性 seed 的 vz 误差时序（4 档偏置）；B) 整段 pos_rms 对"状态级剂量"
         （注入结束时的 |vz 误差| − 同 seed 基线）散点，标出 8 m/s 物理门。
@module  TandemVec-EKF-Report/仿真作图
@details 输入 `sim-data/gate_lockout_inject_<tag>.json` 与同目录 `gate_lockout_<tag>/trace_*.csv`；
         输出 `figs/gate_lockout_inject_<tag>.png`。不重算任何指标，只画已落盘结果。
@frame   N/A（速度误差标量）
@units   时间 s、速度 m/s、位置 m
@note    剂量按"同 seed 基线差值"定义（§4.24bg：0.3 s 偏置有 55–75% 被零偏态吸收，
         名义剂量不是自变量）。
@see     sim/run_gate_lockout_inject.py、导弹6DOF-导航SIL-2026-10-27.md §4.24bg
"""
import argparse
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
FIG = os.path.abspath(os.path.join(HERE, "..", "figs"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="2026-10-29")
    ap.add_argument("--seed", type=int, default=2)
    args = ap.parse_args()
    d = json.load(open(os.path.join(SD, f"gate_lockout_inject_{args.tag}.json"),
                       encoding="utf-8"))
    tdir = os.path.join(SD, f"gate_lockout_{args.tag}_traces")
    biases = d["bias_mps2"]
    seeds = d["seeds"]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.5, 4.6))

    # A) 时序：代表 seed，每档一条
    for b in biases:
        f = os.path.join(tdir, f"trace_s{args.seed}_b{int(b)}.csv")
        t = pd.read_csv(f).set_index("t")
        vz = t.ekf_vz - t.truth_vz
        m = (t.index >= 14.0) & (t.index <= 24.0)
        ax1.plot(t.index[m], vz[m], lw=1.6,
                 label=f"bias {b:.0f} m/s² (nominal {b*0.3:.0f} m/s)")
    ax1.axvspan(15.0, 15.3, color="0.85", zorder=0, label="injection window")
    for y in (8.0, -8.0):
        ax1.axhline(y, color="0.4", ls="--", lw=0.9)
    ax1.set_xlabel("t [s]")
    ax1.set_ylabel(r"$\hat v_z - v_z$ [m/s]")
    ax1.set_title(f"A) vertical velocity error, seed {args.seed}")
    ax1.legend(fontsize=8)
    ax1.grid(alpha=0.3)

    # B) 剂量-误差散点：状态级剂量 = |vz@inj_end| − 同 seed b0 的 |vz@inj_end|
    base = {r["seed"]: abs(r["vz_err_at_inj_end"]) for r in d["rows"] if r["bias"] == 0}
    marks = {1: "o", 2: "s", 3: "^", 4: "D", 5: "v"}
    for s in seeds:
        xs, ys = [], []
        for r in d["rows"]:
            if r["seed"] != s or r["bias"] == 0:
                continue
            xs.append(abs(r["vz_err_at_inj_end"]) - base[s])
            ys.append(r["pos_rms"])
        ax2.scatter(xs, ys, marker=marks.get(s, "o"), s=48, label=f"seed {s}")
        for x, y, r in zip(xs, ys, [r for r in d["rows"]
                                    if r["seed"] == s and r["bias"] != 0]):
            ax2.annotate(f"{r['bias']:.0f}", (x, y), fontsize=7,
                         textcoords="offset points", xytext=(3, 3))
    ax2.axvline(8.0, color="0.4", ls="--", lw=0.9)
    ax2.set_xlabel("state-level dose at injection end [m/s] (baseline-subtracted)")
    ax2.set_ylabel("pos_rms over run [m]")
    ax2.set_title("B) dose vs position error (labels = nominal m/s²)")
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.3)

    fig.tight_layout()
    os.makedirs(FIG, exist_ok=True)
    out = os.path.join(FIG, f"gate_lockout_inject_{args.tag}.png")
    fig.savefig(out, dpi=130)
    print("saved", out)


if __name__ == "__main__":
    main()
