#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""固件控制链对拍作图（SIL vs fwapi.Gnc）。

@file    plot_fw_gnc_parity.py
@brief   A) 期望天向比力 sfup 时序（SIL vs 直调库）与差值；B) 各通道最大偏差条形图。
@module  TandemVec-EKF-Report/仿真作图
@details 输入 SIL `--trace-ctrl` 轨迹，用 fwapi.Gnc 重放并与落盘输出比较。
@frame   NED / FRD
@units   时间 s、比力 m/s²、执行器 %、摆角 deg、角加速度 dps²
@note    只画已能复现的量；残差结论引用 check_fw_gnc_parity.py 的预登记判据。
@see     sim/check_fw_gnc_parity.py、docs/05-控制与分配/CTL-004-固件仿真一体化.md
"""
import argparse
import math
import pathlib
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import fwapi  # noqa: E402


def replay(trace):
    rows = list(pd.read_csv(trace).to_dict("records"))
    g = fwapi.Gnc()
    rec = []
    prev = None
    for r in rows:
        t = float(r["t"])
        dt = 0.005 if prev is None else round(t - prev, 6)
        if dt <= 0:
            dt = 0.005
        prev = t
        o = g.step(
            quat=(r["quat_w"], r["quat_x"], r["quat_y"], r["quat_z"]),
            omega=(r["gx_rps"], r["gy_rps"], r["gz_rps"]),
            vel=(r["vn"], r["ve"], r["vd"]),
            pos=(r["rn"], r["re"], r["rd"]),
            alt_up=r["alt_up"],
            rc=[int(r[f"rc{i}"]) for i in range(9)] + [988] * 7,
            dt=dt,
            link_up=int(r["link"]) != 0,
            failsafe_in_flight=int(r["fs"]) != 0,
            target_alt=float(r["target_alt"]),
            heading_hold_allowed=int(r["hdg_ok"]) != 0,
            pos_hold_degraded=int(r["pos_deg"]) != 0,
            gnss_outage_age_s=float(r["gnss_age"]),
        )
        rec.append((t, o, r))
    g.close()
    return rec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trace", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--title", default="fw_gnc parity")
    args = ap.parse_args()

    rec = replay(args.trace)
    t = np.array([x[0] for x in rec])
    sfup_dll = np.array([x[1]["specific_force_up_mps2"] for x in rec])
    sfup_sil = np.array([float(x[2]["sfup"]) for x in rec])

    keys = {
        "ch1": "servo1 [%]", "ch2": "servo2 [%]", "ch3": "motor1 [%]", "ch4": "motor2 [%]",
        "delta_f": "upper [deg]", "delta_t": "lower [deg]", "dw": "diff [-]",
        "or0": "w_ref roll [dps]", "or1": "w_ref pitch [dps]",
        "or2": "w_ref yaw [dps]", "ar0": "a_ref roll [dps2]",
        "ar1": "a_ref pitch [dps2]", "ar2": "a_ref yaw [dps2]",
    }
    maxd = {}
    for k in keys:
        vals = []
        for _, o, r in rec:
            if k.startswith("ch"):
                a = o[f"ch{k[-1]}_pct"]
                b = float(r[k])
            elif k.startswith("or"):
                a = o["omega_ref_dps"][int(k[-1])]
                b = float(r[k])
            elif k.startswith("ar"):
                a = o["alpha_ref_dps2"][int(k[-1])]
                b = float(r[k])
            elif k in ("delta_f", "delta_t"):
                a = o[k + "_deg"]
                b = float(r[k])
            else:
                a = o[k]
                b = float(r[k])
            vals.append(abs(a - b))
        maxd[k] = max(vals) if vals else 0.0

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.6, 4.6))
    ax1.plot(t, sfup_sil, lw=2.2, label="SIL (firmware + plant)")
    ax1.plot(t, sfup_dll, lw=1.2, ls="--", label="fwapi.Gnc (shared lib)")
    ax1.set_xlabel("t [s]")
    ax1.set_ylabel("specific force up [m/s²]")
    ax1.set_title("A) vertical thrust command: SIL vs direct-call")
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=8)
    ax1b = ax1.twinx()
    ax1b.plot(t, sfup_dll - sfup_sil, color="tab:red", lw=0.9, alpha=0.6)
    ax1b.set_ylabel("Δ (DLL − SIL) [m/s²]", color="tab:red")
    ax1b.tick_params(axis="y", colors="tab:red")

    xs = list(maxd.keys())
    ys = [maxd[k] for k in xs]
    ax2.bar(range(len(xs)), np.maximum(ys, 1e-7), color="tab:blue", alpha=0.8)
    ax2.set_yscale("log")
    ax2.set_xticks(range(len(xs)))
    ax2.set_xticklabels([keys[k] for k in xs], rotation=60, ha="right", fontsize=7)
    ax2.set_ylabel("max |Δ| (log)")
    ax2.set_title("B) worst-channel deviation over run")
    ax2.grid(alpha=0.3, axis="y")
    for i, v in enumerate(ys):
        ax2.text(i, max(v, 1e-7) * 1.4, f"{v:.1e}", ha="center", fontsize=6)

    fig.suptitle(args.title)
    fig.tight_layout()
    fig.savefig(args.out, dpi=130)
    print("saved", args.out, "max sfup diff", round(max(abs(sfup_dll - sfup_sil)), 4))


if __name__ == "__main__":
    main()
