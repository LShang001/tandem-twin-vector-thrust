# -*- coding: utf-8 -*-
"""run_vmax_fault_ab.py — 高速度点上 nav-raw 的门槛收益是否被故障吃掉？

@file    run_vmax_fault_ab.py
@module  analysis/导弹导航 SIL 大规模矩阵（L2）
@brief   固定 24600 N（v_max≈1724 m/s、g_max≈32.3）、10 Hz、seeds 101–130 的子集，
         在 4 个故障格（无 / 振 1 g@80 Hz / 失联 5 s / 叠加）上比较三臂：
           A 默认；B `--nav-raw-imu`；C B + 健康键源头回退。
@details 背景：§4.24bd 已证明 nav-raw 把 30 g 段失效门槛从 1686.9 推到 ≥1724 m/s
         （6 点 0/30）；§4.24h/bb 在 30.06/30.57 g 上证明故障格不新增代价。
         本轮回答"门槛收益在故障叠加的高速度点是否仍然成立"，并记录健康键回退
         在振动格是否被触发（`nrfb`/`imu_detected`）。

预登记判据：
  F1 对照格可复现：A 在"无故障"格失效应在 40%–80%（§4.24bd 的 18/30=60%）。
  F2 振动格：B 失效率 ≤ A，且 B ≤2/20。
  F3 失联格：B 不得比 A 多失效 >2 个 seed（失联是独立失效路径，不承诺修好）。
  F4 叠加格：B 失效率 ≤ A。
  F5 回退惰性：振动格若 `imu_state` 未进入 Failed，C 必须与 B 逐位相同；若触发，
     记录 `nrfb` 帧数并单独报告（不得当作默认惰性）。
边界：L2 SIL；20 seeds/格；单一弹道（10 s 燃烧、24600 N，g_max 32.34 略超 32 g 满量程）；
     不写实机。
"""
from __future__ import annotations

import argparse
import concurrent.futures as cf
import datetime as _dt
import json
import os
import statistics as st
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from sil_matrix import exe_in_temp, md5_of, run_case, save_jsonl

plt.rcParams["font.family"] = "sans-serif"
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

DATA = os.path.normpath(os.path.join(HERE, "..", "sim-data"))
FIGS = os.path.normpath(os.path.join(HERE, "..", "figs"))

BASE = ["--real-nav", "--missile", "--seconds", "45",
        "--imu-fs-g", "32", "--imu-health-accel-range", "313.8",
        "--profile", "mag_clean", "--imu-grade", "icm",
        "--gnss-rate-hz", "10", "--ms-burn", "10", "--ms-thrust", "24600"]
ARMS = [
    ("A_default", []),
    ("B_navraw", ["--nav-raw-imu"]),
    ("C_navraw_fb", ["--nav-raw-imu", "--nav-raw-fault-fallback",
                     "--nav-raw-fb-health"]),
]
FAULTS = [
    ("none", []),
    ("vib1g80", ["--vib", "1.0", "--vib-hz", "80", "--vib-start", "3"]),
    ("out5", ["--gnss-outage", "20:25"]),
    ("vibout", ["--vib", "1.0", "--vib-hz", "80", "--vib-start", "3",
                "--gnss-outage", "20:25"]),
]
SEED_START = 101
SEEDS = 20
CORES = [2, 3, 5, 6, 7, 8, 9, 10, 11, 12]
FAIL_M = 25.0


def run_seed(exe: str, seed: int, core: int) -> list[dict]:
    rows = []
    for cell, fargs in FAULTS:
        for arm, extra in ARMS:
            rec = run_case(exe, BASE + fargs + extra, seed, core)
            rec["arm"] = arm
            rec["cell"] = cell
            rows.append(rec)
    return rows


def summarize(rows):
    out = {}
    for cell, _ in FAULTS:
        out[cell] = {}
        for arm, _ in ARMS:
            sub = [r for r in rows if r["cell"] == cell and r["arm"] == arm]
            p = np.array([r["pos_rms"] for r in sub], dtype=float)
            bad = p > FAIL_M
            out[cell][arm] = {
                "n": int(p.size), "n_fail": int(bad.sum()),
                "rate": round(float(bad.mean()), 3),
                "med": round(float(np.median(p)), 3),
                "p90": round(float(np.percentile(p, 90)), 3),
                "max": round(float(p.max()), 3),
                "att_med": round(float(np.median([r["nav_att_rmse"] for r in sub])), 3),
                "rej_med": float(np.median([r["outlier_rejects"] for r in sub])),
                "reanchor_seeds": int(sum(1 for r in sub if r["reacquire_resets"] > 0)),
                "imu_det_seeds": int(sum(1 for r in sub if r["imu_detected"])),
                "nrfb_max": int(max(r.get("nrfb", 0) for r in sub)),
            }
    return out


def paired(rows, cell, arm_a, arm_b):
    a = {r["seed"]: r for r in rows if r["cell"] == cell and r["arm"] == arm_a}
    b = {r["seed"]: r for r in rows if r["cell"] == cell and r["arm"] == arm_b}
    better = worse = same = 0
    worst = 0.0
    for s in sorted(set(a) & set(b)):
        d = b[s]["pos_rms"] - a[s]["pos_rms"]
        if abs(d) <= 1e-6:
            same += 1
        elif d < 0:
            better += 1
        else:
            worse += 1
            worst = max(worst, d)
    return {"better": better, "worse": worse, "same": same,
            "worst_degradation_m": round(worst, 4)}


def plot(summary, rows, out_png):
    cells = [c for c, _ in FAULTS]
    labels = ["无故障", "振 1g@80", "失联 5 s", "振动+失联"]
    arms = [("A_default", "默认"), ("B_navraw", "nav-raw"),
            ("C_navraw_fb", "nav-raw+回退")]
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.2))

    ax = axes[0]
    idx = np.arange(len(cells))
    w = 0.26
    for k, (arm, lab) in enumerate(arms):
        ax.bar(idx + (k - 1) * w, [summary[c][arm]["rate"] * 100 for c in cells],
               w, label=lab)
    ax.set_xticks(idx)
    ax.set_xticklabels(labels)
    ax.set_ylabel("失效率 [%] (pos_rms>25 m)")
    ax.set_title("24600 N：故障格失效率")
    ax.grid(alpha=0.3, axis="y")
    ax.legend()

    ax = axes[1]
    for cell, lab in zip(cells, labels):
        for arm, ls in (("A_default", "--"), ("B_navraw", "-")):
            p = sorted(r["pos_rms"] for r in rows if r["cell"] == cell and r["arm"] == arm)
            ax.plot(p, np.linspace(0, 1, len(p)), ls,
                    label=f"{lab}/{'默认' if arm == 'A_default' else 'nav-raw'}")
    ax.set_xscale("symlog", linthresh=1.0)
    ax.set_xlabel("pos_rms [m]")
    ax.set_ylabel("CDF")
    ax.set_title("pos_rms 分布")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7)

    ax = axes[2]
    for k, (arm, lab) in enumerate(arms):
        ax.bar(idx + (k - 1) * w, [summary[c][arm]["att_med"] for c in cells],
               w, label=lab)
    ax.set_xticks(idx)
    ax.set_xticklabels(labels)
    ax.set_ylabel("姿态 RMSE 中位 [deg]")
    ax.set_title("姿态")
    ax.grid(alpha=0.3, axis="y")
    ax.legend()

    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", default=exe_in_temp(os.path.join("tv_vmax", "fw_real.exe")))
    ap.add_argument("--seeds", type=int, default=SEEDS)
    ap.add_argument("--seed-start", type=int, default=SEED_START)
    ap.add_argument("--workers", type=int, default=10)
    ap.add_argument("--tag", default="vmax_fault_" + _dt.date.today().isoformat())
    args = ap.parse_args()

    if not os.path.exists(args.exe):
        raise SystemExit(f"未找到 SIL 二进制：{args.exe}")
    print("exe:", args.exe, "md5:", md5_of(args.exe))

    seeds = list(range(args.seed_start, args.seed_start + args.seeds))
    rows = []
    with cf.ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = {pool.submit(run_seed, args.exe, s, CORES[i % len(CORES)]): s
                   for i, s in enumerate(seeds)}
        for fut in cf.as_completed(futures):
            rows.extend(fut.result())

    os.makedirs(DATA, exist_ok=True)
    save_jsonl(os.path.join(DATA, f"{args.tag}.jsonl"), rows)

    summary = summarize(rows)
    pair_ab = {c: paired(rows, c, "A_default", "B_navraw") for c, _ in FAULTS}
    pair_cb = {c: paired(rows, c, "B_navraw", "C_navraw_fb") for c, _ in FAULTS}

    f1 = 0.40 <= summary["none"]["A_default"]["rate"] <= 0.80
    f2 = (summary["vib1g80"]["B_navraw"]["rate"] <= summary["vib1g80"]["A_default"]["rate"]
          and summary["vib1g80"]["B_navraw"]["n_fail"] <= 2)
    f3 = (summary["out5"]["B_navraw"]["n_fail"]
          <= summary["out5"]["A_default"]["n_fail"] + 2)
    f4 = summary["vibout"]["B_navraw"]["rate"] <= summary["vibout"]["A_default"]["rate"]
    # F5：振动格若健康态未报 Failed，C 必须与 B 相同；触发则转人工判读 nrfb。
    vib_cb = pair_cb["vib1g80"]
    vib_same = vib_cb["better"] == 0 and vib_cb["worse"] == 0
    f5 = ("same" if (summary["vib1g80"]["B_navraw"]["imu_det_seeds"] == 0 and vib_same)
          else "check")

    verdict = {
        "date": _dt.date.today().isoformat(),
        "exe": args.exe, "exe_md5": md5_of(args.exe),
        "base": BASE, "faults": {k: v for k, v in FAULTS},
        "arms": {k: v for k, v in ARMS},
        "seeds": [seeds[0], seeds[-1]],
        "fail_threshold_m": FAIL_M,
        "summary": summary,
        "paired_A_vs_B": pair_ab,
        "paired_B_vs_C": pair_cb,
        "criteria": {"F1_control": bool(f1), "F2_vib": bool(f2),
                     "F3_outage": bool(f3), "F4_vib_out": bool(f4),
                     "F5_fallback": f5},
        "note": "L2 SIL；24600 N 的 g_max 32.34 略超 32 g 满量程；20 seeds/格。",
    }
    with open(os.path.join(DATA, f"{args.tag}_verdict.json"), "w", encoding="utf-8") as f:
        json.dump(verdict, f, ensure_ascii=False, indent=1)
    os.makedirs(FIGS, exist_ok=True)
    plot(summary, rows, os.path.join(FIGS, f"{args.tag}.png"))

    print(json.dumps(verdict["criteria"], ensure_ascii=False))
    for c, _ in FAULTS:
        print(c, [(a, summary[c][a]["n_fail"], summary[c][a]["rate"],
                   summary[c][a]["med"], summary[c][a]["nrfb_max"]) for a, _ in ARMS])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
