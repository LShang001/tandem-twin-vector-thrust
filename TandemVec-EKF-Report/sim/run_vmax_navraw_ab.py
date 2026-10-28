# -*- coding: utf-8 -*-
"""run_vmax_navraw_ab.py — nav-raw 能否把 30 g 段的失效门槛（v_max）显著推高？

@file    run_vmax_navraw_ab.py
@module  analysis/导弹导航 SIL 大规模矩阵（L2）
@brief   32 g 生产二进制、10 Hz GNSS、10 s 燃烧，扫描 5 个推力点
         （v_max≈1620–1700 m/s），比较三臂：
           A 默认生产路径；
           B 导航增量绕过加计 LPF（`--nav-raw-imu`）；
           C B + IMU 健康键源头回退（推荐候选，无故障时应与 B 完全相同）。
@details 背景：§4.24ap/at/av 已证明 30 g 段失效入口是"时间对齐误差 × 速度"
         （v·δt 越过 30 m 位置门 → 整帧拒绝 → 双稳锁死），临界 v_max≈1650 m/s；
         §4.24an/ba 证明 nav-raw 在 30.06/30.57 g 单点上 13/50→0/50。本轮回答
         一个不同的问题：**nav-raw 是把整条门槛推高了，还是只在单点碰巧过线？**
         ——若只是单点，换速度剖面就会复现失效；若门槛被推高，则是结构性收益。

预登记判据（跑前写死，跑后不改口径）：
  P1 构建锚：新二进制跑 seeds 51–60 @23900 默认臂，`pos_rms` 必须与归档
     `attrib305_2026-10-28_ab_hz10.jsonl` 同 seed 值**逐位相同**（容差 1e-6）。
     不成立 ⇒ 构建/默认行为变了，本轮跨臂结论仍需配对有效，但必须显式标注。
  P2 单点收益：23900 档 B 的失效率（pos_rms>25 m）相对 A 下降 ≥50% 且至少少
     5 个 seed；逐 seed 配对"变差"≤1 个且幅度 <2 m。
  P3 门槛推移：门槛 = 失效率 ≥25% 的最低推力点对应的实测 v_max 中位。
     B 的门槛必须比 A 高至少一个台阶（≥1 个 ladder step），或 B 在整条扫描
     范围内都 <25%（门槛被推到扫描上界之外，需显式标注"未定位"）才可写"推高"；
     否则只能写"单点改善、门槛未移动"。
  P4 回退惰性：无故障场景下 C 必须与 B 逐位相同（pos_rms 差 ≤1e-6 且 nrfb=0）。
     不成立 ⇒ 健康键回退误触发，候选配置不可用。
  P5 姿态代价：任一点 B 的 nav_att_rmse 中位不得比 A 高 >10%。
  P6 尾部：B 全矩阵 max(pos_max) 不得比 A 的 max 高 >10%。
边界：L2 SIL、单一弹道族（10 s 燃烧）、seeds 101–130（独立于归档 51–100）、
     无故障注入（故障格见 §4.24h/bb）；不写实机。
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
ARCHIVE = os.path.join(DATA, "attrib305_2026-10-28_ab_hz10.jsonl")

BASE = ["--real-nav", "--missile", "--seconds", "45",
        "--imu-fs-g", "32", "--imu-health-accel-range", "313.8",
        "--profile", "mag_clean", "--imu-grade", "icm",
        "--gnss-rate-hz", "10", "--ms-burn", "10"]
LADDER = [int(x) for x in os.environ.get(
    "VM_LADDER", "22650,23000,23500,23900,24300").split(",")]
ARMS = [
    ("A_default", []),
    ("B_navraw", ["--nav-raw-imu"]),
    ("C_navraw_fb", ["--nav-raw-imu", "--nav-raw-fault-fallback",
                     "--nav-raw-fb-health"]),
]
SEED_START = 101
SEEDS = 30
CORES = [2, 3, 5, 6, 7, 8, 9, 10, 11, 12]
FAIL_M = 25.0


def run_seed(exe: str, seed: int, core: int) -> list[dict]:
    rows = []
    for thr in LADDER:
        for arm, extra in ARMS:
            args = BASE + ["--ms-thrust", str(thr)] + extra
            rec = run_case(exe, args, seed, core)
            rec["arm"] = arm
            rec["thrust_n"] = thr
            rows.append(rec)
    return rows


def summarize(rows):
    out = {}
    for arm, _ in ARMS:
        out[arm] = {}
        for thr in LADDER:
            sub = [r for r in rows if r["arm"] == arm and r["thrust_n"] == thr]
            p = np.array([r["pos_rms"] for r in sub], dtype=float)
            bad = p > FAIL_M
            good = ~bad
            out[arm][thr] = {
                "n": int(p.size),
                "n_fail": int(bad.sum()),
                "rate": round(float(bad.mean()), 3),
                "vmax_med": round(float(np.median([r["ms_vmax"] for r in sub])), 1),
                "gmax_med": round(float(np.median([r["ms_gmax"] for r in sub])), 2),
                "good_med": round(float(np.median(p[good])), 3) if good.any() else None,
                "bad_med": round(float(np.median(p[bad])), 3) if bad.any() else None,
                "p90": round(float(np.percentile(p, 90)), 3),
                "max": round(float(p.max()), 3),
                "att_med": round(float(np.median([r["nav_att_rmse"] for r in sub])), 3),
                "rej_med": float(np.median([r["outlier_rejects"] for r in sub])),
                "reanchor_seeds": int(sum(1 for r in sub if r["reacquire_resets"] > 0)),
                "nrfb_max": int(max(r.get("nrfb", 0) for r in sub)),
            }
    return out


def threshold(arm_summary):
    for thr in LADDER:
        s = arm_summary[thr]
        if s["rate"] >= 0.25:
            return thr, s["vmax_med"]
    return None, None


def paired(rows, arm_a: str, arm_b: str):
    out = {}
    for thr in LADDER:
        a = {r["seed"]: r for r in rows if r["arm"] == arm_a and r["thrust_n"] == thr}
        b = {r["seed"]: r for r in rows if r["arm"] == arm_b and r["thrust_n"] == thr}
        seeds = sorted(set(a) & set(b))
        better = worse = same = 0
        worst = 0.0
        for s in seeds:
            d = b[s]["pos_rms"] - a[s]["pos_rms"]
            if abs(d) <= 1e-6:
                same += 1
            elif d < 0:
                better += 1
            else:
                worse += 1
                worst = max(worst, d)
        out[thr] = {"better": better, "worse": worse, "same": same,
                    "worst_degradation_m": round(worst, 4)}
    return out


def plot(summary, paired_ba, out_png):
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.3))
    xs = [summary["A_default"][t]["vmax_med"] for t in LADDER]

    ax = axes[0]
    for arm, label in (("A_default", "默认"), ("B_navraw", "nav-raw"),
                       ("C_navraw_fb", "nav-raw+回退")):
        ys = [summary[arm][t]["rate"] * 100 for t in LADDER]
        ax.plot(xs, ys, "o-", label=label)
    ax.axhline(25, color="grey", ls="--", lw=1, label="25% 门槛判据")
    ax.set_xlabel("实测 v_max 中位 [m/s]")
    ax.set_ylabel("失效率 [%] (pos_rms>25 m)")
    ax.set_title("失效门槛：三臂对比")
    ax.grid(alpha=0.3)
    ax.legend()

    ax = axes[1]
    for arm, label in (("A_default", "默认"), ("B_navraw", "nav-raw")):
        vals = sorted(r["pos_rms"] for r in summary["_raw"]
                      if r["arm"] == arm)
        ax.plot(vals, np.linspace(0, 1, len(vals)), label=label)
    ax.set_xscale("symlog", linthresh=1.0)
    ax.set_xlabel("pos_rms [m]")
    ax.set_ylabel("CDF")
    ax.set_title("全矩阵 pos_rms 分布")
    ax.grid(alpha=0.3)
    ax.legend()

    ax = axes[2]
    width = 0.25
    idx = np.arange(len(LADDER))
    for k, (arm, label) in enumerate((("A_default", "默认"), ("B_navraw", "nav-raw"),
                                      ("C_navraw_fb", "nav-raw+回退"))):
        att = [summary[arm][t]["att_med"] for t in LADDER]
        ax.bar(idx + (k - 1) * width, att, width, label=label)
    ax.set_xticks(idx)
    ax.set_xticklabels([f"{t/1000:.1f} kN" for t in LADDER])
    ax.set_ylabel("姿态 RMSE 中位 [deg]")
    ax.set_title("姿态代价")
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
    ap.add_argument("--tag", default="vmax_navraw_" + _dt.date.today().isoformat())
    ap.add_argument("--anchor", action="store_true",
                    help="只跑 seeds 51-60 @23900 默认臂，与归档逐位比对后退出")
    args = ap.parse_args()

    if not os.path.exists(args.exe):
        raise SystemExit(f"未找到 SIL 二进制：{args.exe}")
    print("exe:", args.exe, "md5:", md5_of(args.exe))

    if args.anchor:
        arch = {}
        with open(ARCHIVE, encoding="utf-8") as f:
            for line in f:
                r = json.loads(line)
                if r["thrust_n"] == 23900:
                    arch[r["seed"]] = r["pos_rms"]
        rows = []
        for seed in range(51, 61):
            args_s = BASE + ["--ms-thrust", "23900"]
            rec = run_case(args.exe, args_s, seed, CORES[0])
            rec["arm"] = "A_default"
            rec["thrust_n"] = 23900
            rows.append(rec)
        worst = 0.0
        for r in rows:
            d = abs(r["pos_rms"] - arch[r["seed"]])
            worst = max(worst, d)
            print(f"seed {r['seed']}: new {r['pos_rms']:.6f} archive {arch[r['seed']]:.6f} d={d:.6f}")
        ok = worst <= 1e-6
        print("P1 构建锚:", "PASS" if ok else "FAIL", f"max|d|={worst:.6g}")
        return 0 if ok else 1

    seeds = list(range(args.seed_start, args.seed_start + args.seeds))
    rows: list[dict] = []
    with cf.ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = {pool.submit(run_seed, args.exe, s, CORES[i % len(CORES)]): s
                   for i, s in enumerate(seeds)}
        for fut in cf.as_completed(futures):
            rows.extend(fut.result())

    os.makedirs(DATA, exist_ok=True)
    save_jsonl(os.path.join(DATA, f"{args.tag}.jsonl"), rows)

    summary = summarize(rows)
    pair_ba = paired(rows, "A_default", "B_navraw")
    pair_cb = paired(rows, "B_navraw", "C_navraw_fb")

    ta, va = threshold(summary["A_default"])
    tb, vb = threshold(summary["B_navraw"])
    # 参考点：默认档位含 23900 时用它，否则用阶梯最高点（扩展扫描）。
    ref_thr = 23900 if 23900 in LADDER else LADDER[-1]
    p2_row = summary["B_navraw"][ref_thr]
    p2_rate_ok = p2_row["rate"] <= 0.5 * summary["A_default"][ref_thr]["rate"]
    p2_seed_ok = (summary["A_default"][ref_thr]["n_fail"] - p2_row["n_fail"]) >= 5
    p2_pair_ok = pair_ba[ref_thr]["worse"] <= 1 and pair_ba[ref_thr]["worst_degradation_m"] < 2.0
    # B 在整条扫描范围内都 <25% ⇒ 门槛被推到扫描上界之外，计为推移（并显式标注未定位）。
    if tb is None and ta is not None:
        p3_ok = True
    elif ta is None:
        p3_ok = False
    else:
        p3_ok = LADDER.index(tb) >= LADDER.index(ta) + 1
    p4_ok = all(v["worse"] == 0 and v["better"] == 0 for v in pair_cb.values()) and \
            all(summary["C_navraw_fb"][t]["nrfb_max"] == 0 for t in LADDER)
    p5_ok = all(summary["B_navraw"][t]["att_med"] <= 1.10 * summary["A_default"][t]["att_med"]
                for t in LADDER)
    max_a = max(summary["A_default"][t]["max"] for t in LADDER)
    max_b = max(summary["B_navraw"][t]["max"] for t in LADDER)
    p6_ok = max_b <= 1.10 * max_a

    verdict = {
        "date": _dt.date.today().isoformat(),
        "exe": args.exe, "exe_md5": md5_of(args.exe),
        "base": BASE, "ladder_N": LADDER,
        "arms": {k: v for k, v in ARMS},
        "seeds": [seeds[0], seeds[-1]],
        "bind_cores": CORES, "fail_threshold_m": FAIL_M,
        "summary": summary,
        "paired_A_vs_B": pair_ba,
        "paired_B_vs_C": pair_cb,
        "threshold": {"A": {"thrust": ta, "vmax": va},
                      "B": {"thrust": tb, "vmax": vb},
                      "B_beyond_scan": bool(tb is None)},
        "criteria": {
            "P1_build_anchor": "见 --anchor 单独运行",
            "P2_single_point": bool(p2_rate_ok and p2_seed_ok and p2_pair_ok),
            "P2_detail": {"ref_thrust": ref_thr, "rate_ok": p2_rate_ok,
                          "seed_ok": p2_seed_ok, "pair_ok": p2_pair_ok},
            "P3_threshold_shift": bool(p3_ok),
            "P4_fallback_inert": bool(p4_ok),
            "P5_att_cost": bool(p5_ok),
            "P6_tail": bool(p6_ok),
            "max_pos_A": round(max_a, 3), "max_pos_B": round(max_b, 3),
        },
        "note": "L2 SIL；失效率按 pos_rms>25 m；双峰格不得只看中位；不写实机。",
    }

    with open(os.path.join(DATA, f"{args.tag}_verdict.json"), "w", encoding="utf-8") as f:
        json.dump(verdict, f, ensure_ascii=False, indent=1)
    os.makedirs(FIGS, exist_ok=True)
    plot({**summary, "_raw": rows}, pair_ba,
         os.path.join(FIGS, f"{args.tag}.png"))

    print(json.dumps(verdict["criteria"], ensure_ascii=False, indent=1))
    print(json.dumps(verdict["threshold"], ensure_ascii=False))
    for arm in ("A_default", "B_navraw", "C_navraw_fb"):
        print(arm, [(t, summary[arm][t]["n_fail"], summary[arm][t]["rate"],
                     summary[arm][t]["vmax_med"]) for t in LADDER])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
