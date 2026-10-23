#!/usr/bin/env python3
"""run_param_mc.py — 参数不确定性 Latin Hypercube 蒙特卡洛（L1 模型仿真）。

@file    run_param_mc.py
@module  analysis/参数不确定性 MC（hifi 6DOF + 固件控制/EKF）
@brief   对质量/CG/惯量/推进/执行器/控制增益做 LHS 采样，跑 A2 定高+位置保持
         +磁矢量+光流场景，输出 P50/P95/P99/最坏案例与参数敏感度排序。
@details 输入：--cases（默认 64 试点）、--workers、--seed、--out；
         处理：每例在 worker 内重置 hifi_core.P/CTRL，派生 a/b/Ix/Iy/kQ/w_hover，
         跑 40 s 闭环；输出 CSV + summary JSON + manifest。
@theory  参数不确定性传播：LHS 分层采样；失败判据为位置/姿态/高度误差超限或非有限。
@frame   NED + FRD；姿态 deg、位置 m。
@units   SI；质量 kg、惯量 kg·m²、系数 N·s²、时间 s。
@note    这是 L1 模型仿真：hifi_core 无气动/无真实传感器标定，参数范围来自
         ESTIMATED/MODEL-DEFAULT，不能写成实机结论。敏感度先用标准化回归/Spearman
         排名，Sobol 需要更大样本与代理模型，后续再做。
@see     hifi_core.py、run_scenes_opt.py、FULL-ENVELOPE-STATUS.md
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent.parent  # 仓库根
sys.path.insert(0, str(HERE.parent / "opt"))
sys.path.insert(0, str(HERE))

import hifi_core as hc  # noqa: E402
import run_scenes_opt as rs  # noqa: E402
from opt_ekf import build_cfg  # noqa: E402

BASE_P = dict(hc.P)
BASE_CTRL = dict(hc.CTRL)
M0 = BASE_P["m"]
BASE_W = hc.w_hover

# name, low, high, log?
PARAMS = [
    ("m", 0.55, 0.85, False),
    ("e_cg", -0.040, 0.040, False),
    ("Ix_scale", 0.60, 1.60, False),
    ("Iy_scale", 0.60, 1.50, False),
    ("kT", 0.80e-5, 1.30e-5, True),
    ("kQ_ratio", 0.020, 0.038, False),
    ("Jp", 0.8e-4, 4.0e-4, True),
    ("tauM", 0.15, 0.50, True),
    ("att_kp", 1.80, 4.20, False),
    ("att_kp_yaw", 2.50, 7.50, False),
    ("rate_kp_rp", 8.0, 25.0, False),
    ("rate_ki_rp", 2.0, 9.0, False),
    ("rate_kp_yaw", 6.0, 18.0, False),
    ("rate_ki_yaw", 1.5, 7.0, False),
    ("alt_pos_kp", 0.50, 1.80, False),
    ("alt_vel_kp", 2.5, 8.0, False),
    ("alt_vel_ki", 0.30, 2.0, False),
]

# 压力尾：只为找边界，不代表正常包线；范围明显超出 ESTIMATED 不确定度。
STRESS_PARAMS = [
    ("m", 0.45, 0.95, False),
    ("e_cg", -0.060, 0.060, False),
    ("Ix_scale", 0.40, 2.20, False),
    ("Iy_scale", 0.40, 1.80, False),
    ("kT", 0.65e-5, 1.50e-5, True),
    ("kQ_ratio", 0.015, 0.045, False),
    ("Jp", 0.4e-4, 6.0e-4, True),
    ("tauM", 0.08, 0.70, True),
    ("att_kp", 0.80, 6.00, False),
    ("att_kp_yaw", 1.0, 10.0, False),
    ("rate_kp_rp", 4.0, 35.0, False),
    ("rate_ki_rp", 0.5, 12.0, False),
    ("rate_kp_yaw", 3.0, 25.0, False),
    ("rate_ki_yaw", 0.5, 10.0, False),
    ("alt_pos_kp", 0.20, 3.00, False),
    ("alt_vel_kp", 1.0, 10.0, False),
    ("alt_vel_ki", 0.10, 3.0, False),
]

# 失败门限（A2 定高+位置保持，40 s，L1）
LIMITS = {"pos_ne": 10.0, "att_deg": 15.0, "alt_err": 5.0}
WINDOW = (12.0, 40.0)


def lhs(n: int, d: int, rng: np.random.Generator) -> np.ndarray:
    """简单 LHS：每维分层 [0,1)，独立随机置换。"""
    u = np.empty((n, d), dtype=float)
    for j in range(d):
        perm = rng.permutation(n)
        u[:, j] = (perm + rng.random(n)) / float(n)
    return u


def map_unit(u: float, lo: float, hi: float, log: bool) -> float:
    """把 [0,1) 分层值映射到参数区间（可选对数）。"""
    if log:
        return float(math.exp(math.log(lo) + u * (math.log(hi) - math.log(lo))))
    return float(lo + u * (hi - lo))


def make_design(n: int, seed: int, params=PARAMS) -> list[dict]:
    """生成 n 例 LHS 参数设计（每维独立分层+随机置换）。"""
    rng = np.random.default_rng(seed)
    u = lhs(n, len(params), rng)
    rows = []
    for i in range(n):
        row = {}
        for j, (name, lo, hi, log) in enumerate(params):
            row[name] = map_unit(float(u[i, j]), lo, hi, log)
        rows.append(row)
    return rows


def apply_case(row: dict) -> float:
    """把一例参数写入 hifi_core/run_scenes_opt 的模块全局；返回 w_hover。"""
    hc.P.clear()
    hc.P.update(BASE_P)
    hc.CTRL.clear()
    hc.CTRL.update(BASE_CTRL)

    m = float(row["m"])
    e = float(row["e_cg"])
    hc.P["m"] = m
    hc.P["a"] = 0.315 - e
    hc.P["b"] = 0.315 + e
    hc.P["Ix"] = BASE_P["Ix"] * float(row["Ix_scale"]) * (m / M0)
    iy = BASE_P["Iy"] * float(row["Iy_scale"]) * (m / M0)
    hc.P["Iy"] = iy
    hc.P["Iz"] = iy
    hc.P["kT"] = float(row["kT"])
    hc.P["kQ"] = float(row["kT"]) * float(row["kQ_ratio"])
    hc.P["Jp"] = float(row["Jp"])
    hc.P["tauM"] = float(row["tauM"])

    hc.CTRL["att_kp"] = float(row["att_kp"])
    hc.CTRL["att_kp_yaw"] = float(row["att_kp_yaw"])
    hc.CTRL["rate_kp_rp"] = float(row["rate_kp_rp"])
    hc.CTRL["rate_ki_rp"] = float(row["rate_ki_rp"])
    hc.CTRL["rate_kp_yaw"] = float(row["rate_kp_yaw"])
    hc.CTRL["rate_ki_yaw"] = float(row["rate_ki_yaw"])
    hc.CTRL["alt_pos_kp"] = float(row["alt_pos_kp"])
    hc.CTRL["alt_vel_kp"] = float(row["alt_vel_kp"])
    hc.CTRL["alt_vel_ki"] = float(row["alt_vel_ki"])

    w_hover = math.sqrt(0.5 * m * hc.P["g"] / hc.P["kT"])
    hc.w_hover = w_hover
    hc.P["w_hover"] = w_hover
    hc.P["w0_floor"] = 0.6 * w_hover
    rs.w_hover = w_hover
    return w_hover


def run_one(item: tuple[int, dict, bool]) -> dict:
    """在 worker 内应用一例参数并运行 40 s 闭环，返回指标+参数。"""
    case_id, row, stress = item
    w_hover = apply_case(row)
    cfgd = build_cfg({})
    opts = {}
    nav_factory = lambda: rs.NavV(cfgd, opts, p0_hdg=0.2, static_gate=True)
    scene = {
        "duration": 40.0,
        "fault": ({"wind_gust": (20.0, 25.0), "gps_lost": (20.0, 25.0)}
                  if stress else {"wind_gust": (20.0, 25.0)}),
        "ref": {
            "alt_hold": 10.0,
            "pos_hold": True,
            "tilt": (10.0, 25.0),
            "thr": 100.0 * w_hover / hc.P["wMax"],
        },
        "mag_vector": True,
        "flow_aiding": True,
        "baro_hz": 2.0,
        "baro_phase": 2,
    }
    try:
        r = rs.run_scene(scene, 40.0, nav_factory, seed=1, seed_dyn=1)
        t = r["t"]
        mask = (t >= WINDOW[0]) & (t <= WINDOW[1])
        pe = r["pos_err"][mask]
        pos_ne = float(np.sqrt(np.mean(pe[:, 0] ** 2 + pe[:, 1] ** 2)))
        att = float(np.sqrt(np.mean(r["att_qerr"][mask] ** 2)) * rs.R2D)
        alt_err = float(np.sqrt(np.mean((r["ealt"][mask] - r["alt"][mask]) ** 2)))
        finite = bool(np.all(np.isfinite(pe)) and np.isfinite(att) and
                      np.isfinite(alt_err))
        fail = (not finite or pos_ne > LIMITS["pos_ne"] or
                att > LIMITS["att_deg"] or alt_err > LIMITS["alt_err"])
        out = {
            "case": case_id,
            "pos_ne": pos_ne,
            "att_deg": att,
            "alt_err": alt_err,
            "finite": int(finite),
            "fail": int(fail),
            "w_hover": w_hover,
        }
    except Exception as exc:  # noqa: BLE001
        out = {
            "case": case_id, "pos_ne": float("nan"), "att_deg": float("nan"),
            "alt_err": float("nan"), "finite": 0, "fail": 1,
            "w_hover": w_hover, "error": repr(exc),
        }
    out.update(row)
    return out


def pct(values: list[float], q: float) -> float:
    """取有限样本的 q 分位（q∈[0,1]）；无有效样本返回 NaN。"""
    v = sorted(x for x in values if math.isfinite(x))
    if not v:
        return float("nan")
    return v[int(q * (len(v) - 1))]


def spearman(x: np.ndarray, y: np.ndarray) -> float:
    """Spearman 相关（无 scipy）：秩相关。"""
    def rank(a):
        order = np.argsort(a)
        r = np.empty(len(a), dtype=float)
        r[order] = np.arange(len(a), dtype=float)
        return r
    rx, ry = rank(x), rank(y)
    rx -= rx.mean()
    ry -= ry.mean()
    den = math.sqrt(float(np.sum(rx * rx) * np.sum(ry * ry)))
    return float(np.sum(rx * ry) / den) if den > 0 else 0.0


def main() -> int:
    """CLI 入口：LHS 设计→并行仿真→CSV/summary/manifest。"""
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases", type=int, default=64)
    ap.add_argument("--workers", type=int, default=max(1, min(12, os.cpu_count() or 1)))
    ap.add_argument("--seed", type=int, default=20261024)
    ap.add_argument("--stress", action="store_true")
    ap.add_argument("--out", default=str(HERE.parent.parent / "sim-data" /
                                          "campaign-2026-10-24" / "param-mc"))
    args = ap.parse_args()
    outdir = Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)
    params = STRESS_PARAMS if args.stress else PARAMS
    design = make_design(args.cases, args.seed, params)
    t0 = time.time()
    rows = []
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        futs = {ex.submit(run_one, (i, row, args.stress)): i for i, row in enumerate(design)}
        done = 0
        for fut in as_completed(futs):
            rows.append(fut.result())
            done += 1
            if done % max(1, args.cases // 8) == 0 or done == args.cases:
                print(f"[progress] {done}/{args.cases} elapsed={time.time()-t0:.1f}s",
                      flush=True)
    rows.sort(key=lambda r: r["case"])
    keys = ["case", "pos_ne", "att_deg", "alt_err", "finite", "fail", "w_hover"]
    keys += [p[0] for p in params] + ["error"]
    with (outdir / "cases.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    fails = sum(int(r["fail"]) for r in rows)
    pos = [float(r["pos_ne"]) for r in rows]
    att = [float(r["att_deg"]) for r in rows]
    alt = [float(r["alt_err"]) for r in rows]
    worst = sorted(rows, key=lambda r: (-int(r["fail"]),
                                        -float(r["pos_ne"]) if math.isfinite(r["pos_ne"]) else 0))[:10]
    sens = []
    for name, *_ in params:
        x = np.array([float(r[name]) for r in rows])
        sens.append((name, spearman(x, np.array(pos)),
                     spearman(x, np.array(att)),
                     spearman(x, np.array(alt))))
    sens.sort(key=lambda s: -(abs(s[1]) + abs(s[2]) + abs(s[3])))
    summary = {
        "cases": len(rows),
        "fail": fails,
        "fail_rate": fails / max(1, len(rows)),
        "pos_ne": {"p50": pct(pos, .50), "p95": pct(pos, .95),
                   "p99": pct(pos, .99), "max": pct(pos, 1.0)},
        "att_deg": {"p50": pct(att, .50), "p95": pct(att, .95),
                    "p99": pct(att, .99), "max": pct(att, 1.0)},
        "alt_err": {"p50": pct(alt, .50), "p95": pct(alt, .95),
                    "p99": pct(alt, .99), "max": pct(alt, 1.0)},
        "limits": LIMITS,
        "sensitivity_spearman": [
            {"param": n, "pos_ne": p, "att_deg": a, "alt_err": e}
            for n, p, a, e in sens
        ],
        "worst_cases": worst,
        "elapsed_s": time.time() - t0,
    }
    try:
        head = subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD"],
                                       text=True).strip()
    except Exception:  # noqa: BLE001
        head = "unknown"
    manifest = {
        "git_head": head,
        "seed": args.seed,
        "cases": args.cases,
        "workers": args.workers,
        "params": [{"name": n, "low": lo, "high": hi, "log": lg}
                   for n, lo, hi, lg in params],
        "limits": LIMITS,
        "window_s": WINDOW,
        "note": "L1 hifi model; parameters ESTIMATED/MODEL-DEFAULT; not HIL/real",
        "stress": bool(args.stress),
    }
    (outdir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    (outdir / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({k: summary[k] for k in ("cases", "fail", "fail_rate",
                                              "pos_ne", "att_deg", "alt_err")},
                     ensure_ascii=False, indent=2))
    print("top sensitivity:", [(s["param"], round(s["pos_ne"], 2))
                                for s in summary["sensitivity_spearman"][:5]])
    print("out:", outdir)
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
