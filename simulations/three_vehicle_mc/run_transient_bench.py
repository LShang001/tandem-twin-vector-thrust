"""run_transient_bench.py — 驾驶杆双拍/突风瞬态下的阻尼误触发基准。

@file    run_transient_bench.py
@module  仿真/三机型快速 MC（瞬态基准）
@brief   固定/网格 tuned/TPE 三种阻尼参数，在姿态参考双拍、风阶跃、风正弦与
         组合场景下比较跟踪误差、饱和、失败率、阻尼触发率
@details 参考/扰动由 fast_mc.simulate 的可选参数注入；默认零输入行为已用 2k
         回归确认与旧版逐字节一致。故障关闭，只研究控制/阻尼瞬态。
@theory  驾驶杆/突风是“非振荡”输入；若阻尼器把跟踪动作误判为振荡，
         gain_active_frac 升高且 track_rms 变差，说明需要输入整形/门控。
         @ref VER-003 §3.8、CTL-003 §2
@frame   悬停存档系小角度等效
@units   deg、m、s、无量纲
@note    ★ L1 模型内瞬态基准；不是驾驶品质/实机适航结论。
@test    py -3.12 run_transient_bench.py --cases 2000
@see     fast_mc.py、ml_optuna_damper.py、VER-003
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from fast_mc import sample_params, simulate
from models import MODELS

SEL = dict(osc_window_s=1.0, osc_threshold=2, osc_decay=0.90,
           osc_recovery=1.01, osc_floor=0.55)
SEL_AGGR = dict(osc_window_s=1.0, osc_threshold=3, osc_decay=0.85,
                osc_recovery=1.01, osc_floor=0.55)


def scenarios(steps: int) -> dict:
    t = np.arange(steps) * 0.005
    roll = np.zeros(steps)
    slow = np.zeros(steps)
    for a, b, v in ((5, 7.5, 10), (7.5, 10, -10), (10, 12.5, 10),
                    (12.5, 15, -10)):
        slow[(t >= a) & (t < b)] = v
    fast = np.zeros(steps)
    for a in range(4, 14, 1):
        fast[(t >= a) & (t < a + 0.5)] = 10
        fast[(t >= a + 0.5) & (t < a + 1)] = -10
    return {
        "hover": dict(ref_roll_deg=roll),
        "doublet_slow": dict(ref_roll_deg=slow),
        "doublet_fast": dict(ref_roll_deg=fast),
        "wind_step": dict(wind_step_mps2=2.0, wind_step_s=5.0),
        "wind_sine": dict(wind_sine_amp=1.5, wind_sine_hz=0.5),
        "combined": dict(ref_roll_deg=fast, wind_sine_amp=1.5, wind_sine_hz=0.5),
    }


def pct(a, q):
    return float(np.percentile(np.asarray(a), q))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases", type=int, default=2000)
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--out", default="sim-data/transient-2026-10-24")
    ap.add_argument("--seed", type=int, default=20261030)
    args = ap.parse_args()
    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)
    model = MODELS["TandemVec"]
    steps = int(args.duration / 0.005)
    p = sample_params(model, args.cases, np.random.default_rng(args.seed))
    p["thrust_loss"] = np.ones(args.cases)
    p["servo_jam"] = np.zeros(args.cases, dtype=bool)
    variants = {"fixed": ("fixed", {}), "selective": ("osc", SEL),
                "selective_aggr": ("osc", SEL_AGGR),
                "detune85": ("fixed", {"gain_scale": 0.85}),
                "detune70": ("fixed", {"gain_scale": 0.70}),
                "lead05": ("lead", {"lead_s": 0.05})}
    rows = []
    for scen, kw in scenarios(steps).items():
        for name, (variant, osc) in variants.items():
            m = simulate(model, p, np.random.default_rng(args.seed + 1),
                         duration_s=args.duration, variant=variant, **osc, **kw)
            row = {
                "scenario": scen, "variant": name,
                "fail_rate": float(m["fail"].mean()),
                "track_rms_roll": pct(m["track_rms_roll_deg"], 50),
                "track_rms_roll_p95": pct(m["track_rms_roll_deg"], 95),
                "max_track_err_p95": pct(m["max_track_err_deg"], 95),
                "sat_frac_mean": float(m["sat_frac"].mean()),
                "sat_frac_p95": pct(m["sat_frac"], 95),
                "gain_min_p05": pct(m["gain_min"], 5),
                "gain_active_mean": float(m["gain_active_frac"].mean()),
                "gain_active_p95": pct(m["gain_active_frac"], 95),
                "final_h_p05": pct(m["final_h"], 5),
                "final_h_p95": pct(m["final_h"], 95),
            }
            rows.append(row)
            print(f"{scen:13s} {name:5s} fail={row['fail_rate']*100:5.2f}% "
                  f"rms={row['track_rms_roll']:6.2f}° "
                  f"maxerr95={row['max_track_err_p95']:6.2f}° "
                  f"gain_act={row['gain_active_mean']*100:5.1f}% "
                  f"gainmin={row['gain_min_p05']:.2f}")
    import csv
    with (out / "transient_bench.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    (out / "transient_bench.json").write_text(
        json.dumps({"seed": args.seed, "cases": args.cases,
                    "duration_s": args.duration,
                    "note": "L1 瞬态基准；故障关闭；不是驾驶品质结论",
                    "rows": rows}, indent=2, ensure_ascii=False),
        encoding="utf-8")

    # 触发率/跟踪误差对比图
    scens = list(scenarios(steps).keys())
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.5))
    width = 0.26
    for j, name in enumerate(variants):
        sub = [next(r for r in rows if r["scenario"] == s and r["variant"] == name)
               for s in scens]
        xs = np.arange(len(scens)) + (j - 1) * width
        axes[0].bar(xs, [r["gain_active_mean"] * 100 for r in sub], width, label=name)
        axes[1].bar(xs, [r["track_rms_roll"] for r in sub], width, label=name)
    for ax, lab in zip(axes, ("damping active (% of 20 s)", "roll tracking RMS (deg)")):
        ax.set_xticks(np.arange(len(scens))); ax.set_xticklabels(scens, rotation=25)
        ax.set_ylabel(lab); ax.legend(); ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "transient_bench.png", dpi=140)
    plt.close(fig)
    print(f"saved: {out/'transient_bench.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
