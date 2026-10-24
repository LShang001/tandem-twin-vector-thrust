# -*- coding: utf-8 -*-
"""run_mega.py — 三机型超大规模快速参数/噪声/误差 MC。

@file    run_mega.py
@module  仿真/三机型快速 MC（超大规模扫描驱动）
@brief   对 TandemVec/共轴/液体三型等效模型跑数万至数十万 case，统计包线、
         失效概率、P50/P95 与参数敏感度，保存 JSON/CSV。
@details 输入：models.py 的反演等效模型；fast_mc.py 的向量化闭环；
         处理：按 chunk 采样参数 → 仿真 → 聚合；输出：summary.json + samples.csv。
@theory  等效姿态/高度串级 PD + 执行器一阶；参数/噪声/故障随机化
         @ref CTL-003 §2、VER-002
@frame   悬停存档系小角度等效
@units   m、deg、s、N、kg
@note    ★ 快速降阶 MC，不是固件 SIL；结果用于参数敏感度/包线，不是实机结论。
@test    py -3.12 run_mega.py --cases 2000 --nominal
@see     models.py、fast_mc.py、VER-002
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from models import MODELS
from fast_mc import sample_params, simulate


def pct(a, q):
    a = a[np.isfinite(a)]
    if a.size == 0:
        return None
    return float(np.percentile(a, q))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases", type=int, default=20000)
    ap.add_argument("--chunk", type=int, default=5000)
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--nominal", action="store_true")
    ap.add_argument("--vehicles", default="TandemVec,CoaxialVTVL,LiquidRocket")
    ap.add_argument("--seed", type=int, default=20261024)
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    out_dir = Path(args.out) if args.out else (
        Path("sim-data") / "three-vehicle-mc-2026-10-24")
    out_dir.mkdir(parents=True, exist_ok=True)
    summary = {
        "seed": args.seed, "cases_per_vehicle": args.cases,
        "duration_s": args.duration, "nominal": args.nominal,
        "vehicles": {}, "note": "降阶快速 MC（非固件 SIL），参数为反演/实飞配置混合假设",
    }
    all_rows = []
    param_names = ["mass_mult", "i_pitch", "i_yaw", "thrust_mult", "tau_mult",
                   "att_kp_mult", "rate_kp_mult", "lever_mult",
                   "sensor_angle_deg", "sensor_rate_dps",
                   "bias_angle_deg", "bias_rate_dps", "wind_up",
                   "torque_noise", "delay_s", "thrust_loss", "servo_jam"]

    for name in [v.strip() for v in args.vehicles.split(",") if v.strip()]:
        model = MODELS[name]
        rows = []
        done = 0
        while done < args.cases:
            n = min(args.chunk, args.cases - done)
            p = sample_params(model, n, rng, nominal=args.nominal)
            m = simulate(model, p, rng, duration_s=args.duration)
            for i in range(n):
                row = dict(vehicle=name, case=done + i,
                           ok=int(m["ok"][i]), fail=int(m["fail"][i]),
                           final_h=float(m["final_h"][i]),
                           max_ang_deg=float(m["max_ang_deg"][i]),
                           max_roll_deg=float(m["max_roll_deg"][i]),
                           max_pitch_deg=float(m["max_pitch_deg"][i]),
                           sat_frac=float(m["sat_frac"][i]))
                for k in param_names:
                    row[k] = (bool(p[k][i]) if p[k].dtype == bool
                              else float(p[k][i]))
                rows.append(row)
            done += n

        ok = np.array([r["ok"] for r in rows], dtype=float)
        fail = np.array([r["fail"] for r in rows], dtype=float)
        ang = np.array([r["max_ang_deg"] for r in rows], dtype=float)
        fh = np.array([r["final_h"] for r in rows], dtype=float)
        sat = np.array([r["sat_frac"] for r in rows], dtype=float)
        # 参数敏感度：与 fail 的 Pearson 相关（只做诊断，不做因果）
        corr = {}
        for k in param_names:
            x = np.array([float(r[k]) for r in rows])
            if np.std(x) > 1e-12 and np.std(fail) > 1e-12:
                corr[k] = float(np.corrcoef(x, fail)[0, 1])
        top = sorted(corr.items(), key=lambda kv: abs(kv[1]), reverse=True)[:6]
        summary["vehicles"][name] = dict(
            cases=len(rows), pass_rate=float(ok.mean()),
            fail_rate=float(fail.mean()),
            final_h_p50=pct(fh, 50), final_h_p05=pct(fh, 5),
            final_h_p95=pct(fh, 95),
            max_ang_p50=pct(ang, 50), max_ang_p95=pct(ang, 95),
            sat_frac_p50=pct(sat, 50), sat_frac_p95=pct(sat, 95),
            top_param_corr=top,
        )
        all_rows.extend(rows)
        print(f"{name}: {len(rows)} cases, pass={ok.mean()*100:.2f}% "
              f"fail={fail.mean()*100:.2f}% max_ang P95={pct(ang,95):.2f}° "
              f"final_h P05/P95={pct(fh,5):.2f}/{pct(fh,95):.2f} m")
        print("  top corr:", ", ".join(f"{k}={v:+.3f}" for k, v in top))

    (out_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    with (out_dir / "samples.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(all_rows[0].keys()))
        writer.writeheader()
        writer.writerows(all_rows)
    print(f"saved: {out_dir/'summary.json'} / {out_dir/'samples.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
