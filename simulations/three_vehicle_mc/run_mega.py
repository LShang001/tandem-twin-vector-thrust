# -*- coding: utf-8 -*-
"""run_mega.py — 三机型超大规模快速参数/噪声/误差 MC。

@file    run_mega.py
@module  仿真/三机型快速 MC（超大规模扫描驱动）
@brief   对 TandemVec/共轴/液体三型等效模型跑数万至数百万 case，统计包线、
         失效概率、P50/P95 与参数敏感度，保存 JSON/CSV。
@details 输入：models.py 的反演等效模型；fast_mc.py 的向量化闭环；
         处理：按 chunk 采样参数 → 仿真 → 流式写 CSV + 分块聚合；
         输出：summary.json + samples.csv。
@theory  等效姿态/高度串级 PD + 执行器一阶；参数/噪声/故障随机化
         @ref CTL-003 §2、VER-002
@frame   悬停存档系小角度等效
@units   m、deg、s、N、kg
@note    ★ 快速降阶 MC，不是固件 SIL；结果用于参数敏感度/包线，不是实机结论。
         流式写 CSV，避免百万行 dict 常驻内存。
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

PARAM_NAMES = ["mass_mult", "i_pitch", "i_yaw", "thrust_mult", "tau_mult",
               "att_kp_mult", "rate_kp_mult", "lever_mult",
               "sensor_angle_deg", "sensor_rate_dps",
               "bias_angle_deg", "bias_rate_dps", "wind_up",
               "torque_noise", "delay_s", "thrust_loss", "servo_jam"]
FIELDNAMES = ["vehicle", "case", "ok", "fail", "final_h", "max_ang_deg",
              "max_roll_deg", "max_pitch_deg", "sat_frac"] + PARAM_NAMES


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
    ap.add_argument("--variant", default="fixed",
                    choices=["fixed", "auth", "osc", "adaptive", "adrc", "lead", "indi",
                             "accel_fb", "phase_lead"])
    ap.add_argument("--osc-window", type=float, default=0.25)
    ap.add_argument("--osc-threshold", type=float, default=6.0)
    ap.add_argument("--osc-decay", type=float, default=0.90)
    ap.add_argument("--osc-recovery", type=float, default=1.01)
    ap.add_argument("--osc-floor", type=float, default=0.55)
    ap.add_argument("--osc-filter-tau", type=float, default=0.05)
    ap.add_argument("--osc-min-rate", type=float, default=2.0)
    ap.add_argument("--osc-ref-gate", type=float, default=0.5)
    ap.add_argument("--gain-scale", type=float, default=1.0)
    ap.add_argument("--adrc-wc", type=float, default=16.0)
    ap.add_argument("--adrc-wo", type=float, default=50.0)
    ap.add_argument("--adrc-b0", type=float, default=1.0)
    ap.add_argument("--lead-s", type=float, default=0.0)
    ap.add_argument("--indi-filter-tau", type=float, default=0.02)
    ap.add_argument("--indi-b0", type=float, default=1.0)
    ap.add_argument("--accel-fb-gain", type=float, default=0.8)
    ap.add_argument("--accel-fb-tau", type=float, default=0.02)
    ap.add_argument("--phase-lead-gain", type=float, default=0.3)
    ap.add_argument("--phase-lead-tau", type=float, default=0.02)
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
        "variant": args.variant,
        "osc_params": {
            "window_s": args.osc_window, "threshold": args.osc_threshold,
            "decay": args.osc_decay, "recovery": args.osc_recovery,
            "floor": args.osc_floor, "filter_tau_s": args.osc_filter_tau,
            "min_rate_dps": args.osc_min_rate, "ref_gate_deg": args.osc_ref_gate,
            "gain_scale": args.gain_scale,
            "adrc": {"wc": args.adrc_wc, "wo": args.adrc_wo, "b0": args.adrc_b0},
            "lead_s": args.lead_s,
            "indi": {"filter_tau_s": args.indi_filter_tau, "b0": args.indi_b0},
            "accel_fb": {"gain": args.accel_fb_gain,
                         "tau_s": args.accel_fb_tau},
            "phase_lead": {"gain": args.phase_lead_gain,
                           "tau_s": args.phase_lead_tau},
        },
        "vehicles": {}, "note": "降阶快速 MC（非固件 SIL），参数为反演/实飞配置混合假设",
    }

    csv_path = out_dir / "samples.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        writer.writeheader()

        for name in [v.strip() for v in args.vehicles.split(",") if v.strip()]:
            model = MODELS[name]
            parts = {k: [] for k in ("ok", "fail", "ang", "fh", "sat")}
            param_parts = {k: [] for k in PARAM_NAMES}
            done = 0
            while done < args.cases:
                n = min(args.chunk, args.cases - done)
                p = sample_params(model, n, rng, nominal=args.nominal)
                m = simulate(model, p, rng, duration_s=args.duration,
                             variant=args.variant,
                             osc_window_s=args.osc_window,
                             osc_threshold=args.osc_threshold,
                             osc_decay=args.osc_decay,
                             osc_recovery=args.osc_recovery,
                             osc_floor=args.osc_floor,
                             osc_filter_tau_s=args.osc_filter_tau,
                             osc_min_rate_dps=args.osc_min_rate,
                             osc_ref_gate_deg=args.osc_ref_gate,
                             gain_scale=args.gain_scale,
                             adrc_wc=args.adrc_wc, adrc_wo=args.adrc_wo,
                             adrc_b0=args.adrc_b0,
                             lead_s=args.lead_s,
                             indi_filter_tau_s=args.indi_filter_tau,
                             indi_b0=args.indi_b0,
                             accel_fb_gain=args.accel_fb_gain,
                             accel_fb_tau_s=args.accel_fb_tau,
                             phase_lead_gain=args.phase_lead_gain,
                             phase_lead_tau_s=args.phase_lead_tau)
                chunk = []
                for i in range(n):
                    row = dict(vehicle=name, case=done + i,
                               ok=int(m["ok"][i]), fail=int(m["fail"][i]),
                               final_h=float(m["final_h"][i]),
                               max_ang_deg=float(m["max_ang_deg"][i]),
                               max_roll_deg=float(m["max_roll_deg"][i]),
                               max_pitch_deg=float(m["max_pitch_deg"][i]),
                               sat_frac=float(m["sat_frac"][i]))
                    for k in PARAM_NAMES:
                        row[k] = (bool(p[k][i]) if p[k].dtype == bool
                                  else float(p[k][i]))
                    chunk.append(row)
                writer.writerows(chunk)
                parts["ok"].append(m["ok"])
                parts["fail"].append(m["fail"])
                parts["ang"].append(m["max_ang_deg"])
                parts["fh"].append(m["final_h"])
                parts["sat"].append(m["sat_frac"])
                for k in PARAM_NAMES:
                    param_parts[k].append(p[k])
                done += n

            ok = np.concatenate(parts["ok"]).astype(float)
            fail = np.concatenate(parts["fail"]).astype(float)
            ang = np.concatenate(parts["ang"])
            fh = np.concatenate(parts["fh"])
            sat = np.concatenate(parts["sat"])
            # 参数敏感度：与 fail 的 Pearson 相关（只做诊断，不做因果）
            corr = {}
            for k in PARAM_NAMES:
                x = np.concatenate(param_parts[k]).astype(float)
                if np.std(x) > 1e-12 and np.std(fail) > 1e-12:
                    corr[k] = float(np.corrcoef(x, fail)[0, 1])
            top = sorted(corr.items(), key=lambda kv: abs(kv[1]), reverse=True)[:6]
            summary["vehicles"][name] = dict(
                cases=int(ok.size), pass_rate=float(ok.mean()),
                fail_rate=float(fail.mean()),
                final_h_p50=pct(fh, 50), final_h_p05=pct(fh, 5),
                final_h_p95=pct(fh, 95),
                max_ang_p50=pct(ang, 50), max_ang_p95=pct(ang, 95),
                sat_frac_p50=pct(sat, 50), sat_frac_p95=pct(sat, 95),
                top_param_corr=top,
            )
            print(f"{name}: {ok.size} cases, pass={ok.mean()*100:.2f}% "
                  f"fail={fail.mean()*100:.2f}% max_ang P95={pct(ang,95):.2f}° "
                  f"final_h P05/P95={pct(fh,5):.2f}/{pct(fh,95):.2f} m")
            print("  top corr:", ", ".join(f"{k}={v:+.3f}" for k, v in top))

    (out_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"saved: {out_dir/'summary.json'} / {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
