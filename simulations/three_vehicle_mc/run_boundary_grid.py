"""run_boundary_grid.py — TandemVec 确定性边界网格与阻尼参数筛选。

@file    run_boundary_grid.py
@module  仿真/三机型快速 MC（边界网格）
@brief  延迟 × 姿态/速率增益 × 故障的确定性网格，比较固定增益与多组振荡阻尼参数
@details 与随机 MC 互补：随机 MC 给概率，网格给失效边界。所有候选共享同一组
         网格点、同一噪声种子，因此可逐点比较 rescued/regressed；输出
         grid.csv + summary.json。
@theory  等效姿态串级 PD + 执行器一阶；延迟/增益/推力损失/舵机卡死
         @ref CTL-003 §2、VER-003 §3.6
@frame   悬停存档系小角度等效
@units   s、deg、N、kg
@note    L1 降阶模型；网格刻意加严，百分比不是实机概率。
@test    py -3.12 run_boundary_grid.py --out sim-data/boundary-grid-2026-10-24
@see     fast_mc.py、models.py、VER-003
"""

from __future__ import annotations

import argparse
import itertools
import json
from pathlib import Path

import numpy as np

from fast_mc import simulate
from models import MODELS

DELAYS = [0.0, 0.03, 0.06, 0.09, 0.12, 0.15, 0.18, 0.21]
ATT_GAINS = [0.8, 1.0, 1.2, 1.4]
RATE_GAINS = [0.8, 1.0, 1.2, 1.4]
FAULTS = ["none", "thrust_loss", "servo_jam"]
REPEATS = 3

CANDIDATES = {
    "fixed": None,
    "osc_default": dict(window=0.25, threshold=6.0, decay=0.90, recovery=1.01, floor=0.55),
    "osc_fast": dict(window=0.15, threshold=4.0, decay=0.90, recovery=1.01, floor=0.45),
    "osc_deep": dict(window=0.25, threshold=6.0, decay=0.85, recovery=1.01, floor=0.40),
    "osc_long": dict(window=0.35, threshold=5.0, decay=0.88, recovery=1.01, floor=0.45),
    "osc_fast_deep_latch": dict(window=0.15, threshold=4.0, decay=0.85, recovery=1.00, floor=0.40),
    "osc_fast_deep_r": dict(window=0.15, threshold=4.0, decay=0.85, recovery=1.01, floor=0.40),
    "osc_faster_deep_r": dict(window=0.10, threshold=4.0, decay=0.85, recovery=1.01, floor=0.40),
    "osc_fast_mid_r": dict(window=0.15, threshold=4.0, decay=0.85, recovery=1.01, floor=0.45),
}


def build_params(n: int) -> tuple[dict, list[dict]]:
    rows = []
    for delay, att, rate, fault, rep in itertools.product(
            DELAYS, ATT_GAINS, RATE_GAINS, FAULTS, range(REPEATS)):
        rows.append(dict(delay_s=delay, att_kp_mult=att, rate_kp_mult=rate,
                         fault=fault, repeat=rep))
    p = {
        "mass_mult": np.ones(n), "i_roll": np.ones(n), "i_pitch": np.ones(n),
        "i_yaw": np.ones(n), "thrust_mult": np.ones(n), "tau_mult": np.ones(n),
        "att_kp_mult": np.array([r["att_kp_mult"] for r in rows]),
        "rate_kp_mult": np.array([r["rate_kp_mult"] for r in rows]),
        "lever_mult": np.ones(n), "sensor_angle_deg": np.full(n, 1.0),
        "sensor_rate_dps": np.full(n, 2.0), "bias_angle_deg": np.full(n, 0.5),
        "bias_rate_dps": np.full(n, 0.5), "wind_up": np.full(n, 0.3),
        "torque_noise": np.full(n, 0.05),
        "delay_s": np.array([r["delay_s"] for r in rows]),
        "thrust_loss": np.array([0.5 if r["fault"] == "thrust_loss" else 1.0
                                 for r in rows]),
        "servo_jam": np.array([r["fault"] == "servo_jam" for r in rows]),
        "init_angle_deg": np.full(n, 1.0), "init_rate_dps": np.full(n, 1.0),
    }
    return p, rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="sim-data/boundary-grid-2026-10-24")
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--seed", type=int, default=20261024)
    args = ap.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    model = MODELS["TandemVec"]
    p, rows = build_params(len(DELAYS) * len(ATT_GAINS) * len(RATE_GAINS)
                           * len(FAULTS) * REPEATS)
    n = len(rows)
    no_fault_mask = np.array([r["fault"] == "none" for r in rows])

    results = {}
    summary = {"grid": {"delays_s": DELAYS, "att_gains": ATT_GAINS,
                        "rate_gains": RATE_GAINS, "faults": FAULTS,
                        "repeats": REPEATS, "cases": n},
               "candidates": {}}
    for name, cfg in CANDIDATES.items():
        rng = np.random.default_rng(args.seed)
        kw = {} if cfg is None else dict(
            osc_window_s=cfg["window"], osc_threshold=cfg["threshold"],
            osc_decay=cfg["decay"], osc_recovery=cfg["recovery"],
            osc_floor=cfg["floor"])
        m = simulate(model, p, rng, duration_s=args.duration,
                     variant="fixed" if cfg is None else "osc", **kw)
        fail = m["fail"].astype(bool)
        ok = m["ok"].astype(bool)
        results[name] = (fail, ok, m)
        by_delay, by_gain, by_fault = [], [], {}
        d = p["delay_s"]
        for lo, hi in [(0.0, 0.06), (0.06, 0.09), (0.09, 0.12),
                       (0.12, 0.15), (0.15, 0.18), (0.18, 9.0)]:
            sel = (d >= lo) & (d < hi) & no_fault_mask
            by_delay.append({"bin": f"{lo:.2f}-{hi:.2f}", "n": int(sel.sum()),
                             "fail": float(fail[sel].mean()) if sel.any() else None})
        for g in ATT_GAINS:
            sel = (p["att_kp_mult"] == g) & no_fault_mask
            by_gain.append({"att_kp": g, "n": int(sel.sum()),
                            "fail": float(fail[sel].mean())})
        for f in FAULTS:
            sel = np.array([r["fault"] == f for r in rows])
            by_fault[f] = {"n": int(sel.sum()), "fail": float(fail[sel].mean()),
                           "pass": float(ok[sel].mean())}
        summary["candidates"][name] = {
            "overall": {"n": n, "fail": float(fail.mean()), "pass": float(ok.mean())},
            "no_fault": {"n": int(no_fault_mask.sum()),
                         "fail": float(fail[no_fault_mask].mean()),
                         "pass": float(ok[no_fault_mask].mean())},
            "by_delay": by_delay, "by_att_gain": by_gain, "by_fault": by_fault,
            "settings": cfg,
        }
        print(f"{name}: overall fail={fail.mean()*100:.1f}% "
              f"no_fault fail={fail[no_fault_mask].mean()*100:.1f}% "
              f"pass={ok.mean()*100:.1f}%")

    fixed_fail = results["fixed"][0]
    for name, (fail, ok, _m) in results.items():
        if name == "fixed":
            continue
        summary["candidates"][name]["vs_fixed"] = {
            "rescued": int((fixed_fail & ~fail).sum()),
            "regressed": int((~fixed_fail & fail).sum()),
            "new_pass": int((~results["fixed"][1] & ok).sum()),
            "lost_pass": int((results["fixed"][1] & ~ok).sum()),
        }

    with (out_dir / "grid.csv").open("w", newline="", encoding="utf-8") as f:
        f.write("candidate,delay_s,att_kp_mult,rate_kp_mult,fault,repeat,ok,fail,"
                "final_h,max_ang_deg\n")
        for name, (fail, ok, m) in results.items():
            for i, r in enumerate(rows):
                f.write(f"{name},{r['delay_s']},{r['att_kp_mult']},"
                        f"{r['rate_kp_mult']},{r['fault']},{r['repeat']},"
                        f"{int(ok[i])},{int(fail[i])},{m['final_h'][i]:.6f},"
                        f"{m['max_ang_deg'][i]:.6f}\n")
    (out_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"saved: {out_dir/'grid.csv'} / {out_dir/'summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
