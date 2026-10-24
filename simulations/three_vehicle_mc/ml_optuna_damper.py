"""ml_optuna_damper.py — 用 Optuna/TPE 贝叶斯优化振荡阻尼参数。

@file    ml_optuna_damper.py
@module  仿真/三机型快速 MC（贝叶斯优化）
@brief   在固定 TandemVec MC 样本上，用 TPE 搜索 window/threshold/decay/floor，
         目标为失败率 + 通过损失惩罚；后在独立 100k 上验证
@details 优化集与验证集使用不同 seed；每个 trial 与 fixed 基线共享噪声种子。
         TPE 为基于历史试验密度估计的贝叶斯优化采样器，适合混合连续/整数参数。
@theory  黑箱目标 f(θ)=fail_rate + λ·lost_pass_rate；TPE 用 good/bad 密度比
         提议下一组 θ，减少昂贵 MC 评估次数。
         @ref VER-003 §3.8、CTL-003 §2
@frame   N/A（超参数搜索）
@units   概率、s、无量纲
@note    ★ L1 模型内参数搜索；不是实机调参，不改变固件默认飞行参数。
@test    py -3.12 ml_optuna_damper.py
@see     fast_mc.py、models.py、VER-003
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import optuna
import pandas as pd

from fast_mc import sample_params, simulate
from models import MODELS


def run_variant(model, p, variant: str, seed: int, **kw):
    rng = np.random.default_rng(seed)
    return simulate(model, p, rng, duration_s=20.0, variant=variant, **kw)


def scores(m, base_ok):
    fail = float(m["fail"].mean())
    ok = float(m["ok"].mean())
    lost = float(np.mean(base_ok & ~m["ok"]))
    return {"fail": fail, "pass": ok, "lost_pass": lost,
            "score": fail + 0.30 * lost + 0.02 * (1.0 - ok)}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="sim-data/ml-2026-10-24")
    ap.add_argument("--opt-cases", type=int, default=8000)
    ap.add_argument("--val-cases", type=int, default=100000)
    ap.add_argument("--trials", type=int, default=60)
    ap.add_argument("--seed", type=int, default=20261027)
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    model = MODELS["TandemVec"]

    p_opt = sample_params(model, args.opt_cases, np.random.default_rng(args.seed))
    base_opt = run_variant(model, p_opt, "fixed", args.seed + 1)
    base_ok = base_opt["ok"].astype(bool)
    print("opt baseline:", scores(base_opt, base_ok))

    def objective(trial: optuna.Trial) -> float:
        window = trial.suggest_float("window_s", 0.08, 0.40)
        threshold = trial.suggest_int("threshold", 3, 10)
        decay = trial.suggest_float("decay", 0.75, 0.95)
        floor = trial.suggest_float("floor", 0.30, 0.60)
        recovery = trial.suggest_float("recovery", 1.005, 1.02)
        m = run_variant(model, p_opt, "osc", args.seed + 1,
                        osc_window_s=window, osc_threshold=threshold,
                        osc_decay=decay, osc_recovery=recovery,
                        osc_floor=floor)
        s = scores(m, base_ok)
        trial.set_user_attr("fail", s["fail"])
        trial.set_user_attr("pass", s["pass"])
        trial.set_user_attr("lost_pass", s["lost_pass"])
        return s["score"]

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    study = optuna.create_study(
        direction="minimize",
        sampler=optuna.samplers.TPESampler(multivariate=True, seed=args.seed))
    study.enqueue_trial({"window_s": 0.15, "threshold": 4, "decay": 0.85,
                         "floor": 0.45, "recovery": 1.01})
    study.optimize(objective, n_trials=args.trials)
    best = study.best_trial
    print("best:", best.value, best.params, best.user_attrs)

    # 独立验证：fixed / grid tuned / optuna best，共享噪声种子
    p_val = sample_params(model, args.val_cases, np.random.default_rng(args.seed + 100))
    m_fixed = run_variant(model, p_val, "fixed", args.seed + 101)
    m_grid = run_variant(model, p_val, "osc", args.seed + 101,
                         osc_window_s=0.15, osc_threshold=4, osc_decay=0.85,
                         osc_recovery=1.01, osc_floor=0.45)
    bp = best.params
    m_opt = run_variant(model, p_val, "osc", args.seed + 101,
                        osc_window_s=bp["window_s"], osc_threshold=bp["threshold"],
                        osc_decay=bp["decay"], osc_recovery=bp["recovery"],
                        osc_floor=bp["floor"])
    base_ok_val = m_fixed["ok"].astype(bool)
    variants = {"fixed": m_fixed, "grid_tuned": m_grid, "optuna": m_opt}
    val = {k: scores(m, base_ok_val) for k, m in variants.items()}
    fixed_fail = m_fixed["fail"].astype(bool)
    for key in ("grid_tuned", "optuna"):
        cand_fail = variants[key]["fail"].astype(bool)
        val[key]["rescued"] = int((fixed_fail & ~cand_fail).sum())
        val[key]["regressed"] = int((~fixed_fail & cand_fail).sum())
    print("validation:", json.dumps(val, indent=2, ensure_ascii=False))

    trials = study.trials_dataframe()
    trials.to_csv(out / "optuna_trials.csv", index=False)
    result = {
        "opt_cases": args.opt_cases, "val_cases": args.val_cases,
        "trials": args.trials, "seed": args.seed,
        "baseline_opt": scores(base_opt, base_ok),
        "best_value": float(best.value), "best_params": best.params,
        "best_user_attrs": best.user_attrs,
        "validation": val,
        "note": "L1 模型内 TPE 搜索；不是实机调参",
    }
    (out / "optuna_damper.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"saved: {out/'optuna_damper.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
