"""ml_optuna_worstcase.py — 用 TPE 主动搜索 TandemVec 最坏参数组合（AI 红队）。

@file    ml_optuna_worstcase.py
@module  仿真/三机型快速 MC（对抗性参数搜索）
@brief   以 max_ang/final_h 构造严重度目标，用 Optuna TPE 在采样边界内主动寻找
         比 1M 随机 MC 更坏的组合；fixed 与 tuned 各跑一轮
@details 这是“主动找最坏”而非概率估计；每个 trial 单 case，最优解再用多个噪声
         种子复核，避免单次噪声偶然。
@theory  随机 MC 估计分布尾部，但低概率角点采样不足；TPE 用历史好/坏点密度
         主动往高严重度区域搜索，可检验“随机样本是否漏掉更坏组合”。
         @ref VER-003 §3.8、CTL-003 §2
@frame  悬停存档系小角度等效（极端角度已超出小角度假设，只作数值边界）
@units   deg、m、s、无量纲
@note    ★ L1 对抗搜索；不是实机概率，也不能外推到真实气动/结构极限。
@test    py -3.12 ml_optuna_worstcase.py
@see     fast_mc.py、run_mega.py、VER-003
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import optuna

from fast_mc import simulate
from models import MODELS


def one_case(model, rng_seed: int, variant: str, params: dict, **osc):
    n = 1
    p = {
        "mass_mult": np.array([params["mass_mult"]]),
        "i_roll": np.array([1.0]), "i_pitch": np.array([params["i_pitch"]]),
        "i_yaw": np.array([params["i_yaw"]]),
        "thrust_mult": np.array([params["thrust_mult"]]),
        "tau_mult": np.array([params["tau_mult"]]),
        "att_kp_mult": np.array([params["att_kp_mult"]]),
        "rate_kp_mult": np.array([params["rate_kp_mult"]]),
        "lever_mult": np.array([params["lever_mult"]]),
        "sensor_angle_deg": np.array([params["sensor_angle_deg"]]),
        "sensor_rate_dps": np.array([params["sensor_rate_dps"]]),
        "bias_angle_deg": np.array([params["bias_angle_deg"]]),
        "bias_rate_dps": np.array([params["bias_rate_dps"]]),
        "wind_up": np.array([params["wind_up"]]),
        "torque_noise": np.array([params["torque_noise"]]),
        "delay_s": np.array([params["delay_s"]]),
        "thrust_loss": np.ones(n), "servo_jam": np.zeros(n, dtype=bool),
        "init_angle_deg": np.zeros(n), "init_rate_dps": np.zeros(n),
    }
    m = simulate(model, p, np.random.default_rng(rng_seed), duration_s=20.0,
                 variant=variant, **osc)
    ang = float(m["max_ang_deg"][0]); h = float(m["final_h"][0])
    finite = bool(np.isfinite(ang) and np.isfinite(h))
    severity = (ang / 60.0 + max(0.0, -h) / 100.0
                + max(0.0, h - 30.0) / 100.0) if finite else 100.0
    return {"max_ang_deg": ang, "final_h": h, "finite": finite,
            "severity": severity, "fail": int(m["fail"][0])}


def search(model, variant: str, trials: int, seed: int, **osc):
    def objective(trial: optuna.Trial) -> float:
        params = {
            "mass_mult": trial.suggest_float("mass_mult", 0.7, 1.3),
            "i_pitch": trial.suggest_float("i_pitch", 0.55, 1.7),
            "i_yaw": trial.suggest_float("i_yaw", 0.55, 1.7),
            "thrust_mult": trial.suggest_float("thrust_mult", 0.75, 1.25),
            "tau_mult": trial.suggest_float("tau_mult", 0.4, 2.5),
            "att_kp_mult": trial.suggest_float("att_kp_mult", 0.6, 1.4),
            "rate_kp_mult": trial.suggest_float("rate_kp_mult", 0.6, 1.4),
            "lever_mult": trial.suggest_float("lever_mult", 0.5, 1.6),
            "sensor_angle_deg": trial.suggest_float("sensor_angle_deg", 0.0, 6.0),
            "sensor_rate_dps": trial.suggest_float("sensor_rate_dps", 0.0, 15.0),
            "bias_angle_deg": trial.suggest_float("bias_angle_deg", -6.0, 6.0),
            "bias_rate_dps": trial.suggest_float("bias_rate_dps", -6.0, 6.0),
            "wind_up": trial.suggest_float("wind_up", -4.5, 4.5),
            "torque_noise": trial.suggest_float("torque_noise", 0.0, 0.5),
            "delay_s": trial.suggest_float("delay_s", 0.0, 0.2),
        }
        r = one_case(model, seed, variant, params, **osc)
        trial.set_user_attr("max_ang_deg", r["max_ang_deg"])
        trial.set_user_attr("final_h", r["final_h"])
        trial.set_user_attr("fail", r["fail"])
        return -r["severity"]

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    study = optuna.create_study(
        direction="minimize",
        sampler=optuna.samplers.TPESampler(multivariate=True, seed=seed))
    study.optimize(objective, n_trials=trials)
    best = study.best_trial
    params = dict(best.params)
    # 多噪声种子复核
    reps = [one_case(model, seed + 1000 + i, variant, params, **osc)
            for i in range(8)]
    return {
        "variant": variant,
        "best_params": params,
        "best_trial": {"max_ang_deg": best.user_attrs["max_ang_deg"],
                       "final_h": best.user_attrs["final_h"],
                       "fail": best.user_attrs["fail"],
                       "severity": -best.value},
        "replicates": reps,
        "replicate_max_ang": [r["max_ang_deg"] for r in reps],
        "replicate_final_h": [r["final_h"] for r in reps],
        "replicate_fail_rate": float(np.mean([r["fail"] for r in reps])),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="sim-data/ml-2026-10-24")
    ap.add_argument("--trials", type=int, default=250)
    ap.add_argument("--seed", type=int, default=20261029)
    args = ap.parse_args()
    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)
    model = MODELS["TandemVec"]
    result = {"trials": args.trials, "seed": args.seed,
              "note": "L1 对抗搜索；非概率，不能外推实机极限", "searches": {}}
    # 注意：fast_mc 的选择器变体名是 "osc"；旧代码把 "tuned" 直接传给
    # simulate，会落入 fixed 分支，使 tuned 与 fixed 结果逐位相同。
    for label, variant, osc in (
            ("fixed", "fixed", {}),
            ("accel_fb", "accel_fb", dict(accel_fb_gain=0.8,
                                          accel_fb_tau_s=0.02)),
            ("osc_tuned", "osc", dict(osc_window_s=0.15, osc_threshold=4,
                                      osc_decay=0.85, osc_recovery=1.01,
                                      osc_floor=0.45))):
        print("search", label)
        result["searches"][label] = search(model, variant, args.trials,
                                           args.seed, **osc)
        print(label, result["searches"][label]["best_trial"])
    (out / "optuna_worstcase.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"saved: {out/'optuna_worstcase.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
