"""ml_failure_surrogate.py — 用 1M MC 数据训练失败代理模型并做可解释性分析。

@file    ml_failure_surrogate.py
@module  仿真/三机型快速 MC（ML 失败代理）
@brief   HistGradientBoosting 失败分类器 + 校准 + 置换重要性 + 延迟×增益风险图；
         分别训练 fixed 与 tuned 数据，比较风险结构和特征重要性
@details 目标变量为 fast_mc 的 fail（非有限/撞地/飞出 30 m/角度 >60°）。
         数据来自 run_mega.py 的 1M TandemVec 独立种子；随机 70/15/15 分层切分。
         校准在验证集拟合 isotonic，再在测试集评估；置换重要性用 ROC-AUC 评分。
@theory  树集成学习失败概率 P(fail|x)，x 为参数/噪声/延迟/故障特征；
         仅用于敏感度与风险图，不作因果推断。
         @ref VER-003 §3.8、CTL-003 §2
@frame   N/A（表格特征）
@units   概率、AUC、Brier、deg、s
@note    ★ L1 模型内代理，不是实机失效概率；外推到采样范围外无效。
@test    py -3.12 ml_failure_surrogate.py --fixed <fixed.csv> --tuned <tuned.csv>
@see     fast_mc.py、run_mega.py、VER-003
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.calibration import calibration_curve
from sklearn.ensemble import HistGradientBoostingClassifier
from sklearn.inspection import permutation_importance
from sklearn.isotonic import IsotonicRegression
from sklearn.metrics import (average_precision_score, brier_score_loss,
                             confusion_matrix, f1_score, log_loss,
                             precision_score, recall_score, roc_auc_score)
from sklearn.model_selection import train_test_split

RAW = ["mass_mult", "i_pitch", "i_yaw", "thrust_mult", "tau_mult",
       "att_kp_mult", "rate_kp_mult", "lever_mult", "sensor_angle_deg",
       "sensor_rate_dps", "bias_angle_deg", "bias_rate_dps", "wind_up",
       "torque_noise", "delay_s", "thrust_loss", "servo_jam"]


def add_features(df: pd.DataFrame) -> pd.DataFrame:
    x = df.copy()
    x["fault"] = ((x["thrust_loss"] < 1.0) | x["servo_jam"].astype(bool)).astype(np.int8)
    x["thrust_eff"] = x["thrust_mult"] * x["thrust_loss"]
    x["delay_ms"] = x["delay_s"] * 1000.0
    x["delay_x_rate"] = x["delay_s"] * x["rate_kp_mult"]
    x["delay_x_att"] = x["delay_s"] * x["att_kp_mult"]
    x["gain_product"] = x["att_kp_mult"] * x["rate_kp_mult"]
    x["authority"] = (x["thrust_mult"] * x["lever_mult"] /
                      (x["mass_mult"] * x["i_pitch"]))
    return x


def features(df: pd.DataFrame, no_fault: bool) -> list[str]:
    drop = {"fail"}
    if no_fault:
        drop |= {"fault", "thrust_loss", "servo_jam", "thrust_eff"}
    return [c for c in df.columns if c not in drop]


def fit_model(x: pd.DataFrame, y: pd.Series, seed: int) -> HistGradientBoostingClassifier:
    model = HistGradientBoostingClassifier(
        max_iter=300, learning_rate=0.06, max_leaf_nodes=31,
        l2_regularization=1.0, early_stopping=True, validation_fraction=0.1,
        n_iter_no_change=20, class_weight="balanced", random_state=seed)
    model.fit(x, y)
    return model


def metrics(y: np.ndarray, prob: np.ndarray, threshold: float) -> dict:
    pred = (prob >= threshold).astype(int)
    tn, fp, fn, tp = confusion_matrix(y, pred, labels=[0, 1]).ravel()
    return {
        "n": int(len(y)), "positives": int(y.sum()),
        "auc": float(roc_auc_score(y, prob)),
        "average_precision": float(average_precision_score(y, prob)),
        "brier": float(brier_score_loss(y, prob)),
        "log_loss": float(log_loss(y, prob)),
        "threshold": float(threshold),
        "precision": float(precision_score(y, pred, zero_division=0)),
        "recall": float(recall_score(y, pred, zero_division=0)),
        "f1": float(f1_score(y, pred, zero_division=0)),
        "confusion": {"tn": int(tn), "fp": int(fp), "fn": int(fn), "tp": int(tp)},
    }


def choose_threshold(y: np.ndarray, prob: np.ndarray) -> float:
    best_t, best_f1 = 0.5, -1.0
    for t in np.linspace(0.01, 0.99, 99):
        f1 = f1_score(y, (prob >= t).astype(int), zero_division=0)
        if f1 > best_f1:
            best_t, best_f1 = float(t), float(f1)
    return best_t


def risk_grid(model, base: pd.DataFrame, feature_names: list[str],
              delay_ms: np.ndarray, rate_gain: np.ndarray,
              calibrator=None) -> pd.DataFrame:
    rows = []
    for d in delay_ms:
        for r in rate_gain:
            rows.append({"delay_ms": float(d), "rate_kp_mult": float(r)})
    grid = pd.DataFrame(rows)
    b = base.drop(columns=["delay_ms", "rate_kp_mult", "delay_s",
                           "delay_x_rate", "delay_x_att", "gain_product"],
                  errors="ignore")
    merged = pd.concat([b.iloc[[0] * len(grid)].reset_index(drop=True), grid], axis=1)
    merged["delay_s"] = merged["delay_ms"] / 1000.0
    merged["delay_x_rate"] = merged["delay_s"] * merged["rate_kp_mult"]
    merged["delay_x_att"] = merged["delay_s"] * merged["att_kp_mult"]
    merged["gain_product"] = merged["att_kp_mult"] * merged["rate_kp_mult"]
    raw = model.predict_proba(merged[feature_names])[:, 1]
    if calibrator is not None:
        raw = calibrator.predict(raw)
    merged["p_fail"] = raw
    return merged[["delay_ms", "rate_kp_mult", "p_fail"]]


def delay_calibration(y: np.ndarray, prob: np.ndarray,
                      delay_s: np.ndarray) -> list[dict]:
    bins = [0.0, 0.03, 0.06, 0.09, 0.12, 0.15, np.inf]
    labels = ["<30ms", "30-60ms", "60-90ms", "90-120ms", "120-150ms", ">=150ms"]
    out = []
    for lo, hi, lab in zip(bins[:-1], bins[1:], labels):
        m = (delay_s >= lo) & (delay_s < hi)
        if not m.any():
            continue
        out.append({"bin": lab, "n": int(m.sum()),
                    "mean_pred": float(prob[m].mean()),
                    "observed": float(y[m].mean())})
    return out


def write_analysis(name: str, df: pd.DataFrame, no_fault: bool,
                   out: Path, seed: int) -> dict:
    if no_fault:
        df = df[df["fault"] == 0].reset_index(drop=True)
    y = df["fail"].astype(int)
    x = df[features(df, no_fault)]
    x_train, x_tmp, y_train, y_tmp = train_test_split(
        x, y, test_size=0.30, stratify=y, random_state=seed)
    x_val, x_test, y_val, y_test = train_test_split(
        x_tmp, y_tmp, test_size=0.50, stratify=y_tmp, random_state=seed + 1)
    model = fit_model(x_train, y_train, seed)
    p_val = model.predict_proba(x_val)[:, 1]
    p_test = model.predict_proba(x_test)[:, 1]
    threshold = choose_threshold(y_val.to_numpy(), p_val)
    raw = metrics(y_test.to_numpy(), p_test, threshold)
    cal_by_delay = {"raw": delay_calibration(y_test.to_numpy(), p_test,
                                             x_test["delay_s"].to_numpy())}

    iso = IsotonicRegression(out_of_bounds="clip")
    iso.fit(p_val, y_val.to_numpy())
    p_test_cal = iso.predict(p_test)
    cal = metrics(y_test.to_numpy(), p_test_cal, threshold)
    cal_by_delay["calibrated"] = delay_calibration(
        y_test.to_numpy(), p_test_cal, x_test["delay_s"].to_numpy())

    # 置换重要性：测试集抽样 50k，AUC 评分
    rng = np.random.default_rng(seed)
    idx = rng.choice(len(x_test), size=min(50000, len(x_test)), replace=False)
    perm = permutation_importance(model, x_test.iloc[idx], y_test.iloc[idx],
                                  scoring="roc_auc", n_repeats=4,
                                  random_state=seed, n_jobs=-1)
    imp = sorted(zip(x_test.columns, perm.importances_mean),
                 key=lambda kv: kv[1], reverse=True)
    importance = [{"feature": k, "auc_drop": float(v)} for k, v in imp]

    base = x.median(numeric_only=True).to_frame().T
    if not no_fault:
        base["servo_jam"] = False
        base["thrust_loss"] = 1.0
        base["fault"] = 0
        base["thrust_eff"] = base["thrust_mult"] * base["thrust_loss"]
    base["delay_s"] = 0.05
    base["delay_ms"] = 50.0
    base["delay_x_rate"] = base["delay_s"] * base["rate_kp_mult"]
    base["delay_x_att"] = base["delay_s"] * base["att_kp_mult"]
    base["gain_product"] = base["att_kp_mult"] * base["rate_kp_mult"]
    base = base.reindex(columns=x_test.columns, fill_value=0)
    grid = risk_grid(model, base, list(x_test.columns),
                     np.linspace(0, 200, 41), np.linspace(0.6, 1.4, 33),
                     calibrator=iso)
    grid.to_csv(out / f"risk_map_{name}.csv", index=False)

    fig, ax = plt.subplots(figsize=(7, 4.5))
    im = ax.pcolormesh(grid.pivot(index="rate_kp_mult", columns="delay_ms",
                                  values="p_fail"), shading="auto",
                       cmap="magma", vmin=0, vmax=max(0.5, grid.p_fail.max()))
    ax.set_xlabel("delay (ms)")
    ax.set_ylabel("rate_kp multiplier")
    ax.set_title(f"P(fail) surrogate — {name}")
    fig.colorbar(im, ax=ax, label="P(fail)")
    fig.tight_layout()
    fig.savefig(out / f"risk_map_{name}.png", dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(5, 5))
    for p, lab in ((p_test, "raw"), (p_test_cal, "isotonic")):
        frac, mean = calibration_curve(y_test, p, n_bins=15, strategy="quantile")
        ax.plot(mean, frac, marker=".", label=lab)
    ax.plot([0, 1], [0, 1], "--", color="gray", label="ideal")
    ax.set_xlabel("predicted P(fail)")
    ax.set_ylabel("observed failure rate")
    ax.set_title(f"Calibration — {name}")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / f"calibration_{name}.png", dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6, 4))
    xs = np.arange(len(cal_by_delay["calibrated"]))
    lab = [d["bin"] for d in cal_by_delay["calibrated"]]
    for key, color in (("raw", "tab:gray"), ("calibrated", "tab:red")):
        d = cal_by_delay[key]
        ax.plot(xs, [q["mean_pred"] for q in d], marker="o", label=f"{key} pred",
                color=color)
    ax.plot(xs, [q["observed"] for q in cal_by_delay["calibrated"]],
            marker="s", linestyle="--", label="observed", color="black")
    ax.set_xticks(xs); ax.set_xticklabels(lab, rotation=30)
    ax.set_ylabel("P(fail)")
    ax.set_title(f"Calibration by delay — {name}")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / f"calibration_by_delay_{name}.png", dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 5))
    top = importance[:12][::-1]
    ax.barh([d["feature"] for d in top], [d["auc_drop"] for d in top])
    ax.set_xlabel("AUC drop (permutation)")
    ax.set_title(f"Permutation importance — {name}")
    fig.tight_layout()
    fig.savefig(out / f"importance_{name}.png", dpi=140)
    plt.close(fig)

    return {"metrics_raw": raw, "metrics_calibrated": cal,
            "calibration_by_delay": cal_by_delay,
            "importance": importance, "model_iter": int(model.n_iter_)}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--fixed", required=True, help="1M fixed samples.csv")
    ap.add_argument("--tuned", required=True, help="1M tuned samples.csv")
    ap.add_argument("--out", default="sim-data/ml-2026-10-24")
    ap.add_argument("--seed", type=int, default=20261024)
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    result = {"seed": args.seed, "note": "L1 模型内失败代理；不是实机概率",
              "models": {}}
    for name, path in (("fixed", args.fixed), ("tuned", args.tuned)):
        df = pd.read_csv(path, usecols=RAW + ["fail"])
        df = add_features(df)
        for no_fault in (False, True):
            label = name + ("_no_fault" if no_fault else "_all")
            sub = df[df["fault"] == 0] if no_fault else df
            print(f"{label}: {len(sub)} rows, fail={sub.fail.mean()*100:.2f}%")
            result["models"][label] = write_analysis(label, df, no_fault,
                                                     out, args.seed)
    (out / "metrics.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"saved: {out/'metrics.json'} (+ risk maps/calibration/importance)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
