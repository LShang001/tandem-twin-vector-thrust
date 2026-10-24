"""ml_active_learning.py — 不确定度主动学习 vs 随机采样（失败边界代理）。

@file    ml_active_learning.py
@module  仿真/三机型快速 MC（主动学习）
@brief   在无故障 TandemVec MC 数据上，比较 entropy 不确定度采样与随机采样在
         相同标签预算下的 AUC/AP/Brier，评估稀有失败边界的标签效率
@details 数据：1M fixed 的 no-fault 子集；固定 150k 测试集，其余为未标注池。
         初始 3k 标签，每轮各加 3k，共 4 轮；多种子重复取均值/标准差。
@theory  主动学习 query-by-uncertainty：优先标注模型最不确定的样本，
         在稀有事件边界上应比随机采样更快降低泛化误差。
         @ref VER-003 §3.8、ml_failure_surrogate.py
@frame   N/A（表格特征）
@units   标签数、AUC、AP、Brier
@note    ★ L1 模型内实验；不是实机失效概率，也不是通用主动学习结论。
@test    py -3.12 ml_active_learning.py --data <fixed-1m.csv>
@see     ml_failure_surrogate.py、fast_mc.py、VER-003
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
from sklearn.ensemble import HistGradientBoostingClassifier
from sklearn.metrics import average_precision_score, brier_score_loss, roc_auc_score
from sklearn.model_selection import train_test_split

from ml_failure_surrogate import RAW, add_features, features


def fit(x: pd.DataFrame, y: pd.Series, seed: int) -> HistGradientBoostingClassifier:
    model = HistGradientBoostingClassifier(
        max_iter=150, learning_rate=0.08, max_leaf_nodes=31,
        l2_regularization=1.0, early_stopping=True, validation_fraction=0.1,
        n_iter_no_change=15, class_weight="balanced", random_state=seed)
    model.fit(x, y)
    return model


def evaluate(model, x: pd.DataFrame, y: pd.Series) -> dict:
    p = model.predict_proba(x)[:, 1]
    return {"auc": float(roc_auc_score(y, p)),
            "average_precision": float(average_precision_score(y, p)),
            "brier": float(brier_score_loss(y, p))}


def run_seed(x_pool, y_pool, x_test, y_test, seed: int,
             initial: int, step: int, rounds: int) -> list[dict]:
    rows = []
    init_idx, rest_idx = train_test_split(
        np.arange(len(x_pool)), train_size=initial, stratify=y_pool,
        random_state=seed)
    remaining = np.setdiff1d(np.arange(len(x_pool)), init_idx, assume_unique=False)
    for strategy in ("random", "uncertainty"):
        labeled = init_idx.copy()
        rem = remaining.copy()
        rng = np.random.default_rng(seed)
        model = fit(x_pool.iloc[labeled], y_pool.iloc[labeled], seed)
        rows.append({"seed": seed, "strategy": strategy, "n_labels": len(labeled),
                     **evaluate(model, x_test, y_test)})
        for _ in range(rounds):
            if strategy == "random":
                pick = rng.choice(rem, size=step, replace=False)
            else:
                p = model.predict_proba(x_pool.iloc[rem])[:, 1]
                p = np.clip(p, 1e-6, 1 - 1e-6)
                ent = -(p * np.log(p) + (1 - p) * np.log(1 - p))
                pick = rem[np.argsort(ent)[-step:]]
            labeled = np.concatenate([labeled, pick])
            rem = np.setdiff1d(rem, pick, assume_unique=False)
            model = fit(x_pool.iloc[labeled], y_pool.iloc[labeled], seed)
            rows.append({"seed": seed, "strategy": strategy,
                         "n_labels": len(labeled),
                         **evaluate(model, x_test, y_test)})
    return rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="1M fixed samples.csv")
    ap.add_argument("--out", default="sim-data/ml-2026-10-24")
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--initial", type=int, default=3000)
    ap.add_argument("--step", type=int, default=3000)
    ap.add_argument("--rounds", type=int, default=4)
    ap.add_argument("--pool", type=int, default=970000)
    ap.add_argument("--test", type=int, default=150000)
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    df = pd.read_csv(args.data, usecols=RAW + ["fail"])
    df = add_features(df)
    df = df[df["fault"] == 0].reset_index(drop=True)
    if len(df) > args.pool + args.test:
        df = df.sample(n=args.pool + args.test, random_state=20261024).reset_index(drop=True)
    x = df[features(df, no_fault=True)]
    y = df["fail"].astype(int)
    x_pool, x_test, y_pool, y_test = train_test_split(
        x, y, test_size=args.test, stratify=y, random_state=20261024)
    print(f"pool={len(x_pool)} test={len(x_test)} positives="
          f"{int(y_pool.sum())}/{int(y_test.sum())}")

    rows = []
    for i in range(args.seeds):
        seed = 20261024 + i
        print(f"seed {seed} ...")
        rows.extend(run_seed(x_pool, y_pool, x_test, y_test, seed,
                             args.initial, args.step, args.rounds))
    dfr = pd.DataFrame(rows)
    dfr.to_csv(out / "active_learning_curve.csv", index=False)
    agg = dfr.groupby(["strategy", "n_labels"]).agg(
        auc_mean=("auc", "mean"), auc_std=("auc", "std"),
        ap_mean=("average_precision", "mean"), ap_std=("average_precision", "std"),
        brier_mean=("brier", "mean"), brier_std=("brier", "std")).reset_index()
    agg.to_csv(out / "active_learning_summary.csv", index=False)

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    for strategy, color in (("random", "tab:gray"), ("uncertainty", "tab:red")):
        s = agg[agg.strategy == strategy]
        axes[0].errorbar(s.n_labels, s.auc_mean, yerr=s.auc_std, marker="o",
                         capsize=3, label=strategy, color=color)
        axes[1].errorbar(s.n_labels, s.ap_mean, yerr=s.ap_std, marker="o",
                         capsize=3, label=strategy, color=color)
    axes[0].set_xlabel("labels"); axes[0].set_ylabel("ROC-AUC")
    axes[0].set_title("Failure-boundary AUC vs label budget")
    axes[1].set_xlabel("labels"); axes[1].set_ylabel("Average precision")
    axes[1].set_title("Rare-failure AP vs label budget")
    for ax in axes:
        ax.legend(); ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "active_learning.png", dpi=140)
    plt.close(fig)

    final = agg[agg.n_labels == agg.n_labels.max()].set_index("strategy")
    summary = {
        "data": args.data, "pool": int(len(x_pool)), "test": int(len(x_test)),
        "seeds": args.seeds, "initial": args.initial, "step": args.step,
        "rounds": args.rounds,
        "final": final.to_dict(orient="index"),
        "note": "L1 模型内主动学习实验；AP 为稀有失败主指标",
    }
    (out / "active_learning.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print(agg.to_string(index=False))
    print(f"saved: {out/'active_learning.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
