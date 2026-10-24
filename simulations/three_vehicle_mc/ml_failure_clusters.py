"""ml_failure_clusters.py — TandemVec 失效模式聚类与 tuned 投影。

@file    ml_failure_clusters.py
@module  仿真/三机型快速 MC（失效模式聚类）
@brief   对 1M fixed 的失败 case 做 KMeans 聚类（仅输入特征），用轮廓系数选 k；
         再统计每簇故障类型/严重度，并把 tuned 失败 case 投影到同一空间
@details 输入特征不含 max_ang/final_h，避免按结果严重度聚类；结果只作
         失效模式探索，不作因果归因。
@theory  无监督聚类把高维失败参数空间压缩为若干可解释输入区域；
         轮廓系数评估簇内紧致/簇间分离。
         @ref VER-003 §3.8
@frame   N/A（标准化表格特征）
@units   无量纲标准化特征；结果统计用 deg、m、s
@note    ★ L1 模型内探索；簇不是物理失效机理，不能直接映射到实机故障树。
@test    py -3.12 ml_failure_clusters.py --fixed <1m-fixed.csv> --tuned <1m-tuned.csv>
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
from sklearn.cluster import KMeans
from sklearn.decomposition import PCA
from sklearn.metrics import silhouette_score
from sklearn.preprocessing import StandardScaler

from ml_failure_surrogate import RAW

CLUSTER_FEATURES = ["delay_s", "att_kp_mult", "rate_kp_mult", "thrust_loss",
                    "servo_jam", "wind_up", "mass_mult", "thrust_mult",
                    "sensor_rate_dps", "bias_angle_deg"]


def fault_label(df: pd.DataFrame) -> pd.Series:
    tl = df["thrust_loss"] < 1.0
    sj = df["servo_jam"].astype(bool)
    out = pd.Series("none", index=df.index)
    out[tl] = "thrust_loss"
    out[sj] = "servo_jam"
    out[tl & sj] = "both"
    return out


def summarize(df: pd.DataFrame, cluster: np.ndarray, centers: pd.DataFrame) -> list[dict]:
    rows = []
    for k in range(len(centers)):
        sub = df[cluster == k]
        if not len(sub):
            continue
        rows.append({
            "cluster": int(k),
            "n": int(len(sub)),
            "share": float(len(sub) / len(df)),
            "fault_none": float((fault_label(sub) == "none").mean()),
            "fault_thrust_loss": float((fault_label(sub) == "thrust_loss").mean()),
            "fault_servo_jam": float((fault_label(sub) == "servo_jam").mean()),
            "median_delay_ms": float(sub.delay_s.median() * 1000),
            "median_att_kp": float(sub.att_kp_mult.median()),
            "median_rate_kp": float(sub.rate_kp_mult.median()),
            "median_wind_up": float(sub.wind_up.median()),
            "median_max_ang_deg": float(sub.max_ang_deg.median()),
            "p95_max_ang_deg": float(np.percentile(sub.max_ang_deg, 95)),
            "median_final_h": float(sub.final_h.median()),
            "p05_final_h": float(np.percentile(sub.final_h, 5)),
            "center": {c: float(centers.loc[k, c]) for c in centers.columns},
        })
    return rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--fixed", required=True)
    ap.add_argument("--tuned", required=True)
    ap.add_argument("--out", default="sim-data/ml-2026-10-24")
    ap.add_argument("--kmax", type=int, default=7)
    ap.add_argument("--seed", type=int, default=20261024)
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    fixed = pd.read_csv(args.fixed, usecols=["case"] + RAW + ["fail", "max_ang_deg", "final_h"])
    tuned = pd.read_csv(args.tuned, usecols=["case"] + RAW + ["fail", "max_ang_deg", "final_h"])
    fixed = fixed[fixed["fail"] == 1].reset_index(drop=True)
    tuned = tuned[tuned["fail"] == 1].reset_index(drop=True)
    xf = fixed[CLUSTER_FEATURES].astype(float)
    scaler = StandardScaler().fit(xf)
    zf = scaler.transform(xf)
    zt = scaler.transform(tuned[CLUSTER_FEATURES].astype(float))

    rng = np.random.default_rng(args.seed)
    idx = rng.choice(len(zf), size=min(10000, len(zf)), replace=False)
    sil = {}
    for k in range(2, args.kmax + 1):
        km = KMeans(n_clusters=k, n_init=10, random_state=args.seed).fit(zf)
        sil[k] = float(silhouette_score(zf[idx], km.labels_[idx]))
    best_k = max(sil, key=sil.get)
    use_k = best_k if best_k >= 3 else min(3, args.kmax)
    print("silhouette:", {k: round(v, 4) for k, v in sil.items()},
          "best_k=", best_k, "use_k=", use_k)

    km = KMeans(n_clusters=use_k, n_init=20, random_state=args.seed).fit(zf)
    centers_z = pd.DataFrame(km.cluster_centers_, columns=CLUSTER_FEATURES)
    centers_raw = pd.DataFrame(scaler.inverse_transform(centers_z),
                               columns=CLUSTER_FEATURES)
    clusters_fixed = km.labels_
    clusters_tuned = km.predict(zt)

    summary_fixed = summarize(fixed, clusters_fixed, centers_raw)
    summary_tuned = summarize(tuned, clusters_tuned, centers_raw)
    # 只保留 fixed 中存在的簇；tuned 投影计数对齐
    tuned_counts = {int(k): int((clusters_tuned == k).sum()) for k in range(use_k)}
    fixed_counts = {int(k): int((clusters_fixed == k).sum()) for k in range(use_k)}

    fixed_out = fixed[["case", "max_ang_deg", "final_h"]].copy()
    fixed_out["cluster"] = clusters_fixed
    fixed_out["fault"] = fault_label(fixed).to_numpy()
    fixed_out.to_csv(out / "failure_clusters_fixed.csv", index=False)
    tuned_out = tuned[["case", "max_ang_deg", "final_h"]].copy()
    tuned_out["cluster"] = clusters_tuned
    tuned_out["fault"] = fault_label(tuned).to_numpy()
    tuned_out.to_csv(out / "failure_clusters_tuned.csv", index=False)
    centers_raw.to_csv(out / "failure_cluster_centers.csv", index=False)

    pca = PCA(n_components=2, random_state=args.seed).fit(zf)
    z2 = pca.transform(zf)
    z2t = pca.transform(zt)
    fig, ax = plt.subplots(figsize=(7, 5.5))
    for k in range(use_k):
        m = clusters_fixed == k
        ax.scatter(z2[m, 0], z2[m, 1], s=2, alpha=0.25,
                   label=f"C{k} (n={fixed_counts[k]})")
    ax.scatter(z2t[:, 0], z2t[:, 1], s=2, alpha=0.20, c="black",
               label=f"tuned fails (n={len(zt)})", marker="x")
    ax.set_xlabel(f"PC1 ({pca.explained_variance_ratio_[0]*100:.1f}%)")
    ax.set_ylabel(f"PC2 ({pca.explained_variance_ratio_[1]*100:.1f}%)")
    ax.set_title("TandemVec failure modes (fixed KMeans; tuned projected)")
    ax.legend(markerscale=4, fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "failure_clusters.png", dpi=140)
    plt.close(fig)

    result = {
        "k": int(use_k), "silhouette": sil, "best_k": int(best_k),
        "fixed_failures": int(len(fixed)), "tuned_failures": int(len(tuned)),
        "fixed_counts": fixed_counts, "tuned_counts": tuned_counts,
        "clusters_fixed": summary_fixed, "clusters_tuned": summary_tuned,
        "pca_explained": pca.explained_variance_ratio_.tolist(),
        "note": "L1 模型内探索；簇是输入区域，不是物理失效机理/因果结论",
    }
    (out / "failure_clusters.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"fixed clusters: {fixed_counts}")
    print(f"tuned clusters: {tuned_counts}")
    print(f"saved: {out/'failure_clusters.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
