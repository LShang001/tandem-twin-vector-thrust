"""巨型 MC 结果的降维分析：分故障桶、推力分桶、最坏案例与 1% 分层样本。

输入目录需含 summary.json 与 samples.csv（run_mega.py 输出）。
输出写回同目录：analysis.json / samples_1pct.csv / worst_cases.csv，
可选 --compare-dir 生成 compare.json（两组 campaign 的通过率/失败率/P95 对比）。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

THRUST_BINS = [0.7, 0.9, 1.05, 1.15, 1.31]
BIN_LABELS = [f"{a:.2f}-{b:.2f}" for a, b in zip(THRUST_BINS[:-1], THRUST_BINS[1:])]


def pct(series: pd.Series, q: float) -> float:
    return float(np.percentile(series.to_numpy(), q))


def bucket_stats(df: pd.DataFrame) -> dict:
    fail_rate = float(df["fail"].mean())
    return {
        "n": int(len(df)),
        "pass_rate": float(df["ok"].mean()),
        "fail_rate": fail_rate,
        "bounded_rate": 1.0 - fail_rate,
        "ang_p95": pct(df["max_ang_deg"], 95),
        "h_p05": pct(df["final_h"], 5),
        "h_p95": pct(df["final_h"], 95),
    }


def analyze_vehicle(df: pd.DataFrame) -> dict:
    fault = (df["thrust_loss"] < 1.0) | df["servo_jam"]
    out = {
        "all": bucket_stats(df),
        "no_fault": bucket_stats(df[~fault]),
        "with_fault": bucket_stats(df[fault]),
        "thrust_bins": [],
    }
    bins = pd.cut(df["thrust_mult"], bins=THRUST_BINS, labels=BIN_LABELS, include_lowest=True)
    for label in BIN_LABELS:
        sub = df[bins == label]
        if len(sub):
            out["thrust_bins"].append(
                {
                    "bin": label,
                    "n": int(len(sub)),
                    "fail": float(sub["fail"].mean()),
                    "ok": float(sub["ok"].mean()),
                }
            )
    for key in ("thrust_loss", "servo_jam"):
        mask = df["thrust_loss"] < 1.0 if key == "thrust_loss" else df["servo_jam"]
        sub = df[mask]
        out[f"only_{key}"] = bucket_stats(sub) if len(sub) else {"n": 0}
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True, help="campaign 目录（含 summary.json/samples.csv）")
    ap.add_argument("--compare-dir", default=None, help="参照 campaign 目录")
    ap.add_argument("--seed", type=int, default=20261024)
    args = ap.parse_args()

    src = Path(args.dir)
    summary = json.loads((src / "summary.json").read_text(encoding="utf-8"))
    df = pd.read_csv(src / "samples.csv")
    analysis = {}
    samples = []
    worst = []

    rng = np.random.default_rng(args.seed)
    for vehicle, group in df.groupby("vehicle", sort=False):
        analysis[vehicle] = analyze_vehicle(group)
        # 1% 分层样本：保持通过/失败/边缘比例
        n_sample = max(1, len(group) // 100)
        idx = rng.choice(group.index.to_numpy(), size=n_sample, replace=False)
        samples.append(group.loc[idx])
        severity = group.assign(
            _sev=group["fail"] * 1000 + group["max_ang_deg"] + (group["final_h"] - 10.0).abs()
        ).sort_values("_sev", ascending=False)
        worst.append(severity.drop(columns="_sev").head(100))

    analysis["meta"] = {
        "dir": src.name,
        "variant": summary.get("variant", "fixed"),
        "cases_per_vehicle": summary.get("cases_per_vehicle"),
        "duration_s": summary.get("duration_s"),
        "nominal": summary.get("nominal"),
        "note": "L1 降阶 MC；pass/fail 之外为边缘带（既未通过也未判失败）",
    }
    (src / "analysis.json").write_text(
        json.dumps(analysis, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    pd.concat(samples).to_csv(src / "samples_1pct.csv", index=False)
    pd.concat(worst).to_csv(src / "worst_cases.csv", index=False)

    if args.compare_dir:
        ref_dir = Path(args.compare_dir)
        ref = json.loads((ref_dir / "summary.json").read_text(encoding="utf-8"))
        cmp = {"reference": ref_dir.name, "candidate": src.name, "vehicles": {}}
        for vehicle in summary["vehicles"]:
            a = ref["vehicles"][vehicle]
            b = summary["vehicles"][vehicle]
            cmp["vehicles"][vehicle] = {
                "pass_rate": [a["pass_rate"], b["pass_rate"]],
                "fail_rate": [a["fail_rate"], b["fail_rate"]],
                "max_ang_p95": [a["max_ang_p95"], b["max_ang_p95"]],
                "final_h_p05": [a["final_h_p05"], b["final_h_p05"]],
                "final_h_p95": [a["final_h_p95"], b["final_h_p95"]],
                "delta_pass_pp": (b["pass_rate"] - a["pass_rate"]) * 100.0,
                "delta_fail_pp": (b["fail_rate"] - a["fail_rate"]) * 100.0,
                "delta_ang_p95_deg": b["max_ang_p95"] - a["max_ang_p95"],
            }
        (src / "compare.json").write_text(
            json.dumps(cmp, indent=2, ensure_ascii=False), encoding="utf-8"
        )

    for vehicle, stats in analysis.items():
        if vehicle == "meta":
            continue
        print(
            f"{vehicle}: n={stats['all']['n']} "
            f"pass={stats['all']['pass_rate']*100:.2f}% "
            f"fail={stats['all']['fail_rate']*100:.2f}% "
            f"angP95={stats['all']['ang_p95']:.2f}° "
            f"| no_fault fail={stats['no_fault']['fail_rate']*100:.2f}% "
            f"| fault fail={stats['with_fault']['fail_rate']*100:.2f}%"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
