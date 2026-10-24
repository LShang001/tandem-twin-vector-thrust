"""同种子两套 MC 的逐 case A/B：确认参数样本一致，统计通过/失败转移。

用法：
  py -3.12 compare_casewise.py \
      --baseline /tmp/mega-fixed-300k-2026-10-24 \
      --candidate sim-data/mega-osc-300k-2026-10-24 \
      --out sim-data/mega-osc-300k-2026-10-24/casewise.json

输入目录需含 samples.csv。仅在参数逐 case 完全一致时给出转移统计，
否则直接报错（防止把不同样本的差异当成策略收益）。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

PARAM_COLS = [
    "mass_mult", "i_pitch", "i_yaw", "thrust_mult", "tau_mult",
    "att_kp_mult", "rate_kp_mult", "lever_mult",
    "sensor_angle_deg", "sensor_rate_dps",
    "bias_angle_deg", "bias_rate_dps", "wind_up",
    "torque_noise", "delay_s", "thrust_loss", "servo_jam",
]


def label(df: pd.DataFrame) -> pd.Series:
    out = pd.Series("marginal", index=df.index)
    out[df["ok"] == 1] = "pass"
    out[df["fail"] == 1] = "fail"
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--baseline", required=True)
    ap.add_argument("--candidate", required=True)
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    base = pd.read_csv(Path(args.baseline) / "samples.csv")
    cand = pd.read_csv(Path(args.candidate) / "samples.csv")
    keys = ["vehicle", "case"]
    merged = base.merge(cand, on=keys, suffixes=("_base", "_cand"), validate="one_to_one")
    if len(merged) != len(base) or len(merged) != len(cand):
        raise SystemExit("case 集合不一致，无法逐 case 对比")

    for col in PARAM_COLS:
        a = merged[f"{col}_base"].to_numpy()
        b = merged[f"{col}_cand"].to_numpy()
        if a.dtype == bool or b.dtype == bool:
            same = bool(np.all(a.astype(bool) == b.astype(bool)))
        else:
            same = bool(np.allclose(a.astype(float), b.astype(float), rtol=0, atol=1e-12))
        if not same:
            raise SystemExit(f"参数列 {col} 不一致：不是同参数 A/B")

    merged["label_base"] = label(merged.rename(columns={"ok_base": "ok", "fail_base": "fail"}))
    merged["label_cand"] = label(merged.rename(columns={"ok_cand": "ok", "fail_cand": "fail"}))
    merged["fault"] = (merged["thrust_loss_base"] < 1.0) | merged["servo_jam_base"].astype(bool)

    out = {"baseline": Path(args.baseline).name, "candidate": Path(args.candidate).name,
           "n": int(len(merged)), "params_identical": True, "vehicles": {}}
    delay_bins = [0.0, 0.03, 0.06, 0.09, 0.12, 0.15, 0.20, np.inf]
    delay_labels = ["<30ms", "30-60ms", "60-90ms", "90-120ms",
                    "120-150ms", "150-200ms", ">=200ms"]
    for vehicle, group in merged.groupby("vehicle", sort=False):
        table = pd.crosstab(group["label_base"], group["label_cand"])
        table = table.reindex(index=["pass", "marginal", "fail"],
                              columns=["pass", "marginal", "fail"], fill_value=0)
        row = {
            "n": int(len(group)),
            "transitions": {a: {b: int(table.loc[a, b]) for b in table.columns}
                            for a in table.index},
            "base_pass": int((group["label_base"] == "pass").sum()),
            "cand_pass": int((group["label_cand"] == "pass").sum()),
            "base_fail": int((group["label_base"] == "fail").sum()),
            "cand_fail": int((group["label_cand"] == "fail").sum()),
        }
        row["rescued"] = int(((group["label_base"] == "fail") &
                              (group["label_cand"] != "fail")).sum())
        row["regressed"] = int(((group["label_base"] != "fail") &
                                (group["label_cand"] == "fail")).sum())
        row["new_pass"] = int(((group["label_base"] != "pass") &
                               (group["label_cand"] == "pass")).sum())
        row["lost_pass"] = int(((group["label_base"] == "pass") &
                                (group["label_cand"] != "pass")).sum())
        row["fault"] = {}
        for key, sub in (("no_fault", group[~group["fault"]]), ("with_fault", group[group["fault"]])):
            row["fault"][key] = {
                "n": int(len(sub)),
                "rescued": int(((sub["label_base"] == "fail") &
                                (sub["label_cand"] != "fail")).sum()),
                "regressed": int(((sub["label_base"] != "fail") &
                                  (sub["label_cand"] == "fail")).sum()),
                "base_fail": int((sub["label_base"] == "fail").sum()),
                "cand_fail": int((sub["label_cand"] == "fail").sum()),
            }
        bins = pd.cut(group["delay_s_base"], bins=delay_bins, labels=delay_labels,
                      include_lowest=True)
        row["delay_bins"] = []
        for name in delay_labels:
            sub = group[bins == name]
            if not len(sub):
                continue
            row["delay_bins"].append({
                "bin": name, "n": int(len(sub)),
                "base_fail": int((sub["label_base"] == "fail").sum()),
                "cand_fail": int((sub["label_cand"] == "fail").sum()),
                "rescued": int(((sub["label_base"] == "fail") &
                                (sub["label_cand"] != "fail")).sum()),
                "regressed": int(((sub["label_base"] != "fail") &
                                  (sub["label_cand"] == "fail")).sum()),
            })
        out["vehicles"][vehicle] = row
        print(f"{vehicle}: base pass/fail={row['base_pass']}/{row['base_fail']} -> "
              f"{row['cand_pass']}/{row['cand_fail']} | rescued={row['rescued']} "
              f"regressed={row['regressed']} new_pass={row['new_pass']} "
              f"lost_pass={row['lost_pass']}")
        print("  delay:", ", ".join(
            f"{b['bin']} fail {b['base_fail']}->{b['cand_fail']}" for b in row["delay_bins"]))

    if args.out:
        Path(args.out).write_text(json.dumps(out, indent=2, ensure_ascii=False), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
