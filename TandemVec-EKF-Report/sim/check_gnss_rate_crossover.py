#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24ab 裁决器：定位 10 Hz 与 20 Hz 的"交越点"（AB1–AB3）。

@file    check_gnss_rate_crossover.py
@brief   8 个过载档（22.0 / 29.0 / 29.4 / 30.1 / 30.5 g 等）× 两档刷新率（10 / 20 Hz）
         × seeds 51–100，逐档比 p90/发生率，找 20 Hz 开始占优的最小过载
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 为什么需要这一轮：§4.24aa 证明"提到 ≥20 Hz"不是无条件正确——22.0/25.6/28.1 g 上 20 Hz 的 p90
         分别差 4.8%/28%/303%，而 30.5 g 上 20 Hz 明显更好 ⇒ 中间必然存在交越，但没测。
         本轮的产出不是一个"最优刷新率"，而是一条**可执行的门槛**：低于它用 10 Hz、高于它用 20 Hz，
         并且必须同时看**失效率**（决定能不能飞）与**精度**（p90，决定飞得多准）。
         ★ 判据只看"同一档位、同一 seed 段"的两臂对比；跨二进制/跨 seed 段的数字不参与。
@frame   NED；pos_rms [m]、nav_att_rmse [deg]
@units   米 / 度 / 个 seed / 牛顿
@note    预登记判据（跑前写死）：
         AB1 完整性：每个 (档位, 刷新率) 组合各 50 条，缺一即整体作废。
         AB2 交越定义：ratio = p90(20 Hz) / p90(10 Hz)。交越点 = **最小的** 档位，使得该档
             ratio < 1.0 **且** 下一档 ratio 也 < 1.0（要求连续两档，防止单点噪声）。
             若所有档 ratio ≥1 ⇒ 记"本包线内未出现交越"。
         AB3 可用性：从交越点起，20 Hz 的 `>25 m` 发生率必须 ≤ 10 Hz 同档；否则交越只是精度交换，
             不能作为"该档改用 20 Hz"的依据（此时以发生率为准，取 10 Hz）。
         判定：AB1∧(AB2 有交越)∧AB3 ⇒ `RATE_CROSSOVER_LOCATED`；
              AB2 无交越 ⇒ `NO_CROSSOVER_IN_ENVELOPE`；AB3 不成立 ⇒ `CROSSOVER_NOT_USABLE`。
@see     sim/run_305g_onset_scan.py（驱动）、报告 §4.24ab
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
FAIL_M = 25.0
# (推力 N, 数据来源 tag, 可选：只取该 tag 的某档)
SOURCES = {
    10: [("ab_hz10", None), ("z_default", 22000)],
    20: [("ab_hz20", None), ("aa_hz20_off", 22000)],
}


def load(tag):
    path = os.path.join(SD, "attrib305_2026-10-28_%s.jsonl" % tag)
    if not os.path.exists(path):
        sys.exit("缺证据文件：%s" % path)
    out = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                r = json.loads(line)
                out.setdefault(int(r["thrust_n"]), {})[int(r["seed"])] = r
    return out


def stat(rows):
    p = np.array([float(r["pos_rms"]) for r in rows.values()])
    rej = np.array([float(r.get("outlier_rejects", 0) or 0) for r in rows.values()])
    return {"n": len(p), "fail_gt25": int((p > FAIL_M).sum()),
            "p90": round(float(np.percentile(p, 90)), 3),
            "max": round(float(p.max()), 3),
            "rej_med": float(np.median(rej))}


def main():
    data = {}
    for rate, srcs in SOURCES.items():
        per_thr = {}
        for tag, only in srcs:
            d = load(tag)
            for thr, rows in d.items():
                if only is not None and thr != only:
                    continue
                per_thr.setdefault(thr, {}).update(rows)
        data[rate] = per_thr

    thrusts = sorted(set(data[10]) & set(data[20]))
    rep = {"per_thrust": {}, "AB1": {}}
    ok1 = True
    for thr in thrusts:
        a, b = stat(data[10][thr]), stat(data[20][thr])
        if a["n"] != 50 or b["n"] != 50:
            ok1 = False
        rep["per_thrust"]["%.2f g" % (thr / 781.8)] = {
            "thrust_n": thr, "hz10": a, "hz20": b,
            "ratio_p90": round(b["p90"] / a["p90"], 3) if a["p90"] > 0 else None}
    rep["AB1"] = {"thrusts": ["%.2f g" % (t / 781.8) for t in thrusts], "pass": ok1 and len(thrusts) >= 5}

    keys = ["%.2f g" % (t / 781.8) for t in thrusts]
    ratios = [rep["per_thrust"][k]["ratio_p90"] for k in keys]
    cross_idx = None
    for i in range(len(ratios) - 1):
        if ratios[i] is not None and ratios[i] < 1.0 and ratios[i + 1] is not None and ratios[i + 1] < 1.0:
            cross_idx = i
            break
    rep["AB2"] = {"ratios": ratios, "crossover_index": cross_idx,
                  "crossover_thrust_n": thrusts[cross_idx] if cross_idx is not None else None,
                  "crossover_g": round(thrusts[cross_idx] / 781.8, 2) if cross_idx is not None else None,
                  "pass": cross_idx is not None}

    usable = True
    if cross_idx is not None:
        for i in range(cross_idx, len(keys)):
            if rep["per_thrust"][keys[i]]["hz20"]["fail_gt25"] > rep["per_thrust"][keys[i]]["hz10"]["fail_gt25"]:
                usable = False
    rep["AB3"] = {"usable_from_crossover": usable,
                  "note": "交越点起 20 Hz 的发生率必须不高于 10 Hz，否则按发生率取 10 Hz"}

    verdict = ("RATE_CROSSOVER_LOCATED" if (rep["AB1"]["pass"] and cross_idx is not None and usable)
               else ("NO_CROSSOVER_IN_ENVELOPE" if cross_idx is None else "CROSSOVER_NOT_USABLE"))
    rep["verdict"] = verdict
    out = os.path.join(SD, "gnss_rate_crossover_2026-10-28.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(rep, f, ensure_ascii=False, indent=1)
    print(json.dumps(rep, ensure_ascii=False, indent=1))
    print("written", out)
    sys.exit(0 if verdict == "RATE_CROSSOVER_LOCATED" else 1)


if __name__ == "__main__":
    main()
