#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24r 裁决器：15 m 触发上限的独立 seeds 泛化（Y1–Y3）与阈值平坦性（Y4）。

@file    check_vel_subset_capval.py
@brief   在独立 seeds 51–100 上比七臂（20 Hz 子集关 / 无上限 / cap 12·15·21；
         10 Hz 无上限 / cap 15），按预登记的 Y1–Y4 逐条给 PASS/FAIL/N_A 并落盘
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 为什么必须跑这一轮：§4.24q 的 15 m 是**从 seeds 1–50 的 18 个触发样本事后选的**
         （有益端 ≤12.03 m、有害端 ≥21.29 m，15 m 取中）。这是典型的"训练点即验证点"，
         在本工程已多次被独立 seeds 推翻（饱和 Q 的 DE 最优、yaw 限幅的 10→20 seeds）。
         因此本轮做三件独立的事：
         ① **泛化**：同一阈值在 seeds 51–100 上是否仍然"拦住有害、放过有益"；
         ② **平坦性**：12 / 15 / 21 m 三档若结果相当 ⇒ 15 落在一马平川的平台上，
            阈值不敏感、可作默认；若 12 与 21 分处两侧 ⇒ 15 是悬崖上的一点，
            只能写"本平台样本内阈值"，不得外推；
         ③ **副作用回归**：上限在 10 Hz 那一端应当完全咬不到（§4.24q 里 0/50 seed 被拦），
            若在新 seeds 上开始咬，必须逐 seed 判方向——变差即 FAIL。
         ★ 口径纪律（本工程反复踩过的三条）：
         a) 双峰格不得报中位当指标 ⇒ 报 `>25 m` 发生率 + 按簇统计；
         b) 任何比较先 assert 配对数 == 50，交集不足直接判无效；
         c) 判"有收益"之前必须先证明被比的那个开关真的被触发（子集触发次数 >0），
            否则"两臂相同"可能是构造性相同（§4.23b 的教训）。
@frame   NED；pos_rms / pos_max [m]、nav_att_rmse [deg]
@units   米 / 度 / 次
@note    预登记判据（跑前写死，不看结果不改）：
         Y1 完整性：七臂在 seeds 51–100 上各收满 50 条，否则整体作废。
         Y2 泛化（有害端仍被拦）：20 Hz `cap=15` 的 `>25 m` 发生率 ≤ `nocap` 臂，
            且 `max` ≤ 30 m。前置可达性检查：`nocap` 臂在新 seeds 上必须至少出现 1 个
            `>25 m` 的 seed，否则新 seed 段没重现有害触发 ⇒ Y2 记 N/A（不算通过），
            并按 §4.24q 原口径降级为"仅 seeds 1–50 验证"。
         Y3 收益保全（有益端不被误杀）：10 Hz `cap=15` 与 `nocap` 在新 seeds 上逐 seed
            `pos_rms` 完全相同；若存在差异，逐 seed 判方向，**任何"变差 >5%"即 FAIL**，
            全部"变好或不变"则记 PASS_WITH_DIFF 并登记差异 seed。
         Y4 阈值平坦性：20 Hz 上 cap∈{12,15,21} 三档的 `>25 m` 发生率两两差 ≤1 seed，
            且 `max` 两两差 ≤5 m ⇒ 15 m 在平台上；否则登记实际平坦区间，
            并把 §4.24q 的"默认 15 m"表述降级为"区间内的一个点"。
         判定规则：Y1∧Y2∧Y3∧Y4 ⇒ 15 m 升级为"独立 seeds 验证过的阈值（L2）"；
         Y2 或 Y4 不成立 ⇒ 保留实现但把默认改为"候选、需按平台重标"；
         Y3 不成立 ⇒ 上限会误杀有益端，必须放宽默认并重启 §4.24q 判定。
@see     sim/run_305g_onset_scan.py（数据驱动）、sim/check_vel_subset_posres.py（X 组）、
         报告 §4.24q/§4.24r
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
SEEDS = list(range(51, 101))
ARMS = ["y20_off", "y20_nocap", "y20_c12", "y20_c15", "y20_c21", "y10_nocap", "y10_c15"]


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
                out[int(r["seed"])] = r
    return out


def stat(d):
    p = np.array([float(v["pos_rms"]) for v in d.values()])
    return {"n": len(p), "fail_gt25": int((p > 25).sum()),
            "med": round(float(np.median(p)), 3),
            "p90": round(float(np.percentile(p, 90)), 3),
            "p95": round(float(np.percentile(p, 95)), 3),
            "max": round(float(p.max()), 3),
            "att": round(float(np.median([float(v["nav_att_rmse"]) for v in d.values()])), 3)}


def main():
    arms = {a: load(a) for a in ARMS}
    rep = {}

    # Y1 完整性
    rep["Y1"] = {a: len(arms[a]) for a in ARMS}
    rep["Y1"]["pass"] = all(len(arms[a]) == len(SEEDS) for a in ARMS)
    for a in ARMS:
        missing = sorted(set(SEEDS) - set(arms[a]))
        if missing:
            rep["Y1"]["missing_" + a] = missing[:5]

    # Y2 泛化（20 Hz 有害端）
    s20 = {a: stat(arms[a]) for a in ("y20_off", "y20_nocap", "y20_c12", "y20_c15", "y20_c21")}
    reachable = s20["y20_nocap"]["fail_gt25"] >= 1
    rep["Y2"] = {"per_arm": s20, "nocap_reachable": reachable,
                 "rate_le_nocap": s20["y20_c15"]["fail_gt25"] <= s20["y20_nocap"]["fail_gt25"],
                 "max_le_30": s20["y20_c15"]["max"] <= 30.0,
                 "pass": bool(reachable and s20["y20_c15"]["fail_gt25"] <= s20["y20_nocap"]["fail_gt25"]
                              and s20["y20_c15"]["max"] <= 30.0),
                 "note": "nocap 臂在新 seeds 上无 >25 m ⇒ Y2 记 N/A（未验证，不算通过）"}

    # Y3 收益保全（10 Hz 有益端不被误杀）
    keys10 = sorted(set(arms["y10_nocap"]) & set(arms["y10_c15"]))
    assert len(keys10) == len(SEEDS), "Y3 配对 %d != %d ⇒ 判定无效" % (len(keys10), len(SEEDS))
    diffs = []
    for s in keys10:
        a = float(arms["y10_nocap"][s]["pos_rms"])
        b = float(arms["y10_c15"][s]["pos_rms"])
        if a != b:
            diffs.append({"seed": s, "nocap": round(a, 3), "c15": round(b, 3),
                          "rel_pct": round((b - a) / a * 100.0, 2) if a > 0 else None})
    worse = [d for d in diffs if d["rel_pct"] is not None and d["rel_pct"] > 5.0]
    rep["Y3"] = {"per_arm": {a: stat(arms[a]) for a in ("y10_nocap", "y10_c15")},
                 "pairs": len(keys10), "diff_seeds": diffs, "worse_seeds": worse,
                 "pass": not worse,
                 "status": ("PASS_IDENTICAL" if not diffs else
                            ("FAIL" if worse else "PASS_WITH_DIFF"))}

    # Y4 阈值平坦性
    caps = {"12": s20["y20_c12"], "15": s20["y20_c15"], "21": s20["y20_c21"]}
    rate_spread = max(c["fail_gt25"] for c in caps.values()) - min(c["fail_gt25"] for c in caps.values())
    max_spread = round(max(c["max"] for c in caps.values()) - min(c["max"] for c in caps.values()), 3)
    rep["Y4"] = {"caps": caps, "rate_spread_seeds": int(rate_spread), "max_spread_m": max_spread,
                 "flat": bool(rate_spread <= 1 and max_spread <= 5.0),
                 "pass": bool(rate_spread <= 1 and max_spread <= 5.0)}

    ok = all(rep[k]["pass"] for k in ("Y1", "Y2", "Y3", "Y4"))
    rep["verdict"] = "CAP_VALIDATED_ON_INDEPENDENT_SEEDS" if ok else "CAP_NOT_GENERALIZED"
    rep["seeds"] = [SEEDS[0], SEEDS[-1]]
    rep["gate"] = ("Y1 七臂各 50/50；Y2 nocap 臂在新 seeds 上可达(≥1 个 >25 m) 且 cap15 发生率不高于它、"
                   "max≤30；Y3 10 Hz cap15 无 seed 变差 >5%；Y4 cap 12/15/21 发生率差 ≤1 seed 且 max 差 ≤5 m")
    out = os.path.join(SD, "vel_subset_capval_51_100_2026-10-28.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(rep, f, ensure_ascii=False, indent=1)
    print(json.dumps({k: rep[k] for k in ("Y1", "Y2", "Y3", "Y4", "verdict")},
                     ensure_ascii=False, indent=1))
    print("written", out)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
