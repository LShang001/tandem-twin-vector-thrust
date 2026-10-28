#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24w 裁决器：子集"每历元配额"门（默认 1 次）的同二进制 A/B——W1–W4。

@file    check_vel_subset_quota.py
@brief   30.5 g / 20 Hz，两段 seeds（1–50、51–100），配额 1 vs 255（取消限制），
         先证"关掉门 == 旧行为"，再判门开启后是否不劣化/是否更好
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 为什么要做这个门（§4.24r–t 的机制结论）：害处不是"某次触发太大"，
         而是"位置残差已经大了还反复触发"（无上限时 3–4 次 ⇒ 44–105 m）；
         单次 21–30 m 的触发往往无害甚至有益（cap35 放行 seed47 的 29.89 m 反而好 2.5 m）。
         配额门把"反复"直接限制住，且**不依赖"米"这个要按平台重标的绝对量**。
         ★ 判定顺序很重要：先证"关掉门等于旧行为"（W2），再谈门本身的收益（W3/W4）。
           若 W2 不成立，说明新代码在门关闭时也改变了行为，本轮全部结论作废。
@frame   NED；pos_rms / pos_max [m]、nav_att_rmse [deg]
@units   米 / 度 / 次 / 个 seed
@note    预登记判据（跑前写死）：
         W1 完整性：四臂各 50/50 收满。
         W2 等价性：`--gnss-vel-subset-maxfires 255` 臂与归档 cap25 臂（`u25_s51`/`u25_s1`，
            旧二进制 tv_32g_q2）在 `pos_rms/pos_max/nav_att_rmse/outlier_rejects/
            reacquire_resets` 五字段上**逐 seed 相同**。
         W3 不劣化：配额 1 臂在每段的 `>25 m` 发生率 ≤ 255 臂，且 `max` ≤ 255 臂 `max` + 3 m。
         W4 收益判定（描述性，不设门槛）：记录配额 1 相对 255 的逐 seed 方向计数；
            若为 0 改善则如实写"无可测收益"。
         判定：W1∧W2∧W3 ⇒ `QUOTA_GATE_SAFE`；W3 不成立 ⇒ `QUOTA_GATE_HARMS`（不启用）。
@see     sim/run_305g_onset_scan.py（驱动，复用）、test_host/test_ekf_15state.cpp T26、报告 §4.24w
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
FAIL_M = 25.0
KEYS = ("pos_rms", "pos_max", "nav_att_rmse", "outlier_rejects", "reacquire_resets")
# (段, 本臂 tag, 参照 tag（旧二进制 cap25）)
SEGS = [("51-100", "w_q1_s51", "w_q255_s51", "u25_s51"),
        ("1-50", "w_q1_s1", "w_q255_s1", "u25_s1")]


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
    return {"n": len(p), "fail_gt25": int((p > FAIL_M).sum()),
            "p90": round(float(np.percentile(p, 90)), 3),
            "max": round(float(p.max()), 3),
            "att": round(float(np.median([float(v["nav_att_rmse"]) for v in d.values()])), 3)}


def main():
    rep = {"per_seg": {}, "W1": {}, "W2": {}}
    counts = {}
    eq_bad = []
    harm = []
    for seg, t1, t255, tref in SEGS:
        q1, q255, ref = load(t1), load(t255), load(tref)
        counts.update({t1: len(q1), t255: len(q255)})
        k = sorted(set(q255) & set(ref))
        assert len(k) == 50, "%s 与参照配对 %d != 50 ⇒ 判定无效" % (seg, len(k))
        diff = {key: [s for s in k if q255[s].get(key) != ref[s].get(key)] for key in KEYS}
        if any(diff.values()):
            eq_bad.append({"seg": seg, "diff": {x: len(v) for x, v in diff.items()}})
        k2 = sorted(set(q1) & set(q255))
        assert len(k2) == 50, "%s 两臂配对 %d != 50" % (seg, len(k2))
        s1, s255 = stat(q1), stat(q255)
        better = sum(1 for s in k2 if float(q1[s]["pos_rms"]) < float(q255[s]["pos_rms"]) - 1e-9)
        worse = sum(1 for s in k2 if float(q1[s]["pos_rms"]) > float(q255[s]["pos_rms"]) + 1e-9)
        rep["per_seg"][seg] = {"quota1": s1, "quota255": s255,
                               "seeds_better": better, "seeds_worse": worse}
        if s1["fail_gt25"] > s255["fail_gt25"] or s1["max"] > s255["max"] + 3.0:
            harm.append(seg)
    rep["W1"] = {"counts": counts, "pass": all(v == 50 for v in counts.values())}
    rep["W2"] = {"mismatch": eq_bad, "pass": not eq_bad,
                 "note": "maxfires=255 臂必须与旧二进制 cap25 臂逐 seed 相同"}
    rep["W3"] = {"segs_adding_harm": harm, "pass": not harm}
    ok = rep["W1"]["pass"] and rep["W2"]["pass"] and rep["W3"]["pass"]
    rep["verdict"] = "QUOTA_GATE_SAFE" if ok else (
        "QUOTA_GATE_HARMS" if rep["W1"]["pass"] and rep["W2"]["pass"] else "A_B_INVALID")
    rep["gate"] = "W1 四臂各 50；W2 255 臂==旧 cap25 臂逐 seed；W3 配额 1 臂发生率不升、max 不超 +3 m"
    out = os.path.join(SD, "vel_subset_quota_2026-10-28.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(rep, f, ensure_ascii=False, indent=1)
    print(json.dumps(rep, ensure_ascii=False, indent=1))
    print("written", out)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
