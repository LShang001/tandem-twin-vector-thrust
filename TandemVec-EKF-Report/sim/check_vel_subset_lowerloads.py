#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24v 裁决器：触发上限 = 25 m 在"弱失稳档"（28.8 g / 30.1 g）上是否仍然只帮忙不添乱。

@file    check_vel_subset_lowerloads.py
@brief   两档推力（22 500 N≈28.8 g、23 500 N≈30.1 g）× 两臂（子集关 / 子集开·默认 25 m）
         × 50 seeds，按 V1–V3 判"没有失效可修时不得引入失效、有失效时不得变差"
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 为什么要补这两档：§4.24o 的 onset 曲线显示 28.8–30.2 g 是 5% 发生率平台，
         而 §4.24q–t 的全部阈值证据都来自 30.5 g 单点。**一个只在失稳档有效的算法，
         若在弱失稳档反而制造失效，就不能算改进。** 这两档正好是"少数 seed 失效"的区间，
         既检验子集能不能修，也检验它会不会把本来干净的 seed 弄坏。
         ★ 两臂同二进制 `tv_32g_q2`（md5 499495e5…，编译期默认 25 m），只差 `--gnss-vel-subset`。
@frame   NED；pos_rms / pos_max [m]、nav_att_rmse [deg]
@units   米 / 度 / 个 seed / 牛顿
@note    预登记判据（跑前写死）：
         V1 完整性：2 档 × 2 臂各收满 50 条。
         V2 不添乱：每档内，子集开臂的 `>25 m` 发生率 ≤ 关臂，且 `max` ≤ 关臂 `max` + 5 m。
         V3 该修的要修：若关臂在 28.8 g 档有 ≥3 个 `>25 m` 的 seed，开臂发生率必须下降
            （>0 才谈"修"，0 个失效的档只要求 V2）。
         判定：V1∧V2∧(V3 或其不适用) ⇒ `NO_HARM_ACROSS_LOADS`；否则 `HARM_AT_LOWER_LOAD`。
@see     sim/run_305g_onset_scan.py（驱动，复用）、报告 §4.24v
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
FAIL_M = 25.0
LADDER = ["22500", "23500"]
ARMS = [("off", "v_off"), ("on", "v_on")]


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
                out[(int(r["thrust_n"]), int(r["seed"]))] = r
    return out


def stat(d, thr):
    v = [float(r["pos_rms"]) for (t, _s), r in d.items() if t == thr]
    p = np.array(v)
    return {"n": len(p), "fail_gt25": int((p > FAIL_M).sum()),
            "p90": round(float(np.percentile(p, 90)), 3),
            "max": round(float(p.max()), 3),
            "att": round(float(np.median([float(r["nav_att_rmse"])
                                          for (t, _s), r in d.items() if t == thr])), 3)}


def main():
    data = {a: load(t) for a, t in ARMS}
    rep = {"per_cell": {}, "V1": {}}
    v1 = {a: len(d) for a, d in data.items()}
    rep["V1"] = {"counts": v1, "pass": all(n == 100 for n in v1.values())}

    bad = []
    v3_ok = True
    for thr in LADDER:
        s = {a: stat(data[a], int(thr)) for a, _ in ARMS}
        rep["per_cell"][thr] = s
        if (s["on"]["fail_gt25"] > s["off"]["fail_gt25"] or
                s["on"]["max"] > s["off"]["max"] + 5.0):
            bad.append(thr)
        if thr == "22500" and s["off"]["fail_gt25"] >= 3:
            v3_ok = s["on"]["fail_gt25"] < s["off"]["fail_gt25"]
    rep["V2"] = {"cells_adding_harm": bad, "pass": not bad}
    rep["V3"] = {"pass": bool(v3_ok),
                 "note": "仅在关臂 28.8 g 档 ≥3 个 >25 m 时生效；否则只要求 V2"}
    ok = rep["V1"]["pass"] and rep["V2"]["pass"] and rep["V3"]["pass"]
    rep["verdict"] = "NO_HARM_ACROSS_LOADS" if ok else "HARM_AT_LOWER_LOAD"
    rep["gate"] = "V1 两臂各 100 条；V2 每档发生率不高于关臂且 max 不超 +5 m；V3 有失效时要变少"
    out = os.path.join(SD, "vel_subset_lowerloads_2026-10-28.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(rep, f, ensure_ascii=False, indent=1)
    print(json.dumps(rep, ensure_ascii=False, indent=1))
    print("written", out)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
