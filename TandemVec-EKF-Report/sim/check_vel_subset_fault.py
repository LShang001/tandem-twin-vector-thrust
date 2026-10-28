#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24u 裁决器：把触发上限放宽到 25 m 之后，故障格是否仍然不劣化（Z1–Z3）。

@file    check_vel_subset_fault.py
@brief   四类故障格（IMU NaN 1 s / GNSS 失联 5 s / GNSS 失联 20 s / 振动 1 g@80 Hz）
         × 两臂（子集关 / 子集开·默认 25 m），按预登记判据 Z1–Z3 给 PASS/FAIL
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 判定逻辑必须"两端都看"：
         ① 子集存在的理由是修 30.5 g 的尾部（主格 20 Hz 上指标在 §4.24s/§4.24t 已给），
            本文件只回答"它有没有在故障格上引入新的坏结果"；
         ② 因此 Z2 用**逐 seed 配对**判"变差"，而不是聚合中位——本轮系列已多次证明
            聚合口径会把单个 seed 的灾难平均掉；
         ③ Z3 是"若 Z2 不成立必须定位"的强制条款：任何变差 seed 都要打印其
            `outlier_rejects / reacquire_resets / nav_att_rmse`，区分是子集自身造成
            还是故障本底。
@frame   NED；pos_rms / pos_max [m]、nav_att_rmse [deg]
@units   米 / 度 / 次 / 个 seed
@note    预登记判据（跑前写死）：
         Z1 完整性：八格（4 故障 × 2 臂）各收满 N seeds，否则整体作废。
         Z2 不劣化：对每个故障格，子集开臂的 `>25 m` 发生率 ≤ 子集关臂，且
            **不存在**"逐 seed pos_rms 变差 >20% 且绝对增量 >2 m"的 seed。
         Z3 定位：若 Z2 有变差 seed，必须为每个这样的 seed 打印拒帧/重锚/姿态，
            并在报告中区分"子集造成"与"故障本底"；不得只报计数字样。
         判定规则：Z1∧Z2 ⇒ `FAULT_MATRIX_CLEAN`；Z2 不成立 ⇒ `FAULT_REGRESSION`，
         默认值必须回退到 15 m 并重跑（本轮不接受"影响很小"这种说法）。
@see     sim/run_vel_subset_fault_matrix.py（数据驱动）、报告 §4.24u
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
TAG = "2026-10-28_z"
CELLS = ["nan1s", "out5", "out20", "vib1g"]
ARMS = ["off", "on"]
FAIL_M = 25.0


def load():
    path = os.path.join(SD, "vel_subset_fault_%s.jsonl" % TAG)
    if not os.path.exists(path):
        sys.exit("缺证据文件：%s" % path)
    data = {(c, a): {} for c in CELLS for a in ARMS}
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                r = json.loads(line)
                data[(r["cell"], r["arm"])][int(r["seed"])] = r
    return data


def stats(d):
    p = np.array([float(v["pos_rms"]) for v in d.values()])
    return {"n": len(p), "fail_gt25": int((p > FAIL_M).sum()),
            "p90": round(float(np.percentile(p, 90)), 3),
            "max": round(float(p.max()), 3),
            "att_med": round(float(np.median([float(v["nav_att_rmse"]) for v in d.values()])), 3)}


def main():
    data = load()
    rep = {"per_cell": {}, "Z1": {}, "Z2": {}}

    z1 = {}
    for c in CELLS:
        for a in ARMS:
            z1["%s|%s" % (c, a)] = len(data[(c, a)])
    ns = {v for v in z1.values()}
    rep["Z1"] = {"counts": z1, "pass": len(ns) == 1 and min(ns) > 0}

    bad = []
    for c in CELLS:
        off, on = data[(c, "off")], data[(c, "on")]
        keys = sorted(set(off) & set(on))
        assert keys, "格 %s 两臂无交集" % c
        s = {"off": stats(off), "on": stats(on)}
        worse = []
        for k in keys:
            a = float(off[k]["pos_rms"])
            b = float(on[k]["pos_rms"])
            if b - a > 2.0 and (b - a) / max(a, 1e-9) > 0.20:
                worse.append({"seed": k, "off": round(a, 3), "on": round(b, 3),
                              "on_rej": on[k].get("outlier_rejects"),
                              "on_rst": on[k].get("reacquire_resets"),
                              "on_att": round(float(on[k]["nav_att_rmse"]), 3)})
        rep["per_cell"][c] = {"stats": s, "pairs": len(keys), "worse_seeds": worse}
        if s["on"]["fail_gt25"] > s["off"]["fail_gt25"] or worse:
            bad.append(c)

    rep["Z2"] = {"cells_with_regression": bad, "pass": not bad}
    rep["Z3"] = {"note": "若 Z2 不成立，worse_seeds 里的拒帧/重锚/姿态用于区分故障本底"}
    ok = rep["Z1"]["pass"] and rep["Z2"]["pass"]
    rep["verdict"] = "FAULT_MATRIX_CLEAN" if ok else "FAULT_REGRESSION"
    rep["gate"] = ("Z1 八格各收满；Z2 每格子集开臂发生率不高于关臂且无"
                   "'变差>20% 且 >2 m' 的 seed")
    out = os.path.join(SD, "vel_subset_fault_%s_verdict.json" % TAG)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(rep, f, ensure_ascii=False, indent=1)
    print(json.dumps({"Z1": rep["Z1"], "Z2": rep["Z2"],
                      "per_cell": {c: rep["per_cell"][c] for c in CELLS},
                      "verdict": rep["verdict"]}, ensure_ascii=False, indent=1))
    print("written", out)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
