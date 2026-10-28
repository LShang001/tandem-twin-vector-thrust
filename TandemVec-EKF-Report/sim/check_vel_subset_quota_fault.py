#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24y 裁决器：配额门在故障格与 10 Hz 上是否安全/是否惰性（Y1–Y3）。

@file    check_vel_subset_quota_fault.py
@brief   五格（IMU NaN 1 s / GNSS 失联 5 s / 20 s / 振动 1 g@80 Hz / 10 Hz 主格）
         × 两臂（配额 1 新默认 / 配额 255 取消），按预登记判据给 PASS/FAIL
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 与 §4.24u 同型（故障矩阵是新门的必过关），但目标不同：§4.24u 问的是
         "放宽幅值门有没有引入失效"，本节问的是"**新增的配额门会不会误伤**"。
         由于 §4.24w 已在 20 Hz 干净段测到配额门完全惰性，本节的核心预期是"五格全惰性"；
         一旦出现差异，就必须逐 seed 判方向（变差即 FAIL）并登记机制线索。
@frame   NED；pos_rms / nav_att_rmse [deg]
@units   米 / 度 / 次 / 个 seed
@note    预登记判据（跑前写死）：
         Y1 完整性：五格 × 两臂各收满 20 条。
         Y2 不劣化：每格配额 1 臂的 `>25 m` 发生率 ≤ 配额 255 臂，且不存在
            "逐 seed pos_rms 变差 >20% 且绝对增量 >2 m"的 seed。
         Y3 惰性记录（描述性，不设门槛）：逐格统计两臂逐 seed 相同的条数；
            全部相同 ⇒ 写"配额门在该格惰性"；有差异 ⇒ 逐条列方向与幅度。
         判定：Y1∧Y2 ⇒ `QUOTA_FAULT_SAFE`；Y2 不成立 ⇒ `QUOTA_FAULT_HARMS`（不启用该门）。
@see     sim/run_vel_subset_quota_fault.py（数据驱动）、报告 §4.24y
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
TAG = "2026-10-28_y"
CELLS = ["nan1s", "out5", "out20", "vib1g", "hz10"]
ARMS = ["q1", "q255"]
FAIL_M = 25.0


def load():
    path = os.path.join(SD, "vel_subset_quota_fault_%s.jsonl" % TAG)
    if not os.path.exists(path):
        sys.exit("缺证据文件：%s" % path)
    d = {(c, a): {} for c in CELLS for a in ARMS}
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                r = json.loads(line)
                d[(r["cell"], r["arm"])][int(r["seed"])] = r
    return d


def stat(d):
    p = np.array([float(v["pos_rms"]) for v in d.values()])
    return {"n": len(p), "fail_gt25": int((p > FAIL_M).sum()),
            "p90": round(float(np.percentile(p, 90)), 3),
            "max": round(float(p.max()), 3),
            "att": round(float(np.median([float(v["nav_att_rmse"]) for v in d.values()])), 3)}


def main():
    data = load()
    rep = {"per_cell": {}, "Y1": {}, "Y2": {}, "Y3": {}}
    counts = {"%s|%s" % (c, a): len(data[(c, a)]) for c in CELLS for a in ARMS}
    rep["Y1"] = {"counts": counts, "pass": all(v == 20 for v in counts.values())}

    bad = []
    for c in CELLS:
        a, b = data[(c, "q1")], data[(c, "q255")]
        keys = sorted(set(a) & set(b))
        assert keys, "格 %s 两臂无交集" % c
        same = sum(1 for k in keys if float(a[k]["pos_rms"]) == float(b[k]["pos_rms"]))
        worse = []
        for k in keys:
            x, y = float(b[k]["pos_rms"]), float(a[k]["pos_rms"])  # x=255 基线, y=配额1
            if y - x > 2.0 and (y - x) / max(x, 1e-9) > 0.20:
                worse.append({"seed": k, "q255": round(x, 3), "q1": round(y, 3)})
        sa, sb = stat(a), stat(b)
        rep["per_cell"][c] = {"q1": sa, "q255": sb, "pairs": len(keys),
                              "seeds_identical": same, "worse_seeds": worse}
        rep["Y3"][c] = ("惰性（20/20 逐 seed 相同）" if same == len(keys)
                        else "%d/%d 相同，差异 %d" % (same, len(keys), len(keys) - same))
        if sa["fail_gt25"] > sb["fail_gt25"] or worse:
            bad.append(c)
    rep["Y2"] = {"cells_with_regression": bad, "pass": not bad}

    ok = rep["Y1"]["pass"] and rep["Y2"]["pass"]
    rep["verdict"] = "QUOTA_FAULT_SAFE" if ok else "QUOTA_FAULT_HARMS"
    rep["gate"] = "Y1 五格两臂各 20；Y2 发生率不升且无'变差>20%且>2 m'的 seed；Y3 惰性描述"
    out = os.path.join(SD, "vel_subset_quota_fault_%s_verdict.json" % TAG)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(rep, f, ensure_ascii=False, indent=1)
    print(json.dumps(rep, ensure_ascii=False, indent=1))
    print("written", out)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
