#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24z 裁决器：把"干净过载档"补齐，给出 22→30.5 g 的完整收敛表（Z1–Z3）。

@file    check_vel_subset_overload_table.py
@brief   三档干净过载（22.0 / 25.5 / 28.1 g）× 两臂（现行默认=子集关 / 推荐配置=20 Hz+子集）
         × seeds 51–100，确认推荐配置在"本来就没有失效"的档位上不添乱、不降精度
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 拼图里还缺的三格：§4.24o 的 onset 曲线显示 22–28 g 是 0–5%，§4.24v 覆盖了
         28.8/30.1/30.5 g 的两臂对照，但 **22/25.5/28.1 g 只有"默认配置 0/50"的记录，
         没有"推荐配置（20 Hz + 子集）"的对照**。若推荐配置在干净档上把精度拉差，
         那它就不是"可以提升"而是"要权衡"。
         ★ 两臂同二进制 `tv_32g_w2`（md5 `fbac7fe9…`，含配额门默认 1），只差运行时参数。
@frame   NED；pos_rms / pos_max [m]、nav_att_rmse [deg]
@units   米 / 度 / 个 seed / 牛顿
@note    预登记判据（跑前写死）：
         Z1 完整性：3 档 × 2 臂各收满 50 条。
         Z2 不添乱：每档推荐配置的 `>25 m` 发生率 ≤ 默认臂，且 `max` ≤ 默认臂 `max` + 3 m。
         Z3 精度记录（描述性）：逐档记录两臂的 p90/max 与逐 seed 方向计数
            （更好/更差/相同），不设门槛，用于判断"推荐配置是否以精度换稳定性"。
         判定：Z1∧Z2 ⇒ `CLEAN_BAND_SAFE`；否则 `CLEAN_BAND_HARMS`。
@see     sim/run_305g_onset_scan.py（驱动，复用）、报告 §4.24z
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
THRUSTS = ["17200", "20000", "22000"]
ARMS = [("default", "z_default"), ("rec", "z_rec")]
FAIL_M = 25.0


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
    p = np.array([float(r["pos_rms"]) for (t, _s), r in d.items() if t == thr])
    return {"n": len(p), "fail_gt25": int((p > FAIL_M).sum()),
            "p90": round(float(np.percentile(p, 90)), 3),
            "max": round(float(p.max()), 3),
            "att": round(float(np.median([float(r["nav_att_rmse"])
                                          for (t, _s), r in d.items() if t == thr])), 3)}


def main():
    data = {a: load(t) for a, t in ARMS}
    rep = {"per_thrust": {}, "Z1": {}, "Z2": {}, "Z3": {}}
    counts = {a: len(d) for a, d in data.items()}
    rep["Z1"] = {"counts": counts, "pass": all(n == 150 for n in counts.values())}

    harm = []
    for thr in THRUSTS:
        s = {a: stat(data[a], int(thr)) for a, _ in ARMS}
        keys = sorted({k for k in data["default"] if k[0] == int(thr)} &
                      {k for k in data["rec"] if k[0] == int(thr)})
        better = sum(1 for k in keys
                     if float(data["rec"][k]["pos_rms"]) < float(data["default"][k]["pos_rms"]) - 1e-9)
        worse = sum(1 for k in keys
                    if float(data["rec"][k]["pos_rms"]) > float(data["default"][k]["pos_rms"]) + 1e-9)
        same = len(keys) - better - worse
        rep["per_thrust"]["%.1f g" % (int(thr) / 781.8)] = {
            "default": s["default"], "rec": s["rec"],
            "seeds_better": better, "seeds_worse": worse, "seeds_same": same}
        if (s["rec"]["fail_gt25"] > s["default"]["fail_gt25"] or
                s["rec"]["max"] > s["default"]["max"] + 3.0):
            harm.append(thr)
    rep["Z2"] = {"thrusts_adding_harm": harm, "pass": not harm}
    rep["Z3"] = {"caliber": "逐档记录 p90/max 与逐 seed 方向；不设门槛",
                 "note": "若某档 rec 臂 p90 明显更差，需在报告里写明是交换而非提升"}
    ok = rep["Z1"]["pass"] and rep["Z2"]["pass"]
    rep["verdict"] = "CLEAN_BAND_SAFE" if ok else "CLEAN_BAND_HARMS"
    rep["gate"] = "Z1 两臂各 150；Z2 每档发生率不升、max 不超 +3 m；Z3 描述性"
    out = os.path.join(SD, "vel_subset_overload_table_2026-10-28.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(rep, f, ensure_ascii=False, indent=1)
    print(json.dumps(rep, ensure_ascii=False, indent=1))
    print("written", out)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
