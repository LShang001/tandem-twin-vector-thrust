#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24q 裁决器：速度子集"位置残差 ≤15 m 才触发"反向判据的 X1–X4。

@file    check_vel_subset_posres.py
@brief   对齐三臂（10 Hz 加上限 / 20 Hz 加上限 / 20 Hz 取消上限）与两个旧口径基线，
         按 §4.24q 预登记的 X1–X4 逐条给 PASS/FAIL 并落盘
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 与 §4.24p 的 posgate 修复相反：那次要求"位置门越门(>30 m)"，实测把 10 Hz 的
         有益触发全部拦掉（发生率 0/50 退回 18/50），而 20 Hz 那个 105 m 的 seed 触发时
         位置门本来就已越门，所以两头都不对。本判据用**触发时刻的位置残差大小**（15 m 上限）
         作为一致性代理，12 个触发 seed 的分离带是 12.0 → 21.3 m。
         ★ X2 与 X3 必须同时成立——只验有害侧会误杀有益端，这是 §4.24p 的直接教训。
         所有比较先 assert 配对数，交集不足直接判无效。
@frame   NED；pos_rms / pos_max [m]、nav_att_rmse [deg]
@units   米 / 度 / 次
@see     sim/run_305g_onset_scan.py（数据）、报告 §4.24q
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
WORST_SEEDS = [16, 20, 28, 37, 45, 47]   # §4.24p 中在 20 Hz 被子集推坏的 6 个 seed
FILES = {
    "b10_off": "onset50",            # 10 Hz 子集关
    "b10_old": "vel50sub",           # 10 Hz 子集开（无上限，旧口径）
    "b20_off": "hz20only",           # 20 Hz 子集关
    "b20_old": "vel50sub_hz20",      # 20 Hz 子集开（无上限，旧口径）
    "q10": "q10",                    # 10 Hz 子集开 + 15 m 上限
    "q20": "q20",                    # 20 Hz 子集开 + 15 m 上限
    "q20_nocap": "q20nocap",         # 20 Hz 子集开 + 上限取消（应等于 b20_old）
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
                out[int(r["seed"])] = r
    return out


def stat(d):
    p = np.array([float(v["pos_rms"]) for v in d.values()])
    return {"n": len(p), "fail_gt25": int((p > 25).sum()),
            "med": round(float(np.median(p)), 3),
            "p95": round(float(np.percentile(p, 95)), 3),
            "max": round(float(p.max()), 3),
            "att": round(float(np.median([float(v["nav_att_rmse"]) for v in d.values()])), 3)}


def pair(a, b, label):
    keys = sorted(set(a) & set(b))
    assert len(keys) == 50, "%s 配对 %d != 50 ⇒ 判定无效" % (label, len(keys))
    return keys


def main():
    arms = {k: load(v) for k, v in FILES.items()}
    rep = {}

    # X1 完整性
    rep["X1"] = {k: len(v) for k, v in arms.items()}
    rep["X1"]["pass"] = all(len(v) == 50 for v in arms.values())

    # X2 收益保全（10 Hz）
    s10 = {k: stat(arms[k]) for k in ("b10_off", "b10_old", "q10")}
    rep["X2"] = {"per_arm": s10,
                 "pass": (s10["q10"]["fail_gt25"] == 0 and s10["q10"]["max"] <= 20.0)}

    # X3 伤害消除（20 Hz）
    keys = pair(arms["q20"], arms["b20_off"], "X3")
    viol = [{"seed": s,
             "off": round(float(arms["b20_off"][s]["pos_rms"]), 3),
             "q20": round(float(arms["q20"][s]["pos_rms"]), 3)}
            for s in WORST_SEEDS
            if float(arms["q20"][s]["pos_rms"]) > float(arms["b20_off"][s]["pos_rms"]) * 1.1 + 1e-9]
    s20 = {k: stat(arms[k]) for k in ("b20_off", "b20_old", "q20")}
    rep["X3"] = {"per_arm": s20, "worsened_target_seeds_still_worse": viol,
                 "incidence_le_baseline": s20["q20"]["fail_gt25"] <= s20["b20_off"]["fail_gt25"],
                 "max_le_30": s20["q20"]["max"] <= 30.0,
                 "pass": (not viol and s20["q20"]["fail_gt25"] <= s20["b20_off"]["fail_gt25"]
                          and s20["q20"]["max"] <= 30.0)}

    # X4 上限取消 == 旧口径（同二进制只差开关）
    keys4 = pair(arms["q20_nocap"], arms["b20_old"], "X4")
    diff = [s for s in keys4
            if float(arms["q20_nocap"][s]["pos_rms"]) != float(arms["b20_old"][s]["pos_rms"])]
    rep["X4"] = {"pairs": len(keys4), "diff_seeds": diff, "pass": not diff}

    ok = all(rep[k]["pass"] for k in ("X1", "X2", "X3", "X4"))
    rep["verdict"] = "CRITERION_ACCEPTED" if ok else "REVERT_CRITERION"
    rep["gate"] = ("X2 10 Hz 仍 0/50 且 max≤20；X3 六个受害 seed 全部不再变差、"
                   "20 Hz 发生率不高于子集关基线、max≤30；X4 取消上限与旧口径逐 seed 相同")
    out = os.path.join(SD, "vel_subset_posres_verdict_2026-10-28.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(rep, f, ensure_ascii=False, indent=1)
    print(json.dumps({k: rep[k] for k in ("X1", "X2", "X3", "X4", "verdict")},
                     ensure_ascii=False, indent=1))
    print("written", out)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
