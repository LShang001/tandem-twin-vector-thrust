#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24p 裁决器：速度子集软恢复能否压平 §4.24o 的 onset 发生率台阶（V1–V4，跑前写死）。

@file    check_vel_subset_onset.py
@brief   对齐基线臂与 `--gnss-vel-subset 1` 臂的逐 seed 结果，按 §4.24p 预登记的
         V1（完整性）/V2（有效性）/V3（惰性）/V4（代价面）逐条给 PASS/FAIL 并落盘
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 判据原文见报告 §4.24p。所有比较先 assert 配对数，交集不足直接判无效——
         本工程已 4 次踩"交集为 0 仍打印逐项相同"这一类空洞绿灯。
         同二进制只差 runtime 开关 ⇒ 差异只能来自该通道本身。
@frame   NED；pos_rms [m]、nav_att_rmse [deg]、outlier_rejects/reacquire_resets [次]
@units   米 / 度 / 计数
@note    V2 的"无一变差 >10%"按逐 seed 相对差判定；基线为 0 时跳过（不可比）
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
BASE_FILES = ["attrib305_2026-10-28_onset.jsonl", "attrib305_2026-10-28_onsetfine.jsonl",
              "attrib305_2026-10-28_onset50.jsonl"]
VEL_FILES = ["attrib305_2026-10-28_vel6sub.jsonl", "attrib305_2026-10-28_vel50sub.jsonl",
             "attrib305_2026-10-28_vellazy.jsonl"]
LAZY_CELLS = [(17200, 20), (22000, 20)]
MID_CELLS = [(22500, 20), (23000, 20), (23500, 20), (23700, 20), (23800, 20)]


def load(files):
    out = {}
    for fn in files:
        path = os.path.join(SD, fn)
        if not os.path.exists(path):
            sys.exit("缺证据文件：%s" % path)
        with open(path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line:
                    r = json.loads(line)
                    out[(int(r["thrust_n"]), int(r["seed"]))] = r
    return out


def cell(d, thr, n):
    return {s: d[(thr, s)] for s in range(1, n + 1) if (thr, s) in d}


def rate_gt25(c):
    v = np.array([float(r["pos_rms"]) for r in c.values()])
    return int((v > 25.0).sum()), len(v), float(np.median(v)), float(np.percentile(v, 90))


def main():
    base, vel = load(BASE_FILES), load(VEL_FILES)
    rep = {}

    # ---- V1 完整性 ----
    cells = LAZY_CELLS + MID_CELLS + [(23900, 20), (23900, 50)]
    v1 = {}
    for thr, n in cells:
        v1["%d@%d" % (thr, n)] = {"base": len(cell(base, thr, n)), "vel": len(cell(vel, thr, n))}
    rep["V1"] = {"per_cell_counts": v1,
                 "pass": all(v1[k]["base"] == v1[k]["vel"] and v1[k]["base"] > 0 for k in v1)}

    # ---- V2 有效性：30.5 g / 50 seeds ----
    b, v = cell(base, 23900, 50), cell(vel, 23900, 50)
    assert len(b) == 50 and len(v) == 50, "V2 配对 %d/%d != 50 ⇒ 判定无效" % (len(b), len(v))
    fb, nb, _, _ = rate_gt25(b)
    fv, nv, med_v, p90_v = rate_gt25(v)
    worse = []
    for s in sorted(set(b) & set(v)):
        x, y = float(b[s]["pos_rms"]), float(v[s]["pos_rms"])
        if x > 0 and (y - x) / x > 0.10:
            worse.append({"seed": s, "base": round(x, 3), "vel": round(y, 3)})
    rep["V2"] = {"base_fail": "%d/%d" % (fb, nb), "vel_fail": "%d/%d" % (fv, nv),
                 "vel_med": round(med_v, 3), "vel_p90": round(p90_v, 3),
                 "worse_gt10pct": worse, "pass": (fv <= 6 and not worse)}

    # ---- V3 惰性：两档非失效区逐 seed 必须完全相同 ----
    lazy = {}
    for thr, n in LAZY_CELLS:
        b2, v2 = cell(base, thr, n), cell(vel, thr, n)
        assert len(b2) == n and len(v2) == n, "V3 配对不足 (%d/%d) ⇒ 判定无效" % (len(b2), len(v2))
        diff = [s for s in b2 if float(b2[s]["pos_rms"]) != float(v2[s]["pos_rms"])]
        lazy["%d" % thr] = {"pairs": len(b2), "diff_seeds": diff}
    rep["V3"] = {"per_cell": lazy, "pass": all(not x["diff_seeds"] for x in lazy.values())}

    # ---- V4 代价面：逐档两臂对照 ----
    tab = {}
    for thr, n in MID_CELLS + [(23900, 20)]:
        b3, v3 = cell(base, thr, n), cell(vel, thr, n)
        if not b3 or not v3:
            continue
        fb3, nb3, mb3, p90b = rate_gt25(b3)
        fv3, nv3, mv3, p90v = rate_gt25(v3)
        tab["%d@%d" % (thr, n)] = {
            "g": round(thr / 781.8, 2),
            "fail_base": "%d/%d" % (fb3, nb3), "fail_vel": "%d/%d" % (fv3, nv3),
            "med_base": round(mb3, 3), "med_vel": round(mv3, 3),
            "p90_base": round(p90b, 3), "p90_vel": round(p90v, 3),
            "att_base": round(float(np.median([float(r["nav_att_rmse"]) for r in b3.values()])), 3),
            "att_vel": round(float(np.median([float(r["nav_att_rmse"]) for r in v3.values()])), 3),
            "posmax_base": round(float(np.median([float(r["pos_max"]) for r in b3.values()])), 3),
            "posmax_vel": round(float(np.median([float(r["pos_max"]) for r in v3.values()])), 3),
            "rej_base": float(np.median([float(r.get("outlier_rejects", 0)) for r in b3.values()])),
            "rej_vel": float(np.median([float(r.get("outlier_rejects", 0)) for r in v3.values()])),
            "reanch_base": sum(1 for r in b3.values() if float(r.get("reacquire_resets", 0)) > 0),
            "reanch_vel": sum(1 for r in v3.values() if float(r.get("reacquire_resets", 0)) > 0),
        }
    rep["V4"] = {"per_cell": tab}
    mid_no_drop = [k for k, x in tab.items()
                   if int(x["fail_vel"].split("/")[0]) >= int(x["fail_base"].split("/")[0])]
    rep["V4"]["mid_cells_no_improvement"] = mid_no_drop

    rep["verdict"] = ("UPGRADE_RECOMMENDED" if (rep["V1"]["pass"] and rep["V2"]["pass"]
                                                and rep["V3"]["pass"]) else "KEEP_DEFAULT_OFF")
    rep["gate"] = ("V1 完整性 + V2 (30.5 g 发生率 ≤6/50 且无一 seed 变差 >10%) + "
                   "V3 非失效区逐 seed 不变 ⇒ 才允许升级推荐；否则只登记边界")
    out = os.path.join(SD, "vel_subset_onset_verdict_2026-10-28.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(rep, f, ensure_ascii=False, indent=1)
    print(json.dumps({k: rep[k] for k in ("V1", "V2", "V3", "verdict")},
                     ensure_ascii=False, indent=1))
    print("V4 mid cells with no improvement:", rep["V4"]["mid_cells_no_improvement"])
    print("written", out)


if __name__ == "__main__":
    main()
