#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24t 续（Y7）：把上界从 "U>35" 继续往上顶到出现劣化为止。

@file    check_vel_subset_capupper2.py
@brief   用与 Y6b **完全相同**的劣化定义，判 cap∈{40,50} 两档（两段 seeds），给出 U 或 "U>50"
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details Y6 的结果：cap 25/30/35 相对 cap15 都没有出现 ">25 m 失效"或"max 抬升 >5 m"，
         即 **U > 35 m**；而 nocap（无阈值）在两段上分别是 2/50 与 6/50、max 58.4 / 105.0 m。
         ⇒ 上界在 (35, 无阈值) 之间。§4.24q 的追踪里最大的"首次触发残差"是 seed28 的 39.86 m
         （它在 nocap 下变成 104.97 m），所以 U 很可能就卡在 35–40 m 之间。
         本轮只补两个点（40、50），判据不改（改判据=移动门柱），目的是**给默认值找出双侧裕度**：
         现在已知下界 L=14，若 U≈40，则平台宽 26 m，默认取 25 m 有 +11/−15 m 的裕度；
         若 U>50，则平台更宽，但"nocap 会炸"这件事仍然成立 ⇒ 上界一定存在，只是本弹道的
         残差分布没把样本推到那里，届时如实登记"U 未被本数据集界定"。
@frame   NED；pos_rms [m]
@units   米 / 个 seed
@note    预登记判据（与 Y6b 同定义，不新增）：
         Y7a 参照一致性：本轮不重跑 cap25，改为**直接读 §4.24t 归档的 cap25 统计**做基线；
              若缺文件即退出（不允许用别的档当基线）。
         Y7b 上界：U = 使"任一段出现 `>25 m` 失效，或 `max` 相对 cap25 增加 >5 m"的最小 cap∈{40,50}；
              两档都不满足 ⇒ 记 U>50 并明确"本数据集未界定上界"。
         Y7c 决策口径：只有当下界 L 与上界 U 都被实测、且 U−L≥10 m 时，才允许提出
              "把默认从 15 m 挪到区间内一点"的建议；否则维持"下界已测、上界未测"的单侧表述。
@see     sim/check_vel_subset_capupper.py（Y6）、报告 §4.24t
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
FAIL_M = 25.0
BASE_REF = "vel_subset_capupper_2026-10-28.json"
ARMS = [("v40_s51", 51, "40"), ("v50_s51", 51, "50"),
        ("v40_s1", 1, "40"), ("v50_s1", 1, "50")]
SEG_OF = {51: "51-100", 1: "1-50"}


def load(tag, start):
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
    keys = list(range(start, start + 50))
    missing = sorted(set(keys) - set(out))
    assert not missing, "%s 缺 seed：%s" % (tag, missing[:5])
    return {s: out[s] for s in keys}


def stat(d):
    p = np.array([float(v["pos_rms"]) for v in d.values()])
    return {"n": len(p), "fail_gt25": int((p > FAIL_M).sum()),
            "p90": round(float(np.percentile(p, 90)), 3),
            "p95": round(float(np.percentile(p, 95)), 3),
            "max": round(float(p.max()), 3),
            "att": round(float(np.median([float(v["nav_att_rmse"]) for v in d.values()])), 3)}


def main():
    ref_path = os.path.join(SD, BASE_REF)
    if not os.path.exists(ref_path):
        sys.exit("缺 Y6 基线 %s ⇒ 不允许换基线，本轮作废" % BASE_REF)
    base = json.load(open(ref_path, encoding="utf-8"))["per_segment"]
    per = {seg: {c: dict(base[seg]["25"]) for c in ["25"]} for seg in ("51-100", "1-50")}

    flags = {}
    for tag, start, cap in ARMS:
        seg = SEG_OF[start]
        s = stat(load(tag, start))
        per[seg][cap] = s
        flags["%s_cap%s" % (seg, cap)] = {
            "fail": s["fail_gt25"] > base[seg]["25"]["fail_gt25"],
            "max_up_gt5": s["max"] > base[seg]["25"]["max"] + 5.0}

    first_u = None
    for cap in ("40", "50"):
        if any(flags["%s_cap%s" % (seg, cap)]["fail"] or
               flags["%s_cap%s" % (seg, cap)]["max_up_gt5"] for seg in per):
            first_u = cap
            break
    L = 14
    U = int(first_u) if first_u else None
    rep = {
        "per_segment": per,
        "Y7a": {"baseline": "%s 的 cap25（两段）" % BASE_REF, "pass": True},
        "Y7b": {"per_cap_flags": flags, "upper_edge_cap": first_u,
                "upper_edge_measured": first_u is not None,
                "text": ("U = %s m" % first_u if first_u else
                         "U > 50 m：40/50 两档都无劣化 ⇒ 本数据集未界定上界"
                         "（但 nocap 一定炸，说明上界存在、只是没被这些 seed 推到）"),
                "pass": True},
        "Y7c": {"L_measured": L, "U_measured": U, "width_m": (U - L) if U else None,
                "may_propose_default_move": bool(U and (U - L) >= 10), "pass": True},
    }
    ok = rep["Y7a"]["pass"] and rep["Y7b"]["pass"] and rep["Y7c"]["pass"]
    rep["verdict"] = ("UPPER_EDGE_MEASURED" if (ok and U) else "UPPER_EDGE_BEYOND_50")
    out = os.path.join(SD, "vel_subset_capupper2_2026-10-28.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(rep, f, ensure_ascii=False, indent=1)
    print(json.dumps({k: rep[k] for k in ("Y7b", "Y7c", "verdict")}, ensure_ascii=False, indent=1))
    print("written", out)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
