#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24t 裁决器：量出触发上限的**上边界**（Y6a–Y6c），把 [L, U) 补成可写的区间。

@file    check_vel_subset_capupper.py
@brief   在 cap∈{25,30,35} × 两段 seeds（1–50 / 51–100）上找最小的使结果开始劣化的 cap，
         与 §4.24s 已测出的下界 L=14 合成平台区间 [L, U)
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details §4.24s（Y5）的结论：下界已测出（cap 12→13→14→15 的发生率为 5→2→0→0，单调，
         L=14），但**上界没测出来**——cap25 与 cap15 相比没有任何一段出现 `>25 m` 失效，
         `max` 也不升（22.219 / 23.206 m），反而在 6 个 seed 上把结果往"接近子集关"的方向挪了
         ±1 m，并在 seeds 68/85 上比 cap15 更好（19.52→18.67、20.24→17.91）。
         也就是说：nocap 的 44.9–105 m 尾部来自**残差 >25 m 的那些触发**，
         而 ≤25 m 的触发即使幅值判据"应该"拦它（§4.24q 标的 6 个有害 seed 首次残差 21.3–22.1 m），
         单放行一次也只是 ±1 m 量级的变化，不是灾难。
         ⇒ 这一轮唯一要回答的问题是：**上边界在哪里**，从而决定默认值该不该从 15 挪到区间中部。
         ★ 纪律：不改 §4.24s 的判据再跑（那是移动门柱）；本轮是新判据 Y6，跑前写死。
         ★ 若 U 与 L 之间宽度足够，默认值改动的验收必须包含"默认值变更 → 宿主 T25 断言同步"，
           不能只改代码注释。
@frame   NED；pos_rms [m]、pos_max [m]、nav_att_rmse [deg]
@units   米 / 个 seed
@note    预登记判据（跑前写死，不看结果改）：
         Y6a 参照一致性：本轮的 cap25 臂必须与 §4.24s 归档的 cap25 **逐 seed 完全相同**
              （两段各 50 条、5 个字段），否则说明构建或参数漂了，本轮整体作废。
         Y6b 上界定义：U = 使"任一段出现 `>25 m` 失效，或 `max` 相对 cap25 增加 >5 m"
              的最小 cap∈{30, 35}；若两档都不满足 ⇒ 记 U>35，并明确"上界仍未测出，
              只在 nocap（无阈值）处确认存在 44.9–105 m 的尾部"。
         Y6c 区间表述：只有当 L 与 U 都实测出且 U − L ≥ 5 m，才允许把结论写成
              "平台区间 [L, U) + 默认取区间内一点"；否则保持"下界已测、上界未测"的单侧表述。
              发生率一律按段报，双峰不报中位当精度。
         判定规则：Y6a 成立是硬前提；Y6b 给出 U 或"U>35"二选一；
              Y6c 决定 §8/§4.24q 的措辞能否升级。**默认值是否改到区间中部，本轮只出证据不出决定**
              （改默认要过宿主单测 + 门禁 + 两段复跑，单独一步做）。
@see     sim/check_vel_subset_capband.py（Y5）、sim/run_305g_onset_scan.py（数据驱动）、报告 §4.24s/§4.24t
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
FAIL_M = 25.0
REF_KEYS = ("pos_rms", "pos_max", "nav_att_rmse", "outlier_rejects", "reacquire_resets")
# (本轮 tag, 段, seed 起, cap, 参照 tag（§4.24s 归档，用于 Y6a）)
ARMS = [("u25_s51", "51-100", 51, "25", "y20_c25"),
        ("u30_s51", "51-100", 51, "30", None),
        ("u35_s51", "51-100", 51, "35", None),
        ("u25_s1", "1-50", 1, "25", "p20_c25"),
        ("u30_s1", "1-50", 1, "30", None),
        ("u35_s1", "1-50", 1, "35", None)]


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


def want(start):
    return list(range(start, start + 50))


def stat(d, keys):
    p = np.array([float(d[s]["pos_rms"]) for s in keys])
    return {"n": len(keys), "fail_gt25": int((p > FAIL_M).sum()),
            "p90": round(float(np.percentile(p, 90)), 3),
            "p95": round(float(np.percentile(p, 95)), 3),
            "max": round(float(p.max()), 3),
            "att": round(float(np.median([float(d[s]["nav_att_rmse"]) for s in keys])), 3)}


def main():
    data = {tag: load(tag) for tag, _seg, _start, _cap, _ref in ARMS}
    per = {}
    for tag, seg, start, cap, ref in ARMS:
        keys = want(start)
        missing = sorted(set(keys) - set(data[tag]))
        assert not missing, "%s 缺 seed：%s" % (tag, missing[:5])
        per.setdefault(seg, {})[cap] = stat(data[tag], keys)
    rep = {"per_segment": per}

    # Y6a 参照一致性：cap25 两臂必须与 §4.24s 归档逐 seed 相同
    checks = {}
    for tag, seg, start, cap, ref in ARMS:
        if ref is None:
            continue
        keys = want(start)
        refd = load(ref)
        assert len(set(keys) & set(refd)) == 50, "%s 与 %s 配对不足 50" % (tag, ref)
        diff = {k: [s for s in keys if data[tag][s].get(k) != refd[s].get(k)] for k in REF_KEYS}
        checks["%s|%s" % (seg, cap)] = {k: len(v) for k, v in diff.items()}
    rep["Y6a"] = {"diff_counts": checks,
                  "pass": all(v == 0 for c in checks.values() for v in c.values())}

    # Y6b 上界
    u = {}
    for seg, base_cap, base_tag in (("51-100", "25", "y20_c25"), ("1-50", "25", "p20_c25")):
        base = per[seg][base_cap]
        for cap in ("30", "35"):
            cur = per[seg][cap]
            u["%s_cap%s" % (seg, cap)] = {
                "fail": cur["fail_gt25"] > base["fail_gt25"],
                "max_up_gt5": cur["max"] > base["max"] + 5.0}
    first_u = None
    for cap in ("30", "35"):
        if any(u["%s_cap%s" % (seg, cap)]["fail"] or u["%s_cap%s" % (seg, cap)]["max_up_gt5"]
               for seg in per):
            first_u = cap
            break
    rep["Y6b"] = {"per_cap_flags": u, "upper_edge_cap": first_u,
                  "upper_edge_measured": first_u is not None,
                  "text": ("上界 U = %s m（该档相对 cap25 出现失效或 max 抬升 >5 m）" % first_u
                           if first_u else
                           "U > 35 m：30/35 两档都没有劣化 ⇒ 上界仍未测出，"
                           "尾部只在完全无阈值(nocap)时出现（44.9–105 m）"),
                  "pass": True}

    # Y6c 区间表述
    L = 14
    U = int(first_u) if first_u else None
    rep["Y6c"] = {"L_measured": L, "U_measured": U,
                  "width_m": (U - L) if U else None,
                  "may_write_interval": bool(U and (U - L) >= 5),
                  "caliber": "发生率按段报；双峰不报中位；默认值变更本轮只出证据不出决定",
                  "pass": True}

    ok = rep["Y6a"]["pass"] and rep["Y6b"]["pass"] and rep["Y6c"]["pass"]
    rep["verdict"] = ("UPPER_EDGE_MEASURED" if (ok and U) else
                      ("UPPER_EDGE_BEYOND_35" if ok else "REFERENCE_MISMATCH_INVALID"))
    rep["gate"] = ("Y6a cap25 与 §4.24s 归档 5 字段逐 seed 相同；Y6b 给出 U 或 U>35；"
                   "Y6c 只有 U−L≥5 才允许写区间")
    out = os.path.join(SD, "vel_subset_capupper_2026-10-28.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(rep, f, ensure_ascii=False, indent=1)
    print(json.dumps({k: rep[k] for k in ("Y6a", "Y6b", "Y6c", "verdict")},
                     ensure_ascii=False, indent=1))
    print("written", out)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
