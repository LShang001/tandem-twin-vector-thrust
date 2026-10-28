#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24s 裁决器：把 15 m 触发上限从"单点"改写成"平台区间"的边界映射（Y5a–Y5c）。

@file    check_vel_subset_capband.py
@brief   在 20 Hz 上扫 cap∈{12,13,14,15,18,21,25}（seeds 51–100）与 cap∈{15,25}（seeds 1–50），
         定位"再收紧就误杀有益触发"的下界 L 与"再放宽就放行有害触发"的上界 U
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 为什么要这一轮：§4.24r（Y1–Y4）判 **CAP_NOT_GENERALIZED**，但失败的原因不是
         "15 m 在新 seed 上失效"——恰恰相反，20 Hz 新 seed 上 cap15 与 cap21 都是 **0/50**
         且逐 seed 相同、max 22.2 m，而**收紧到 12 m 反而出现 5/50 失效**（比不加阈值的
         2/50 还差）。⇒ Y4 的"三档发生率差 ≤1 seed 即平坦"这条预登记判据把"12 更差"
         记成了"不平坦"，这本身是有信息量的：**阈值不是越小越保守**。
         因此本轮的目标不是重测 15，而是**把区间量出来**：
         下界 L = 最小的使发生率为 0 的 cap；上界 U = 开始出现有害放行的 cap。
         ★ 沟通纪律：两段 seed 的曲线必须并列报，不得合并；只报平台区间与两侧发生率，
           不给"最优阈值"（本工程已多次证明按样本挑单点会在下一段样本上翻面）。
@frame   NED；pos_rms [m]、nav_att_rmse [deg]
@units   米 / 个 seed
@note    预登记判据（跑前写死）：
         Y5a 下界单调性：seeds 51–100 上 cap 由 12→13→14→15 的发生率必须**单调不增**；
              若非单调 ⇒ 下界不是单一阈值，登记为"下界形态未知"并停止给区间。
              记录 L = 使发生率为 0 的最小 cap。
         Y5b 上界可达性：若 cap25 相对 cap15 在任一段 seed 上出现"发生率升高 或 max 升高 >5 m"
              ⇒ 上界 U ∈ (21,25]，区间写成 [L, U)；
              若两段的 cap25 与 cap15 逐 seed 相同 ⇒ 上界未测出，登记 U>25 并明确写
              "上界未测，15 m 的裕度只有下界一侧有证据"。
         Y5c 口径：双峰段只报发生率 + 按簇的 max/p95，不报中位当精度指标；
              所有比较先 assert 配对数（seeds 51–100 段 50、seeds 1–50 段 50）。
         判定：Y5a∧(Y5b 两种结果之一明确) ⇒ 把 §8 的单点表述替换为"区间 [L,U) 的保守下端"；
              Y5a 不成立 ⇒ 撤回"区间"表述，只保留"15 m 在两段 seed 上均为 0/50"的事实。
@see     sim/check_vel_subset_capval.py（Y1–Y4）、sim/run_305g_onset_scan.py（数据驱动）、
         报告 §4.24r/§4.24s
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
FAIL_M = 25.0
# (tag, 段名, seed 起, cap 标签)
ARMS = [("y20_c12", "51-100", 51, "12"), ("y20_c13", "51-100", 51, "13"),
        ("y20_c14", "51-100", 51, "14"), ("y20_c15", "51-100", 51, "15"),
        ("y20_c18", "51-100", 51, "18"), ("y20_c21", "51-100", 51, "21"),
        ("y20_c25", "51-100", 51, "25"),
        ("p20_c15", "1-50", 1, "15"), ("p20_c25", "1-50", 1, "25")]
# p20_c15 直接复用 §4.24q 已归档的 20 Hz cap15 臂（tag=q20，同二进制 tv_32g_q1、seeds 1–50），
# 不重跑 ⇒ 省 50 次；两份证据的 exe md5 与参数在 attrib305_2026-10-28_q20_verdict.json 里一致。
TAG_ALIAS = {"p20_c15": "q20"}


def load(tag, start):
    real = TAG_ALIAS.get(tag, tag)
    path = os.path.join(SD, "attrib305_2026-10-28_%s.jsonl" % real)
    if not os.path.exists(path):
        sys.exit("缺证据文件：%s" % path)
    out = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                r = json.loads(line)
                out[int(r["seed"])] = r
    want = list(range(start, start + 50))
    missing = sorted(set(want) - set(out))
    assert not missing, "%s 缺 seed：%s" % (tag, missing[:5])
    return {s: out[s] for s in want}


def stat(d):
    p = np.array([float(v["pos_rms"]) for v in d.values()])
    return {"n": len(p), "fail_gt25": int((p > FAIL_M).sum()),
            "p90": round(float(np.percentile(p, 90)), 3),
            "p95": round(float(np.percentile(p, 95)), 3),
            "max": round(float(p.max()), 3),
            "att": round(float(np.median([float(v["nav_att_rmse"]) for v in d.values()])), 3)}


def identical(a, b):
    """逐 seed pos_rms 完全相同 ⇒ True；同时返回不同的 seed 列表。"""
    diff = [s for s in sorted(a) if float(a[s]["pos_rms"]) != float(b[s]["pos_rms"])]
    return (not diff), diff


def main():
    arms = {tag: load(tag, start) for tag, _, start, _ in ARMS}
    per = {}
    for tag, seg, _start, cap in ARMS:
        per.setdefault(seg, {})[cap] = stat(arms[tag])

    rep = {"per_segment": per}

    # Y5a 下界单调性（seeds 51–100，cap 12→13→14→15）
    low = [per["51-100"][c]["fail_gt25"] for c in ("12", "13", "14", "15")]
    mono = all(low[i] >= low[i + 1] for i in range(len(low) - 1))
    zeros = [c for c in ("12", "13", "14", "15", "18", "21", "25")
             if per["51-100"][c]["fail_gt25"] == 0]
    rep["Y5a"] = {"rates_12_13_14_15": low, "monotone_nonincreasing": mono,
                  "L_first_zero_cap": min(zeros) if zeros else None,
                  "pass": bool(mono)}

    # Y5b 上界可达性：cap25 vs cap15（两段分别比）
    same51, diff51 = identical(arms["y20_c15"], arms["y20_c25"])
    same150, diff150 = identical(arms["p20_c15"], arms["p20_c25"])
    worse51 = (per["51-100"]["25"]["fail_gt25"] > per["51-100"]["15"]["fail_gt25"]
               or per["51-100"]["25"]["max"] > per["51-100"]["15"]["max"] + 5.0)
    worse150 = (per["1-50"]["25"]["fail_gt25"] > per["1-50"]["15"]["fail_gt25"]
                or per["1-50"]["25"]["max"] > per["1-50"]["15"]["max"] + 5.0)
    bounded = worse51 or worse150
    rep["Y5b"] = {"c25_vs_c15_identical_51_100": same51, "diff_seeds_51_100": diff51[:8],
                  "c25_vs_c15_identical_1_50": same150, "diff_seeds_1_50": diff150[:8],
                  "upper_edge_measured": bool(bounded),
                  "verdict_text": ("上界 U ∈ (21,25]（cap25 相对 cap15 出现失效/max 抬升）"
                                   if bounded else
                                   "上界未测出（cap25 与 cap15 两段均逐 seed 相同或无劣化）⇒ "
                                   "只能写 U>25，15 m 的下界有证据、上界没有"),
                  "pass": True}

    rep["Y5c"] = {"caliber": "发生率+按簇 max/p95；双峰不报中位；两段并列不合并",
                  "pass": True}

    ok = rep["Y5a"]["pass"] and rep["Y5b"]["pass"] and rep["Y5c"]["pass"]
    rep["verdict"] = ("BAND_MEASURED" if (ok and rep["Y5b"]["upper_edge_measured"]) else
                      ("BAND_LOWER_ONLY" if ok else "MONOTONICITY_BROKEN"))
    rep["gate"] = ("Y5a cap12→15 发生率单调不增并给出首个零发生率 cap；"
                   "Y5b 明确 cap25 相对 cap15 是否出现劣化（上界测出或未测出）；"
                   "Y5c 口径（发生率+簇、两段并列）")
    out = os.path.join(SD, "vel_subset_capband_2026-10-28.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(rep, f, ensure_ascii=False, indent=1)
    print(json.dumps({k: rep[k] for k in ("Y5a", "Y5b", "verdict")}, ensure_ascii=False, indent=1))
    print("written", out)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
