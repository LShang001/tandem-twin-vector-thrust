# -*- coding: utf-8 -*-
"""杆臂盲区修复在 30.5 g 触发场景上的四路对照汇总（逐 seed 配对）。
@file    summarize_lever_fix_ab.py
@brief   合并 revlv/fixlv 两份 jsonl，按 (二进制, 子集开关) 四组给出位置统计与
         逐 seed 配对差值，回答"修复前 ON 是否比 OFF 更差、修复后 ON 是否恢复"。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 四组来自 revcmp（修复前）与 fixcmp（修复后）两份 jsonl：逐组给位置中位/p95/max、
         >25 m 计数与拒帧/重锚/子集恢复次数；再按 seed 配对给 ON−OFF 的中位差与
         变差/变好 seed 数，用于回答"修复前 ON 是否比 OFF 差、修复后 ON 是否恢复"。
@frame   NED
@units   m

用法：py -3.12 sim/summarize_lever_fix_ab.py
"""
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(os.path.dirname(HERE), "sim-data")
GROUPS = [("revlv", "修复前", "pop50_off"), ("revlv", "修复前", "pop50_on"),
          ("fixlv", "修复后", "pop50_off"), ("fixlv", "修复后", "pop50_on")]
TAGS = {"revlv": "revcmp_2026-10-27", "fixlv": "fixcmp_2026-10-27"}


def load(tag):
    """读 sil32g_velrec_<tag>.jsonl，按 arm 分组、seed 索引，返回 {arm: {seed: 记录}}；文件缺失返回空 dict。"""
    rows = {}
    path = os.path.join(DATA, f"sil32g_velrec_{tag}.jsonl")
    if not os.path.exists(path):
        return rows
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            r = json.loads(line)
            rows.setdefault(r["arm"], {})[r["seed"]] = r
    return rows


def q(vals, p):
    """取分位数（最近秩法：排序后按 p 取下标），空序列返回 NaN。"""
    if not vals:
        return float("nan")
    v = sorted(vals)
    i = min(len(v) - 1, int(p * len(v)))
    return v[i]


def main():
    """命令行入口：按修复前/后 × 子集关/开四组打印位置统计，并打印两轮 ON−OFF 的逐 seed 配对中位差与变差/变好 seed 数。"""
    cache = {t: load(TAGS[t]) for t in TAGS}
    print(f"{'组':<16}{'n':>4}{'中位':>9}{'p95':>9}{'max':>9}{'>25m':>7}"
          f"{'拒帧':>8}{'重锚':>7}{'vsub':>7}")
    print("-" * 74)
    per = {}
    for tag, lab, arm in GROUPS:
        d = cache[tag].get(arm, {})
        pr = [r["pos_rms"] for r in d.values()
              if isinstance(r.get("pos_rms"), (int, float))
              and math.isfinite(r["pos_rms"])]
        per[(tag, arm)] = d
        rej = sum(r.get("outlier_rejects", 0) for r in d.values())
        rec = sum(r.get("reacquire_resets", 0) for r in d.values())
        vs = sum(r.get("vel_sub_rec", 0) for r in d.values())
        print(f"{lab}/{arm[-3:]:<9}{len(pr):>4}{q(pr, .5):>9.2f}"
              f"{q(pr, .95):>9.2f}{max(pr) if pr else float('nan'):>9.2f}"
              f"{sum(1 for x in pr if x > 25):>7}{rej:>8}{rec:>7}{vs:>7}")
    print()
    for tag, lab in (("revlv", "修复前"), ("fixlv", "修复后")):
        off, on = per.get((tag, "pop50_off"), {}), per.get((tag, "pop50_on"), {})
        seeds = sorted(set(off) & set(on))
        delta = [on[s]["pos_rms"] - off[s]["pos_rms"] for s in seeds
                 if math.isfinite(on[s].get("pos_rms", float("nan")))
                 and math.isfinite(off[s].get("pos_rms", float("nan")))]
        if not delta:
            continue
        worse = sum(1 for x in delta if x > 0.5)
        better = sum(1 for x in delta if x < -0.5)
        print(f"{lab}：逐 seed 配对 n={len(delta)}  ON−OFF 中位 {q(delta, .5):+.2f} m"
              f"，ON 变差>0.5m 的 seed {worse} 个，变好 {better} 个")


if __name__ == "__main__":
    main()
