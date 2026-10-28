#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24p W1–W4 裁决器：速度子集"位置门同时越门"修复是否成立。

@file    check_vel_subset_posgate.py
@brief   对齐 2×2（GNSS 10/20 Hz × posgate 0/1）与两个基线臂，按 §4.24p 预登记的
         W1–W4 逐条给 PASS/FAIL，并落盘 JSON 供报告引用
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 背景：V5 发现"≥20 Hz 与速度子集软恢复不可叠加"——子集在位置门未越门时介入，
         把速度拉对而位置留在旧值，打破位置-速度一致性，此后位置新息持续越门
         （SIL gate_pos 30→1000 帧），30.5 g/20 Hz 从 7.7 m 恶化到 105 m RMS。
         修复＝要求位置门同时越门（`--gnss-vel-subset-posgate 1`，代码默认 true）。
         所有比较先 assert 配对数；posgate 两臂必须来自**同一二进制**，
         否则比较无效（本工程已多次因跨二进制归因出错）。
@frame   NED；pos_rms / pos_max [m]，nav_att_rmse [deg]
@units   米 / 度
@note    W2 的"不差于 20 Hz 基线"用逐 seed 配对；W3 的"不吃收益"用发生率与 p95
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
TAGS = {
    "b10_off": "onset50",          # 10 Hz 子集关（基线）
    "b10_pg0": "vel50sub",         # 10 Hz 子集开 + 旧触发（posgate 隐式 0）
    "b20_off": "hz20only",         # 20 Hz 子集关（基线）
    "b20_pg0": "vel50sub_hz20",    # 20 Hz 子集开 + 旧触发
    "w10_pg1": "w10pg1",           # 10 Hz 子集开 + 新触发（修复后）
    "w20_pg1": "w20pg1",           # 20 Hz 子集开 + 新触发（修复后）
    "w_off_pg0": "wsuboff_pg0",    # 子集关 + posgate=0（W4 惰性）
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


def stats(d):
    p = np.array([float(v["pos_rms"]) for v in d.values()])
    return {"n": len(p), "fail": int((p > 25).sum()), "med": round(float(np.median(p)), 3),
            "p95": round(float(np.percentile(p, 95)), 3), "max": round(float(p.max()), 3),
            "att": round(float(np.median([float(v["nav_att_rmse"]) for v in d.values()])), 3)}


def main():
    arms = {k: load(v) for k, v in TAGS.items()}
    rep = {}

    # W1 完整性
    rep["W1"] = {k: arms[k].__len__() for k in arms}
    rep["W1"]["pass"] = all(len(arms[k]) == 50 for k in arms)

    def worse_cnt(a, b):
        """a 相对 b 变差 (>10%) 的 seed 列表（a、b 同 seed 集）"""
        keys = sorted(set(a) & set(b))
        assert len(keys) == 50, "配对 %d != 50 ⇒ 判定无效" % len(keys)
        return [s for s in keys
                if float(a[s]["pos_rms"]) > float(b[s]["pos_rms"]) * 1.10 + 1e-9]

    # W2 修好 V5 的伤害（20 Hz）
    s20 = {k: stats(arms[k]) for k in ("b20_off", "b20_pg0", "w20_pg1")}
    w2_worse = worse_cnt(arms["w20_pg1"], arms["b20_off"])
    rep["W2"] = {"per_arm": s20, "worse_than_20hz_baseline": w2_worse,
                 "max_halved": s20["w20_pg1"]["max"] <= 0.5 * s20["b20_pg0"]["max"],
                 "pass": (s20["w20_pg1"]["max"] <= 0.5 * s20["b20_pg0"]["max"]
                          and not w2_worse)}

    # W3 不吃 §4.16 收益（10 Hz）
    s10 = {k: stats(arms[k]) for k in ("b10_off", "b10_pg0", "w10_pg1")}
    # 附加：若 10 Hz 下子集过去只在"双门同时越门"时触发，则新旧臂应逐 seed 完全相同
    # —— 这是"修复在 10 Hz 上什么都没改"的最强形式（跨二进制位相同）。
    k10 = sorted(set(arms["w10_pg1"]) & set(arms["b10_pg0"]))
    assert len(k10) == 50, "W3 配对 %d != 50 ⇒ 判定无效" % len(k10)
    same10 = [s for s in k10
              if float(arms["w10_pg1"][s]["pos_rms"]) == float(arms["b10_pg0"][s]["pos_rms"])]
    rep["W3"] = {"per_arm": s10, "pg0_pg1_identical_seeds": len(same10),
                 "pass": (s10["w10_pg1"]["fail"] == s10["b10_pg0"]["fail"]
                          and s10["w10_pg1"]["p95"] <= 1.2 * s10["b10_pg0"]["p95"])}

    # W4 父通道关闭时子开关无副作用。
    # ★ 这条同时是"新二进制对默认路径惰性"的跨二进制证明：
    #   b10_off 来自修复前的 tv_32g_hf13，w_off_pg0 来自修复后的 tv_32g_w1，
    #   两者只差本次 posgate 修复；若逐 seed 相同 ⇒ 修复在父通道关闭时零影响。
    keys = sorted(set(arms["b10_off"]) & set(arms["w_off_pg0"]))
    assert len(keys) == 50, "W4 配对 %d != 50 ⇒ 判定无效" % len(keys)
    diff = [s for s in keys
            if float(arms["b10_off"][s]["pos_rms"]) != float(arms["w_off_pg0"][s]["pos_rms"])]
    rep["W4"] = {"pairs": len(keys), "diff_seeds": diff, "pass": not diff}

    rep["verdict"] = ("FIX_ACCEPTED" if all(rep[k]["pass"] for k in ("W1", "W2", "W3", "W4"))
                      else "REVERT_FIX_AND_RECORD_LIMITATION")
    rep["gate"] = ("W1 五臂各 50 seeds；W2 20 Hz 尾部 max 至少砍半且无一 seed 比 20 Hz 基线差 >10%；"
                   "W3 10 Hz 发生率与 p95 不劣于旧触发；W4 子集关时 posgate 开关逐 seed 无副作用")
    out = os.path.join(SD, "vel_subset_posgate_verdict_2026-10-28.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(rep, f, ensure_ascii=False, indent=1)
    print(json.dumps({k: rep[k] for k in ("W1", "W2", "W3", "W4", "verdict")},
                     ensure_ascii=False, indent=1))
    print("written", out)


if __name__ == "__main__":
    main()
