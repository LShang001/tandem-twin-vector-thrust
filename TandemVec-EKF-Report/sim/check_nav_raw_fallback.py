#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24m 判据裁决器：导航增量源头故障回退（J1–J5，跑前已写死）。

@file    check_nav_raw_fallback.py
@brief   读 `_nr_{base,raw,fb,inert}` 与旧臂 `_fzraw0`，按 §4.24m 预登记的
         J1–J5 逐条给出 PASS/FAIL 与依据数值，落盘 JSON 供报告引用
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 判据原文见报告 §4.24m；本脚本不修改判据，只做裁决。
         所有比较先 assert 配对数，交集为 0 时直接报错退出（防空洞绿灯）。
@frame   指标取 SIL JSON 的 pos_rms（全程位置 RMSE，m）
@units   长度 m；百分比相对各自参照臂
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
SUF = {"base": "_nrb", "raw": "_nrr", "fb": "_nrf", "inert": "_nri",
       "rin": "_nrin"}


def load(tag, suf):
    path = os.path.join(SD, f"missile-sil-2026-10-28_{tag}.jsonl")
    if not os.path.exists(path):
        sys.exit(f"缺证据文件：{path}")
    out = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            r = json.loads(line)
            cfg = r["config"]
            assert cfg.endswith(suf), (cfg, suf)
            out.setdefault(cfg[:-len(suf)].rstrip("_"), {})[r["seed"]] = r
    return out


def med(recs, cell, key="pos_rms"):
    v = np.array([r[key] for r in recs[cell].values()], float)
    assert len(v) >= 15, f"{cell} 样本 {len(v)} < 15 ⇒ 口径异常"
    return float(np.median(v)), int(len(v))


def main():
    B = load("nr_base", SUF["base"])
    R = load("nr_raw", SUF["raw"])
    F = load("nr_fb", SUF["fb"])
    I = load("nr_inert", SUF["inert"])
    N = load("nr_rin", SUF["rin"])   # 非故障格上的纯 raw 臂（J4 参照）
    fault_cells = sorted(B)
    assert fault_cells == sorted(R) == sorted(F), (fault_cells, sorted(R), sorted(F))
    res = {"cells": fault_cells, "checks": {}, "detail": {}}

    # J1 / J2：回退把冻结格拉回默认水平
    for name, cell in (("J1_imu_stuck", "imu_stuck_icm_full"),
                       ("J2_imu_stuck2s", "imu_stuck2s_icm_full")):
        mb, _ = med(B, cell)
        mr, _ = med(R, cell)
        mf, n = med(F, cell)
        ok = mf <= 1.1 * mb
        res["checks"][name] = dict(pass_=bool(ok), base=round(mb, 4), raw=round(mr, 4),
                                   fb=round(mf, 4), n=n, gate="fb <= 1.1 x base")
    # J3：NaN 两格不因回退变差
    worst3 = 0.0
    for cell in ("imu_nan_icm_full", "imu_nan2s_icm_full"):
        mr, _ = med(R, cell)
        mf, _ = med(F, cell)
        worst3 = max(worst3, mf / mr - 1.0)
    res["checks"]["J3_nan_cells"] = dict(pass_=bool(worst3 <= 0.05),
                                         worst_rel=round(worst3 * 100, 2),
                                         gate="fb vs raw <= +5%")
    # J4：非故障格逐 seed 相同 + nrfb=0
    inert_cells = sorted(I)
    same, nrfb = {}, {}
    assert set(N) == set(inert_cells), (sorted(N), inert_cells)
    for cell in inert_cells:
        seeds = sorted(set(I[cell]) & set(N[cell]))
        assert len(seeds) >= 15, f"{cell} 配对 {len(seeds)} < 15 ⇒ 无效"
        same[cell] = int(sum(1 for s in seeds
                             if I[cell][s]["pos_rms"] == N[cell][s]["pos_rms"]))
        nrfb[cell] = int(max(int(r.get("nrfb", -1)) for r in I[cell].values()))
    ok4 = all(v == len(sorted(set(I[c]) & set(N[c]))) for c, v in same.items()) and \
        all(v == 0 for v in nrfb.values())
    res["checks"]["J4_inert"] = dict(pass_=bool(ok4), same_seed_count=same,
                                     nrfb_max=nrfb, gate="非故障格 raw+回退 == 纯raw 逐 seed 相同且 nrfb=0")
    # J4b：故障格必须真的触发回退（否则 J1/J2 是空洞绿灯）
    fb_frames = {c: int(max(int(r.get("nrfb", -1)) for r in F[c].values())) for c in fault_cells}
    ok4b = all(v > 0 for v in fb_frames.values())
    res["checks"]["J4b_fallback_executed"] = dict(pass_=bool(ok4b), nrfb_max=fb_frames,
                                                  gate="故障格 nrfb > 0（分支真被执行）")
    # J5：新构建的 raw 臂与旧 40-seed 臂（tv_hf6）seeds 1-20 逐位相同
    OLD = load("fzraw0", "_fz0")
    ok5, det5 = True, {}
    compared = [c for c in fault_cells if c in OLD]
    det5["skipped"] = [c for c in fault_cells if c not in OLD]
    assert len(compared) >= 3, f"J5 可比格仅 {len(compared)} ⇒ 参照不足，判据不可下"
    for cell in compared:
        key = cell
        seeds = sorted(set(R[cell]) & set(OLD[key]))
        seeds = [s for s in seeds if s <= 20]
        assert len(seeds) >= 15, f"{cell} 与旧臂配对 {len(seeds)} < 15 ⇒ 无效"
        nd = int(sum(1 for s in seeds if R[cell][s]["pos_rms"] != OLD[key][s]["pos_rms"]))
        det5[cell] = f"{len(seeds) - nd}/{len(seeds)} 相同"
        ok5 = ok5 and nd == 0
    res["checks"]["J5_raw_path_unchanged"] = dict(pass_=bool(ok5), detail=det5,
                                                  gate="tv_hf9 raw == tv_hf6 raw（seeds 1-20）")
    # 逐 seed 明细（供报告表格）
    res["detail"] = {cell: {
        "base_med": round(med(B, cell)[0], 4), "raw_med": round(med(R, cell)[0], 4),
        "fb_med": round(med(F, cell)[0], 4),
        "fb_vs_base_pct": round((med(F, cell)[0] / med(B, cell)[0] - 1) * 100, 2),
        "raw_vs_base_pct": round((med(R, cell)[0] / med(B, cell)[0] - 1) * 100, 2),
        "fb_better_than_raw_seeds": int(sum(1 for s in F[cell]
                                            if s in R[cell] and
                                            F[cell][s]["pos_rms"] < R[cell][s]["pos_rms"])),
        "n_seeds": len(F[cell])} for cell in fault_cells}
    verdict = all(v["pass_"] for v in res["checks"].values())
    res["verdict_all_pass"] = bool(verdict)
    out = os.path.join(SD, "nav_raw_fallback_verdict_2026-10-28.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=1)
    print(json.dumps(res, ensure_ascii=False, indent=1))
    print("=> 全部判据", "PASS" if verdict else "未全通过", " 落盘", out)


if __name__ == "__main__":
    main()
