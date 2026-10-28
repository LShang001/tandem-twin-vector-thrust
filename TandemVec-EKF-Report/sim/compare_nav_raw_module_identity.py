#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24m 固件化身份校验：SIL 健康键回退改用 `NavImuIncrementSelect` 模块后是否逐 seed 不变。

@file    compare_nav_raw_module_identity.py
@brief   对齐两份 47 格健康键矩阵 jsonl（改前 `tv_hf10` / 改后重建二进制），
         按 (config, seed) 配对逐字段比对，配对数不足即判无效
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 背景：`fw_sil_env_main.cpp` 的内联锁存（Failed 拍置满 10 000、其后递减）
         换成固件模块 `include/NavImuIncrementSelect.h` 的 `select()`，
         语义等价但代码不同。固件化必须证明"换实现 = 换二进制结果零差异"，
         否则 §4.24m 的 J/K 组数字与新二进制不可比。
         判据（跑前写死）：I1 配对数 = 旧文件行数（470）；I2 `pos_rms`/`pos_max`/
         `nav_att_rmse`/`nrfb` 四个字段逐 seed 完全相等 ⇒ 身份成立；
         任一不等即列出差异格与最大相对偏差，判"实现不等价"。
@units   长度 m、姿态 deg、计数帧；相对偏差无量纲
@note    两份 jsonl 的 config 后缀必须一致（同一 `--name-suffix`），否则配对为 0
"""
import json
import os
import sys

FIELDS = ("pos_rms", "pos_max", "nav_att_rmse", "nrfb")


def load(path):
    out = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            r = json.loads(line)
            out[(r["config"], r["seed"])] = r
    return out


def main():
    if len(sys.argv) != 4:
        sys.exit("用法: py compare_nav_raw_module_identity.py <old.jsonl> <new.jsonl> <out.json>")
    old_path, new_path, out_path = sys.argv[1:4]
    old, new = load(old_path), load(new_path)
    keys = sorted(set(old) & set(new))
    verdict = {
        "old_rows": len(old), "new_rows": len(new), "paired": len(keys),
        "I1_pairing_ok": len(keys) == len(old) == len(new),
    }
    if not verdict["I1_pairing_ok"]:
        verdict["I2_identical"] = False
        verdict["note"] = "配对数不等于两侧行数 ⇒ 判定无效（防空洞绿灯）"
        print(json.dumps(verdict, ensure_ascii=False, indent=1))
        sys.exit(1)

    diffs = {}
    for k in keys:
        for fld in FIELDS:
            a, b = old[k].get(fld), new[k].get(fld)
            if a != b:
                diffs.setdefault(fld, []).append(
                    {"config": k[0], "seed": k[1], "old": a, "new": b})
    for fld in FIELDS:
        d = diffs.get(fld, [])
        rel = []
        for it in d:
            if isinstance(it["old"], float) and it["old"] != 0.0:
                rel.append(abs(it["new"] - it["old"]) / abs(it["old"]))
        verdict[f"{fld}_diff_count"] = len(d)
        verdict[f"{fld}_max_rel"] = round(max(rel), 6) if rel else 0.0
    verdict["diff_examples"] = {f: diffs[f][:5] for f in FIELDS if f in diffs}
    verdict["I2_identical"] = all(verdict[f"{f}_diff_count"] == 0 for f in FIELDS)
    verdict["gate"] = ("I1 配对 %d/%d 且 I2 四字段逐 seed 相等 ⇒ 模块实现与原内联锁存等价"
                       % (len(keys), len(old)))
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(verdict, f, ensure_ascii=False, indent=1)
    print(json.dumps(verdict, ensure_ascii=False, indent=1))
    sys.exit(0 if (verdict["I1_pairing_ok"] and verdict["I2_identical"]) else 1)


if __name__ == "__main__":
    main()
