# -*- coding: utf-8 -*-
"""两条 SIL 矩阵 jsonl 的逐 seed 配对比较：默认不变性 与 候选臂净效应。

@file    compare_hfconf.py
@brief   按 (config, seed) 对齐两份 `missile-sil-*.jsonl`，输出逐格配对差异
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 用途有两个，都要求"同 seed 配对"而不是比中位数：
         ① **默认不变性**：新二进制（接了高频振动通道，但默认宏仍选中频通道+否决关）
            与改动前归档逐 seed 必须**逐位相同**；任何一格不同都说明接线改到了默认路径。
         ② **候选臂净效应**：`--vib-veto 1 --vib-hf-source 1 --vib-hf-enter 1.0`
            相对默认只在真强振动格不同，其余格逐位相同。
         中位数相同不代表逐 seed 相同，因此这里同时报
          `n_diff`（有多少 seed 的 pos_rms 差 > 1e-6）与 `max|Δ|`。
"""
import json
import os
import statistics as st
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_DATA = os.path.normpath(os.path.join(HERE, "..", "sim-data"))
KEYS = ["pos_rms", "pos_max", "nav_att_rmse", "nav_vel_rms", "ad_app", "ad_blk"]


def load(path):
    out = {}
    with open(path, encoding="utf-8") as fh:
        for ln in fh:
            ln = ln.strip()
            if not ln:
                continue
            d = json.loads(ln)
            cfg = d.get("config")
            if not cfg:
                continue
            out.setdefault(cfg, {})[d.get("seed")] = d
    return out


def main():
    if len(sys.argv) < 3:
        print("用法: compare_hfconf.py <A.jsonl> <B.jsonl> [key]")
        return 2
    a = load(os.path.join(SIM_DATA, sys.argv[1]) if not os.path.isabs(sys.argv[1])
             else sys.argv[1])
    b = load(os.path.join(SIM_DATA, sys.argv[2]) if not os.path.isabs(sys.argv[2])
             else sys.argv[2])
    key = sys.argv[3] if len(sys.argv) > 3 else "pos_rms"
    common = sorted(set(a) & set(b))
    if not common:
        # 两份矩阵通常只差 config 末尾的运行标签（`_hfA` / `_wide`）。
        # 不做这一步会得到"交集 0 → 逐 seed 不同 0 → 默认不变"的**空洞绿灯**。
        strip = lambda s: s.rsplit("_", 1)[0]
        a = {strip(k): v for k, v in a.items()}
        b = {strip(k): v for k, v in b.items()}
        common = sorted(set(a) & set(b))
        print("[note] config 标签不同，已按去掉末段 `_tag` 后对齐")
    if not common:
        print("!! 两文件的 config 完全无法对齐，比较无意义（拒绝给出'逐项不变'）")
        return 2
    print(f"cells A={len(a)} B={len(b)} 交集={len(common)}  key={key}")
    n_diff_total = 0
    for cfg in common:
        seeds = sorted(set(a[cfg]) & set(b[cfg]))
        if not seeds:
            continue
        da = [b[cfg][s].get(key, 0) - a[cfg][s].get(key, 0) for s in seeds]
        nd = sum(1 for x in da if abs(x) > 1e-6)
        n_diff_total += nd
        med_a = st.median([a[cfg][s].get(key, 0) for s in seeds])
        med_b = st.median([b[cfg][s].get(key, 0) for s in seeds])
        flag = "  <<< 不同" if nd else ""
        print(f"{cfg:<34} n={len(seeds):<3} {med_a:>9.3f} -> {med_b:>9.3f} "
              f"({(med_b - med_a) / med_a * 100 if abs(med_a) > 1e-9 else 0:+6.1f}%)"
              f"  逐seed不同 {nd:>3}/{len(seeds)}  max|Δ|={max(abs(x) for x in da):.3f}"
              f"{flag}")
    print(f"\n合计逐 seed 不同：{n_diff_total}"
          + ("  ⇒ 默认路径逐项不变（新接线惰性）" if n_diff_total == 0 else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
