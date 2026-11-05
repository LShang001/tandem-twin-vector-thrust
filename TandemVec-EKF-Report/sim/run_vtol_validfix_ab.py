# -*- coding: utf-8 -*-
"""VTOL 生产口径回归：陈旧 τ̂ 修复 + WindowGap 门 是否影响主机型。

@file    run_vtol_validfix_ab.py
@brief   两个二进制（修复前/修复后）跑 VTOL 三磁剖面 ×10 seeds，比较验收通过数
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 为什么必须做：`auto_delay_valid` 移出成功分支后，估计器一拒绝就立刻停止
         应用 τ̂。VTOL 悬停段常年 `NoExcitation` 拒绝 ⇒ 修复前可能一直在用陈旧的
         导弹式 τ̂；这会在主机型上改变行为，必须量化而不是假设"VTOL 用不到"。
         判据沿用标准验收门：hold_end ≤5 m、max_roll ≤20°、pos_rms ≤3 m。
@frame   NED（hold_end_rms / pos_rms 为位置误差）；max_roll 为机体滚转角（deg）
@units   m（位置误差）、deg（滚转）、s（仿真时长）；通过数/seed 数为计数
"""
import concurrent.futures as cf
import hashlib
import json
import os
import re
import statistics as st
import subprocess
import sys

import psutil

JRE = re.compile(r"SIL_REAL_JSON (\{.*\})")
T = r"C:/Users/12631/AppData/Local/Temp"
ARMS = [("修复前(tv_gap)", f"{T}/tv_gap/fw_real.exe"),
        ("修复后(tv_gap2)", f"{T}/tv_gap2/fw_real.exe")]
PROFILES = ["mag_clean", "mag_nominal", "mag_disturbed"]
NSEED = 10


def run(exe, prof, seed):
    """跑一次 VTOL 悬停 SIL（绑 CPU2，超时 600 s 则 kill），返回末行 SIL_REAL_JSON 指标；无则 None。"""
    cmd = [exe, "--real-nav", "--profile", prof, "--pos-hold", "--flow",
           "--seconds", "40", "--seed", str(seed)]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                         text=True, env=os.environ.copy())
    try:
        psutil.Process(p.pid).cpu_affinity([2])
    except Exception:
        pass
    try:
        out, err = p.communicate(timeout=600)
    except subprocess.TimeoutExpired:
        p.kill()
        return None
    m = JRE.findall((out or "") + (err or ""))
    return json.loads(m[-1]) if m else None


def passed(r):
    """单例验收判据：hold_end_rms ≤5 m 且 max_roll ≤20° 且 pos_rms ≤3 m 才算通过。"""
    return (r["hold_end_rms"] <= 5.0 and r["max_roll"] <= 20.0 and
            r["pos_rms"] <= 3.0)


def main():
    """命令行入口：三磁剖面 × 两臂 × 10 seeds，打印通过数与 hold/roll/pos 指标中位数。

    任一臂二进制缺失直接返回 2（不猜结果）；结果写
    sim-data/vtol_validfix_ab_2026-10-27.json（含逐条原始记录与失败 seed 列表）。
    """
    for n, e in ARMS:
        if not os.path.exists(e):
            print("MISSING:", n, e)
            return 2
        print(f"arm {n}: md5 "
              f"{hashlib.md5(open(e,'rb').read()).hexdigest()[:8]}")
    print(f"{'剖面':<16}{'臂':<18}{'通过':>6}{'hold中位':>10}{'hold最大':>10}"
          f"{'roll最大':>10}{'pos中位':>9}")
    res = {}
    for prof in PROFILES:
        for n, e in ARMS:
            with cf.ThreadPoolExecutor(max_workers=8) as pool:
                rows = [r for r in pool.map(
                    lambda s: run(e, prof, s), range(1, NSEED + 1)) if r]
            ok = sum(1 for r in rows if passed(r))
            he = [r["hold_end_rms"] for r in rows]
            print(f"{prof:<16}{n:<18}{ok:>4}/{len(rows)}{st.median(he):>10.2f}"
                  f"{max(he):>10.2f}{max(r['max_roll'] for r in rows):>10.2f}"
                  f"{st.median([r['pos_rms'] for r in rows]):>9.2f}")
            res[f"{prof}|{n}"] = {"pass": ok, "n": len(rows),
                                  "fails": [r["seed"] for r in rows
                                            if not passed(r)],
                                  "rows": rows}
    path = (r"D:/纵列双发矢量推力飞行器/TandemVec-EKF-Report/sim-data/"
            "vtol_validfix_ab_2026-10-27.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(res, fh, ensure_ascii=False)
    print("json:", path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
