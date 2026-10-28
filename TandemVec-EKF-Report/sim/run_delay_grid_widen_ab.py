# -*- coding: utf-8 -*-
"""放宽 GNSS 延时搜索网格上界：在"端点判无效 + 残差门 150 m/s"之下还有没有必要？

@file    run_delay_grid_widen_ab.py
@brief   同一二进制只差 `--delay-grid-max-ms` / `--delay-cap-ms` / `--delay-max-resid`，
         在 5 个延迟档 + 4 个对照格上做三臂 A/B
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 背景：§4.19 曾因"放宽网格后 150 ms 档整帧拒绝 0→40、250 ms 档 523→591"
         决定不放宽默认（保持 0~150 ms）。但那时**没有**这两样东西：
         ①端点/近端点一律判无效（§4.20 M-附，默认生效）；
         ②应用侧自报残差门 150 m/s（§4.22.1e，已默认开）。
         本轮宿主 T10 又证明：真延时 200 ms 在默认网格上只能报 `Boundary`，
         把上界抬到 300 ms 立刻命中 τ̂=199.5 ms ⇒ "大延迟估不出"是**搜索范围**问题，
         不是信息问题。于是问题变成：放宽后估出的 τ̂ 能不能安全地用？

三臂：
  G0 默认                 （网格 150 ms、CAP 宏默认、残差门 150）
  G1 放宽网格+CAP         `--delay-grid-max-ms 300 --delay-cap-ms 300`
  G2 G1 且关残差门        再加 `--delay-max-resid 0`（用于分离残差门的保护作用）

预登记判据（跑前写死）：
  P1 有效性：延 200/250 ms 档 G1 的 `ad_tau_ms` 中位 ≥ 注入值的 0.85 且 `ad_app`>0；
     不成立说明应用侧仍被挡，本档不进入收益判定。
  P2 收益：延 200 ms 档 G1 `pos_rms` ≤ 0.6 × G0；延 250 ms 同。
  P3 无副作用：洁净 / 5 Hz / 巡航失联 5 s / 振 2 g 四格 G1 ≤ 1.1 × G0，
     且 `outlier_rejects` 合计不比 G0 增加（§4.19 的副作用指标）。
  P4 归因：G1 变差而 G2 变好 ⇒ 残差门在大延迟下确实在挡坏 τ̂（保留放宽+保留残差门）；
     G1 与 G2 同样差 ⇒ 放宽本身有害 ⇒ 维持 §4.19 决定不放宽。
  默认值：只有 P1+P2+P3 同时成立才讨论把 `MAX_TAU_S` 从 0.150 提高；
  即便如此仍需实机接收机延迟标定，不得凭 SIL 下默认。
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
EXE = os.environ.get(
    "EXE", r"C:/Users/12631/AppData/Local/Temp/tv_hf3/fw_real.exe")
BASE = ["--real-nav", "--missile", "--seconds", "45", "--gnss-rate-hz", "10",
        "--profile", "mag_clean", "--imu-grade", "icm"]
ARMS = [
    ("G0 默认", []),
    ("G1 网格300+CAP300", ["--delay-grid-max-ms", "300", "--delay-cap-ms", "300"]),
    ("G2 G1+关残差门", ["--delay-grid-max-ms", "300", "--delay-cap-ms", "300",
                        "--delay-max-resid", "0"]),
]
# (名称, 追加参数, 注入延迟 ms 或 None)
CONDS = [
    ("洁净", [], None),
    ("延50ms", ["--gnss-latency-ms", "50"], 50.0),
    ("延100ms", ["--gnss-latency-ms", "100"], 100.0),
    ("延150ms", ["--gnss-latency-ms", "150"], 150.0),
    ("延200ms", ["--gnss-latency-ms", "200"], 200.0),
    ("延250ms", ["--gnss-latency-ms", "250"], 250.0),
    ("5Hz", ["--gnss-rate-hz", "5"], None),
    ("巡航失联5s", ["--gnss-outage", "20:25"], None),
    ("振2g_8.4g", ["--ms-thrust", "8000", "--vib", "2.0", "--vib-hz", "120",
                   "--vib-start", "1.5"], None),
]
NSEED = 20
KEYS = ["pos_rms", "pos_max", "nav_att_rmse", "nav_vel_rms", "ad_tau_ms",
        "ad_app", "ad_blk", "outlier_rejects", "reacquire_resets"]


def run(extra, arm, seed):
    cmd = [EXE] + BASE + extra + arm + ["--seed", str(seed)]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                         text=True, env=os.environ.copy())
    try:
        psutil.Process(p.pid).cpu_affinity([2])
    except Exception:
        pass
    try:
        out, err = p.communicate(timeout=900)
    except subprocess.TimeoutExpired:
        p.kill()
        return None
    m = JRE.findall((out or "") + (err or ""))
    return json.loads(m[-1]) if m else None


def med(rows, key):
    vals = [r.get(key, 0) or 0 for r in rows]
    return st.median(vals) if vals else float("nan")


def main():
    if not os.path.exists(EXE):
        print("MISSING EXE:", EXE)
        return 2
    md5 = hashlib.md5(open(EXE, "rb").read()).hexdigest()
    print("exe md5", md5[:8])
    res = {}
    for cname, extra, inj in CONDS:
        per = {}
        for label, arm in ARMS:
            with cf.ThreadPoolExecutor(max_workers=8) as pool:
                per[label] = [r for r in pool.map(
                    lambda s: run(extra, arm, s), range(1, NSEED + 1)) if r]
        labels = [l for l, _ in ARMS]
        print(f"\n=== {cname} (注入 {inj if inj else '-'} ms) ===")
        for key in KEYS:
            row = "  {:<16}".format(key)
            row += "".join(f"{med(per[l], key):>16.3f}" for l in labels)
            print(row)
        p0 = med(per[labels[0]], "pos_rms")
        p1 = med(per[labels[1]], "pos_rms")
        p2 = med(per[labels[2]], "pos_rms")
        print(f"  → G1/G0 {p1 / p0 if p0 else float('nan'):.3f}   "
              f"G2/G0 {p2 / p0 if p0 else float('nan'):.3f}")
        res[cname] = {"inject_ms": inj,
                      "med": {l: {k: med(per[l], k) for k in KEYS} for l in labels},
                      "rows": {l: per[l] for l in labels}}

    def m(cell, arm, key):
        return res[cell]["med"][arm][key]
    L = [l for l, _ in ARMS]
    crit = {}
    for cell in ("延200ms", "延250ms"):
        inj = res[cell]["inject_ms"]
        crit[f"P1 {cell} τ̂ 命中"] = (m(cell, L[1], "ad_tau_ms") >= 0.85 * inj and
                                  m(cell, L[1], "ad_app") > 0)
        crit[f"P2 {cell} 收益≤0.6"] = (m(cell, L[1], "pos_rms") <=
                                    0.6 * m(cell, L[0], "pos_rms"))
    for cell in ("洁净", "5Hz", "巡航失联5s", "振2g_8.4g"):
        crit[f"P3 {cell} 无副作用"] = (m(cell, L[1], "pos_rms") <=
                                    1.1 * m(cell, L[0], "pos_rms"))
        crit[f"P3 {cell} 拒帧不增"] = (m(cell, L[1], "outlier_rejects") <=
                                    m(cell, L[0], "outlier_rejects") + 1)
    print("\n=== 预登记判据 ===")
    for k, v in crit.items():
        print(f"  {'✓' if v else '✗'} {k}")
    p1_ok = all(v for k, v in crit.items() if k.startswith("P1"))
    p2_ok = all(v for k, v in crit.items() if k.startswith("P2"))
    p3_ok = all(v for k, v in crit.items() if k.startswith("P3"))
    print(f"\nP1(有效性)={p1_ok}  P2(收益)={p2_ok}  P3(无副作用)={p3_ok}")
    verdict = "可讨论放宽 MAX_TAU_S（仍需实机延迟标定）" if (p1_ok and p2_ok and p3_ok) \
        else "维持 §4.19 决定：不放宽默认网格"
    print("VERDICT:", verdict)
    path = ("D:/纵列双发矢量推力飞行器/TandemVec-EKF-Report/sim-data/"
            "delay_grid_widen_ab_2026-10-28.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump({"exe_md5": md5, "criteria": crit, "verdict": verdict,
                   "rows": res}, fh, ensure_ascii=False)
    print("json:", path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
