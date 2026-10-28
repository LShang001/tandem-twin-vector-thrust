# -*- coding: utf-8 -*-
"""拟合窗 GNSS 缺口门的 runtime A/B：消除"纯失联变差"同时保住"失联+真延时"收益。

@file    run_gap_gate_ab.py
@brief   同一二进制只差 `--delay-max-gap S`，在 6 个导弹工况上比较三档缺口门
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 背景：`outage_veto_ab_2026-10-27.json`（二进制 e3142caf，20 seeds/档，逐档
         用 `gnss_stale_peak_s` 证明失联真的发生）显示取消 vib 否决后：
         纯失联两档系统性变差（助推 5 s：2.635→2.934 m，20/20 变差；
         巡航 20 s：12.296→12.899 m，20/20 变差），
         但"失联 20 s + 延时 100 ms"组合从 18.695→11.425 m（−38.9%，10/10 变好）。
         机制：`GnssStale` 只挡得住失联**期间**，重捕获后 20 s 拟合窗仍跨着缺口，
         τ̂ 混合缺口前后两段却继续被应用（ad_app 2→139）。
         新门 `WindowGap`：窗内 GNSS 最大间隔超阈值即拒绝估计。

预登记判据（跑前写死，跑后不改口径）：
  G1 消除退化：纯失联两档（助推 5 s / 巡航 20 s）在某档缺口门下相对"门关"
     的 pos_rms 变化 ≤ +1%（即回到 veto 开时的水平）；
  G2 收益保留：组合档（失联 20 s + 延 100 ms）pos_rms ≤ 12.5 m
     （相对无补偿 18.695 至少 −33%）；
  G3 无自我饿死：洁净档与单延 100 ms 档 pos_rms 相对"门关"变化 ≤ +2%，
     且 ad_app 不得归零（缺口门不能把功能永久关掉）；
  选档规则：满足 G1+G2+G3 的档里取**阈值最大**者（阈值越大越不敏感、越不容易
  误伤正常丢帧）；若无一档同时满足，则默认保持 0（关闭）并记录为负结果。
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
EXE = (sys.argv[1] if len(sys.argv) > 1 else
       r"C:/Users/12631/AppData/Local/Temp/tv_gap/fw_real.exe")
OUT_TAG = (sys.argv[2] if len(sys.argv) > 2 else "gap")
BASE = ["--real-nav", "--missile", "--seconds", "45", "--gnss-rate-hz", "10",
        "--profile", "mag_clean", "--imu-grade", "icm"]
ARMS = [("门关", "0"), ("门0.5s", "0.5"), ("门2.0s", "2.0")]
# (名称, 追加参数, seeds, 预期失联时长 s 或 None)
CONDS = [
    ("洁净", [], 20, None),
    ("助推失联5s", ["--gnss-outage", "5:10"], 20, 5.0),
    ("巡航失联20s", ["--gnss-outage", "20:40"], 20, 20.0),
    ("延100ms", ["--gnss-latency-ms", "100"], 20, None),
    ("失联20s+延100ms", ["--gnss-outage", "20:40", "--gnss-latency-ms", "100"],
     10, 20.0),
    ("失联5s+延100ms", ["--gnss-outage", "20:25", "--gnss-latency-ms", "100"],
     10, 5.0),
]


def run(extra, gap, seed):
    cmd = [EXE] + BASE + extra + ["--delay-max-gap", gap, "--seed", str(seed)]
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


def med(rows, key):
    vals = [r.get(key, 0) or 0 for r in rows]
    return st.median(vals) if vals else float("nan")


def main():
    if not os.path.exists(EXE):
        print("MISSING EXE:", EXE)
        return 2
    print("exe md5", hashlib.md5(open(EXE, "rb").read()).hexdigest()[:8])
    print(f"{'工况':<18}{'指标':<14}" + "".join(f"{a[0]:>14}" for a in ARMS))
    print("-" * 66)
    res = {}
    for cname, extra, nseed, exp_age in CONDS:
        per = {}
        for label, gap in ARMS:
            with cf.ThreadPoolExecutor(max_workers=8) as pool:
                per[label] = [r for r in pool.map(
                    lambda s: run(extra, gap, s), range(1, nseed + 1)) if r]
        age_pk = max([r.get("gnss_stale_peak_s", 0.0) for lbl, _ in ARMS
                      for r in per[lbl]])
        valid = "OK" if (exp_age is None or age_pk >= exp_age * 0.8) else "INVALID"
        print(f"\n{cname}  [{valid}] age峰 {age_pk:.2f}s / 预期 "
              f"{exp_age if exp_age else '-'}")
        for key in ["pos_rms", "pos_max", "nav_att_rmse", "ad_app",
                    "reacquire_resets"]:
            line = f"{'':<18}{key:<14}"
            for label, _ in ARMS:
                line += f"{med(per[label], key):>14.3f}"
            print(line)
        ref = per[ARMS[0][0]]
        for label, _ in ARMS[1:]:
            cur = per[label]
            n = min(len(ref), len(cur))
            d = [cur[i]["pos_rms"] - ref[i]["pos_rms"] for i in range(n)]
            print(f"{'':<18}  {label}: Δpos 中位 {st.median(d):+.3f} m  "
                  f"变差 {sum(1 for x in d if x > 0.05)}/变好 "
                  f"{sum(1 for x in d if x < -0.05)}/同 "
                  f"{sum(1 for x in d if abs(x) <= 0.05)} (n={n})")
        res[cname] = {"valid": valid, "age_peak": age_pk,
                      "arms": {k: per[k] for k, _ in ARMS}}
    path = ("D:/纵列双发矢量推力飞行器/TandemVec-EKF-Report/sim-data/"
            f"gap_gate_ab_2026-10-27_{OUT_TAG}.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump({"exe_md5": hashlib.md5(open(EXE, "rb").read()).hexdigest(),
                   "rows": res}, fh, ensure_ascii=False)
    print("\njson:", path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
