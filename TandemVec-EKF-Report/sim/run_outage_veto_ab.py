# -*- coding: utf-8 -*-
"""用正确的 `--gnss-outage` 重跑"失联 × 振动否决"两臂（取代作废的 `--loss` 档）。

@file    run_outage_veto_ab.py
@brief   同一二进制只差 `--vib-veto 0|1`，在真实 GNSS 失联段上检验默认值翻转的代价
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 背景：§4.20.5 与 §4.21.1 的"失联"两档误用了 `--loss`（推力保留比例，需
         `--fault 3/5` 才生效），实为洁净弹道重复 ⇒ 作废。本脚本用 `--gnss-outage A:B`
         重做，并同时记录 `gnss_age` 与 `reacquire_resets`，确保失联注入真的发生
         （判据：失联臂的 gnss_age 峰值必须 ≥ 失联时长，否则该臂无效、不看结论）。

预登记问题与判据（跑前写死）：
  Q1 翻转在真实失联下是否有害？—— 失联段应用侧本应有 `age>5 s` 新鲜度闸（block bit 1），
     故预期 20 s 失联两臂逐项相同；若不同且 veto 关更差 >10%，需要补失联侧门控。
  Q2 5 s 级短失联下 veto 关是否仍无害？—— 差值 |Δpos_rms| ≤5% 视为无害。
  Q3 组合（失联 + 100 ms 延时）下 veto 关是否保留收益？—— 期望 veto 关明显更好。
  有效性前置：每档必须打印 `gnss_age` 峰值与 `reacquire_resets`，
  失联时长与 age 峰值不符的档一律标 INVALID，不参与判定。
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
EXE = r"C:/Users/12631/AppData/Local/Temp/tv_guard/fw_real.exe"
BASE = ["--real-nav", "--missile", "--seconds", "45", "--gnss-rate-hz", "10",
        "--profile", "mag_clean", "--imu-grade", "icm"]
ARMS = [("veto开", "1"), ("veto关(默认)", "0")]
# (名称, 追加参数, seeds, 预期失联时长 s 或 None)
CONDS = [
    ("洁净", [], 20, None),
    ("助推失联5s", ["--gnss-outage", "5:10"], 20, 5.0),
    ("巡航失联5s", ["--gnss-outage", "20:25"], 20, 5.0),
    ("巡航失联20s", ["--gnss-outage", "20:40"], 20, 20.0),
    ("失联20s+延100ms", ["--gnss-outage", "20:40", "--gnss-latency-ms", "100"],
     10, 20.0),
]
KEYS = ["pos_rms", "pos_max", "nav_att_rmse", "nav_vel_rms",
        "gnss_stale_peak_s", "reacquire_resets", "outlier_rejects", "ad_app"]


def run(extra, veto, seed):
    cmd = [EXE] + BASE + extra + ["--vib-veto", veto, "--seed", str(seed)]
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


def main():
    if not os.path.exists(EXE):
        print("MISSING EXE:", EXE)
        return 2
    print("exe md5", hashlib.md5(open(EXE, "rb").read()).hexdigest()[:8])
    res = {}
    for cname, extra, nseed, exp_age in CONDS:
        per = {}
        for label, veto in ARMS:
            with cf.ThreadPoolExecutor(max_workers=8) as pool:
                per[label] = [r for r in pool.map(
                    lambda s: run(extra, veto, s), range(1, nseed + 1)) if r]
        ref, cur = per[ARMS[0][0]], per[ARMS[1][0]]
        age_pk = max([r.get("gnss_stale_peak_s", 0.0) for r in ref] +
                     [r.get("gnss_stale_peak_s", 0.0) for r in cur])
        valid = "OK" if (exp_age is None or age_pk >= exp_age * 0.8) else "INVALID"
        print(f"\n=== {cname}  [{valid}] age峰 {age_pk:.2f}s / 预期 "
              f"{exp_age if exp_age else '-'}  rst "
              f"{sum(r.get('reacquire_resets', 0) for r in ref)}→"
              f"{sum(r.get('reacquire_resets', 0) for r in cur)} ===")
        for key in KEYS:
            a = st.median([r.get(key, 0) or 0 for r in ref])
            b = st.median([r.get(key, 0) or 0 for r in cur])
            print(f"  {key:<18}{a:>12.3f} → {b:>12.3f}  "
                  f"({(b - a) / a * 100 if a else 0:+.1f}%)")
        n = min(len(ref), len(cur))
        d = [cur[i]["pos_rms"] - ref[i]["pos_rms"] for i in range(n)]
        print(f"  配对 Δpos: 变差 {sum(1 for x in d if x > 0.05)} / 变好 "
              f"{sum(1 for x in d if x < -0.05)} / 同 "
              f"{sum(1 for x in d if abs(x) <= 0.05)}  中位 {st.median(d):+.3f} m")
        res[cname] = {"valid": valid, "age_peak": age_pk,
                      "arms": {k: per[k] for k, _ in ARMS}}
    path = ("D:/纵列双发矢量推力飞行器/TandemVec-EKF-Report/sim-data/"
            "outage_veto_ab_2026-10-27.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump({"exe_md5": hashlib.md5(open(EXE, "rb").read()).hexdigest(),
                   "rows": res}, fh, ensure_ascii=False)
    print("\njson:", path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
