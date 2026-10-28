# -*- coding: utf-8 -*-
"""默认值翻转（VIB_VETO 1→0）对导弹主工况的影响：隔离 A/B。

@file    check_veto_flip_impact.py
@brief   同一源码只差 BFS_NAVIGATION_GNSS_DELAY_AUTO_VIB_VETO，比较导弹关键工况
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 为什么必须做：归档导弹主表全部在"否决开"下产出，而翻转默认值会让自动延时
         估计在高过载段真的被应用；若洁净基线（0 ms 注入）被推离 1.5 m 级，
         整份报告的基线数字都要重述。判据：逐 seed 配对差 + 中位数，不看单点。
"""
import collections
import concurrent.futures as cf
import hashlib
import json
import os
import re
import statistics as st
import subprocess

import psutil

JRE = re.compile(r"SIL_REAL_JSON (\{.*\})")
BASE = ["--real-nav", "--missile", "--seconds", "45", "--gnss-rate-hz", "10",
        "--profile", "mag_clean", "--imu-grade", "icm"]
T = r"C:/Users/12631/AppData/Local/Temp"
ARMS = [("veto开", T + r"/tv_adt_base/fw_real.exe"),
        ("veto关(新默认)", T + r"/tv_vv_def/fw_real.exe")]
# (名称, 追加参数, seeds)
# ★ 2026-10-27 勘误：原用 `--loss F`，那是推力保留比例（需 --fault 3/5），
#   对 GNSS 失联完全空跑；失联一律用 `--gnss-outage 起:止`。
CONDS = [
    ("clean 0ms", ["--gnss-latency-ms", "0"], 50),
    ("延迟 100ms", ["--gnss-latency-ms", "100"], 20),
    ("延迟 200ms", ["--gnss-latency-ms", "200"], 20),
    ("失联5s", ["--gnss-outage", "20:25"], 20),
    ("失联20s", ["--gnss-outage", "20:40"], 20),
    ("8.4 g", ["--ms-thrust", "8000"], 20),
    ("spoof30", ["--gnss-spoof", "30"], 20),
]
KEYS = ["pos_rms", "pos_max", "nav_att_rmse", "nav_vel_rms", "outlier_rejects",
        "reacquire_resets", "ad_app"]


def run(exe, extra, seed):
    cmd = [exe] + BASE + extra + ["--seed", str(seed)]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                         text=True, env=os.environ.copy())
    try:
        psutil.Process(p.pid).cpu_affinity([2])
    except Exception:
        pass
    out, err = p.communicate(timeout=900)
    m = JRE.findall((out or "") + (err or ""))
    if not m:
        return None
    return json.loads(m[-1].replace("nan", "NaN").replace("inf", "Infinity"))


def main():
    for name, exe in ARMS:
        print(f"arm {name}: md5 "
              f"{hashlib.md5(open(exe,'rb').read()).hexdigest()[:8]}")
    rows_all = []
    print(f"{'工况':<16}{'n':>4}  {'pos_rms veto开→关':>22}  {'中位差':>9}"
          f"  {'变差/变好/同':>16}  {'拒帧开→关':>12}  {'app开→关':>12}")
    print("-" * 104)
    for cname, extra, nseed in CONDS:
        per = {}
        for name, exe in ARMS:
            with cf.ThreadPoolExecutor(max_workers=10) as pool:
                per[name] = [r for r in pool.map(
                    lambda s: run(exe, extra, s), range(1, nseed + 1)) if r]
        a = per[ARMS[0][0]]
        b = per[ARMS[1][0]]
        n = min(len(a), len(b))
        da = [a[i]["pos_rms"] for i in range(n)]
        db = [b[i]["pos_rms"] for i in range(n)]
        diff = [db[i] - da[i] for i in range(n)]
        worse = sum(1 for d in diff if d > 0.05)
        better = sum(1 for d in diff if d < -0.05)
        same = n - worse - better
        rj = (sum(r.get("outlier_rejects", 0) for r in a),
              sum(r.get("outlier_rejects", 0) for r in b))
        ap = (st.median([r.get("ad_app", 0) for r in a]),
              st.median([r.get("ad_app", 0) for r in b]))
        print(f"{cname:<16}{n:>4}  {st.median(da):>9.2f} → {st.median(db):<9.2f}"
              f"{st.median(diff):>11.2f}  {worse:>4}/{better}/{same:<7}"
              f"  {rj[0]:>5}→{rj[1]:<5}  {ap[0]:>5.0f}→{ap[1]:<5.0f}")
        for i in range(n):
            rows_all.append({"cond": cname, "seed": i + 1,
                             "off": {k: a[i].get(k) for k in KEYS},
                             "on": {k: b[i].get(k) for k in KEYS}})
    path = (r"D:/纵列双发矢量推力飞行器/TandemVec-EKF-Report/sim-data/"
            "veto_flip_impact_2026-10-27.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump({"arms": {k: hashlib.md5(open(v, 'rb').read()).hexdigest()
                            for k, v in ARMS}, "rows": rows_all},
                  fh, ensure_ascii=False)
    print("json:", path)


if __name__ == "__main__":
    main()
