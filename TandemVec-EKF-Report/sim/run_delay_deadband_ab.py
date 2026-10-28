# -*- coding: utf-8 -*-
"""自动延时应用死区的同一二进制 runtime A/B。

@file    run_delay_deadband_ab.py
@brief   只改 `--delay-deadband`，检验"估计差小于一个网格步就不替换先验"能否消掉洁净档代价
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 背景：取消 `vib_high` 否决后，自动延时在 100 ms 档把位置 RMSE 从 17.86 压到
         4.89 m（20/20 变好），但 0 ms 洁净档从 1.39 抬到 1.56 m（45/50 变差）。
         trace 取证：洁净档 τ̂ 中位 11 ms [p5 8, p95 19.6]，而设施真实管线年龄≈15 ms，
         即 4 ms 的差落在估计器自己的 5 ms 网格步之内——用不可分辨的差去改先验。
         死区取 5 ms（一个网格步），100 ms 档 τ̂≈89 ms 与先验差 74 ms，不受影响。

预注册判据（跑前写死）：
  D1 洁净 0 ms：死区 5 ms 档的 pos_rms 中位回到无否决基线 1.39 m（±0.05 m）；
  D2 收益保留：100 ms 档 pos_rms 中位 ≤5.2 m（相对无死区 4.89 m 损失 ≤7%）；
  D3 边界无害：25/50 ms 档相对无死区劣化 ≤10%（若超过，说明死区吞掉了真延时）；
  D4 死区 10 ms 只作为敏感性对照，不参与默认值决策。
  D1–D3 同时成立才把默认死区设为 5 ms；任一不成立则默认保持 0 并记录代价。
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
EXE = r"C:/Users/12631/AppData/Local/Temp/tv_db/fw_real.exe"
BASE = ["--real-nav", "--missile", "--seconds", "45", "--gnss-rate-hz", "10",
        "--profile", "mag_clean", "--imu-grade", "icm"]
ARMS = [("无死区", "0"), ("死区5ms", "0.005"), ("死区10ms", "0.010")]
NSEED = 10
CONDS = [
    ("洁净0ms", ["--gnss-latency-ms", "0"]),
    ("延时25ms", ["--gnss-latency-ms", "25"]),
    ("延时50ms", ["--gnss-latency-ms", "50"]),
    ("延时100ms", ["--gnss-latency-ms", "100"]),
    ("延时200ms", ["--gnss-latency-ms", "200"]),
]


def run(extra, db, seed):
    cmd = [EXE] + BASE + extra + ["--delay-deadband", db, "--seed", str(seed)]
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
    head = f"{'工况':<12}{'指标':<12}" + "".join(f"{a[0]:>14}" for a in ARMS)
    print(head)
    print("-" * len(head))
    res = {}
    for cname, extra in CONDS:
        per = {}
        for label, db in ARMS:
            with cf.ThreadPoolExecutor(max_workers=8) as pool:
                per[label] = [r for r in pool.map(
                    lambda s: run(extra, db, s), range(1, NSEED + 1)) if r]
        res[cname] = per
        for key in ["pos_rms", "pos_max", "nav_att_rmse"]:
            line = f"{cname:<12}{key:<12}"
            for label, _ in ARMS:
                line += f"{st.median([r[key] for r in per[label]]):>14.3f}"
            print(line)
        line = f"{'':<12}{'ad_app':<12}"
        for label, _ in ARMS:
            line += f"{st.median([r.get('ad_app', 0) for r in per[label]]):>14.0f}"
        print(line)
        ref = per[ARMS[0][0]]
        for label, _ in ARMS[1:]:
            cur = per[label]
            n = min(len(ref), len(cur))
            d = st.median([cur[i]["pos_rms"] - ref[i]["pos_rms"]
                           for i in range(n)])
            print(f"{'':<12}  {label}: Δpos={d:+.3f} m vs 无死区")
    path = ("D:/纵列双发矢量推力飞行器/TandemVec-EKF-Report/sim-data/"
            "delay_deadband_ab_2026-10-27.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump({"exe_md5": hashlib.md5(open(EXE, "rb").read()).hexdigest(),
                   "nseed": NSEED, "rows": res}, fh, ensure_ascii=False)
    print("json:", path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
