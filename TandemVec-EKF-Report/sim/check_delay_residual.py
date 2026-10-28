# -*- coding: utf-8 -*-
"""自动延时补偿的"该不该应用"判据取证：估计器自报残差能否分开有害格与有益格。

@file    check_delay_residual.py
@brief   逐格统计 pos_rms 与 `ad_resid/ad_sharp/ad_app/ad_valid`，找可分离阈值
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 背景：默认二进制再基线（§4.22.3）发现取消 `vib_high` 否决带来两个严重尾部——
         `vib2g_8p4g` 位置 RMSE 3.07→28.06 m、`imu_stuck` 3.11→34.37 m；而 100 ms
         延时档同一切换拿到 −12.75 m。说明"该不该应用 τ̂"需要一个**与振动无关、
         来自估计器自身拟合质量**的判据。本脚本量出各格的自报残差分布，检验
         是否存在干净的分隔带（有害格残差显著大于有益格，且与 100 ms 档不重叠）。
"""
import concurrent.futures as cf
import hashlib
import json
import os
import re
import statistics as st
import subprocess

import psutil

JRE = re.compile(r"SIL_REAL_JSON (\{.*\})")
EXE = r"C:/Users/12631/AppData/Local/Temp/tv_gap2/fw_real.exe"
BASE = ["--real-nav", "--missile", "--seconds", "45", "--gnss-rate-hz", "10",
        "--profile", "mag_clean", "--imu-grade", "icm"]
SEEDS = list(range(1, 9))
# (标签, 追加参数, 期望类别)
CONDS = [
    ("洁净", [], "ref"),
    ("延100ms", ["--gnss-latency-ms", "100"], "有益"),
    ("延50ms", ["--gnss-latency-ms", "50"], "有益"),
    ("延200ms", ["--gnss-latency-ms", "200"], "饱和"),
    ("振2g_8.4g", ["--ms-thrust", "8000", "--vib", "2.0", "--vib-hz", "120",
                   "--vib-start", "1.5"], "有害"),
    ("振1g_8.4g", ["--ms-thrust", "8000", "--vib", "1.0", "--vib-hz", "80",
                   "--vib-start", "1.5"], "有害?"),
    ("IMU冻结1s", ["--imu-fault", "stuck", "--imu-fault-window", "12:13"],
     "有害"),
    ("IMU_NaN1s", ["--imu-fault", "nan", "--imu-fault-window", "12:13"], "参考"),
    ("助推失联5s", ["--gnss-outage", "5:10"], "有害?"),
    ("巡航失联20s", ["--gnss-outage", "20:40"], "有害?"),
]


def run(extra, seed):
    cmd = [EXE] + BASE + extra + ["--seed", str(seed)]
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
    print("exe md5", hashlib.md5(open(EXE, "rb").read()).hexdigest()[:8])
    print(f"{'工况':<14}{'类别':<8}{'pos_rms':>10}{'ad_resid':>11}"
          f"{'res_p90':>9}{'ad_sharp':>10}{'ad_app':>8}{'ad_valid':>9}")
    print("-" * 84)
    res = {}
    for tag, extra, kind in CONDS:
        with cf.ThreadPoolExecutor(max_workers=8) as pool:
            rows = [r for r in pool.map(lambda s: run(extra, s), SEEDS) if r]
        if not rows:
            continue

        def med(k):
            return st.median([r.get(k, 0) or 0 for r in rows])

        resid = sorted(r.get("ad_resid", 0) or 0 for r in rows)
        p90 = resid[min(len(resid) - 1, int(0.9 * len(resid)))]
        print(f"{tag:<14}{kind:<8}{med('pos_rms'):>10.2f}{med('ad_resid'):>11.2f}"
              f"{p90:>9.2f}{med('ad_sharp'):>10.3f}{med('ad_app'):>8.0f}"
              f"{med('ad_valid'):>9.0f}")
        res[tag] = {"kind": kind, "rows": rows}
    path = ("D:/纵列双发矢量推力飞行器/TandemVec-EKF-Report/sim-data/"
            "delay_residual_separation_2026-10-27.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump({"exe_md5": hashlib.md5(open(EXE, "rb").read()).hexdigest(),
                   "rows": res}, fh, ensure_ascii=False)
    print("json:", path)


if __name__ == "__main__":
    main()
