# -*- coding: utf-8 -*-
"""应用侧"拟合质量门"的 runtime A/B：残差门 + 重锚保持门能否收掉尾部。

@file    run_apply_gate_ab.py
@brief   同一二进制只差 `--delay-max-resid` / `--delay-disc-hold`，逐格比较
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 依据 `delay_residual_separation_2026-10-27.json` 的分离表：
         有害格自报残差 IMU 冻结 349、巡航失联 20 s 190，有益格 洁净 50 / 延 50 ms 41 /
         延 100 ms 43 / 振 2 g 37 —— 150 m/s 的阈值应当只挡住前两者。
         ★ 但振 2 g 档残差 37 **低于**洁净档 50，任何残差阈值都区分不了它，
         而它的 τ̂=96.5 ms（真值 15 ms）与真正的 100 ms 档 τ̂=89.1 ms 无法区分
         ⇒ 预登记：本门**预期修不好振 2 g**，若实测修好了说明分离表有误，要回头查设施。

预登记判据：
  R1 `imu_stuck` pos_rms 从 34.16 回到 ≤5 m（残差门生效）；
  R2 `巡航失联20s` pos_rms 回到 ≤12.4 m（veto 开时 12.30）；
  R3 有益格不变：洁净 ≤2.55、延 50 ms ≤4.30、延 100 ms ≤5.00、延 200 ms ≤39.0；
  R4 `vib2g_8p4g` 允许仍差（预期失败），但必须记录，不得据此调阈值去凑。
  R1+R2+R3 同时成立才考虑把默认从 0 改成一个有限值；否则默认保持 0（关闭）。

★ 本脚本为**历史证据**（8 seeds）：其结果已按 20 seeds 复核并在报告 §4.22.1e 更正
  （振 2 g −80% → −20%，R1 的 IMU 冻结不由本门解决，见 §4.22.1d）。
  `--delay-disc-hold` 开关对应的 `Discontinuity` 重锚保持门因六格无可测收益已删除，
  用当前二进制再跑本脚本会被"未识别参数硬失败"拦下——这是设施的正确行为。
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
EXE = r"C:/Users/12631/AppData/Local/Temp/tv_res/fw_real.exe"
BASE = ["--real-nav", "--missile", "--seconds", "45", "--gnss-rate-hz", "10",
        "--profile", "mag_clean", "--imu-grade", "icm"]
# (标签, 追加参数)
ARMS = [
    ("基线(门全关)", ["--delay-max-resid", "-1", "--delay-disc-hold", "-1"]),
    ("残差150", ["--delay-max-resid", "150", "--delay-disc-hold", "-1"]),
    ("残差150+重锚200", ["--delay-max-resid", "150", "--delay-disc-hold", "200"]),
    ("仅重锚200", ["--delay-max-resid", "-1", "--delay-disc-hold", "200"]),
]
CONDS = [
    ("洁净", []),
    ("延50ms", ["--gnss-latency-ms", "50"]),
    ("延100ms", ["--gnss-latency-ms", "100"]),
    ("延200ms", ["--gnss-latency-ms", "200"]),
    ("IMU冻结1s", ["--imu-fault", "stuck", "--imu-fault-window", "12:13"]),
    ("巡航失联20s", ["--gnss-outage", "20:40"]),
    ("振2g_8.4g", ["--ms-thrust", "8000", "--vib", "2.0", "--vib-hz", "120",
                   "--vib-start", "1.5"]),
    ("振1g_8.4g", ["--ms-thrust", "8000", "--vib", "1.0", "--vib-hz", "80",
                   "--vib-start", "1.5"]),
]
NSEED = 8


def run(extra, arm, seed):
    cmd = [EXE] + BASE + extra + arm + ["--seed", str(seed)]
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
    print(f"{'工况':<14}" + "".join(f"{a[0]:>18}" for a in ARMS))
    print("-" * (14 + 18 * len(ARMS)))
    res = {}
    for tag, extra in CONDS:
        per = {}
        for label, arm in ARMS:
            with cf.ThreadPoolExecutor(max_workers=8) as pool:
                per[label] = [r for r in pool.map(
                    lambda s: run(extra, arm, s), range(1, NSEED + 1)) if r]
        line = f"{'pos_rms':<14}"
        for label, _ in ARMS:
            line += f"{st.median([r['pos_rms'] for r in per[label]]):>18.2f}"
        app = f"{'ad_app':<14}"
        for label, _ in ARMS:
            app += f"{st.median([r.get('ad_app', 0) for r in per[label]]):>18.0f}"
        print(f"{tag:<14}" + "".join(
            f"{st.median([r['pos_rms'] for r in per[l]]):>18.2f}"
            for l, _ in ARMS))
        print(app)
        res[tag] = {l: per[l] for l, _ in ARMS}
    path = ("D:/纵列双发矢量推力飞行器/TandemVec-EKF-Report/sim-data/"
            "apply_gate_ab_2026-10-27.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump({"exe_md5": hashlib.md5(open(EXE, "rb").read()).hexdigest(),
                   "rows": res}, fh, ensure_ascii=False)
    print("json:", path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
