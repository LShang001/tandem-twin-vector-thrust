# -*- coding: utf-8 -*-
"""振动估计器的频段响应：生产 IMU 软件 LPF 之后还能不能看见机械振动。

@file    check_vib_band_response.py
@brief   同一洁净弹道叠加不同频率 1 g 正弦振动，测高通加速度 RMS 的增量
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 为什么必须做：`ins_vibration_adaptive.h` 的 0.5 Hz 高通设计目的是"把机械
         振动从真实加速度里分离出来"，但它吃的是 `sensor_imu.cpp` 2 kHz 软件 LPF
         （α=0.02/0.05 → 截止约 6/16 Hz）之后、再按 200 Hz 送进导航的信号。
         若 80 Hz 振动的 RMS 增量远小于低频振动，则该检测器对弹上真实振动频段
         几乎无响应，`vib_high` 的含义就不是"有振动"而是"机动剧烈"。
         基线用同二进制零振动档，逐档给增量 Δrms[g]/每 1 g，避免跨二进制比较。
@frame   NED（pos_rms 为北东地水平位置误差；振动 RMS 为加速度标量模长，无轴向）
@units   g（vib_rms_g 与振动幅值）、m（pos_rms）、Hz（振动频率）、
         g/g（每 1 g 振动的 RMS 灵敏度）、无量纲（vib_high 占空比）
"""
import json
import os
import re
import statistics as st
import subprocess

import psutil

JRE = re.compile(r"SIL_REAL_JSON (\{.*\})")
BASE = ["--real-nav", "--missile", "--seconds", "45", "--gnss-rate-hz", "10",
        "--profile", "mag_clean", "--imu-grade", "icm"]
EXE = r"C:/Users/12631/AppData/Local/Temp/tv_vib/fw_real.exe"
TR = r"C:/Users/12631/AppData/Local/Temp/vibband"
# (标签, 振动频率 Hz, 幅值 g)；None=零振动基线
CONDS = [("零振动", None, 0.0), ("10Hz", 10.0, 1.0), ("20Hz", 20.0, 1.0),
         ("40Hz", 40.0, 1.0), ("80Hz", 80.0, 1.0), ("120Hz", 120.0, 1.0),
         ("80Hz_3g", 80.0, 3.0)]
SEEDS = [1, 2]


def run(tag, hz, amp):
    """对一个振动档跑全部种子，取每个种子的振动 RMS、vib_high 占空比与位置 RMSE。

    参数：tag 为档名兼迹线文件名前缀；hz 为振动频率（Hz，None=零振动基线）；
    amp 为振幅（g）。只统计 t>3 s 的飞行段，避免初始化段稀释统计量。
    返回：长度等于 SEEDS 的列表，每项 (vib_rms_g 中位, vib_high 均值, pos_rms 或 NaN)。
    """
    os.makedirs(TR, exist_ok=True)
    extra = [] if hz is None else ["--vib", str(amp), "--vib-hz", str(hz),
                                   "--vib-start", "1.5"]
    out = []
    for s in SEEDS:
        csv = f"{TR}/{tag}_s{s}.csv"
        cmd = [EXE] + BASE + extra + ["--seed", str(s), "--trace-csv", csv]
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, text=True,
                             env=os.environ.copy())
        try:
            psutil.Process(p.pid).cpu_affinity([2])
        except Exception:
            pass
        o, e = p.communicate(timeout=900)
        j = JRE.findall((o or "") + (e or ""))
        with open(csv, encoding="utf-8") as fh:
            head = fh.readline().strip().split(",")
            rows = [dict(zip(head, ln.strip().split(","))) for ln in fh
                    if ln.strip()]
        fly = [r for r in rows if float(r["t"]) > 3.0]
        rms = [float(r["vib_rms_g"]) for r in fly]
        hi = [float(r["vib_high"]) for r in fly]
        out.append((st.median(rms), st.mean(hi),
                    json.loads(j[-1])["pos_rms"] if j else float("nan")))
    return out


def main():
    """命令行入口：逐档测振动估计器的频段响应，打印增量/灵敏度并落盘 JSON。

    以同一二进制、同一弹道的零振动档为基线，逐档打印振动频率、RMS 中位、
    相对零振动的增量 Δrms、vib_high 占空比、位置 RMSE 与每 1 g 的灵敏度（g/g）。
    输出：`sim-data/vib_band_response_2026-10-27.json`。
    """
    print(f"{'档':<10}{'rms_med[g]':>12}{'Δ vs 零':>10}{'vib_high':>10}"
          f"{'pos_rms':>10}")
    print("-" * 54)
    base = None
    res = {}
    for tag, hz, amp in CONDS:
        rows = run(tag, hz, amp)
        rms = st.median([r[0] for r in rows])
        hi = st.mean([r[1] for r in rows])
        pr = st.median([r[2] for r in rows])
        if base is None:
            base = rms
        d = rms - base
        sens = f"{d/amp:.3f} g/g" if amp > 0 else ""
        print(f"{tag:<10}{rms:>12.3f}{d:>+10.3f}{hi*100:>9.0f}%{pr:>10.2f}"
              f"  {sens}")
        res[tag] = {"rms": rms, "delta": d, "hi": hi, "pos_rms": pr,
                    "amp_g": amp}
    import json
    path = ("D:/纵列双发矢量推力飞行器/TandemVec-EKF-Report/sim-data/"
            "vib_band_response_2026-10-27.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(res, fh, ensure_ascii=False)
    print("json:", path)


if __name__ == "__main__":
    main()
