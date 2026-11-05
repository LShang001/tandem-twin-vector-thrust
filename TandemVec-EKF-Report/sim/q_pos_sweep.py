# -*- coding: utf-8 -*-
"""Q_pos / P0_pos 一致性扫描（研究模型，非固件 SIL）。
@file    q_pos_sweep.py
@brief   位置过程噪声 Q_pos 扫描。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 分析位置协方差与 RMSE 的权衡。
@frame   NED（pos_rmse / vel_rmse 为北东地三轴误差范数）；NEES 无量纲
@units   m（位置 RMSE）、m/s（速度 RMSE）；NEES / NIS 为归一化比值
"""
import json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import run_scenarios as R
from ekf_core import Ekf15, cfg as base_cfg, HOME_LLA, G_MPS2

OUT = os.path.join(HERE, "..", "sim-data", "q_pos_sweep_2026-10-26.json")

def make_filter(qpos, p0scale):
    """构造并初始化为静止的 15 态 EKF，注入位置过程噪声与初始位置协方差倍率。

    参数：qpos 为位置过程噪声谱密度（写入 `q_pos_psd_m2ps`）；
    p0scale 为初始位置协方差倍率（写入 `p0_pos_scale`）。
    返回：以 HOME_LLA 为原点、零速度、比力 (0,0,−g) 初始化的 Ekf15 实例。
    """
    c = dict(base_cfg)
    c["q_pos_psd_m2ps"] = qpos
    c["p0_pos_scale"] = p0scale
    e = Ekf15(c)
    e.home_lla = HOME_LLA.copy()
    e.initialize(np.array([0.0, 0.0, -G_MPS2]), np.zeros(3), HOME_LLA)
    return e

def run_scenario(name, qpos, p0scale, seeds=10, duration=60.0):
    """跑一个场景档的多随机种子仿真，返回各指标的中位与尾部聚合。

    参数：name 为场景名（nominal/highnoise/delay/outage5，决定量测噪声、量测年龄与
    失联窗）；qpos/p0scale 见 make_filter；seeds 为种子数；duration 为时长（s）。
    返回：{指标+"_seedmed": 各 seed 之间的中位}；NEES 的 p95/max 另给 "_seedmax"。
    """
    R.make_ekf = lambda: make_filter(qpos, p0scale)
    if name == "nominal":
        noise, age, outage = None, 0.0, (None, None)
    elif name == "highnoise":
        noise, age, outage = (5.0, 8.0, 0.5, 0.8), 0.0, (None, None)
    elif name == "delay":
        noise, age, outage = None, 0.015, (None, None)
    elif name == "outage5":
        noise, age, outage = None, 0.0, (20.0, 25.0)
    rows = []
    for seed in range(seeds):
        R.RNG = np.random.default_rng(seed + 100)
        r = R.simulate_gnss(duration, (5.0, 0.0, 0.0), gnss_hz=10, age_s=age,
                            gnss_noise=noise, outage=outage, rng_seed=seed,
                            replay=True)
        pe, ve = r["pos_err"], r["vel_err"]
        nees_p, nees_v = r["nees_pos_full"], r["nees_vel_full"]
        nis = r["nis"][:, 1] if len(r["nis"]) else np.array([np.nan])
        rows.append(dict(
            pos_rmse=float(np.sqrt(np.mean(np.sum(pe * pe, axis=1)))),
            vel_rmse=float(np.sqrt(np.mean(np.sum(ve * ve, axis=1)))),
            nees_pos_med=float(np.nanmedian(nees_p)),
            nees_pos_p95=float(np.nanpercentile(nees_p, 95)),
            nees_pos_max=float(np.nanmax(nees_p)),
            nees_vel_med=float(np.nanmedian(nees_v)),
            nees_vel_p95=float(np.nanpercentile(nees_v, 95)),
            nis_med=float(np.nanmedian(nis))))
    agg = {}
    for k in rows[0]:
        vals = [x[k] for x in rows if np.isfinite(x[k])]
        agg[k + "_seedmed"] = float(np.median(vals)) if vals else float("nan")
        if k in ("nees_pos_p95", "nees_pos_max", "nees_vel_p95"):
            agg[k + "_seedmax"] = float(np.max(vals)) if vals else float("nan")
    return agg

def main():
    """命令行入口：Q_pos × P0_pos × 四场景做网格扫描并落盘。

    网格：Q_pos ∈ {0, 1e-4, 1e-3, 1e-2, 3e-2}，P0_pos 倍率 ∈ {1, 2}，
    场景 ∈ {nominal, highnoise, delay, outage5}。
    输出：`sim-data/q_pos_sweep_2026-10-26.json`，键为 `q<Q>_p0<P0>_<场景>`。
    """
    scales = [0.0, 1e-4, 1e-3, 1e-2, 3e-2]
    p0s = [1.0, 2.0]
    out = {}
    for q in scales:
        for p0 in p0s:
            for sc in ("nominal", "highnoise", "delay", "outage5"):
                key = f"q{q:g}_p0{p0:g}_{sc}"
                print("run", key, flush=True)
                out[key] = run_scenario(sc, q, p0)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(out, open(OUT, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("wrote", OUT)

if __name__ == "__main__":
    main()
