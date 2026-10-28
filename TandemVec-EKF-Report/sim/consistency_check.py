# -*- coding: utf-8 -*-
"""位置/速度 NEES 的分布级一致性核验（研究模型，非固件 SIL）。
@file    consistency_check.py
@brief   EKF 协方差一致性核验。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 用 Python 同构 15 态 EKF 做 NEES/NIS 分布级检查。
@frame   N/A
@units   N/A

NEES 按 3 自由度归一化（除以 3），因此一致时中位≈0.789、p95≈2.60；
单步 max 随样本数增长，必须用同样本量的卡方蒙特卡洛分布判断，不能直接当尾部失配。
"""
import json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import run_scenarios as R
from ekf_core import Ekf15, cfg as base_cfg, HOME_LLA, G_MPS2

OUT = os.path.join(HERE, "..", "sim-data", "consistency_check_2026-10-26.json")
NPZ = os.path.join(HERE, "..", "sim-data", "consistency_check_2026-10-26.npz")


def make_filter(qpos=0.0, p0scale=1.0):
    c = dict(base_cfg)
    c["q_pos_psd_m2ps"] = qpos
    c["p0_pos_scale"] = p0scale
    e = Ekf15(c)
    e.home_lla = HOME_LLA.copy()
    e.initialize(np.array([0.0, 0.0, -G_MPS2]), np.zeros(3), HOME_LLA)
    return e


def run(scenario, seeds, qpos=0.0):
    R.make_ekf = lambda: make_filter(qpos)
    noise, age, outage = None, 0.0, (None, None)
    if scenario == "highnoise":
        noise = (5.0, 8.0, 0.5, 0.8)
    elif scenario == "delay":
        age = 0.015
    elif scenario == "outage5":
        outage = (20.0, 25.0)
    nees_p, nees_v, nis_l = [], [], []
    per_seed = []
    for seed in seeds:
        R.RNG = np.random.default_rng(seed + 100)
        r = R.simulate_gnss(60.0, (5.0, 0.0, 0.0), gnss_hz=10, age_s=age,
                            gnss_noise=noise, outage=outage, rng_seed=seed,
                            replay=True)
        p = r["nees_pos_full"]; v = r["nees_vel_full"]
        n = r["nis"][:, 1] if len(r["nis"]) else np.array([np.nan])
        nees_p.append(p); nees_v.append(v); nis_l.append(n)
        per_seed.append(dict(seed=seed, nees_pos_med=float(np.nanmedian(p)),
                             nees_pos_max=float(np.nanmax(p)),
                             nees_pos_p95=float(np.nanpercentile(p, 95)),
                             nees_vel_med=float(np.nanmedian(v)),
                             pos_rmse=float(np.sqrt(np.mean(np.sum(r["pos_err"] ** 2, 1))))))
    return dict(nees_p=np.concatenate(nees_p), nees_v=np.concatenate(nees_v),
                nis=np.concatenate(nis_l), per_seed=per_seed)


def chi2_over_df_quantiles(df, n_mc, trials=4000):
    """χ²(df)/df 的分位数，以及同样本量 n_mc 下 max 的分布分位数。"""
    rng = np.random.default_rng(20261026)
    qs = [0.5, 0.9, 0.95, 0.99, 0.999]
    one = rng.chisquare(df, size=200000) / df
    q1 = {str(q): float(np.quantile(one, q)) for q in qs}
    maxima = np.empty(trials)
    chunk = 2000
    for i in range(0, trials, chunk):
        k = min(chunk, trials - i)
        s = rng.chisquare(df, size=(k, n_mc)) / df
        maxima[i:i + k] = s.max(axis=1)
    qmax = {str(q): float(np.quantile(maxima, q)) for q in (0.5, 0.9, 0.95, 0.99)}
    return q1, qmax


def main():
    seeds_main = list(range(100, 110))
    seeds_old = list(range(0, 5))
    out = dict(meta=dict(seeds_main=seeds_main, seeds_old=seeds_old, duration_s=60.0,
                         dt_s=0.005, nees_note="NEES 已按 3 自由度归一化"))
    arrays = {}
    for sc in ("nominal", "highnoise", "delay", "outage5"):
        for tag, seeds in (("main", seeds_main), ("old", seeds_old)):
            if tag == "old" and sc != "nominal":
                continue
            print("run", sc, tag, flush=True)
            r = run(sc, seeds, qpos=0.0)
            key = f"{sc}_{tag}"
            n_epochs = len(r["nees_p"])
            q1p, qmaxp = chi2_over_df_quantiles(3, n_epochs)
            q1v, qmaxv = chi2_over_df_quantiles(3, len(r["nees_v"]))
            out[key] = dict(
                n_epochs=int(n_epochs),
                nees_pos_med=float(np.nanmedian(r["nees_p"])),
                nees_pos_p95=float(np.nanpercentile(r["nees_p"], 95)),
                nees_pos_max=float(np.nanmax(r["nees_p"])),
                nees_pos_frac_gt3=float(np.mean(r["nees_p"] > 3.0)),
                nees_vel_med=float(np.nanmedian(r["nees_v"])),
                nees_vel_p95=float(np.nanpercentile(r["nees_v"], 95)),
                nees_vel_max=float(np.nanmax(r["nees_v"])),
                nis_med=float(np.nanmedian(r["nis"])),
                nis_mean=float(np.nanmean(r["nis"])),
                nis_p95=float(np.nanpercentile(r["nis"], 95)),
                per_seed=r["per_seed"],
                chi2_df3_quantiles=q1p, chi2_df3_max_quantiles=qmaxp,
                chi2_df3_vel_max_quantiles=qmaxv,
                chi2_df6_quantiles=chi2_over_df_quantiles(6, len(r["nis"]))[0])
            arrays[f"{key}_nees_pos"] = r["nees_p"]
            arrays[f"{key}_nees_vel"] = r["nees_v"]
            arrays[f"{key}_nis"] = r["nis"]
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    np.savez_compressed(NPZ, **arrays)
    json.dump(out, open(OUT, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("wrote", OUT)


if __name__ == "__main__":
    main()
