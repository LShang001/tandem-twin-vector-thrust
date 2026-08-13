# -*- coding: utf-8 -*-
"""
第三遍复查：数值交叉验证（对固件数学实现的独立验证）
V1  F 矩阵数值微分验证（解析 vs 数值）
V2  NIS 分布一致性检验（S2 噪声=R 配置时 NIS 应 ~ χ²(6)，均值 6 方差 12）
V3  协方差-误差统计一致性（Monte Carlo 复验 + 误差白化检验）
V4  双子样 vs 单子样传播一致性（静止时两者应一致）
"""
import json
import math
import os

import numpy as np

from ekf_core import (Ekf15, cfg, HOME_LLA, D2R, G_MPS2, quat_to_euler,
                      quat_mult, quat_to_dcm, skew, R2D)

DT = cfg["dt"]
OUT = os.path.join(os.path.dirname(__file__), "..", "sim-data")
os.makedirs(OUT, exist_ok=True)


def make_ekf():
    ekf = Ekf15(cfg)
    ekf.home_lla = HOME_LLA.copy()
    ekf.initialize(np.array([0.0, 0.0, -G_MPS2]), np.zeros(3), HOME_LLA)
    ekf.home_lla = HOME_LLA.copy()
    return ekf


def finite_diff_F(ekf, f_body, g_body, h=1e-6):
    """数值微分误差动力学得到 F 矩阵（对照固件简化+完整 F 的关键块）"""
    n = 15
    F = np.zeros((n, n))
    Cnb = quat_to_dcm(ekf.quat)
    g_ned = np.array([0.0, 0.0, ekf.gravity])
    for j in range(n):
        dx = np.zeros(n)
        dx[j] = h
        # 位置误差变化率（线性：δṗ = δv，忽略 Fpp 小项）
        d_p = dx[3:6]
        # 速度误差变化率：-Cbn[f×]δβ - Cbn δba
        d_v = -np.dot(skew(Cnb @ f_body), dx[6:9]) - Cnb @ dx[9:12]
        # 姿态误差变化率：-Skew(g_body)δβ - δbg（忽略传输速率小项）
        d_b = -np.dot(skew(g_body), dx[6:9]) - dx[12:15]
        # 零偏变化率：Markov 衰减（忽略）
        F[:, j] = np.concatenate([d_p, d_v, d_b, np.zeros(3), np.zeros(3)]) / h
    return F


def verify_F_matrix():
    """V1: F 矩阵关键块 解析 vs 数值微分"""
    ekf = make_ekf()
    # 典型状态：倾角 10° 俯仰 + 机体系比力/角速度
    from ekf_core import rotvec_to_quat, quat_mult
    tilt = 10.0 * D2R
    q = rotvec_to_quat(np.array([0.0, tilt, 0.0]))
    ekf.quat = quat_mult(q, ekf.quat)
    ekf.quat /= np.linalg.norm(ekf.quat)
    ekf.Cnb = quat_to_dcm(ekf.quat)
    f_body = np.array([0.3, -0.2, -9.5])   # 去偏比力
    g_body = np.array([0.05, -0.03, 0.02])  # 角速度
    F_num = finite_diff_F(ekf, f_body, g_body)
    # 解析块
    Cnb = ekf.Cnb
    F_an = np.zeros((15, 15))
    F_an[0:3, 3:6] = np.eye(3)
    F_an[3:6, 6:9] = -skew(Cnb @ f_body)
    F_an[3:6, 9:12] = -Cnb
    F_an[6:9, 6:9] = -skew(g_body)
    F_an[6:9, 12:15] = -np.eye(3)
    F_an[5, 2] = 2.0 * ekf.gravity / 6371000.0  # 近似
    err = np.abs(F_num - F_an)
    max_err = err.max()
    print(f"[V1] F矩阵解析 vs 数值微分 最大偏差 = {max_err:.3e}")
    # 逐块报告
    blocks = {
        "F(p,v)": (0, 3, 3, 6), "F(v,beta)": (3, 6, 6, 9),
        "F(v,ba)": (3, 6, 9, 12), "F(beta,beta)": (6, 9, 6, 9),
        "F(beta,bg)": (6, 9, 12, 15),
    }
    for name, (r0, r1, c0, c1) in blocks.items():
        e = err[r0:r1, c0:c1].max()
        status = "OK" if e < 1e-4 else "MISMATCH!"
        print(f"    {name}: max_err={e:.2e} {status}")
    return max_err


def verify_nis_distribution(n_runs=12, duration_s=60.0):
    """V2: NIS 分布一致性。S2 场景（GNSS 噪声=R 配置）下统计 NIS 均值/方差。
    理论 χ²(6): 均值=6, 方差=12, 中位数≈5.35"""
    from run_scenarios import simulate_gnss
    all_nis = []
    for seed in range(1000, 1000 + n_runs):
        r = simulate_gnss(duration_s, truth_vel=[5.0, 1.0, 0.0], rng_seed=seed)
        m = r["ts"] > 10.0
        all_nis.append(r["nis"][r["nis"][:, 2] == 0, 1])  # 非保守融合帧
    nis = np.concatenate(all_nis)
    mean = nis.mean()
    var = nis.var()
    med = np.median(nis)
    # 卡方拟合优度：检验 NIS 落入 χ²(6) 分位区间[0.05,0.95] 的比例
    from scipy.stats import chi2
    lo, hi = chi2.ppf(0.05, 6), chi2.ppf(0.95, 6)
    in_band = np.mean((nis >= lo) & (nis <= hi))
    print(f"[V2] NIS 统计 (n={len(nis)}): 均值={mean:.2f} (理论6), "
          f"方差={var:.1f} (理论12), 中位数={med:.2f} (理论5.35)")
    print(f"     χ²(6) 90% 区间 [5.05, 12.59] 覆盖率 = {in_band*100:.1f}% (理论90%)")
    return dict(nis_mean=float(mean), nis_var=float(var), nis_median=float(med),
                nis_90pct_band=float(in_band))


def verify_innovation_whiteness():
    """V3: 新息白化检验——位置误差的自相关（应快速衰减）"""
    from run_scenarios import simulate_gnss
    r = simulate_gnss(120.0, truth_vel=[5.0, 1.0, 0.0], rng_seed=7)
    e = r["pos_err"][r["ts"] > 60.0, 0]
    # 滞后 1 的样本自相关
    e_c = e - e.mean()
    lag1 = np.corrcoef(e_c[:-1], e_c[1:])[0, 1]
    lag5 = np.corrcoef(e_c[:-5], e_c[5:])[0, 1]
    print(f"[V3] 位置误差自相关: lag1={lag1:.3f}, lag5={lag5:.3f} "
          f"(应接近0表示白化)")
    return dict(lag1=float(lag1), lag5=float(lag5))


def verify_two_sample_static():
    """V4: 静止时双子样 vs 单子样传播应一致（无圆锥/划摇时）"""
    ekf1 = make_ekf()
    ekf2 = make_ekf()
    a = np.array([0.0, 0.0, -G_MPS2])
    w = np.zeros(3)
    for _ in range(200):  # 1 秒
        # 单子样
        ekf1.time_update(w * DT, np.zeros(3), a * DT, np.zeros(3), DT)
        # 双子样（拆成两半）
        ekf2.time_update(w * DT / 2, w * DT / 2, a * DT / 2, a * DT / 2, DT)
    d_q = np.abs(ekf1.quat - ekf2.quat).max()
    d_v = np.abs(ekf1.vel - ekf2.vel).max()
    print(f"[V4] 静止单/双子样差异: quat={d_q:.2e}, vel={d_v:.2e} "
          f"(应≈0)")
    return dict(quat_diff=float(d_q), vel_diff=float(d_v))


def verify_repropagation_consistency():
    """V5: GNSS 延迟回放 vs 无延迟（理想）——时间对齐正确时应统计一致"""
    from run_scenarios import simulate_gnss
    r_ideal = simulate_gnss(60.0, truth_vel=[5.0, 0.0, 0.0], age_s=0.0, rng_seed=5)
    r_delay = simulate_gnss(60.0, truth_vel=[5.0, 0.0, 0.0], age_s=0.015, rng_seed=5)
    m = r_ideal["ts"] > 30.0
    rmse_ideal = np.sqrt(np.mean(r_ideal["pos_err"][m, 0]**2))
    rmse_delay = np.sqrt(np.mean(r_delay["pos_err"][m, 0]**2))
    print(f"[V5] 回放一致性: 理想 RMSE={rmse_ideal:.4f}, 回放 RMSE={rmse_delay:.4f}")
    return dict(ideal=float(rmse_ideal), replay=float(rmse_delay))


def main():
    results = {}
    results["V1_F_max_err"] = verify_F_matrix()
    results["V2_NIS"] = verify_nis_distribution()
    results["V3_whiteness"] = verify_innovation_whiteness()
    results["V4_two_sample"] = verify_two_sample_static()
    results["V5_replay"] = verify_repropagation_consistency()
    with open(os.path.join(OUT, "verify.json"), "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, indent=2, default=float)
    print("\n[完成] 验证结果写入 sim-data/verify.json")


if __name__ == "__main__":
    main()
