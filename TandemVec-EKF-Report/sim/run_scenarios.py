# -*- coding: utf-8 -*-
"""
EKF 组合导航仿真场景 —— 生成分析报告数值素材
场景:
  S1 静基座对准 + 静止辅助(ZUPT/Gravity/StaticGyro)零偏收敛
  S2 GNSS 位置/速度融合收敛 (含 3σ 包络)
  S3 GNSS 失联 → 协方差膨胀 → 重捕获低增益融合
  S4 GNSS 延迟回放 (15ms) 与无延迟对比
  S5 双矢量航向融合 vs 纯陀螺积分漂移
  S6 NIS 野值门控
输出: sim-data/*.json, fig/*.png
"""
import json
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

from ekf_core import (Ekf15, cfg, cfg_p0, HOME_LLA, ZUPT_DIV, GRAV_DIV,
                      SG_DIV, D2R, R2D, G_MPS2, quat_to_euler, wrap_pi,
                      quat_mult, quat_conj)

OUT = os.path.join(os.path.dirname(__file__), "..", "sim-data")
FIG = os.path.join(os.path.dirname(__file__), "..", "fig")
os.makedirs(OUT, exist_ok=True)
os.makedirs(FIG, exist_ok=True)

RNG = np.random.default_rng(42)
DT = cfg["dt"]
HZ = int(1.0 / DT)


def quat_rot_v(q, v):
    """四元数 (w,x,y,z) 旋转向量；q=单位元时返回 v"""
    qn = np.asarray(q)
    if qn.shape == (3,):
        qn = np.append(1.0, qn)
    qn = qn / np.linalg.norm(qn)
    return quat_mult(quat_mult(qn, np.append(0.0, v)), quat_conj(qn))[1:]


def make_ekf(init_att_deg=0.0):
    ekf = Ekf15(cfg)
    ekf.home_lla = HOME_LLA.copy()
    # 初始静止：机体系比力 = (0,0,-g)（水平）
    accel0 = np.array([0.0, 0.0, -G_MPS2])
    gyro0 = np.zeros(3)
    ekf.initialize(accel0, gyro0, HOME_LLA)
    ekf.home_lla = HOME_LLA.copy()
    return ekf


def simulate_static(duration_s, true_bias_ba, true_bias_bg):
    """S1: 静基座，无 GNSS，静止辅助。返回零偏估计/协方差轨迹"""
    ekf = make_ekf()
    n = int(duration_s * HZ)
    ts, ba_est, bg_est, p_att, p_bg, conf = [], [], [], [], [], []
    frame = 0
    gyro_avg = np.zeros(3)
    gyro_cnt = 0
    for k in range(n):
        t = k * DT
        # 传感器仿真：真值静止
        a_true = np.array([0.0, 0.0, -G_MPS2])
        w_true = np.zeros(3)
        a_meas = a_true + true_bias_ba + RNG.normal(0, cfg["accel_std"], 3)
        w_meas = w_true + true_bias_bg + RNG.normal(0, cfg["gyro_std"], 3)
        ekf.time_update(w_meas * DT, np.zeros(3), a_meas * DT, np.zeros(3), DT)
        # 静止确认 (20 帧后)
        if k >= 20:
            frame += 1
            gyro_avg += w_meas
            gyro_cnt += 1
            if frame % ZUPT_DIV == 0:
                ekf.measurement_update_velocity(np.zeros(3), 0.10, 0.14)
            if frame % GRAV_DIV == 1:
                ekf.measurement_update_gravity(a_meas, 0.10)
            if frame % SG_DIV == 3:
                # StaticGyro 用 250ms 静止均值抑制单帧白噪声（对齐固件静止均值语义）
                gyro_mean = gyro_avg / gyro_cnt
                ekf.measurement_update_static_gyro(gyro_mean, np.full(3, 0.0001))
                gyro_avg = np.zeros(3)
                gyro_cnt = 0
        ts.append(t)
        ba_est.append(ekf.ba.copy())
        bg_est.append(ekf.bg.copy())
        p_att.append(np.diag(ekf.P)[6:9].copy())
        p_bg.append(np.diag(ekf.P)[12:15].copy())
        conf.append(1.0 if k >= 20 else 0.0)
    return (np.array(ts), np.array(ba_est), np.array(bg_est),
            np.array(p_att), np.array(p_bg), np.array(conf))


def simulate_gnss(duration_s, truth_vel, gnss_hz=10, age_s=0.0,
                  gnss_noise=None, outage=(None, None), rng_seed=7,
                  ahrs_fallback=False, replay=True):
    """S2/S3/S4: 恒定速度直线飞行 + GNSS 采样融合。
    gnss_noise: (pos_ne_std, pos_d_std, vel_ne_std, vel_d_std)
    ahrs_fallback: 失联期间用重力方向量测约束 roll/pitch（对齐固件无GNSS+运动时
                   的 AHRS 姿态辅助分支，yaw 噪声=π 忽略）"""
    if gnss_noise is None:
        gnss_noise = (2.0, 3.0, 0.15, 0.25)
    rng = np.random.default_rng(rng_seed)
    ekf = make_ekf()
    n = int(duration_s * HZ)
    ts, pos_err, vel_err, p_pos, p_vel, nis_hist = [], [], [], [], [], []
    gnss_k = 0
    outage_start, outage_end = outage
    # 真实轨迹: 前 2s 从静止线性加速到巡航 (与 EKF 静止初始化一致)，之后匀速
    v_ned_true = np.zeros(3)
    pos_ned_true = np.zeros(3)
    pos_hist = []
    v_hist = []
    for k in range(n):
        t = k * DT
        if t < 2.0:
            v_ned_true = np.array(truth_vel, dtype=float) * (t / 2.0)
        else:
            v_ned_true = np.array(truth_vel, dtype=float)
        # 真实比力（含加速段水平加速度）
        a_ned_true = np.zeros(3)
        if t < 2.0:
            a_ned_true = np.array(truth_vel, dtype=float) / 2.0
        # 真实比力（机体水平朝北 => Cnb=I, 比力 = a_ned - g_ned）
        a_true = a_ned_true - np.array([0, 0, G_MPS2])
        w_true = np.zeros(3)
        a_meas = a_true + RNG.normal(0, cfg["accel_std"], 3)
        w_meas = w_true + RNG.normal(0, cfg["gyro_std"], 3)
        # 加计零偏真值
        a_meas += np.array([0.03, -0.02, 0.05])
        ekf.time_update(w_meas * DT, np.zeros(3), a_meas * DT, np.zeros(3), DT)
        pos_ned_true += v_ned_true * DT
        in_outage = (outage_start is not None and
                     outage_start <= t < outage_end)
        if in_outage and ahrs_fallback:
            # 无 GNSS 运动时：重力方向约束 roll/pitch（加速度可信时）
            if abs(np.linalg.norm(a_meas) - G_MPS2) < 2.0 and \
                    np.linalg.norm(w_meas) < 1.2:
                if k % 4 == 0:
                    ekf.measurement_update_gravity(a_meas, 0.08)
        if not in_outage and k % (HZ // gnss_hz) == 0:
            z_pos = pos_ned_true + rng.normal(0, gnss_noise[0], 3) * np.array(
                [1, 1, gnss_noise[1] / gnss_noise[0]])
            z_vel = v_ned_true + rng.normal(0, gnss_noise[2], 3)
            R_std = list(gnss_noise)
            # 模拟 GNSS 量测确实滞后：量测值是 age_s 之前的真值
            if age_s > 0.0:
                hist_idx = int(round(age_s / DT))
                if k - hist_idx >= 0:
                    z_pos = pos_hist[k - hist_idx] + rng.normal(
                        0, gnss_noise[0], 3) * np.array(
                        [1, 1, gnss_noise[1] / gnss_noise[0]])
                    z_vel = v_hist[k - hist_idx] + rng.normal(
                        0, gnss_noise[2], 3)
                else:
                    z_pos = pos_ned_true + rng.normal(0, gnss_noise[0], 3) * \
                        np.array([1, 1, gnss_noise[1] / gnss_noise[0]])
                    z_vel = v_ned_true + rng.normal(0, gnss_noise[2], 3)
            hist = ekf.hist if replay else []
            res = ekf.measurement_update_gnss(z_pos, z_vel, R_std,
                                               age_s=age_s, hist=hist)
            if res["fused"]:
                nis_hist.append((t, res["nis"], res.get("conservative", False)))
        pos_hist.append(pos_ned_true.copy())
        v_hist.append(v_ned_true.copy())
        est_pos = ekf.ned_from_lla(ekf.lla, HOME_LLA)
        ts.append(t)
        pos_err.append(est_pos - pos_ned_true)
        vel_err.append(ekf.vel - v_ned_true)
        p_pos.append(np.sqrt(np.diag(ekf.P)[0:3]))
        p_vel.append(np.sqrt(np.diag(ekf.P)[3:6]))
    return dict(ts=np.array(ts), pos_err=np.array(pos_err),
                vel_err=np.array(vel_err), p_pos=np.array(p_pos),
                p_vel=np.array(p_vel), nis=np.array(nis_hist))


def simulate_dual_vector_yaw(duration_s, speed, flow_noise_std=0.08,
                             turn_mid_s=None):
    """S5: GPS 地速 + 光流速度 → 双矢量航向。与纯陀螺积分对比。
    turn_mid_s: 中途转弯时刻（检验转弯保护跳过）"""
    rng = np.random.default_rng(11)
    ekf = make_ekf()
    ekf.bg = np.array([0.0, 0.0, 0.004])   # 注入真实 Z 轴陀螺零偏
    ekf.P[12:15, 12:15] = (0.00698**2) * np.eye(3)
    n = int(duration_s * HZ)
    ts, yaw_ekf, yaw_pure, yaw_true, yaw_meas_lst = [], [], [], [], []
    yaw_true_v = 0.0
    yaw_pure_v = 0.0
    for k in range(n):
        t = k * DT
        yaw_rate = 0.0
        if turn_mid_s is not None and abs(t - turn_mid_s) < 0.5:
            yaw_rate = 1.0   # 转弯 ~57 deg/s
        yaw_true_v += yaw_rate * DT
        # 真实机体系
        g_ned = np.array([0.0, 0.0, -G_MPS2])
        # 机体朝 yaw 方向：比力旋转
        cy, sy = math.cos(yaw_true_v), math.sin(yaw_true_v)
        Cnb = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.0]])
        a_true = Cnb @ g_ned
        w_true = np.array([0.0, 0.0, yaw_rate])
        a_meas = a_true + RNG.normal(0, cfg["accel_std"], 3)
        w_meas = w_true + ekf.bg + RNG.normal(0, cfg["gyro_std"], 3)
        dth = w_meas * DT
        ekf.time_update(w_meas * DT, np.zeros(3), a_meas * DT, np.zeros(3), DT)
        # 纯陀螺积分航向（对比）
        yaw_pure_v += w_meas[2] * DT
        # 双矢量量测：GPS 地速 = 航迹角，光流侧滑
        if k % (HZ // 10) == 0 and t > 1.0:
            if yaw_rate == 0.0 and speed > 1.0:
                v_gps = np.array([speed * math.cos(yaw_true_v),
                                  speed * math.sin(yaw_true_v)])
                track = math.atan2(v_gps[1], v_gps[0])
                v_flow_body = np.array([speed, 0.0]) + rng.normal(
                    0, flow_noise_std * speed, 2)
                slip = math.atan2(v_flow_body[1], v_flow_body[0])
                est_yaw = wrap_pi(track - slip)
                yaw_meas_lst.append((t, est_yaw))
                noise = max(0.03, 0.08 / (speed - 1.0 + 1.0))
                ekf.measurement_update_yaw(est_yaw, noise)
        ts.append(t)
        yaw_ekf.append(quat_to_euler(ekf.quat)[0])
        yaw_pure.append(wrap_pi(yaw_pure_v))
        yaw_true.append(yaw_true_v)
    return dict(ts=np.array(ts), yaw_ekf=np.array(yaw_ekf),
                yaw_pure=np.array(yaw_pure), yaw_true=np.array(yaw_true),
                yaw_meas=np.array(yaw_meas_lst))


def simulate_nis_outlier(duration_s, gnss_hz=10):
    """S6: 注入 GNSS 位置野值，观察 NIS 门控拒绝"""
    rng = np.random.default_rng(13)
    ekf = make_ekf()
    n = int(duration_s * HZ)
    v_true = np.array([2.0, 0.0, 0.0])
    pos_true = np.zeros(3)
    ts, nis_l, fused_l = [], [], []
    for k in range(n):
        t = k * DT
        a_meas = np.array([0.0, 0.0, -G_MPS2]) + rng.normal(0, 0.05, 3)
        w_meas = rng.normal(0, 0.0015, 3)
        ekf.time_update(w_meas * DT, np.zeros(3), a_meas * DT, np.zeros(3), DT)
        pos_true += v_true * DT
        if k % (HZ // gnss_hz) == 0:
            z_pos = pos_true + rng.normal(0, 1.5, 3)
            if 3.0 <= t < 3.2:      # 野值：横向跳 30m
                z_pos[0] += 30.0
            if 5.0 <= t < 5.2:      # 野值：纵向跳 -25m
                z_pos[1] += 25.0
            z_vel = v_true + rng.normal(0, 0.1, 3)
            res = ekf.measurement_update_gnss(z_pos, z_vel,
                                              [2.0, 3.0, 0.15, 0.25],
                                              age_s=0.0, hist=[])
            ts.append(t)
            nis_l.append(res["nis"])
            fused_l.append(1 if res["fused"] else 0)
    return np.array(ts), np.array(nis_l), np.array(fused_l)


def main():
    summary = {}

    # ================= S1 静基座收敛 =================
    print("[S1] 静基座零偏收敛仿真 (120s)...")
    ts1, ba1, bg1, patt1, pbg1, _ = simulate_static(
        120.0, true_bias_ba=np.array([0.08, -0.05, 0.12]),
        true_bias_bg=np.array([0.003, -0.002, 0.004]))
    fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    axes[0].plot(ts1, ba1 * 1000.0, lw=1.2)
    axes[0].axhline(80, ls="--", c="k", lw=0.8, label="真值 x=80mg")
    axes[0].axhline(-50, ls="--", c="gray", lw=0.8, label="真值 y=-50mg")
    axes[0].axhline(120, ls=":", c="gray", lw=0.8, label="真值 z=120mg")
    axes[0].set_ylabel("加速度零偏估计 (mg)")
    axes[0].set_ylim(-80, 150)
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)
    axes[1].plot(ts1, bg1 * R2D * 3600.0, lw=1.2)
    axes[1].axhline(0.003 * R2D * 3600, ls="--", c="k", lw=0.8, label="真值 x")
    axes[1].axhline(-0.002 * R2D * 3600, ls="--", c="gray", lw=0.8, label="真值 y")
    axes[1].axhline(0.004 * R2D * 3600, ls=":", c="gray", lw=0.8, label="真值 z")
    axes[1].set_ylabel("陀螺零偏估计 (deg/h)")
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.3)
    axes[2].semilogy(ts1, np.sqrt(patt1) * R2D, lw=1.2, label="roll/pitch/yaw σ")
    axes[2].semilogy(ts1, np.sqrt(pbg1) * R2D * 3600, lw=1.2, ls="--",
                     label="gyro bias σ (deg/h)")
    axes[2].set_xlabel("时间 (s)")
    axes[2].set_ylabel("姿态/零偏不确定性 σ")
    axes[2].legend(fontsize=8)
    axes[2].grid(alpha=0.3, which="both")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "s1_static_zupt.png"), dpi=150)
    plt.close(fig)
    # 统计
    summary["s1"] = {
        "ba_final_mg": (ba1[-1] * 1000).round(2).tolist(),
        "bg_final_degh": (bg1[-1] * R2D * 3600).round(3).tolist(),
        "att_sigma_final_deg": (np.sqrt(patt1[-1]) * R2D).round(3).tolist(),
        "bg_sigma_final_degh": (np.sqrt(pbg1[-1]) * R2D * 3600).round(3).tolist(),
    }
    print("  末态加速度零偏(mg):", summary["s1"]["ba_final_mg"])
    print("  末态陀螺零偏(deg/h):", summary["s1"]["bg_final_degh"])

    # ================= S2 GNSS 融合收敛 =================
    print("[S2] GNSS 融合收敛仿真 (120s, 10Hz)...")
    r2 = simulate_gnss(120.0, truth_vel=[5.0, 1.0, 0.0], rng_seed=7)
    fig, axes = plt.subplots(2, 2, figsize=(10, 7))
    for i, (name, err, p) in enumerate(
            [("北向 (N)", r2["pos_err"][:, 0], r2["p_pos"][:, 0]),
             ("东向 (E)", r2["pos_err"][:, 1], r2["p_pos"][:, 1])]):
        ax = axes[0, i]
        ax.plot(r2["ts"], err, lw=1.0, label="估计误差")
        ax.plot(r2["ts"], 3 * p, ls="--", c="r", lw=0.9, label="3σ 包络")
        ax.plot(r2["ts"], -3 * p, ls="--", c="r", lw=0.9)
        ax.set_title(f"位置误差 {name}")
        ax.set_ylabel("m")
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)
    for i, (name, err, p) in enumerate(
            [("北向 (N)", r2["vel_err"][:, 0], r2["p_vel"][:, 0]),
             ("东向 (E)", r2["vel_err"][:, 1], r2["p_vel"][:, 1])]):
        ax = axes[1, i]
        ax.plot(r2["ts"], err, lw=1.0, label="估计误差")
        ax.plot(r2["ts"], 3 * p, ls="--", c="r", lw=0.9, label="3σ 包络")
        ax.plot(r2["ts"], -3 * p, ls="--", c="r", lw=0.9)
        ax.set_title(f"速度误差 {name}")
        ax.set_xlabel("时间 (s)")
        ax.set_ylabel("m/s")
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "s2_gnss_fusion.png"), dpi=150)
    plt.close(fig)
    # 稳态统计 (取后 60s)
    m = r2["ts"] > 60.0
    summary["s2"] = {
        "pos_ne_steady_rmse_m": np.sqrt(
            np.mean(r2["pos_err"][m, 0]**2 + r2["pos_err"][m, 1]**2)).round(3),
        "pos_d_steady_rmse_m": np.sqrt(np.mean(r2["pos_err"][m, 2]**2)).round(3),
        "vel_steady_rmse_mps": np.sqrt(
            np.mean(r2["vel_err"][m]**2)).round(3),
        "pos_sigma_steady_m": np.sqrt(np.diag(
            r2["p_pos"][-1]**2)).round(3).tolist(),
        "nis_accept_rate": (len(r2["nis"]) and
                            np.mean(r2["nis"][:, 2] == 0)).round(4),
    }
    print("  稳态位置 RMSE (N/E 平面):", summary["s2"]["pos_ne_steady_rmse_m"], "m")
    print("  稳态速度 RMSE:", summary["s2"]["vel_steady_rmse_mps"], "m/s")

    # ================= S3 失联重捕获 =================
    print("[S3] GNSS 失联重捕获仿真 (90s, 30-80s 失联, AHRS 姿态回退)...")
    r3 = simulate_gnss(90.0, truth_vel=[4.0, 0.0, 0.0], rng_seed=9,
                       outage=(30.0, 80.0), ahrs_fallback=True)
    fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    axes[0].plot(r3["ts"], r3["pos_err"][:, 0], lw=1.0, label="北向误差")
    axes[0].axvspan(30, 80, color="orange", alpha=0.25, label="GNSS 失联")
    axes[0].plot(r3["ts"], 3 * r3["p_pos"][:, 0], ls="--", c="r", lw=0.8)
    axes[0].plot(r3["ts"], -3 * r3["p_pos"][:, 0], ls="--", c="r", lw=0.8)
    axes[0].set_ylabel("北向位置误差 (m)")
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)
    axes[1].semilogy(r3["ts"], r3["p_pos"][:, 0], lw=1.2, label="σ_N")
    axes[1].semilogy(r3["ts"], r3["p_pos"][:, 2], lw=1.2, label="σ_D")
    axes[1].axvspan(30, 80, color="orange", alpha=0.25)
    axes[1].set_ylabel("位置不确定性 σ (m)")
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.3, which="both")
    axes[2].plot(r3["ts"], r3["vel_err"][:, 0], lw=1.0, label="北向速度误差")
    axes[2].axvspan(30, 80, color="orange", alpha=0.25)
    axes[2].set_xlabel("时间 (s)")
    axes[2].set_ylabel("速度误差 (m/s)")
    axes[2].legend(fontsize=8)
    axes[2].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "s3_outage_reacq.png"), dpi=150)
    plt.close(fig)
    m2 = (r3["ts"] >= 80.0) & (r3["ts"] <= 90.0)
    summary["s3"] = {
        "pos_drift_peak_m": np.abs(r3["pos_err"][
            (r3["ts"] >= 30) & (r3["ts"] <= 80), 0]).max().round(2),
        "pos_sigma_peak_m": r3["p_pos"][
            (r3["ts"] >= 30) & (r3["ts"] <= 80), 0].max().round(2),
        "reacq_pos_rmse_10s_m": np.sqrt(np.mean(
            r3["pos_err"][m2, 0]**2)).round(3),
    }
    print("  失联峰值漂移:", summary["s3"]["pos_drift_peak_m"], "m")

    # ================= S4 延迟回放对比 =================
    print("[S4] GNSS 延迟回放对比 (60s)...")
    # 三路对照：无延迟（理想）/ 延迟+回放（固件架构）/ 延迟+错误时刻直接融合
    r4_ideal = simulate_gnss(60.0, truth_vel=[5.0, 0.0, 0.0], age_s=0.0,
                             rng_seed=5)
    r4_delay = simulate_gnss(60.0, truth_vel=[5.0, 0.0, 0.0], age_s=0.015,
                             rng_seed=5, replay=True)
    r4_wrong = simulate_gnss(60.0, truth_vel=[5.0, 0.0, 0.0], age_s=0.015,
                             rng_seed=5, replay=False)
    fig, ax = plt.subplots(1, 1, figsize=(8, 4.5))
    ax.plot(r4_ideal["ts"], r4_ideal["pos_err"][:, 0], lw=1.0, c="k",
            label="理想：无延迟")
    ax.plot(r4_delay["ts"], r4_delay["pos_err"][:, 0], lw=1.0, ls="--",
            label="延迟+回放（固件架构）")
    ax.plot(r4_wrong["ts"], r4_wrong["pos_err"][:, 0], lw=1.0, ls=":",
            c="r", label="延迟+错误时刻直接融合")
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("北向位置误差 (m)")
    ax.legend(fontsize=9)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "s4_delay_replay.png"), dpi=150)
    plt.close(fig)
    m3 = r4_ideal["ts"] > 30.0
    summary["s4"] = {
        "ideal_pos_ne_rmse_m": np.sqrt(np.mean(
            r4_ideal["pos_err"][m3, 0]**2)).round(3),
        "delay_replay_pos_ne_rmse_m": np.sqrt(np.mean(
            r4_delay["pos_err"][m3, 0]**2)).round(3),
        "wrong_time_pos_ne_rmse_m": np.sqrt(np.mean(
            r4_wrong["pos_err"][m3, 0]**2)).round(3),
    }
    print("  无延迟 RMSE:", summary["s4"]["ideal_pos_ne_rmse_m"],
          "| 回放 RMSE:", summary["s4"]["delay_replay_pos_ne_rmse_m"],
          "| 错误时刻 RMSE:", summary["s4"]["wrong_time_pos_ne_rmse_m"])

    # ================= S5 双矢量航向融合 =================
    print("[S5] 双矢量航向融合 (80s, 8m/s)...")
    r5 = simulate_dual_vector_yaw(80.0, speed=8.0)
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    axes[0].plot(r5["ts"], np.unwrap(r5["yaw_true"]) * R2D, c="k", lw=1.5,
                 label="真实航向")
    axes[0].plot(r5["ts"], np.unwrap(r5["yaw_ekf"]) * R2D, lw=1.2,
                 label="EKF+双矢量融合")
    axes[0].plot(r5["ts"], np.unwrap(r5["yaw_pure"]) * R2D, ls="--", lw=1.0,
                 label="纯陀螺积分")
    if len(r5["yaw_meas"]):
        disp = r5["yaw_meas"][:, 1] - r5["yaw_true"][0]
        axes[0].plot(r5["yaw_meas"][:, 0],
                     np.unwrap(disp) * R2D + r5["yaw_true"][0] * R2D,
                     ".", ms=2, alpha=0.4, label="双矢量量测")
    axes[0].set_ylabel("航向 (deg)")
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)
    axes[1].plot(r5["ts"], (np.unwrap(r5["yaw_ekf"]) - np.unwrap(
        r5["yaw_true"])) * R2D, lw=1.2, label="EKF 航向误差")
    axes[1].plot(r5["ts"], (np.unwrap(r5["yaw_pure"]) - np.unwrap(
        r5["yaw_true"])) * R2D, lw=1.2, ls="--", label="纯陀螺航向误差")
    axes[1].set_xlabel("时间 (s)")
    axes[1].set_ylabel("航向误差 (deg)")
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "s5_dual_vector_yaw.png"), dpi=150)
    plt.close(fig)
    m4 = r5["ts"] > 10.0
    summary["s5"] = {
        "ekf_yaw_rmse_deg": np.sqrt(np.mean(
            (np.unwrap(r5["yaw_ekf"][m4]) - np.unwrap(r5["yaw_true"][m4]))**2
        )).round(3),
        "pure_yaw_rmse_deg": np.sqrt(np.mean(
            (np.unwrap(r5["yaw_pure"][m4]) - np.unwrap(r5["yaw_true"][m4]))**2
        )).round(3),
        "yaw_meas_count": len(r5["yaw_meas"]),
    }
    print("  EKF 航向 RMSE:", summary["s5"]["ekf_yaw_rmse_deg"], "deg")
    print("  纯陀螺航向 RMSE:", summary["s5"]["pure_yaw_rmse_deg"], "deg")

    # ================= S6 NIS 野值门控 =================
    print("[S6] NIS 野值门控仿真 (10s)...")
    ts6, nis6, fused6 = simulate_nis_outlier(10.0)
    fig, axes = plt.subplots(2, 1, figsize=(9, 5.5), sharex=True)
    axes[0].plot(ts6, nis6, lw=1.0, marker="o", ms=3)
    axes[0].axhline(30.0, ls="--", c="r", lw=1.0, label="NIS 门限=30")
    axes[0].axvline(3.0, ls=":", c="orange", lw=0.8)
    axes[0].axvline(5.0, ls=":", c="orange", lw=0.8)
    axes[0].set_ylabel("NIS 统计量")
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)
    axes[1].step(ts6, fused6, where="post", lw=1.2, c="g")
    axes[1].set_yticks([0, 1])
    axes[1].set_yticklabels(["拒绝", "融合"])
    axes[1].set_xlabel("时间 (s)")
    axes[1].set_ylabel("GNSS 帧状态")
    axes[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "s6_nis_gating.png"), dpi=150)
    plt.close(fig)
    reject_frac = 1.0 - fused6.mean()
    summary["s6"] = {
        "nis_mean": nis6[fused6 == 1].mean().round(2),
        "outlier_reject_frac": round(float(reject_frac), 4),
        "nis_max": nis6.max().round(1),
    }
    print("  野值拒绝率:", summary["s6"]["outlier_reject_frac"])

    # ================= 汇总保存 =================
    with open(os.path.join(OUT, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    print("\n[完成] 所有场景数据已写入", OUT)


if __name__ == "__main__":
    main()
