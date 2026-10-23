# -*- coding: utf-8 -*-
"""opt_ekf.py — EKF 优化探索框架：变体 EKF + 静止检测/分级 + 轨迹与场景引擎。

@file    opt_ekf.py
@module  analysis/EKF 优化研究（变体与场景引擎）
@brief   在 ekf_core.Ekf15 上增加可开关算法变体、固件语义静止辅助与轨迹/场景运行器。
@details 输入：场景规格（轨迹 + GNSS/辅助开关 + 故障）与参数覆盖 dict；
         处理：build_cfg/build_p0 生成固件现值配置 → Scenario 逐历元推进真值轨迹
               → OptEkf 传播/更新 → 统计 RMSE、NIS、接受率、航向踢动；
         输出：指标 dict（scenario.run() 返回值），由 run_*.py 汇总成 JSON。
         开关全关时与 ekf_core 现状行为一致（等价性由 run_baseline 复现）。
@theory  15 态 ESKF 与 Joseph/NIS 见 @ref REF-121 §2.2.1 p.33、§2.2.4 p.42–43、§7.3.1 p.255–259；
         静止检测四条件（|a|≈g、|ω|≈0、Δa、Δω）+ 迟滞 @ref TandemVec_FCS/include/ins_static_detector.h；
         三档静止辅助分级 @ref TandemVec_FCS/include/ins_static_aid_profile.h；
         失联协方差膨胀与重捕获低增益 @ref REF-121 §7.3.1 p.255–259（约束一致性）；
         NIS 仅是一致性检验、对软故障不敏感 @ref docs/08-参考资料/notes/02-卡尔曼滤波理论.md §1.13。
@frame  NED + FRD；四元数 (w,x,y,z)；双矢量航向 H 取 6:9（失准角行）为固件口径，
         仿真旧版 7:10 已确认错位（见 @ref docs/08-参考资料/notes/05-仿真验证.md §2）。
@units  SI；噪声 std 用 m/s²、m/s、rad、rad/s；角度内部 rad。
@note    ★ P0 口径（2026-10-23 修正）：FW["p0_hdg"] 取固件现值 0.2 rad；
         历史 π 是修复前值，仅用于显式覆盖复现旧失效链（spec["p0_hdg"]=π）。
         候选/基线对比统一用 CUR_SPEC={"p0_hdg": 0.2}（见 upgrade_candidates.py）。
         GNSS R 地板（2/3/0.15/0.25）经实测**不可收紧**；Q 加计 0.25 在无实机震动数据前不动。
@see     run_baseline.py、run_sweep.py、run_variants.py、upgrade_candidates.py、
         @ref TandemVec-EKF-Report/优化研究-算法与参数优化研究报告.md
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from ekf_core import (Ekf15, cfg, cfg_p0, HOME_LLA, D2R, R2D, G_MPS2,
                      EarthModel, quat_mult, quat_conj, quat_to_dcm,
                      quat_to_euler, rotvec_to_quat, wrap_pi, skew)

# ---------------------------------------------------------------------------
# 固件实机运行参数默认值（来源见各注释）
# ---------------------------------------------------------------------------
FW = {
    # —— EKF 过程噪声（navigation_task.cpp 初始化噪声块 872-893）——
    "accel_std": 0.25,           # 加计白噪声 std (m/s²)
    "accel_bias_std": 0.01,      # 加计零偏 Markov 驱动 (m/s²)
    "accel_tau": 100.0,          # 相关时间 (s)
    "gyro_std": 0.0015,          # 陀螺白噪声 std (rad/s)
    "gyro_bias_std": 0.00524,    # 陀螺零偏 Markov 驱动 (rad/s, ≈0.3°/s)
    "gyro_tau": 180.0,           # 相关时间 (s)
    # —— 初始协方差（同上）——
    "p0_pos": 2.0, "p0_vel": 0.08, "p0_att": 0.08, "p0_hdg": 0.2,
    "p0_ba": 0.196, "p0_bg": 0.00698,
    # —— GNSS 动态权重地板（ins_gnss_dynamic_weight.h）——
    "r_pos_ne": 2.0, "r_pos_d": 3.0, "r_vel_ne": 0.15, "r_vel_d": 0.25,
    # —— NIS 门限（ekf_15_state.h 编译期常量）——
    "nis_gnss": 30.0, "nis_zupt": 1000.0, "nis_grav": 18.0,
    "nis_sg": 24.0, "nis_yaw": 12.0, "nis_baro": 16.0,
    # —— ZUPT/速度物理残差门限 ——
    "zupt_res_rej": 5.5, "vel_res_rej": 8.0,
    # —— 失联膨胀（GNSS_OUTAGE_*_GROWTH）——
    "outage_start": 2.0, "outage_cap": 4.0,
    "outage_rate": [1.2**2, 1.2**2, 1.8**2, 0.35**2, 0.35**2, 0.5**2],
    # —— 重捕获（GNSS_REACQUIRE_*）——
    "reacq_r_infl": [2.0**2, 2.0**2, 4.0**2, 0.5**2, 0.5**2, 0.7**2],
    "reacq_nis2": 80.0, "reacq_window": 6.0,
    # —— 静止检测器（navigation_task.cpp:738-745）——
    "sd_a_lo": 9.5, "sd_a_hi": 10.1, "sd_g": 0.01,
    "sd_da": 0.5, "sd_dg": 0.05, "sd_enter": 20, "sd_exit": 5,
    "sd_dwell_frames": 60,
    # —— 静止辅助三档（ins_static_aid_profile.h）——
    "tier1_grav": 0.18, "tier1_vel_ne": 0.18, "tier1_vel_d": 0.24,
    "tier2_grav": 0.10, "tier2_vel_ne": 0.10, "tier2_vel_d": 0.14,
    "tier3_grav": 0.03, "tier3_vel_ne": 0.02, "tier3_vel_d": 0.04,
    "tier1_frames": 36, "tier2_frames": 100, "tier2_conf": 0.80,
    "sg_noise": 1e-4, "sg_res_rej": 0.05,
    # —— 双矢量航向（navigation_task.cpp:394-524）——
    "dv_min_speed": 1.0, "dv_max_turn": 0.35, "dv_ratio_tol": 0.35,
    "dv_base_noise": 0.08, "dv_min_noise": 0.03,
    # —— 气压辅助（BFS_EKF_BARO_ALTITUDE_UPDATE）——
    "baro_noise": 0.35, "baro_res_rej": 50.0,
    # —— GNSS 延迟 ——
    "gnss_age": 0.015,
}


def build_cfg(overrides=None, opts=None):
    """构造 EKF cfg 字典（含 Ekf15 可调项），默认=固件现值"""
    c = dict(cfg)
    c.update({
        "accel_std": FW["accel_std"], "accel_bias_std": FW["accel_bias_std"],
        "accel_tau": FW["accel_tau"], "gyro_std": FW["gyro_std"],
        "gyro_bias_std": FW["gyro_bias_std"], "gyro_tau": FW["gyro_tau"],
        "nis_gnss": FW["nis_gnss"], "nis_zupt": FW["nis_zupt"],
        "nis_grav": FW["nis_grav"], "nis_sg": FW["nis_sg"],
        "nis_yaw": FW["nis_yaw"],
        "zupt_res_rej": FW["zupt_res_rej"], "vel_res_rej": FW["vel_res_rej"],
        "outage_start": FW["outage_start"], "outage_cap": FW["outage_cap"],
        "outage_rate": list(FW["outage_rate"]),
        "reacq_r_infl": list(FW["reacq_r_infl"]),
        "reacq_nis2": FW["reacq_nis2"],
    })
    if overrides:
        c.update(overrides)
    return c


def build_p0(overrides=None):
    """构造初始协方差 P0 参数 dict（默认=固件现值）。

    @brief   把 cfg_p0 复制为可变 dict，再按变体覆盖。
    @param[in] overrides 覆盖项，如 {"hdg": 0.2}（修复后的航向初值）
    @return  dict：pos/vel/att/hdg/ba/bg 的 1σ 初值
    @note    航向项 hdg=π 是修复前口径；候选基线统一覆盖为 0.2 rad
             （伪磁确定性初始化语义，见 2026-10-21 P0 修复）。
    @ref     docs/08-参考资料/notes/05-仿真验证.md §1.1（基线口径）
    """
    p = dict(cfg_p0)
    if overrides:
        p.update(overrides)
    return p


# ---------------------------------------------------------------------------
# 静止检测器（固件 ins_static_detector.h 语义）
# ---------------------------------------------------------------------------
class StaticDetector:
    """固件语义静止检测器（四条件 + 迟滞 + 置信度）。

    四条件：|a|∈[sd_a_lo,sd_a_hi]（≈g）、|ω|<sd_g、|Δa|<sd_da、|Δω|<sd_dg。
    进入需连续 sd_enter 帧，退出需连续 sd_exit 帧（迟滞防抖）；
    置信度 = min(四子分, dwell 进度)，供三档静止辅助选择档位。

    @note   ★ 已知机理风险：水平匀速飞行与真静止在 IMU 上不可区分（|a|≈g、|ω|≈0、
           Δa≈0），会误确认静止 → GNSS 速度被强制 0 → 6 维联合更新 NIS 爆炸 →
           位置一并被拒 → 纯惯导漂移。固件修复是"GNSS 速度>0.8 m/s 强制退出
           （只出不进）"，本类以 force_exit() 提供该出口，由导航侧按外部速度调用。
           @ref docs/08-参考资料/notes/03-组合导航与鲁棒性.md §4.4
    """

    def __init__(self, prm):
        self.p = prm
        self.enter_cnt = 0
        self.exit_cnt = 0
        self.confirmed = False
        self.frames = 0
        self.confidence = 0.0
        self._last_a = None
        self._last_g = None

    def update(self, accel, gyro):
        p = self.p
        a_norm = float(np.linalg.norm(accel))
        g_norm = float(np.linalg.norm(gyro))
        ok_a = p["sd_a_lo"] <= a_norm <= p["sd_a_hi"]
        ok_g = g_norm < p["sd_g"]
        ok_d = True
        if self._last_a is not None:
            ok_d = (float(np.linalg.norm(accel - self._last_a)) < p["sd_da"]
                    and float(np.linalg.norm(gyro - self._last_g)) < p["sd_dg"])
        self._last_a = accel.copy()
        self._last_g = gyro.copy()
        ok = ok_a and ok_g and ok_d
        if ok:
            self.exit_cnt = 0
            if not self.confirmed:
                self.enter_cnt += 1
                if self.enter_cnt >= p["sd_enter"]:
                    self.confirmed = True
                    self.frames = 0
            else:
                self.frames += 1
        else:
            self.enter_cnt = 0
            if self.confirmed:
                self.exit_cnt += 1
                if self.exit_cnt >= p["sd_exit"]:
                    self.confirmed = False
                    self.frames = 0
        # 置信度（四子分 min + dwell）
        g = 9.80665
        tol = p["sd_a_hi"] - g
        s1 = max(0.0, 1.0 - abs(a_norm - g) / tol)
        s2 = max(0.0, 1.0 - g_norm / p["sd_g"])
        s3 = 1.0 if self._last_a is None else max(
            0.0, 1.0 - float(np.linalg.norm(accel - self._last_a)) / p["sd_da"])
        s4 = 1.0 if self._last_g is None else max(
            0.0, 1.0 - float(np.linalg.norm(gyro - self._last_g)) / p["sd_dg"])
        raw = min(s1, s2, s3, s4)
        dwell = min(1.0, self.frames / p["sd_dwell_frames"]) if self.confirmed else 0.0
        self.confidence = min(raw, dwell) if self.confirmed else 0.0
        return self.confirmed

    def force_exit(self):
        """外部证据（如 GNSS 速度）强制退出静止，防误静止巡航"""
        if self.confirmed:
            self.confirmed = False
            self.frames = 0
            self.enter_cnt = 0
            self.confidence = 0.0


# ---------------------------------------------------------------------------
# 静止辅助分级（固件 ins_static_aid_profile.h 语义）
# ---------------------------------------------------------------------------
class StaticAid:
    """静止辅助三档分级（固件 ins_static_aid_profile.h 语义）。

    档位由置信度与已确认静止帧数决定：tier1（刚确认）/tier2（frames≥100 且
    置信度≥0.80）/tier3（更久），档位越高量测噪声越小、分频越稀。

    @return tier() 返回 1/2/3；divs() 返回 (ZUPT, 重力, 静止陀螺) 的分频数；
            noise() 返回 (重力, 速度NE, 速度D) 的量测噪声 std
    @note    分频是相对 200 Hz 基的；tier1 的 4/8/8 与固件一致。
    """

    def __init__(self, prm):
        self.p = prm
        self.frame_cnt = 0   # 本次静止会话帧号

    def tier(self, det, cov_healthy=True):
        p = self.p
        if det.confidence < 0.35 or det.frames < p["tier1_frames"]:
            return 1
        if det.confidence < p["tier2_conf"] or det.frames < p["tier2_frames"]:
            return 2
        return 3

    def divs(self, tier):
        if tier == 1:
            return 4, 8, 8
        if tier == 2:
            return 8, 16, 16
        return 16, 32, 32

    def noise(self, tier):
        p = self.p
        if tier == 1:
            return p["tier1_grav"], p["tier1_vel_ne"], p["tier1_vel_d"]
        if tier == 2:
            return p["tier2_grav"], p["tier2_vel_ne"], p["tier2_vel_d"]
        return p["tier3_grav"], p["tier3_vel_ne"], p["tier3_vel_d"]


# ---------------------------------------------------------------------------
# 变体 EKF
# ---------------------------------------------------------------------------
class OptEkf(Ekf15):
    """在 Ekf15 基础上增加可开关算法变体。开关全关 = 现状行为。"""

    def __init__(self, cfgd, opts=None):
        super().__init__(cfgd)
        self.opts = opts or {}
        self.base_reacq_r_infl = list(self.reacq_r_infl)
        # 自适应 R 状态
        self._adr_win = []
        # 重捕获膨胀衰减状态
        self._reacq_elapsed = 0.0
        # 气压辅助
        self._last_baro_t = -1e9
        self._baro_rej_streak = 0
        self.meas_log = {"gnss_nis": [], "zupt_nis": [], "grav_nis": [],
                         "sg_nis": [], "yaw_nis": [], "baro_nis": [],
                         "gnss_fused": 0, "gnss_rej": 0, "gnss_conservative": 0,
                         "zupt_fused": 0, "zupt_rej": 0, "grav_fused": 0,
                         "grav_rej": 0, "sg_fused": 0, "sg_rej": 0,
                         "baro_fused": 0, "baro_rej": 0, "yaw_fused": 0,
                         "yaw_rej": 0}

    # ---- 重力方向 / 静止陀螺：日志包装 ----
    def measurement_update_gravity(self, accel_body, noise_rad):
        r = super().measurement_update_gravity(accel_body, noise_rad)
        if r.get("fused"):
            self.meas_log["grav_fused"] += 1
            self.meas_log["grav_nis"].append(r["nis"])
        else:
            self.meas_log["grav_rej"] += 1
        return r

    def measurement_update_static_gyro(self, gyro_body, noise):
        r = super().measurement_update_static_gyro(gyro_body, noise)
        if r.get("fused"):
            self.meas_log["sg_fused"] += 1
            self.meas_log["sg_nis"].append(r["nis"])
        else:
            self.meas_log["sg_rej"] += 1
        return r

    def measurement_update_velocity(self, z_vel, std_ne, std_d):
        r = super().measurement_update_velocity(z_vel, std_ne, std_d)
        if r.get("fused"):
            self.meas_log["zupt_fused"] += 1
            self.meas_log["zupt_nis"].append(r["nis"])
        else:
            self.meas_log["zupt_rej"] += 1
        return r

    # ---- GNSS 分离序贯更新：位置 3 维 + 速度 3 维独立门控 ----
    # 价值：误静止巡航时速度新息大只拒速度子更新，位置继续融合（不再整体漂移）
    def measurement_update_gnss_split(self, z_pos_ned, z_vel_ned, R_std,
                                      age_s=0.0, hist=None):
        # 延迟回放：找历史快照
        restored = False
        best = -1
        if age_s > 0 and hist is not None:
            acc = 0.0
            best_err = age_s
            for i in range(len(hist) - 1, 0, -1):
                acc += hist[i]["dt"]
                err = abs(acc - age_s)
                if err <= best_err:
                    best_err = err
                    best = i - 1
                if acc >= age_s:
                    break
            if best >= 0 and best + 1 < len(hist):
                self.restore(hist[best])
                restored = True
        # ---- 位置子更新 ----
        Hp = np.zeros((3, 15))
        Hp[0:3, 0:3] = np.eye(3)
        Rp = np.diag([R_std[0]**2, R_std[0]**2, R_std[1]**2])
        yp = z_pos_ned - self.ned_from_lla(self.lla, self.home_lla)
        Sp = Hp @ self.P @ Hp.T + Rp
        nisp = yp @ np.linalg.solve(Sp, yp)
        fused_pos = nisp <= self.opts.get("nis_gnss3", 22.0)
        if fused_pos:
            Kp = self.P @ Hp.T @ np.linalg.solve(Sp, np.eye(3))
            I15 = np.eye(15)
            self.P = (I15 - Kp @ Hp) @ self.P @ (I15 - Kp @ Hp).T + \
                Kp @ Rp @ Kp.T
            self.P = self.stabilize(self.P)
            self.inject(Kp @ yp)
        # ---- 速度子更新（含重捕获逻辑）----
        Hv = np.zeros((3, 15))
        Hv[0:3, 3:6] = np.eye(3)
        Rv = np.diag([R_std[2]**2, R_std[2]**2, R_std[3]**2])
        yv = z_vel_ned - self.vel
        Sv = Hv @ self.P @ Hv.T + Rv
        nisv = yv @ np.linalg.solve(Sv, yv)
        fused_vel = False
        conservative = False
        if nisv <= self.opts.get("nis_gnss3", 22.0):
            fused_vel = True
        elif self.time_since_gnss >= self.outage_start or \
                self.gnss_reacquire_timer > 0:
            R2 = Rv.copy()
            R2[0, 0] += self.reacq_r_infl[3]
            R2[1, 1] += self.reacq_r_infl[4]
            R2[2, 2] += self.reacq_r_infl[5]
            S2 = Hv @ self.P @ Hv.T + R2
            nis2 = yv @ np.linalg.solve(S2, yv)
            if nis2 <= self.reacq_nis2:
                Sv, Rv, nisv = S2, R2, nis2
                fused_vel = True
                conservative = True
        if fused_vel:
            Kv = self.P @ Hv.T @ np.linalg.solve(Sv, np.eye(3))
            I15 = np.eye(15)
            self.P = (I15 - Kv @ Hv) @ self.P @ (I15 - Kv @ Hv).T + \
                Kv @ Rv @ Kv.T
            self.P = self.stabilize(self.P)
            self.inject(Kv @ yv)
        # 重传播
        if restored:
            for j in range(best + 1, len(hist)):
                inc = hist[j].get("inc")
                if inc is None:
                    continue
                dth1, dth2, dv1, dv2, ddt = inc
                self.time_update(dth1, dth2, dv1, dv2, ddt, record=False)
            self.time_since_gnss = 0.0
        if fused_pos or fused_vel:
            self.time_since_gnss = 0.0
            if conservative:
                self.gnss_reacquire_timer = 6.0
        return {"fused": fused_pos or fused_vel, "fused_pos": fused_pos,
                "fused_vel": fused_vel, "nis_pos": float(nisp),
                "nis_vel": float(nisv), "conservative": conservative}

    # ---- GNSS 更新：自适应 R / 重捕获衰减 ----
    def measurement_update_gnss(self, z_pos_ned, z_vel_ned, R_std,
                                age_s=0.0, hist=None):
        R_std = list(R_std)
        if self.opts.get("adaptive_r"):
            R_std = self._adapt_r(z_pos_ned, z_vel_ned, R_std, age_s, hist)
        if self.opts.get("reacq_decay") and self.gnss_reacquire_timer > 0:
            frac = self.gnss_reacquire_timer / max(
                self.opts.get("reacq_window", 6.0), 1e-9)
            f = 0.25 + 0.75 * frac
            self.reacq_r_infl = [v * f for v in self.base_reacq_r_infl]
        else:
            self.reacq_r_infl = list(self.base_reacq_r_infl)
        return super().measurement_update_gnss(z_pos_ned, z_vel_ned, R_std,
                                               age_s=age_s, hist=hist)

    def _adapt_r(self, z_pos_ned, z_vel_ned, R_std, age_s, hist):
        """新息方差自适应：滚动窗口估计新息协方差，超标时放大 R。
        预演残差（用当前状态近似，不触发状态变更）"""
        o = self.opts
        pos_err = z_pos_ned - self.ned_from_lla(self.lla, self.home_lla)
        vel_err = z_vel_ned - self.vel
        inn = np.concatenate([pos_err, vel_err])
        self._adr_win.append(inn.copy())
        if len(self._adr_win) > o.get("adr_window", 40):
            self._adr_win.pop(0)
        # 以预测新息方差 P_ii + R 为基准，估计新息放大因子
        Pdiag = np.diag(self.P)[0:6]
        R2 = np.array([R_std[0]**2, R_std[0]**2, R_std[1]**2,
                       R_std[2]**2, R_std[2]**2, R_std[3]**2])
        base_var = Pdiag + R2
        obs_var = np.var(np.array(self._adr_win), axis=0) + 1e-12
        ratio = np.sqrt(obs_var / base_var)
        # 只放大不放小；逐通道限幅
        scale = np.clip(ratio, 1.0, o.get("adr_max_scale", 8.0))
        scale = scale.reshape(-1)
        return [R_std[0] * scale[0], R_std[1] * scale[2],
                R_std[2] * scale[3], R_std[3] * scale[5]]

    def _decay_reacq(self, R_std):
        """重捕获窗口内膨胀量逐帧衰减（默认恒定 → 变体）"""
        pass  # 在父类逻辑后处理：重捕获状态由 gnss_reacquire_timer 维护

    # ---- ZUPT：残差自适应噪声（固件 ins_static_aid_profile.h:113-138）----
    def measurement_update_velocity_adaptive(self, z_vel, std_ne, std_d):
        y = z_vel - self.vel
        res = float(np.linalg.norm(y))
        if res > 1.5:
            self.meas_log["zupt_rej"] += 1
            return {"fused": False, "nis": float("nan"), "rejected": "residual"}
        scale = 1.0 if res <= 0.2 else (
            1.0 + 5.0 * (res - 0.2) if res <= 0.6
            else 3.0 + (3.0 / 0.9) * (res - 0.6))
        scale = min(scale, 6.0)
        return self.measurement_update_velocity(
            z_vel, std_ne * scale, std_d * scale)

    # ---- 气压辅助（固件 BFS_EKF_BARO_ALTITUDE_UPDATE 语义）----
    def measurement_update_baro(self, alt_up_m, noise_std=None):
        """一维高度量测（向上为正）。H = -e2（见推导）。"""
        noise = noise_std if noise_std is not None else FW["baro_noise"]
        # lla[2] 是 WGS-84 高度（向上为正），不是 NED down；
        # 旧实现写成 -(lla-home) 会让“当前高度”变成负值，气压修正反向。
        alt_est = self.lla[2] - self.home_lla[2]
        y = alt_up_m - alt_est
        if abs(y) > FW["baro_res_rej"]:
            self.meas_log["baro_rej"] += 1
            self._baro_recover(y)
            return {"fused": False, "nis": float("nan"), "rejected": "residual"}
        H = np.zeros((1, 15))
        # y = alt_up - alt_est = -(D_meas - D_est) = -δp_D（向上为正）
        # 状态 x[2] 是 NED down 误差，故 H=-e₂；旧 +e₂ 符号反了，
        # 会把气压修正推向错误方向并让 P_DD 坍缩、后续 NIS 全拒。
        H[0, 2] = -1.0
        R = np.array([[noise**2]])
        S = H @ self.P @ H.T + R
        nis = float(y * y / S[0, 0])
        self.meas_log["baro_nis"].append(nis)
        if nis > self.opts.get("nis_baro", FW["nis_baro"]):
            self.meas_log["baro_rej"] += 1
            self._baro_recover(y)
            return {"fused": False, "nis": nis, "rejected": "nis"}
        K = self.P @ H.T / S[0, 0]
        I15 = np.eye(15)
        self.P = (I15 - K @ H) @ self.P @ (I15 - K @ H).T + K @ R @ K.T
        self.P = self.stabilize(self.P)
        dx = K.ravel() * y
        self.inject(dx)
        self._baro_rej_streak = 0
        self.meas_log["baro_fused"] += 1
        return {"fused": True, "nis": nis}

    def _baro_recover(self, y):
        """气压连续被拒且残差大时膨胀 P_DD，避免高度估计锁死在旧值。"""
        self._baro_rej_streak += 1
        if abs(y) > 5.0 and self._baro_rej_streak >= 10:
            self.P[2, 2] += (abs(y) * 0.5) ** 2
            self.P = self.stabilize(self.P)
            self._baro_rej_streak = 0

    # ---- P0-2 概念验证：机体系磁矢量更新 ----
    def measurement_update_mag_vector(self, mag_body, mag_ned, noise_ut=1.0):
        """3D 磁矢量更新；H=[m_b×]（右乘体轴误差）。仅仿真概念验证。"""
        Cbn = quat_to_dcm(self.quat).T
        pred = Cbn @ np.asarray(mag_ned, float)
        y = np.asarray(mag_body, float) - pred
        H = np.zeros((3, 15))
        # 与固件 notes/06 约定一致：δm_b = [m_b×] δβ^b
        H[:, 6:9] = skew(pred)
        R = (noise_ut ** 2) * np.eye(3)
        S = H @ self.P @ H.T + R
        nis = float(y @ np.linalg.solve(S, y))
        if nis > self.opts.get("nis_mag", 30.0):
            self.meas_log["yaw_rej"] += 1
            return {"fused": False, "nis": nis}
        K = self.P @ H.T @ np.linalg.solve(S, np.eye(3))
        K *= float(self.opts.get("mag_k_scale", 1.0))
        I15 = np.eye(15)
        self.P = (I15 - K @ H) @ self.P @ (I15 - K @ H).T + K @ R @ K.T
        self.P = self.stabilize(self.P)
        self.inject(K @ y)
        self.meas_log["yaw_fused"] += 1
        return {"fused": True, "nis": nis}

    # ---- H2 修复版标量航向（真航向轴投影，固件 2026-08-12 修复）----
    def measurement_update_yaw_h2(self, z_yaw, noise_rad):
        yaw = quat_to_euler(self.quat)[0]
        y = wrap_pi(z_yaw - yaw)
        if abs(y) > 1.0:
            self.meas_log["yaw_rej"] += 1
            return {"fused": False}
        # 真航向轴（NED 竖直向下）在机体系的投影 = C_b^n 第三行；
        # 状态索引必须是 6..8（roll/pitch/yaw 误差），不是 7..9。
        # 2026-10 修复：旧实现写 7:10，把航向新息注入加计零偏 x（见 notes/05）。
        # 理论对应：秦永元《惯性导航（第三版）》§9.8.2 姿态误差方程（p.320–322）；
        # 本实现按固件 ApplyYawUpdateDetailed() 的 H 口径 6..8 对齐。
        Cbn = quat_to_dcm(self.quat).T      # C^b_n：机体系 -> NED
        h_axis = Cbn[2, :]                  # NED z 轴在机体系分量
        H = np.zeros((1, 15))
        H[0, 6:9] = h_axis
        R = np.array([[noise_rad**2]])
        S = H @ self.P @ H.T + R
        nis = float(y * y / S[0, 0])
        self.meas_log["yaw_nis"].append(nis)
        # 重捕获式再锚定：NIS 拒绝后用膨胀 R 保守融合（防 P 坍缩后失锁）
        conservative = False
        if nis > self.opts.get("nis_yaw", FW["nis_yaw"]) and \
                self.opts.get("yaw_reacq"):
            R2 = np.array([[(noise_rad + 0.08)**2]])
            S2 = H @ self.P @ H.T + R2
            nis2 = float(y * y / S2[0, 0])
            if nis2 <= self.opts.get("yaw_reacq_nis2", 20.0):
                R, S, nis = R2, S2, nis2
                conservative = True
            else:
                self.meas_log["yaw_rej"] += 1
                return {"fused": False, "nis": nis}
        elif nis > self.opts.get("nis_yaw", FW["nis_yaw"]):
            self.meas_log["yaw_rej"] += 1
            return {"fused": False, "nis": nis}
        K = self.P @ H.T / S[0, 0]
        I15 = np.eye(15)
        self.P = (I15 - K @ H) @ self.P @ (I15 - K @ H).T + K @ R @ K.T
        self.P = self.stabilize(self.P)
        # 航向不确定度地板（无磁力计：航向仅弱可观，防止 P 坍缩失锁）
        floor = self.opts.get("yaw_p_floor", 0.0)
        if floor > 0.0 and self.P[8, 8] < floor**2:
            self.P[8, 8] = floor**2
        dx = K.ravel() * y
        self.inject(dx)
        self.meas_log["yaw_fused"] += 1
        return {"fused": True, "nis": nis, "conservative": conservative}


# ---------------------------------------------------------------------------
# 轨迹发生器：静基座 / 水平巡航转弯 / VTOL 过渡 / 悬停
# ---------------------------------------------------------------------------
class TrajGen:
    """按相位表生成真值轨迹与 IMU/量测。姿态由指令角速度运动学积分。"""

    def __init__(self, dt, rng):
        self.dt = dt
        self.rng = rng
        self.pos = np.zeros(3)      # NED
        self.vel = np.zeros(3)      # NED
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.quat = np.array([1.0, 0, 0, 0])
        self.t = 0.0
        # 真值零偏（可注入漂移/跳变）
        self.ba_true = np.zeros(3)
        self.bg_true = np.zeros(3)
        # 传感器真实噪声（与 EKF 假设 Q 分离：真实 IMU ≈0.05 m/s²，
        # 固件 EKF Q 假设 0.25 —— 两者差异正是 Q 标定的研究对象）
        self.a_noise = 0.05
        self.g_noise = 0.0015
        self.vib_amp = 0.0
        self.vib_hz = 80.0
        # 场景状态
        self.phases = []
        self.phase_idx = 0
        self.phase_t = 0.0
        self._phase_state = {}

    def _sync_quat(self):
        self.quat = quat_from_yp(self.yaw, self.pitch, self.roll)

    # ---- 相位定义 ----
    def add_static(self, dur):
        self.phases.append(("static", dur, {}))

    def add_cruise(self, dur, v, yaw_rate=0.0, accel_t=2.0):
        """水平巡航：速度矢量（NED），可选定常偏航率转弯"""
        self.phases.append(("cruise", dur, dict(v=np.array(v, float),
                                                yr=yaw_rate, accel_t=accel_t)))

    def add_transition(self, dur, pitch_deg0, pitch_deg1, a_ned, speed_out):
        """VTOL 过渡：俯仰 0→90°（或反向），NED 加速度，出段速度"""
        self.phases.append(("transition", dur,
                            dict(p0=pitch_deg0, p1=pitch_deg1,
                                 a=np.array(a_ned, float),
                                 v_out=np.array(speed_out, float))))

    def add_hover(self, dur, vib_amp=0.0, disturb=None):
        """悬停：机头朝天（俯仰90°），可选震动"""
        self.phases.append(("hover", dur, dict(vib=vib_amp, dist=disturb)))

    def add_turn(self, dur, yaw_rate):
        """原地偏航（速度恒定矢量旋转）"""
        self.phases.append(("turn", dur, dict(yr=yaw_rate)))

    # ---- 主步进：返回 (t, a_meas, w_meas, pos_true, vel_true, quat_true) ----
    def step(self, a_override=None):
        if self.phase_idx >= len(self.phases):
            return None
        kind, dur, prm = self.phases[self.phase_idx]
        t = self.t
        dt = self.dt
        if self.phase_t + dt > dur:
            self.phase_idx += 1
            self.phase_t = 0.0
            if self.phase_idx >= len(self.phases):
                return None
            kind, dur, prm = self.phases[self.phase_idx]
        tp = self.phase_t / max(dur, 1e-9)   # 相位进度 0-1

        yaw_rate = 0.0
        pitch_rate = 0.0
        a_ned = np.zeros(3)
        if kind == "static":
            self.vel = np.zeros(3)
        elif kind == "cruise":
            yr = prm["yr"]
            accel_t = prm["accel_t"]
            yaw_rate = yr
            v_base = prm["v"]
            v_dir = np.array([math.cos(self.yaw), math.sin(self.yaw), 0.0])
            speed = float(np.linalg.norm(v_base))
            v_target = v_dir * speed
            if accel_t > 0 and self.phase_t < accel_t:
                s = self.phase_t / accel_t
                s_prev = max(self.phase_t - dt, 0.0) / accel_t
                a_ned = v_target * (s - s_prev) / dt
            else:
                s = 1.0
            self.vel = v_target * s
            if yr != 0.0:
                # 协调转弯向心加速度
                a_ned = a_ned + yr * float(np.linalg.norm(self.vel)) * \
                    np.array([-math.sin(self.yaw), math.cos(self.yaw), 0.0])
        elif kind == "turn":
            yaw_rate = prm["yr"]
            cy, sy = math.cos(yaw_rate * dt), math.sin(yaw_rate * dt)
            v = self.vel.copy()
            self.vel = np.array([cy * v[0] - sy * v[1],
                                 sy * v[0] + cy * v[1], v[2]])
            a_ned = yaw_rate * np.array([-self.vel[1], self.vel[0], 0.0])
        elif kind == "transition":
            p0, p1 = math.radians(prm["p0"]), math.radians(prm["p1"])
            # 余弦平滑俯仰剖面 + 差分求俯仰角速度（真值陀螺输入）
            s_new = 0.5 - 0.5 * math.cos(math.pi * tp)
            s_prev = 0.5 - 0.5 * math.cos(
                math.pi * max(tp - dt / max(dur, 1e-9), 0.0))
            pitch_new = p0 + (p1 - p0) * s_new
            pitch_prev = p0 + (p1 - p0) * s_prev
            pitch_rate = (pitch_new - pitch_prev) / dt
            self.pitch = pitch_new
            a_ned = prm["a"]
            self.vel = self.vel + a_ned * dt
        elif kind == "hover":
            decel_t = prm.get("decel_t", 0.0)
            st = self._phase_state
            if decel_t > 0 and "v0" not in st:
                st["v0"] = self.vel.copy()
            self.pitch = math.radians(90.0)
            if decel_t > 0 and self.phase_t < decel_t:
                v0 = st["v0"]
                a_ned = -v0 / decel_t
                self.vel = v0 * (1.0 - self.phase_t / decel_t)
            else:
                self.vel = np.zeros(3)
                if prm.get("dist") is not None:
                    a_ned = np.array(prm["dist"], float)
        else:
            raise ValueError(kind)

        # 机体角速度（ZYX 运动学，roll=0）：w_b = [-ψ̇ sinθ, θ̇, ψ̇ cosθ]
        sp = math.sin(self.pitch)
        cp = math.cos(self.pitch)
        w_b = np.array([-yaw_rate * sp, pitch_rate, yaw_rate * cp])
        # 姿态状态推进
        self.yaw = wrap_pi(self.yaw + yaw_rate * dt)
        self.pitch = self.pitch + pitch_rate * dt
        self._sync_quat()
        # 位置推进
        self.pos = self.pos + self.vel * dt

        # 真值比力：f_b = C_n^b (a_ned - g_ned)
        Cnb = quat_to_dcm(self.quat)
        g_ned = np.array([0.0, 0.0, G_MPS2])
        f_b = Cnb.T @ (a_ned - g_ned)
        # 传感器
        vib = self.vib_amp * np.sin(2 * math.pi * self.vib_hz * t)
        a_meas = f_b + self.ba_true + self.rng.normal(0, self.a_noise, 3) \
            + vib * np.array([1.0, 1.0, 1.0])
        w_meas = w_b + self.bg_true + self.rng.normal(0, self.g_noise, 3)
        if a_override is not None:
            a_meas = a_override + self.ba_true + vib * np.ones(3) + \
                self.rng.normal(0, self.a_noise, 3)

        self.t += dt
        self.phase_t += dt
        return t, a_meas, w_meas, self.pos.copy(), self.vel.copy(), \
            self.quat.copy()


def quat_from_yp(yaw, pitch, roll):
    """ZYX(yaw,pitch,roll) → 四元数 (w,x,y,z)。

    @brief   轨迹发生器按欧拉角给定真值姿态时的变换。
    @param[in] yaw,pitch,roll 姿态角 [rad]
    @return  四元数 (w,x,y,z)
    @formula q = q_z(yaw)⊗q_y(pitch)⊗q_x(roll)（3-2-1 顺序）
    @ref     MOD-002 §3、§4.1（注意本项目 euler 提取对 theta 取负）
    @note    与 quat_to_euler 互逆；若混用符号会导致真值轨迹姿态反号，
             场景 1/2 的误差统计随之整体偏大。
    """
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    return np.array([
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        cy * sp * cr + sy * cp * sr,
        sy * cp * cr - cy * sp * sr])


def quat_angle_err(q_est, q_true):
    """估计/真值四元数之间的旋转角误差（无欧拉奇异），rad"""
    qe = quat_mult(q_est, quat_conj(q_true))
    return 2.0 * math.acos(max(-1.0, min(1.0, abs(qe[0]))))


def heading_of(q):
    """航向：取机体系 x/z 中 NE 投影更大的轴，atan2(E,N)。
    悬停（x 朝天）自动退化为 z 轴投影，水平退化为欧拉 yaw，全程无奇异。"""
    C = quat_to_dcm(q)
    px = math.hypot(C[0, 0], C[1, 0])
    pz = math.hypot(C[0, 2], C[1, 2])
    if pz > px:
        return math.atan2(C[1, 2], C[0, 2])
    return math.atan2(C[1, 0], C[0, 0])


# ---------------------------------------------------------------------------
# 场景运行器
# ---------------------------------------------------------------------------
class Scenario:
    """运行一个场景，返回时间序列 + 汇总指标。

    spec: dict(
      name, dur, phases=[...],           # TrajGen 相位
      ba=[..], bg=[..],                  # 真值零偏
      gnss=dict(noise=(2,3,0.15,0.25), hz=10, age=0.015,
                outage=(t0,t1) 或 None, multipath=None, outliers=[]),
      static_aid=dict(enable=True, tier=True, zupt_adapt=False),
      baro=dict(enable=False, hz=25, noise=0.35),
      yaw_aid=dict(enable=False, hz=10, noise=None, h2=True),
      ahrs_fallback=dict(enable=False),   # 无 GNSS 运动时重力辅助
      vib_amp=0.0,
    )
    ekf_factory: callable() -> OptEkf
    """

    def __init__(self, spec, ekf_factory):
        self.spec = spec
        self.make_ekf = ekf_factory
        self.dt = 0.005
        self.HZ = 200

    def run(self, seed=1, verbose=False):
        rng = np.random.default_rng(seed)
        spec = self.spec
        self.prm = dict(FW)
        self.prm.update(spec.get("fw_overrides", {}))
        prm = self.prm
        ekf = self.make_ekf()
        ekf.home_lla = HOME_LLA.copy()
        # 初始化：静止水平
        ekf.initialize(np.array([0.0, 0.0, -G_MPS2]), np.zeros(3), HOME_LLA)
        # 悬停构型初始化覆盖（机头朝天，最小旋转法无法识别悬停）
        if spec.get("init_hover"):
            qh = quat_from_yp(0.0, math.radians(90.0), 0.0)
            ekf.quat = qh.copy()
            ekf.Cnb = quat_to_dcm(qh)
            ekf.bg = np.zeros(3) - ekf.Cnb.T @ np.zeros(3)
        # 初始航向不确定度覆盖（研究参数）
        if spec.get("p0_hdg") is not None:
            ekf.P[8, 8] = spec["p0_hdg"]**2
        trj = TrajGen(self.dt, rng)
        for ph in spec["phases"]:
            kind = ph[0]
            if kind == "static":
                trj.add_static(ph[1])
            elif kind == "cruise":
                trj.add_cruise(ph[1], ph[2]["v"],
                               yaw_rate=ph[2].get("yaw_rate", 0),
                               accel_t=ph[2].get("accel_t", 2.0))
            elif kind == "transition":
                d = ph[2]
                trj.add_transition(ph[1], d["p0"], d["p1"], d["a"], d["v_out"])
            elif kind == "hover":
                trj.add_hover(ph[1], vib_amp=spec.get("vib_amp", 0.0),
                              disturb=ph[2].get("disturb"))
            elif kind == "turn":
                trj.add_turn(ph[1], ph[2].get("yaw_rate", 0.3))
        trj.ba_true = np.array(spec.get("ba", [0.0, 0.0, 0.0]), float)
        trj.bg_true = np.array(spec.get("bg", [0.0, 0.0, 0.0]), float)
        if spec.get("vib_amp"):
            trj.vib_amp = spec["vib_amp"]

        det = StaticDetector(self.prm)
        aid = StaticAid(self.prm)
        gnss = spec.get("gnss", {})
        gnss_noise = gnss.get("noise", (2.0, 3.0, 0.15, 0.25))
        gnss_r = gnss.get("r_floor", gnss_noise)
        gnss_hz = gnss.get("hz", 10)
        gnss_age = gnss.get("age", FW["gnss_age"])
        outage = gnss.get("outage")
        multipath = gnss.get("multipath")   # (t0,t1,pos_scale)
        outliers = gnss.get("outliers", [])  # [(t0, dur, n_m, m_e, m_d)]
        baro = spec.get("baro", dict(enable=True, hz=25, noise=0.35))
        yaw_aid = spec.get("yaw_aid", {})
        ahrs_fb = spec.get("ahrs_fallback", {})
        aid_on = spec.get("static_aid", {}).get("enable", True)
        use_tier = spec.get("static_aid", {}).get("tier", True)
        zupt_adapt = spec.get("static_aid", {}).get("zupt_adapt", False)

        n = int(spec["dur"] / self.dt)
        rec = dict(t=[], pos_err=[], vel_err=[], att_err=[], yaw_err=[],
                   p_pos=[], p_vel=[], p_att=[], ba=[], bg=[], static=[],
                   conf=[], q_est=[], q_true=[], pitch_true=[], pitch_est=[])
        gyro_avg = np.zeros(3)
        gyro_cnt = 0
        for k in range(n):
            t = k * self.dt
            # 零偏跳变注入（故障场景）
            for bj in spec.get("bias_jumps", []):
                if t >= bj["t"] and not bj.get("applied"):
                    trj.ba_true = trj.ba_true + np.array(bj.get("ba", [0, 0, 0]), float)
                    trj.bg_true = trj.bg_true + np.array(bj.get("bg", [0, 0, 0]), float)
                    bj["applied"] = True
            out = trj.step()
            if out is None:
                break
            t, a_meas, w_meas, pos_t, vel_t, quat_t = out
            ekf.time_update(w_meas * self.dt, np.zeros(3),
                            a_meas * self.dt, np.zeros(3), self.dt)
            # 静止辅助
            static_now = det.update(a_meas, w_meas)
            if aid_on and static_now:
                gyro_avg += w_meas
                gyro_cnt += 1
                tier = aid.tier(det)
                g_n, v_n, v_d = aid.noise(tier) if use_tier else (
                    prm["tier2_grav"], prm["tier2_vel_ne"], prm["tier2_vel_d"])
                dz, dg, ds = aid.divs(tier) if use_tier else (4, 8, 8)
                aid.frame_cnt += 1
                c = aid.frame_cnt
                if c % dz == 0:
                    if zupt_adapt:
                        ekf.measurement_update_velocity_adaptive(
                            np.zeros(3), v_n, v_d)
                    else:
                        ekf.measurement_update_velocity(np.zeros(3), v_n, v_d)
                if c % dg == 1:
                    ekf.measurement_update_gravity(a_meas, g_n)
                if c % ds == 3:
                    # 静止陀螺：250ms 均值抑制单帧白噪声（固件语义）
                    ekf.measurement_update_static_gyro(
                        gyro_avg / max(gyro_cnt, 1), np.full(3, prm["sg_noise"]))
                    gyro_avg = np.zeros(3)
                    gyro_cnt = 0
            # GNSS
            in_outage = outage is not None and outage[0] <= t < outage[1]
            mp = multipath is not None and multipath[0] <= t < multipath[1]
            if not in_outage and k % (self.HZ // gnss_hz) == 0:
                noise = list(gnss_noise)
                if mp:
                    noise[0] *= multipath[2]
                z_pos = pos_t + rng.normal(0, noise[0], 3) * np.array(
                    [1, 1, noise[1] / max(noise[0], 1e-9)])
                z_vel = vel_t + rng.normal(0, noise[2], 3)
                # GNSS 速度门控静止退出（修复候选）：
                # 速度超阈 → 强制退出静止（仅退出不进入，非对称）
                if gnss.get("static_gate") and \
                        float(np.hypot(z_vel[0], z_vel[1])) > \
                        gnss.get("static_gate_thr", 1.5):
                    det.force_exit()
                    static_now = False
                if static_now:
                    # ★ 固件语义（navigation_task.cpp:1148-1153）：
                    #   静止时 GNSS 速度量测强制=0，防止速度噪声经
                    #   F(v,β) 零偏残差耦合随机注入航向（弱可观噪声放大）
                    z_vel = np.zeros(3)
                for (ot0, odur, om_n, om_e, om_d) in outliers:
                    if ot0 <= t < ot0 + odur:
                        z_pos += np.array([om_n, om_e, om_d])
                hist = ekf.hist if not self.spec.get("no_replay") else []
                if gnss.get("split"):
                    res = ekf.measurement_update_gnss_split(
                        z_pos, z_vel, gnss_r, age_s=gnss_age, hist=hist)
                    if res.get("fused"):
                        ekf.meas_log["gnss_fused"] += 1
                        if res.get("conservative"):
                            ekf.meas_log["gnss_conservative"] += 1
                        ekf.meas_log["gnss_nis"].append(max(
                            res.get("nis_pos", 0), res.get("nis_vel", 0)))
                    else:
                        ekf.meas_log["gnss_rej"] += 1
                else:
                    res = ekf.measurement_update_gnss(z_pos, z_vel, gnss_r,
                                                      age_s=gnss_age, hist=hist)
                    if res.get("fused"):
                        ekf.meas_log["gnss_fused"] += 1
                        if res.get("conservative"):
                            ekf.meas_log["gnss_conservative"] += 1
                        ekf.meas_log["gnss_nis"].append(res["nis"])
                    else:
                        ekf.meas_log["gnss_rej"] += 1
            # 气压
            if baro.get("enable") and k % (self.HZ // baro.get("hz", 25)) == 0:
                z_alt = -pos_t[2] + rng.normal(0, baro.get("noise", prm["baro_noise"]))
                ekf.measurement_update_baro(z_alt)
            # 双矢量航向
            if yaw_aid.get("enable") and k % (self.HZ // yaw_aid.get("hz", 10)) == 0 \
                    and not static_now:
                speed = float(np.hypot(vel_t[0], vel_t[1]))
                turn = float(np.linalg.norm(w_meas))
                if speed > prm["dv_min_speed"] and turn < prm["dv_max_turn"]:
                    track = math.atan2(vel_t[1], vel_t[0])
                    slip = rng.normal(0, 0.03)
                    est_yaw = wrap_pi(track - slip)
                    noise = max(prm["dv_min_noise"],
                                prm["dv_base_noise"] / (speed - 1.0 + 1.0)) * \
                        yaw_aid.get("noise_scale", 1.0)
                    if yaw_aid.get("h2", True):
                        ekf.measurement_update_yaw_h2(est_yaw, noise)
                    else:
                        ekf.measurement_update_yaw(est_yaw, noise)
            # 无 GNSS 运动时 AHRS 回退（重力方向辅助）
            if ahrs_fb.get("enable") and in_outage:
                if abs(float(np.linalg.norm(a_meas)) - G_MPS2) < 2.0 and \
                        float(np.linalg.norm(w_meas)) < ahrs_fb.get("gyro_thr", 0.5):
                    if k % 4 == 0:
                        ekf.measurement_update_gravity(a_meas, ahrs_fb.get("noise", 0.08))
            # 记录
            est_pos = ekf.ned_from_lla(ekf.lla, HOME_LLA)
            att_err = quat_angle_err(ekf.quat, quat_t)
            yaw_err = wrap_pi(heading_of(ekf.quat) - heading_of(quat_t))
            rec["t"].append(t)
            rec["pos_err"].append(est_pos - pos_t)
            rec["vel_err"].append(ekf.vel - vel_t)
            rec["att_err"].append(att_err)
            rec["yaw_err"].append(yaw_err)
            rec["p_pos"].append(np.sqrt(np.diag(ekf.P)[0:3]))
            rec["p_vel"].append(np.sqrt(np.diag(ekf.P)[3:6]))
            rec["p_att"].append(np.sqrt(np.diag(ekf.P)[6:9]))
            rec["ba"].append(ekf.ba - trj.ba_true)
            rec["bg"].append(ekf.bg - trj.bg_true)
            rec["static"].append(1 if static_now else 0)
            rec["conf"].append(det.confidence)
            rec["q_est"].append(ekf.quat.copy())
            rec["q_true"].append(quat_t.copy())
            rec["pitch_true"].append(trj.pitch)
            rec["pitch_est"].append(quat_to_euler(ekf.quat)[1])
        self.rec = {kk: np.array(vv) for kk, vv in rec.items()}
        self.ekf = ekf
        self.trj = trj
        return self

    def metrics(self, t0=None, t1=None):
        r = self.rec
        m = np.ones(len(r["t"]), bool)
        if t0 is not None:
            m &= r["t"] >= t0
        if t1 is not None:
            m &= r["t"] <= t1
        if m.sum() < 2:
            return {}
        pe = np.asarray(r["pos_err"])[m]
        ve = np.asarray(r["vel_err"])[m]
        ae = np.asarray(r["att_err"])[m]
        ye = np.asarray(r["yaw_err"])[m]
        out = {
            "pos_ne_rmse": float(np.sqrt(np.mean(pe[:, 0]**2 + pe[:, 1]**2))),
            "pos_d_rmse": float(np.sqrt(np.mean(pe[:, 2]**2))),
            "vel_ne_rmse": float(np.sqrt(np.mean(ve[:, 0]**2 + ve[:, 1]**2))),
            "vel_d_rmse": float(np.sqrt(np.mean(ve[:, 2]**2))),
            "att_rmse_deg": float(np.sqrt(np.mean(ae**2)) * R2D),
            "yaw_rmse_deg": float(np.sqrt(np.mean(ye**2)) * R2D),
            "pos_ne_max": float(np.max(np.hypot(pe[:, 0], pe[:, 1]))),
            "pos_d_max": float(np.max(np.abs(pe[:, 2]))),
        }
        # 3σ 覆盖
        pp = np.asarray(r["p_pos"])[m]
        pv = np.asarray(r["p_vel"])[m]
        out["pos_3sig_cov"] = float(np.mean(
            (np.abs(pe) <= 3 * pp).all(axis=1)))
        out["vel_3sig_cov"] = float(np.mean(
            (np.abs(ve) <= 3 * pv).all(axis=1)))
        # 零偏收敛误差
        out["ba_err"] = float(np.linalg.norm(np.asarray(r["ba"])[-1]))
        out["bg_err_radps"] = float(np.linalg.norm(np.asarray(r["bg"])[-1]))
        # NIS 统计
        ml = self.ekf.meas_log
        for key in ("gnss", "zupt", "grav", "sg", "yaw", "baro"):
            lst = ml[key + "_nis"]
            if lst:
                out[key + "_nis_mean"] = float(np.mean(lst))
        out["gnss_accept"] = (ml["gnss_fused"] / max(
            1, ml["gnss_fused"] + ml["gnss_rej"]))
        out["gnss_conservative"] = ml["gnss_conservative"]
        out.update({k: ml[k] for k in ("zupt_fused", "zupt_rej", "grav_fused",
                                       "grav_rej", "sg_fused", "sg_rej",
                                       "baro_fused", "baro_rej", "yaw_fused",
                                       "yaw_rej")})
        return out
