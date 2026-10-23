# -*- coding: utf-8 -*-
"""upgrade_candidates.py — 导航升级候选 A/B/C 仿真验证 + ZUPT 误静止风险场景。

@file    upgrade_candidates.py
@module  analysis/EKF 优化研究（候选验证）
@brief   在基线之上"只做加法"地实现候选 A/B/C，并提供 selftest 与套件运行器。
@details 约束：不修改任何既有基线文件（opt_ekf.py / ekf_core.py / run_baseline.py /
         run_variants.py），本文件只新增子类与场景：
           * UpgradeEkf(OptEkf)      —— 候选 A（磁航向：门控+自适应 R+鲁棒更新）、
                                        候选 B（GNSS 子集故障隔离 + 速度重锚）、
                                        候选 C（振动自适应 Q）
           * UpgradeScenario(Scenario) —— 复刻基线 run() 的逐行语义，额外注入磁量测/
                                        高频 Q 调度/逐历元日志；--selftest 要求与基线
                                        指标逐项差 <1e-9（空升级不引入行为漂移）
           * 场景电池：基线 8 场景 + SC9_zupt_false_static / SC9b_zupt_lowspeed（P0 风险）
                       + SC7_hi_vib（强化震动）
           * 指标：均值±标准差（≥5 种子）、分场景统计窗口、逐历元隔离/踢动/失锁统计
         用法：--selftest / --suite P0|A|B|C|all --seeds N --workers M
         输出：TandemVec-EKF-Report/sim-data/opt/upgrade_candidates.json
@theory  双矢量/磁航向量测与倾斜补偿 @ref docs/08-参考资料/notes/04-航向与磁辅助.md；
         GNSS 子集 FDI（位置 χ²(3)=25.90 / 速度 χ²(3)=16.27）与速度软重锚
         @ref REF-121 §7.3.1 p.255–259、@ref docs/08-参考资料/notes/03-组合导航与鲁棒性.md §3.1–§3.2；
         ZUPT 外部参照门控 @ref docs/08-参考资料/notes/03-组合导航与鲁棒性.md §3.3/§4.4；
         振动自适应 Q @ref REF-121 §3.10 p.129–139。
@frame  NED + FRD；误差状态索引 6–8 为失准角、8 为航向。
@units   SI；角度 rad（结论 deg）、位置 m、速度 m/s、NIS 与 σ 无量纲。
@note    ★ 口径：候选基线统一用 CUR_SPEC={"gnss":{"static_gate":True},"p0_hdg":0.2} 与
         CUR_CFG={"nis_zupt":16.3}（见 @ref docs/08-参考资料/notes/05-仿真验证.md §1.1）；
         双矢量航向用 yaw_h2_fix=True（H=6..8 固件口径），legacy 7:10 只作连续性参照。
         单次套件 ≈290 次运行；每格 5 种子不足以定论发散概率（需 ≥20 种子 + 实机数据）。
@see     opt_ekf.py、run_baseline.py、p15_consider_probe.py、upgrade_diag.py、
         @ref TandemVec-EKF-Report/导航算法升级评估-专业文献调研-2026-10.md
"""
import argparse
import copy
import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, ".."))
sys.path.insert(0, _HERE)
from ekf_core import (HOME_LLA, R2D, G_MPS2, quat_conj, quat_mult,
                      quat_to_dcm, quat_to_euler, wrap_pi, D2R)
from opt_ekf import (FW, OptEkf, Scenario, StaticAid, StaticDetector, TrajGen,
                     build_cfg, heading_of, quat_from_yp)
from run_baseline import AID_DEF, GNSS_DEF, base_scenarios

OUT_DIR = os.path.abspath(os.path.join(_HERE, "..", "..", "sim-data", "opt"))
OUT_JSON = os.path.join(OUT_DIR, "upgrade_candidates.json")

# ===========================================================================
# 0. 固件现值基线定义（2026-08 三项修复之后：F1 静态门控 / F2 P0=0.2 /
#    P1 ZUPT NIS 16.3）。所有候选都在这个基线上做 A/B，而不是在报告 §二 的
#    修复前固件口径上做（那会把 F1/F2 的收益重复计入候选）。
# ===========================================================================
CUR_SPEC = {"gnss": {"static_gate": True}, "p0_hdg": 0.2}
CUR_CFG = {"nis_zupt": 16.3}
CUR_OPTS = {}

# 磁模型（μT，NED）：水平 27 / 垂直向下 39 / 磁偏角 -2°（华南量级）
MAG_BH, MAG_BD, MAG_DEC = 27.0, 39.0, math.radians(-2.0)
MAG_B_TOTAL = math.hypot(MAG_BH, MAG_BD)          # 47.43 μT
MAG_DIP = math.atan2(MAG_BD, MAG_BH)              # 55.3°
MAG_B_NED = np.array([MAG_BH * math.cos(MAG_DEC),
                      MAG_BH * math.sin(MAG_DEC), MAG_BD])


# ===========================================================================
# 1. 工具
# ===========================================================================
def deep_merge(dst, src):
    """递归合并两个 dict（src 覆盖 dst，嵌套 dict 逐层合并），返回新对象不改入参。"""
    out = copy.deepcopy(dst)
    for k, v in (src or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def scenario_spec(name):
    """baseline 8 场景 + 本文件新增场景（SC9*/SC7_hi）"""
    base = base_scenarios()
    if name in base:
        return copy.deepcopy(base[name])
    if name in EXTRA_SCENARIOS:
        return copy.deepcopy(EXTRA_SCENARIOS[name])
    raise KeyError(name)


# --- Lead 要求的 P0 风险场景：GNSS 失联 + 平滑巡航 → 静止检测误确认 ---
#     2s 静止 → 5 m/s 匀速 20s（GNSS 正常，F1 门控每历元强制退出静止）
#     → 失联 40s（保持匀速；期间没有 GNSS epoch，F1 门控没有出口）
#     → 10s GNSS 恢复
_CRUISE_5 = [("static", 2.0, {}),
             ("cruise", 20.0, dict(v=[5.0, 0.0, 0.0], accel_t=1.0)),
             ("cruise", 40.0, dict(v=[5.0, 0.0, 0.0], accel_t=0.0)),
             ("cruise", 10.0, dict(v=[5.0, 0.0, 0.0], accel_t=0.0))]
_CRUISE_1 = [("static", 2.0, {}),
             ("cruise", 20.0, dict(v=[1.0, 0.0, 0.0], accel_t=1.0)),
             ("cruise", 40.0, dict(v=[1.0, 0.0, 0.0], accel_t=0.0)),
             ("cruise", 10.0, dict(v=[1.0, 0.0, 0.0], accel_t=0.0))]

EXTRA_SCENARIOS = {
    "SC9_zupt_false_static": dict(
        dur=72.0, phases=_CRUISE_5,
        ba=[0.08, -0.05, 0.12], bg=[0.003, -0.002, 0.004],
        gnss=dict(**GNSS_DEF, outage=(22.0, 62.0)),
        static_aid=dict(AID_DEF),
    ),
    "SC9b_zupt_lowspeed": dict(
        dur=72.0, phases=_CRUISE_1,
        ba=[0.08, -0.05, 0.12], bg=[0.003, -0.002, 0.004],
        gnss=dict(**GNSS_DEF, outage=(22.0, 62.0)),
        static_aid=dict(AID_DEF),
    ),
    "SC7_hi_vib": dict(
        dur=60.0,
        phases=[("hover", 60.0, dict(disturb=None))],
        ba=[0.08, -0.05, 0.12], bg=[0.003, -0.002, 0.004],
        gnss=GNSS_DEF, vib_amp=0.15, yaw_aid=dict(enable=True),
        static_aid=dict(AID_DEF), init_hover=True,
    ),
}


def mag_azimuth_error(m_b, quat_est, dec):
    """磁力计 → NED 航向误差观测 y（rad）。

    用估计姿态把机体系磁量测旋到 NED，取其水平方位角与磁偏角之差：
        m_n = C^n_b(est) · m_b
        az  = atan2(m_n[E], m_n[N])
        y   = wrap(dec − az)
    符号自检见 `--selftest`：若估计航向 = 真值 + δ，则 y = −δ。
    """
    m_n = quat_to_dcm(quat_est) @ np.asarray(m_b, float)
    az = math.atan2(m_n[1], m_n[0])
    return wrap_pi(dec - az), az, m_n


class MagAid:
    """磁力计模拟：真值场 + 机体系硬磁/软磁 + NED 慢变干扰 + 白噪声。

    mode='ideal'     : 真值场 + 白噪声（0.3 μT，≈0.6°航向量级）
    mode='disturbed' : 硬磁 [4.5,-3.0,3.5] μT + 软磁 diag(1.06,0.95,1.04)
                       + 慢变干扰 2.5 μT（周期 70-110s）+ 白噪声 0.6 μT
    门控/自适应 R 用的**标称模型**始终是无干扰的 MAG_B_NED（真实系统也不知道
    自己当天有没有被磁干扰）。
    """

    def __init__(self, cfg):
        self.cfg = dict(cfg or {})
        self.mode = self.cfg.get("mode", "ideal")
        self.disturbed = self.mode == "disturbed"
        self.sigma_meas = float(self.cfg.get(
            "noise_ut", 0.6 if self.disturbed else 0.3))
        self.hard_iron = np.array([4.5, -3.0, 3.5])
        self.soft = np.diag([1.06, 0.95, 1.04])
        self.dist_amp = float(self.cfg.get("dist_ut", 2.5))
        self._dist_phase = np.array([0.7, 2.1, 0.0])

    def field_ned(self, t):
        if not self.disturbed:
            return MAG_B_NED.copy()
        d = self.dist_amp * np.array([
            math.sin(2 * math.pi * t / 90.0 + self._dist_phase[0]),
            math.sin(2 * math.pi * t / 70.0 + self._dist_phase[1]),
            2.0 * math.sin(2 * math.pi * t / 110.0)])
        return MAG_B_NED + d

    def measure(self, quat_true, t, rng):
        m_b = quat_to_dcm(quat_true).T @ self.field_ned(t)
        if self.disturbed:
            m_b = self.soft @ m_b + self.hard_iron
        return m_b + rng.normal(0, self.sigma_meas, 3)


# ===========================================================================
# 2. 候选 EKF
# ===========================================================================
class UpgradeEkf(OptEkf):
    """候选 A/B/C 的 EKF 子类：磁航向（门控+自适应 σ+鲁棒更新）、GNSS 子集 FDI+速度重锚、振动自适应 Q。

    所有候选开关默认关闭；`A_DEF/B_DEF/C_DEF` 为各候选的默认参数表，
    由 `VARIANTS[suite]` 组装成 opts 传入。开关全关时与 `OptEkf` 行为一致。
    """

    # --- 候选 A 参数 ---
    A_DEF = dict(
        sigma_base_deg=1.0,        # 干净磁量测的基础 1σ
        k_norm=0.01,               # rad per μT（|B| 偏差 → 航向 σ）
        k_dip=1.5,                 # rad per rad（倾角偏差 → 航向 σ）
        k_track=1.0,               # rad per rad（与 GNSS 航迹不一致 → 航向 σ）
        k_att=1.5,                 # 姿态不确定度 → 航向 σ（dip 耦合系数 ~1.4）
        gate_norm_ut=5.0,          # |B| 偏差硬门
        gate_dip_deg=6.0,          # 倾角偏差硬门
        gate_track_deg=20.0,       # 与 GNSS 航迹一致性硬门
        gate_res_deg=45.0,         # 与滤波残差硬门（防野值）
        gate_tilt_deg=8.0,         # 倾斜补偿不确定度硬门（过渡/悬停期磁航向不可用）
        sigma_max_deg=25.0,
        nis_max=12.0,
    )
    # --- 候选 B 参数 ---
    B_DEF = dict(nis_sub=22.0, reanchor_after=10, reanchor_kind="reset",
                 reanchor_std=2.0)

    def __init__(self, cfgd, opts=None):
        super().__init__(cfgd, opts)
        o = opts or {}
        self.consider_heading = bool(o.get("consider_heading", False))
        self._consider_active = False
        self.mag_dec = MAG_DEC
        self.mag_par = dict(self.A_DEF)
        self.mag_par.update(o.get("mag", {}) or {})
        self.b_par = dict(self.B_DEF)
        self.b_par.update(o.get("fdi", {}) or {})
        # 候选 A 状态/日志
        self.mag_fused = 0
        self.mag_rej_norm = 0
        self.mag_rej_dip = 0
        self.mag_rej_track = 0
        self.mag_rej_res = 0
        self.mag_rej_nis = 0
        self.mag_nis = []
        self.kicks = []        # (t, src, kick_rad)
        # 候选 B 状态/日志
        self.vel_rej_streak = 0
        self.reanchors = 0
        self.reanchor_log = []
        self.iso_pos_fused = 0
        self.iso_pos_rej = 0
        self.iso_vel_fused = 0
        self.iso_vel_rej = 0
        self.fdi_events = 0
        self.epoch_log = []
        self.mag_log = []
        self._t = 0.0

    # ---------------- P1-5：航向 consider（近似） ----------------
    # 注意：这是“把 GNSS 更新注入的航向分量清零”的近似实现；协方差仍按
    # 标准 KF 更新，不是严格 Schmidt。用于仿真趋势判断，不作为固件实现。
    def measurement_update_gnss(self, *args, **kwargs):
        old = self._consider_active
        self._consider_active = self.consider_heading
        try:
            return super().measurement_update_gnss(*args, **kwargs)
        finally:
            self._consider_active = old

    def measurement_update_gnss_split(self, *args, **kwargs):
        old = self._consider_active
        self._consider_active = self.consider_heading
        try:
            return super().measurement_update_gnss_split(*args, **kwargs)
        finally:
            self._consider_active = old

    def inject(self, dx):
        if getattr(self, "_consider_active", False):
            dx = np.array(dx, copy=True)
            dx[8] = 0.0
        super().inject(dx)

    # ---------------- 双矢量航向（H2）索引修正版 ----------------
    def yaw_update_h2_fixed(self, z_yaw, noise_rad):
        """对照固件 ApplyYawUpdateDetailed（ekf_15_state.h:3162-3226）。

        ★ baseline `opt_ekf.py:measurement_update_yaw_h2()` 写的是
          `H[0, 7:10] = h_axis`，而固件是 `h_yaw(0, 6..8)`——整体右移一格。
          水平姿态下 h_axis=[0,0,1] 实际落在 **加计零偏 x（索引 9）** 上：
          航向新息不修正航向，反而把加计零偏 x 推走（单点注入实证见 --selftest）。
          本方法按固件口径用 6:9，作为候选 A/B 的对照基线。
        """
        h_before = heading_of(self.quat)
        yaw_est = quat_to_euler(self.quat)[0]
        y = wrap_pi(z_yaw - yaw_est)
        if abs(y) > 1.0:
            self.meas_log["yaw_rej"] += 1
            return {"fused": False, "nis": float("nan"), "rejected": "residual"}
        H = np.zeros((1, 15))
        H[0, 6:9] = quat_to_dcm(self.quat).T[2, :]   # 固件口径 6..8
        R = np.array([[noise_rad ** 2]])
        S = H @ self.P @ H.T + R
        nis = float(y * y / S[0, 0])
        if nis > self.nis_yaw:
            self.meas_log["yaw_rej"] += 1
            return {"fused": False, "nis": nis, "rejected": "nis"}
        K = self.P @ H.T / S[0, 0]
        I15 = np.eye(15)
        self.P = (I15 - K @ H) @ self.P @ (I15 - K @ H).T + K @ R @ K.T
        self.P = self.stabilize(self.P)
        self.inject(K.ravel() * y)
        self.meas_log["yaw_fused"] += 1
        self.meas_log["yaw_nis"].append(nis)
        self.kicks.append((self._t, "yaw", abs(
            wrap_pi(heading_of(self.quat) - h_before))))
        return {"fused": True, "nis": nis}

    # ---------------- 候选 A：磁航向更新 ----------------
    def mag_az_jac(self, m_n):
        """磁方位角观测的**精确**机体系雅可比 ∂az/∂ε（ε: q_est⊗exp(ε)）。

        az = atan2(m_n[1], m_n[0])，m_n = C^n_b(q) m_b；
        ∂az/∂ε_i = [−m_nE, m_nN, 0]/(m_nN²+m_nE²) · (c_i × m_n)，c_i = C^n_b 第 i 列。
        与数值微分逐位一致（见 --selftest 5）。固件式的"真航向轴投影"(C_b^n 第 2 行)
        只在水平姿态成立：大俯仰下与数值雅可比差一个转置/符号（见自检 5）。
        """
        Bh2 = max(m_n[0] ** 2 + m_n[1] ** 2, 1e-12)
        g = np.array([-m_n[1], m_n[0], 0.0]) / Bh2
        C = quat_to_dcm(self.quat)
        return np.array([g @ np.cross(C[:, i], m_n) for i in range(3)])

    def mag_update(self, m_b, sigma_rad, robust=False, extra=None):
        p = self.mag_par
        h_before = heading_of(self.quat)
        y, az, m_n = mag_azimuth_error(m_b, self.quat, self.mag_dec)
        H = np.zeros((1, 15))
        H[0, 6:9] = self.mag_az_jac(m_n)
        sig = float(np.clip(sigma_rad, math.radians(0.3),
                            math.radians(p["sigma_max_deg"])))
        R = np.array([[sig ** 2]])
        S = H @ self.P @ H.T + R
        nis = float(y * y / S[0, 0])
        if robust:
            # Huber 型有界影响：等效 σ 不下于 |y|/2.5，避免 P 坍缩后硬拒锁死
            sig_eff = min(max(sig, abs(y) / 2.5),
                          math.radians(p["sigma_max_deg"]))
            R = np.array([[sig_eff ** 2]])
            S = H @ self.P @ H.T + R
            nis = float(y * y / S[0, 0])
            fused = True
        else:
            fused = nis <= p["nis_max"]
        if not fused:
            self.mag_rej_nis += 1
            if extra is not None:
                extra["mag_fused"] = False
                extra["mag_nis"] = nis
            return {"fused": False, "nis": nis}
        K = self.P @ H.T / S[0, 0]
        I15 = np.eye(15)
        self.P = (I15 - K @ H) @ self.P @ (I15 - K @ H).T + K @ R @ K.T
        self.P = self.stabilize(self.P)
        self.inject(K.ravel() * y)
        self.mag_fused += 1
        self.mag_nis.append(nis)
        kick = abs(wrap_pi(heading_of(self.quat) - h_before))
        self.kicks.append((self._t, "mag", kick))
        if extra is not None:
            extra["mag_fused"] = True
            extra["mag_nis"] = nis
            extra["mag_kick"] = kick
        return {"fused": True, "nis": nis, "y": y, "sigma": sig}

    # ---------------- 候选 B：GNSS 子集隔离 + 速度重锚 ----------------
    @staticmethod
    def _subset_update(ekf, H, R3, kind, z):
        """用**当前**状态重算新息后做一次 3 维 Joseph 更新。"""
        if kind == "p":
            y = np.asarray(z, float) - ekf.ned_from_lla(ekf.lla, ekf.home_lla)
        else:
            y = np.asarray(z, float) - ekf.vel
        S = H @ ekf.P @ H.T + R3
        K = ekf.P @ H.T @ np.linalg.solve(S, np.eye(3))
        I15 = np.eye(15)
        ekf.P = (I15 - K @ H) @ ekf.P @ (I15 - K @ H).T + K @ R3 @ K.T
        ekf.P = ekf.stabilize(ekf.P)
        ekf.inject(K @ y)
        return float(y @ np.linalg.solve(S, y))

    def reanchor_velocity(self, z_vel):
        """对照固件 ResetVelocityToExternal（ekf_15_state.h:1326-1338）：
        速度态重置到外部测速 + 速度协方差块抬升 + 清空回放历史。"""
        kind = self.b_par["reanchor_kind"]
        if kind == "reset":
            self.vel = np.asarray(z_vel, float).copy()
            self.P[3:6, 3:6] = (self.b_par["reanchor_std"] ** 2) * np.eye(3)
            self.hist = []
        else:  # 'inflate'：只膨胀 P，不动速度态
            self.P[3:6, 3:6] += (self.b_par["reanchor_std"] ** 2) * np.eye(3)
        self.P = self.stabilize(self.P)
        self.vel_rej_streak = 0
        self.reanchors += 1
        self.reanchor_log.append((self._t, kind,
                                  float(np.linalg.norm(z_vel))))

    def gnss_update(self, mode, z_pos, z_vel, R_std, age_s=0.0, hist=None):
        """mode='cur'  : 固件现值 6 维联合（含 NIS 门控/重捕获/延迟回放）
        mode='fdi'   : 先试 6 维；仅当 6 维被拒才做位置/速度 3 维子集检验，
                       只更新通过的子集；速度连续 N 次被拒 → 软重锚。
        """
        h_before = heading_of(self.quat)
        res6 = self.measurement_update_gnss(z_pos, z_vel, R_std,
                                           age_s=age_s, hist=hist)
        out = dict(res6)
        out.update(mode=mode, nis6=float(res6.get("nis", float("nan"))),
                   fused_p=bool(res6.get("fused")),
                   fused_v=bool(res6.get("fused")), isolated=False,
                   reanchored=False, nis_p=float("nan"),
                   nis_v=float("nan"))
        if res6.get("fused"):
            kick = abs(wrap_pi(heading_of(self.quat) - h_before))
            self.kicks.append((self._t, "gnss", kick))
            self.vel_rej_streak = 0
            return out
        if mode == "cur":
            self.vel_rej_streak += 1
            return out
        # ---- 子集检验（P 为 6 维拒绝后的当前值，与固件"整体拒绝"时刻一致）----
        self.fdi_events += 1
        P = self.P
        y_p = np.asarray(z_pos, float) - self.ned_from_lla(self.lla, self.home_lla)
        y_v = np.asarray(z_vel, float) - self.vel
        Hp = np.zeros((3, 15))
        Hp[0:3, 0:3] = np.eye(3)
        Hv = np.zeros((3, 15))
        Hv[0:3, 3:6] = np.eye(3)
        Rp = np.diag([R_std[0] ** 2, R_std[0] ** 2, R_std[1] ** 2])
        Rv = np.diag([R_std[2] ** 2, R_std[2] ** 2, R_std[3] ** 2])
        Sp = Hp @ P @ Hp.T + Rp
        Sv = Hv @ P @ Hv.T + Rv
        nis_p = float(y_p @ np.linalg.solve(Sp, y_p))
        nis_v = float(y_v @ np.linalg.solve(Sv, y_v))
        # P0-1 固件用位置/速度分开的门限；旧 B 套件用统一 nis_sub 保持可比。
        ok_p = nis_p <= self.b_par.get("nis_sub_pos", self.b_par["nis_sub"])
        ok_v = nis_v <= self.b_par.get("nis_sub_vel", self.b_par["nis_sub"])
        # 位置先、速度后；应用时用当前 P 重算 S/K（序贯更新）
        if ok_p:
            self._subset_update(self, Hp, Rp, "p", z_pos)
            self.iso_pos_fused += 1
        else:
            self.iso_pos_rej += 1
        if ok_v:
            self._subset_update(self, Hv, Rv, "v", z_vel)
            self.iso_vel_fused += 1
            self.vel_rej_streak = 0
        else:
            self.iso_vel_rej += 1
            self.vel_rej_streak += 1
        if ok_p or ok_v:
            self.time_since_gnss = 0.0
        # 速度通道软重锚（连续 N 次速度子集被拒 → 参照 ResetVelocityToExternal）
        if self.vel_rej_streak >= self.b_par["reanchor_after"]:
            self.reanchor_velocity(z_vel)
            out["reanchored"] = True
        kick = abs(wrap_pi(heading_of(self.quat) - h_before))
        self.kicks.append((self._t, "gnss", kick))
        out.update(fused=bool(ok_p or ok_v), fused_p=bool(ok_p),
                   fused_v=bool(ok_v), isolated=True, nis_p=nis_p,
                   nis_v=nis_v, nis=max(nis_p, nis_v))
        return out

    # ---------------- 候选 C：振动自适应 Q ----------------
    def set_accel_std(self, std):
        std = float(std)
        if abs(std - self._accel_std_cur) < 1e-6:
            return
        self._accel_std_cur = std
        self.rw[0:3, 0:3] = std ** 2 * np.eye(3)
        self.cfg["accel_std"] = std


# ===========================================================================
# 3. 场景运行器（baseline 语义复刻 + 钩子）
# ===========================================================================
class UpgradeScenario(Scenario):
    """逐行复刻 opt_ekf.Scenario.run，插入候选量测/调度与日志。"""

    def __init__(self, spec, ekf_factory, up=None):
        super().__init__(spec, ekf_factory)
        self.up = dict(up or {})
        self.zupt_events = []     # (t, fused, |v_est| before)
        self.mag_info = []

    def run(self, seed=1, verbose=False):
        rng = np.random.default_rng(seed)
        spec = self.spec
        self.prm = dict(FW)
        self.prm.update(spec.get("fw_overrides", {}))
        prm = self.prm
        ekf = self.make_ekf()
        ekf.home_lla = HOME_LLA.copy()
        ekf.initialize(np.array([0.0, 0.0, -G_MPS2]), np.zeros(3), HOME_LLA)
        if spec.get("init_hover"):
            qh = quat_from_yp(0.0, math.radians(90.0), 0.0)
            ekf.quat = qh.copy()
            ekf.Cnb = quat_to_dcm(qh)
            ekf.bg = np.zeros(3) - ekf.Cnb.T @ np.zeros(3)
        if spec.get("p0_hdg") is not None:
            ekf.P[8, 8] = spec["p0_hdg"] ** 2
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
        multipath = gnss.get("multipath")
        outliers = gnss.get("outliers", [])
        baro = spec.get("baro", dict(enable=True, hz=25, noise=0.35))
        yaw_aid = spec.get("yaw_aid", {})
        ahrs_fb = spec.get("ahrs_fallback", {})
        aid_on = spec.get("static_aid", {}).get("enable", True)
        use_tier = spec.get("static_aid", {}).get("tier", True)
        zupt_adapt = spec.get("static_aid", {}).get("zupt_adapt", False)

        # ---- 候选配置 ----
        up = self.up
        mag_cfg = up.get("mag")
        mag_aid = MagAid(mag_cfg) if (mag_cfg and mag_cfg.get("enable", True)) \
            else None
        mag_hz = (mag_cfg or {}).get("hz", 10)
        mag_gate = (mag_cfg or {}).get("gate", True)
        mag_adapt = (mag_cfg or {}).get("adapt", True)
        mag_robust = (mag_cfg or {}).get("robust", False)
        gnss_mode = up.get("gnss_mode", "cur")
        q_cfg = up.get("q_sched")
        q_win = []
        if q_cfg is not None:
            ekf._accel_std_cur = float(ekf.cfg["accel_std"])
        last_track = None
        last_track_t = -1e9
        guard_on = bool(up.get("zupt_ext_ref_guard", False))
        guard_no_ext_s = 0.0
        guard_cap = False
        guard_disable = False

        n = int(spec["dur"] / self.dt)
        rec = dict(t=[], pos_err=[], vel_err=[], att_err=[], yaw_err=[],
                   p_pos=[], p_vel=[], p_att=[], ba=[], bg=[], static=[],
                   conf=[], q_est=[], q_true=[], pitch_true=[], pitch_est=[],
                   # 追加：P0/候选 C 分析用
                   speed_est=[], speed_true=[], p_vv_ne=[], p_vv_d=[],
                   accel_std=[])
        gyro_avg = np.zeros(3)
        gyro_cnt = 0
        for k in range(n):
            t = k * self.dt
            ekf._t = t
            for bj in spec.get("bias_jumps", []):
                if t >= bj["t"] and not bj.get("applied"):
                    trj.ba_true = trj.ba_true + np.array(bj.get("ba", [0, 0, 0]), float)
                    trj.bg_true = trj.bg_true + np.array(bj.get("bg", [0, 0, 0]), float)
                    bj["applied"] = True
            out = trj.step()
            if out is None:
                break
            t, a_meas, w_meas, pos_t, vel_t, quat_t = out
            # ---- 候选 C：高频方差调度 accel_std（每 0.5s 更新一次）----
            if q_cfg is not None:
                q_win.append(a_meas.copy())
                if len(q_win) > 200:
                    q_win.pop(0)
                if k % 100 == 0 and len(q_win) >= 100:
                    A = np.asarray(q_win)
                    d = np.diff(A, axis=0)
                    sig_hf = float(np.mean(np.std(d, axis=0)) / math.sqrt(2.0))
                    std_new = float(np.clip(sig_hf, *q_cfg.get(
                        "bounds", (0.05, 0.25))))
                    ekf.set_accel_std(std_new)
            ekf.time_update(w_meas * self.dt, np.zeros(3),
                            a_meas * self.dt, np.zeros(3), self.dt)
            # 静止辅助
            static_now = det.update(a_meas, w_meas)
            in_outage = outage is not None and outage[0] <= t < outage[1]
            mp = multipath is not None and multipath[0] <= t < multipath[1]
            if guard_on:
                # 理论依据：REF-121 §8.2.3 p.291（阻尼必须外来信息）；
                # 与固件 P0-3 同构：仅 GNSS 出场视为外部速度参照；SC9 无光流。
                if in_outage:
                    guard_no_ext_s += self.dt
                else:
                    guard_no_ext_s = 0.0
                guard_cap = guard_no_ext_s >= 3.0
                guard_disable = guard_no_ext_s >= 30.0
            if aid_on and static_now:
                gyro_avg += w_meas
                gyro_cnt += 1
                tier = aid.tier(det)
                g_n, v_n, v_d = aid.noise(tier) if use_tier else (
                    prm["tier2_grav"], prm["tier2_vel_ne"], prm["tier2_vel_d"])
                if guard_cap:
                    v_n = max(v_n, 0.30)
                    v_d = max(v_d, 0.30)
                dz, dg, ds = aid.divs(tier) if use_tier else (4, 8, 8)
                aid.frame_cnt += 1
                c = aid.frame_cnt
                if c % dz == 0:
                    vb = float(np.linalg.norm(ekf.vel))
                    if guard_on and (guard_disable or
                                     (guard_no_ext_s > 0.0 and vb > 0.75)):
                        # 外部参照门控：拒绝 ZUPT，保留 Gravity/StaticGyro。
                        self.zupt_events.append((t, False, vb))
                    elif zupt_adapt:
                        rr = ekf.measurement_update_velocity_adaptive(
                            np.zeros(3), v_n, v_d)
                        self.zupt_events.append((t, bool(rr.get("fused")), vb))
                    else:
                        rr = ekf.measurement_update_velocity(
                            np.zeros(3), v_n, v_d)
                        self.zupt_events.append((t, bool(rr.get("fused")), vb))
                if c % dg == 1:
                    ekf.measurement_update_gravity(a_meas, g_n)
                if c % ds == 3:
                    ekf.measurement_update_static_gyro(
                        gyro_avg / max(gyro_cnt, 1), np.full(3, prm["sg_noise"]))
                    gyro_avg = np.zeros(3)
                    gyro_cnt = 0
            # GNSS
            if not in_outage and k % (self.HZ // gnss_hz) == 0:
                noise = list(gnss_noise)
                if mp:
                    noise[0] *= multipath[2]
                z_pos = pos_t + rng.normal(0, noise[0], 3) * np.array(
                    [1, 1, noise[1] / max(noise[0], 1e-9)])
                z_vel = vel_t + rng.normal(0, noise[2], 3)
                if gnss.get("static_gate") and \
                        float(np.hypot(z_vel[0], z_vel[1])) > \
                        gnss.get("static_gate_thr", 1.5):
                    det.force_exit()
                    static_now = False
                gnss_forced_zero = False
                if static_now:
                    z_vel = np.zeros(3)
                    gnss_forced_zero = True
                fault_p = bool(mp)
                for (ot0, odur, om_n, om_e, om_d) in outliers:
                    if ot0 <= t < ot0 + odur:
                        z_pos = z_pos + np.array([om_n, om_e, om_d])
                        fault_p = True
                hist = ekf.hist if not spec.get("no_replay") else []
                res = ekf.gnss_update(gnss_mode, z_pos, z_vel, gnss_r,
                                      age_s=gnss_age, hist=hist) \
                    if isinstance(ekf, UpgradeEkf) else \
                    ekf.measurement_update_gnss(z_pos, z_vel, gnss_r,
                                                age_s=gnss_age, hist=hist)
                if res.get("fused"):
                    ekf.meas_log["gnss_fused"] += 1
                    if res.get("conservative"):
                        ekf.meas_log["gnss_conservative"] += 1
                    ekf.meas_log["gnss_nis"].append(
                        float(res.get("nis", 0.0)))
                else:
                    ekf.meas_log["gnss_rej"] += 1
                # 双矢量航迹（供磁门控使用，用 GNSS 速度量测本身）
                spd_track = float(np.hypot(z_vel[0], z_vel[1]))
                if spd_track > 1.0 and not gnss_forced_zero:
                    last_track = math.atan2(z_vel[1], z_vel[0])
                    last_track_t = t
                if isinstance(ekf, UpgradeEkf):
                    ekf.epoch_log.append(dict(
                        t=t, nis6=float(res.get("nis", float("nan"))),
                        nis_p=float(res.get("nis_p", float("nan"))),
                        nis_v=float(res.get("nis_v", float("nan"))),
                        fused=bool(res.get("fused")),
                        fused_p=bool(res.get("fused_p", res.get("fused"))),
                        fused_v=bool(res.get("fused_v", res.get("fused"))),
                        isolated=bool(res.get("isolated")),
                        reanchored=bool(res.get("reanchored")),
                        fault_p=fault_p,
                        raw_pos_err=float(np.hypot(z_pos[0] - pos_t[0],
                                                   z_pos[1] - pos_t[1])),
                        raw_pos_err_d=float(abs(z_pos[2] - pos_t[2])),
                        raw_vel_err=float(np.linalg.norm(z_vel - vel_t))))
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
                        if up.get("yaw_h2_fix"):
                            ekf.yaw_update_h2_fixed(est_yaw, noise)
                        else:
                            ekf.measurement_update_yaw_h2(est_yaw, noise)
                    else:
                        ekf.measurement_update_yaw(est_yaw, noise)
            # ---- 候选 A：磁航向（10 Hz）----
            if mag_aid is not None and k % (self.HZ // mag_hz) == 0:
                m_b = mag_aid.measure(quat_t, t, rng)
                info = self._mag_gate(ekf, m_b, last_track,
                                      t - last_track_t, mag_gate, mag_adapt,
                                      in_outage)
                if info["ok"]:
                    extra = {}
                    ekf.mag_update(m_b, info["sigma"], robust=mag_robust,
                                   extra=extra)
                    info.update(extra)
                self.mag_info.append(dict(t=t, **info))
                if isinstance(ekf, UpgradeEkf):
                    ekf.mag_log.append(dict(t=t, **{
                        kk: vv for kk, vv in info.items() if kk != "t"}))
            # 无 GNSS 运动时 AHRS 回退
            if ahrs_fb.get("enable") and in_outage:
                if abs(float(np.linalg.norm(a_meas)) - G_MPS2) < 2.0 and \
                        float(np.linalg.norm(w_meas)) < ahrs_fb.get("gyro_thr", 0.5):
                    if k % 4 == 0:
                        ekf.measurement_update_gravity(a_meas, ahrs_fb.get("noise", 0.08))
            # 记录
            est_pos = ekf.ned_from_lla(ekf.lla, HOME_LLA)
            att_err = _quat_angle_err(ekf.quat, quat_t)
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
            rec["speed_est"].append(float(np.linalg.norm(ekf.vel)))
            rec["speed_true"].append(float(np.linalg.norm(vel_t)))
            rec["p_vv_ne"].append(float(0.5 * (ekf.P[3, 3] + ekf.P[4, 4])))
            rec["p_vv_d"].append(float(ekf.P[5, 5]))
            rec["accel_std"].append(float(ekf.cfg.get("accel_std", 0.25)))
        self.rec = {kk: np.array(vv) for kk, vv in rec.items()}
        self.ekf = ekf
        self.trj = trj
        return self

    # ---- 磁干扰门控 + 自适应 R ----
    def _mag_gate(self, ekf, m_b, track, track_age, gate_on, adapt_on,
                  in_outage):
        p = ekf.mag_par
        Cnb = quat_to_dcm(ekf.quat)
        m_n = Cnb @ m_b
        norm = float(np.linalg.norm(m_b))
        norm_dev = abs(norm - MAG_B_TOTAL)
        dip_meas = math.atan2(m_n[2], math.hypot(m_n[0], m_n[1]))
        dip_dev = abs(dip_meas - MAG_DIP)
        y, az, _ = mag_azimuth_error(m_b, ekf.quat, MAG_DEC)
        # 与 GNSS 航迹的一致性：比较"磁隐含航向误差"与"航迹隐含航向误差"，
        # 两者都相对滤波航向，滤波航向本身误差自动抵消（与 F2 无关）。
        r_track = None
        if track is not None and track_age < 1.5 and not in_outage:
            psi_est = heading_of(ekf.quat)
            dpsi_track = wrap_pi(psi_est - track)      # est − track
            r_track = abs(wrap_pi(y + dpsi_track))     # y − (track − est)
        reasons = []
        if gate_on:
            if norm_dev > p["gate_norm_ut"]:
                reasons.append("norm")
            if dip_dev > math.radians(p["gate_dip_deg"]):
                reasons.append("dip")
            if r_track is not None and \
                    r_track > math.radians(p["gate_track_deg"]):
                reasons.append("track")
            if abs(y) > math.radians(p["gate_res_deg"]):
                reasons.append("res")
            if math.hypot(ekf.P[6, 6], ekf.P[7, 7]) > math.radians(
                    p["gate_tilt_deg"]):
                reasons.append("tilt")
        if reasons:
            for r in reasons:
                if r == "norm":
                    ekf.mag_rej_norm += 1
                elif r == "dip":
                    ekf.mag_rej_dip += 1
                elif r == "track":
                    ekf.mag_rej_track += 1
                else:
                    ekf.mag_rej_res += 1
            return dict(ok=False, reasons=reasons, norm_dev=norm_dev,
                        dip_dev_deg=dip_dev * R2D,
                        r_track_deg=(r_track * R2D) if r_track is not None
                        else float("nan"), y_deg=y * R2D)
        # 自适应 R：环境偏差 + 姿态不确定度
        sig = math.radians(p["sigma_base_deg"])
        if adapt_on:
            sig += p["k_norm"] * norm_dev
            sig += p["k_dip"] * dip_dev
            if r_track is not None:
                sig += p["k_track"] * r_track
            sig += p["k_att"] * math.hypot(ekf.P[6, 6], ekf.P[7, 7])
        sig = float(np.clip(sig, math.radians(0.3),
                            math.radians(p["sigma_max_deg"])))
        return dict(ok=True, reasons=[], sigma=float(sig), norm_dev=norm_dev,
                    dip_dev_deg=dip_dev * R2D,
                    r_track_deg=(r_track * R2D) if r_track is not None
                    else float("nan"), y_deg=y * R2D)


def _quat_angle_err(q_est, q_true):
    """等价 opt_ekf.quat_angle_err"""
    qe = quat_mult(q_est, quat_conj(q_true))
    return 2.0 * math.acos(max(-1.0, min(1.0, abs(qe[0]))))


# ===========================================================================
# 4. 指标
# ===========================================================================
def extra_metrics(sc, t0=None, t1=None):
    """在 Scenario.metrics 之外追加候选专用指标。"""
    r = sc.rec
    ekf = sc.ekf
    t = r["t"]
    m = np.ones(len(t), bool)
    if t0 is not None:
        m &= t >= t0
    if t1 is not None:
        m &= t <= t1
    if m.sum() < 2:
        return {}
    out = {}
    # 航向踢动（每次融合造成的估计航向变化）
    kicks = [k for (tt, s, k) in getattr(ekf, "kicks", [])
             if (t0 is None or tt >= t0) and (t1 is None or tt <= t1)]
    if kicks:
        out["yaw_kick_max_deg"] = float(np.max(kicks) * R2D)
        out["yaw_kick_p95_deg"] = float(np.percentile(kicks, 95) * R2D)
        out["yaw_kick_n"] = float(len(kicks))
    else:
        out["yaw_kick_max_deg"] = float("nan")
    out["yaw_max_err_deg"] = float(np.max(np.abs(r["yaw_err"][m])) * R2D)
    out["speed_est_min"] = float(np.min(r["speed_est"][m]))
    out["speed_err_max"] = float(np.max(np.abs(r["speed_est"] - r["speed_true"])[m]))
    out["p_vv_ne_mean"] = float(np.mean(r["p_vv_ne"][m]))
    out["static_frac"] = float(np.mean(r["static"][m]))
    out["accel_std_mean"] = float(np.mean(r["accel_std"][m]))
    # ZUPT 事件（窗口内）
    ev = [(tt, f) for (tt, f, _) in sc.zupt_events
          if (t0 is None or tt >= t0) and (t1 is None or tt <= t1)]
    out["zupt_fused_win"] = float(sum(1 for _, f in ev if f))
    out["zupt_rej_win"] = float(sum(1 for _, f in ev if not f))
    # 候选 A 计数
    if isinstance(ekf, UpgradeEkf):
        out["mag_fused"] = float(ekf.mag_fused)
        out["mag_rej_norm"] = float(ekf.mag_rej_norm)
        out["mag_rej_dip"] = float(ekf.mag_rej_dip)
        out["mag_rej_track"] = float(ekf.mag_rej_track)
        out["mag_rej_res"] = float(ekf.mag_rej_res)
        out["mag_rej_nis"] = float(ekf.mag_rej_nis)
        n_tot = ekf.mag_fused + ekf.mag_rej_norm + ekf.mag_rej_dip + \
            ekf.mag_rej_track + ekf.mag_rej_res + ekf.mag_rej_nis
        out["mag_skip_frac"] = float(
            (n_tot - ekf.mag_fused) / n_tot) if n_tot else float("nan")
        # 候选 B 逐历元统计
        ep = [e for e in ekf.epoch_log
              if (t0 is None or e["t"] >= t0) and (t1 is None or e["t"] <= t1)]
        out["gnss_epochs"] = float(len(ep))
        if ep:
            iso = [e for e in ep if e["isolated"]]
            out["fdi_epochs"] = float(len(iso))
            vel_rej = [e for e in ep if not e["fused_v"]]
            out["vel_rej_frac"] = float(len(vel_rej) / len(ep))
            out["pos_rej_frac"] = float(sum(
                1 for e in ep if not e["fused_p"]) / len(ep))
            # 失锁持续：最长连续速度子集被拒（按 10 Hz 历元）
            best = cur = 0
            for e in ep:
                if not e["fused_v"]:
                    cur += 1
                    best = max(best, cur)
                else:
                    cur = 0
            out["vel_unlock_longest_s"] = float(best * (1.0 / 10.0))
            # 误隔离率：被拒但原始量测本身健康（位置 <6m / 速度 <0.75m/s）
            p_rej = [e for e in ep if not e["fused_p"]]
            v_rej = vel_rej
            out["false_iso_p_rate"] = float(
                sum(1 for e in p_rej if e["raw_pos_err"] < 6.0) /
                len(p_rej)) if p_rej else float("nan")
            out["false_iso_v_rate"] = float(
                sum(1 for e in v_rej if e["raw_vel_err"] < 0.75) /
                len(v_rej)) if v_rej else float("nan")
            out["iso_pos_fused"] = float(sum(1 for e in ep if e["isolated"]
                                             and e["fused_p"]))
            out["iso_vel_fused"] = float(sum(1 for e in ep if e["isolated"]
                                             and e["fused_v"]))
            out["reanchors"] = float(ekf.reanchors)
            out["nis6_mean"] = float(np.nanmean([e["nis6"] for e in ep]))
        # P0 场景：失联期 ZUPT/速度坍塌时序
        spec = sc.spec.get("gnss", {})
        if spec.get("outage"):
            o0, o1 = spec["outage"]
            mm = (t >= o0) & (t <= o1)
            out["outage_speed_est_min"] = float(np.min(r["speed_est"][mm]))
            out["outage_speed_est_end"] = float(r["speed_est"][mm][-1])
            out["outage_speed_true_end"] = float(r["speed_true"][mm][-1])
            out["outage_pos_err_end"] = float(np.hypot(
                r["pos_err"][mm][-1][0], r["pos_err"][mm][-1][1]))
            out["outage_pos_err_max"] = float(np.max(np.hypot(
                r["pos_err"][mm][:, 0], r["pos_err"][mm][:, 1])))
            out["outage_static_frac"] = float(np.mean(r["static"][mm]))
            fz = [(tt, f) for (tt, f, _) in sc.zupt_events
                  if o0 <= tt <= o1]
            out["outage_zupt_fused"] = float(sum(1 for _, f in fz if f))
            out["outage_zupt_rej"] = float(sum(1 for _, f in fz if not f))
            out["outage_first_zupt_fuse_s"] = float(
                next((tt - o0 for tt, f in fz if f), -1.0))
            fr = np.where(r["static"][mm] > 0)[0]
            out["outage_static_confirm_s"] = float(
                fr[0] * sc.dt if len(fr) else -1.0)
        out["reanchor_n"] = float(ekf.reanchors)
    return out


def run_one(task):
    """单个 (variant, scenario, seed, window) 运行 —— 进程池 worker"""
    suite, vname, scn, seed, win = task
    v = VARIANTS[suite][vname]
    spec = scenario_spec(scn)
    spec = deep_merge(spec, v.get("spec", {}))
    cfgd = build_cfg(v.get("cfg", {}))
    up = v.get("up", {})
    up = copy.deepcopy(up)
    up.setdefault("mag", None)
    up["mag"] = deep_merge({"enable": True}, up.get("mag") or {}) \
        if up.get("mag") else None
    factory = lambda: UpgradeEkf(cfgd, copy.deepcopy(up.get("ekf_opts", {})))
    sc = UpgradeScenario(spec, factory, up)
    sc.run(seed=seed)
    t0, t1 = win if win else (None, None)
    met = dict(sc.metrics(t0=t0, t1=t1))
    met.update(extra_metrics(sc, t0, t1))
    return dict(suite=suite, variant=vname, scenario=scn, seed=seed,
                metrics={k: float(vv) for k, vv in met.items()
                         if _is_num(vv)})


def _is_num(x):
    try:
        return isinstance(x, (int, float, np.floating, np.integer)) and \
            not (isinstance(x, float) and math.isnan(x))
    except Exception:
        return False


# ===========================================================================
# 5. 变体定义
# ===========================================================================
def _cur(spec_extra=None, cfg_extra=None, up_extra=None):
    return dict(spec=deep_merge(CUR_SPEC, spec_extra or {}),
                cfg=dict(CUR_CFG, **(cfg_extra or {})),
                up=up_extra or {})


def _mag_up(**kw):
    m = {"enable": True, "hz": 10}
    m.update(kw)
    return {"mag": m}


VARIANTS = {
    # ---------------- 候选 A：磁航向辅助 ----------------
    # 全部变体在**修正后**的双矢量航向基线上跑（yaw_h2_fix=True，
    # 对照固件 h_yaw(0,6..8)）；A_nomag_legacy/B_cur_legacy 仅保留名称，
    # opt_ekf H 索引修复后与 A_nomag/B_cur 等价，不再作为“错位”对照。
    "A": {
        "A_nomag_legacy": dict(**_cur(),
                               desc="[历史名] 无磁基线；opt_ekf H 索引修复后与 A_nomag 等价，仅保留名称"),
        "A_nomag": dict(**_cur(up_extra={"yaw_h2_fix": True}),
                        desc="无磁·双矢量航向 H 索引修正（固件口径 6..8）"),
        "A_ideal": dict(**_cur(up_extra={"yaw_h2_fix": True,
                                         **_mag_up(mode="ideal", gate=True,
                                                   adapt=True, robust=False)}),
                        desc="理想磁：真值场+0.3μT 噪声，门控+自适应R"),
        "A_dist_raw": dict(**_cur(up_extra={"yaw_h2_fix": True,
                                            **_mag_up(mode="disturbed",
                                                      gate=False, adapt=False)}),
                           desc="受扰磁·裸用：固定 R=1°，无门控（对照，展示危害）"),
        "A_dist_gate": dict(**_cur(up_extra={"yaw_h2_fix": True,
                                             **_mag_up(mode="disturbed",
                                                       gate=True, adapt=True,
                                                       robust=False)}),
                            desc="受扰磁·门控+自适应R+硬 NIS 门（候选 A 标准形态）"),
        "A_dist_robust": dict(**_cur(up_extra={"yaw_h2_fix": True,
                                               **_mag_up(mode="disturbed",
                                                         gate=True, adapt=True,
                                                         robust=True)}),
                              desc="受扰磁·门控+自适应R+鲁棒软更新（候选 A 增强形态）"),
    },
    # ---------------- 候选 B：GNSS 故障隔离 / 速度重锚 ----------------
    "B": {
        "B_cur_legacy": dict(**_cur(),
                             desc="[历史名] 固件现值无磁基线；opt_ekf H 索引修复后与 B_cur 等价（仅 SC3 有双矢量航向）"),
        "B_cur": dict(**_cur(up_extra={"yaw_h2_fix": True}),
                      desc="固件现值：6 维联合更新，NIS 拒绝即整体放弃（航向 H 修正）"),
        "B_fdi": dict(**_cur(up_extra={
            "yaw_h2_fix": True, "gnss_mode": "fdi",
            "ekf_opts": {"fdi": {"reanchor_after": 10 ** 9}}}),
            desc="仅子集隔离（不重锚）：6 维拒绝 → 3 维子集检验，只更新通过子集"),
        "B_fdi_infl": dict(**_cur(up_extra={
            "yaw_h2_fix": True, "gnss_mode": "fdi",
            "ekf_opts": {"fdi": {"reanchor_kind": "inflate"}}}),
            desc="子集隔离 + 速度连续 10 次被拒 → 仅膨胀 P（软重锚）"),
        "B_fdi_reset": dict(**_cur(up_extra={
            "yaw_h2_fix": True, "gnss_mode": "fdi",
            "ekf_opts": {"fdi": {"reanchor_kind": "reset"}}}),
            desc="子集隔离 + 速度重锚（重置速度态+抬 P+清回放，对照 ResetVelocityToExternal）"),
        "B_fdi_fw": dict(**_cur(up_extra={
            "yaw_h2_fix": True, "gnss_mode": "fdi",
            "ekf_opts": {"fdi": {"reanchor_kind": "inflate",
                                 "nis_sub_pos": 25.90,
                                 "nis_sub_vel": 16.27}}}),
            desc="P0-1 固件阈值：位置 χ²(3)=25.90、速度 χ²(3)=16.27；软重锚膨胀 P"),
    },
    # ---------------- 候选 C：振动自适应 Q ----------------
    "C": {
        "C_cur": dict(**_cur(), desc="固件现值：accel_std 固定 0.25"),
        "C_adapt": dict(**_cur(up_extra={"q_sched": {"bounds": (0.05, 0.25)}}),
                        desc="IMU 高频方差调度 accel_std ∈[0.05,0.25]"),
        "C_q05": dict(**_cur(cfg_extra={"accel_std": 0.05}),
                      desc="固定 Q 标定值 0.05（离线标定参照）"),
    },
    # ---------------- P0：ZUPT 误静止风险 ----------------
    # 理论依据：秦永元等《卡尔曼滤波与组合导航原理（第3版）》§8.2.3 p.291：
    # 间接法滤波的阻尼必须利用来自其他导航系统的外来信息，不能用自身信息；
    # §7.3.1 p.255：系统级 FDI 用 χ² 检验。
    "P0": {
        "P0_cur": dict(**_cur(),
                       desc="固件现值（含 ZUPT 残差自适应 1.5 m/s 跳过门 + NIS 16.3）"),
        "P0_guard": dict(**_cur(up_extra={"zupt_ext_ref_guard": True}),
                         desc="固件现值 + 外部参照门控（3 s 封顶/0.75 m/s 拒绝/30 s 停用）"),
        "P0_guard08": dict(**_cur(spec_extra={"gnss": {"static_gate_thr": 0.8}},
                                  up_extra={"zupt_ext_ref_guard": True}),
                           desc="外部参照门控 + GNSS 静止退出门 1.5→0.8 m/s（补 1 m/s 盲区）"),
        "P0_noadapt": dict(**_cur(spec_extra={"static_aid": {"zupt_adapt": False}}),
                           desc="关闭静止辅助残差自适应（只留 EKF 5.5 m/s 残差门 + NIS 16.3）"),
        "P0_pre_p1": dict(**_cur(spec_extra={"static_aid": {"zupt_adapt": False}},
                                 cfg_extra={"nis_zupt": 1000.0}),
                          desc="P1 修复前口径（无自适应跳过 + ZUPT NIS 1000）"),
    },
}

# 场景 + 统计窗口（None=全窗口）
SUITE_EVAL = {
    "A": {
        "A_nomag_legacy": [("SC3_cruise_turn", (30.0, 90.0)),
                           ("SC4_transition", None), ("SC7_hover_vib", None)],
        "A_nomag": [("SC3_cruise_turn", (30.0, 90.0)), ("SC4_transition", None),
                    ("SC5_outage_reacq", None), ("SC7_hover_vib", None)],
        "A_ideal": [("SC3_cruise_turn", (30.0, 90.0)), ("SC4_transition", None),
                    ("SC5_outage_reacq", None), ("SC7_hover_vib", None)],
        "A_dist_raw": [("SC3_cruise_turn", (30.0, 90.0)), ("SC4_transition", None),
                       ("SC5_outage_reacq", None), ("SC7_hover_vib", None)],
        "A_dist_gate": [("SC3_cruise_turn", (30.0, 90.0)), ("SC4_transition", None),
                        ("SC5_outage_reacq", None), ("SC7_hover_vib", None)],
        "A_dist_robust": [("SC3_cruise_turn", (30.0, 90.0)),
                          ("SC4_transition", None), ("SC5_outage_reacq", None),
                          ("SC7_hover_vib", None)],
    },
    "B": {
        "B_cur_legacy": [("SC3_cruise_turn", (30.0, 90.0))],
        "B_cur": [("SC3_cruise_turn", (30.0, 90.0)), ("SC5_outage_reacq", None),
                  ("SC6_outliers", None), ("SC8_bias_jump", (30.0, 60.0))],
        "B_fdi": [("SC3_cruise_turn", (30.0, 90.0)), ("SC5_outage_reacq", None),
                  ("SC6_outliers", None), ("SC8_bias_jump", (30.0, 60.0))],
        "B_fdi_infl": [("SC3_cruise_turn", (30.0, 90.0)),
                       ("SC5_outage_reacq", None), ("SC6_outliers", None),
                       ("SC8_bias_jump", (30.0, 60.0))],
        "B_fdi_reset": [("SC3_cruise_turn", (30.0, 90.0)),
                        ("SC5_outage_reacq", None), ("SC6_outliers", None),
                        ("SC8_bias_jump", (30.0, 60.0))],
        "B_fdi_fw": [("SC3_cruise_turn", (30.0, 90.0)),
                     ("SC5_outage_reacq", None), ("SC6_outliers", None),
                     ("SC8_bias_jump", (30.0, 60.0))],
    },
    "C": {
        "C_cur": [("SC2_cruise_gnss", (60.0, 120.0)), ("SC7_hover_vib", None),
                  ("SC7_hi_vib", None), ("SC8_bias_jump", (30.0, 60.0))],
        "C_adapt": [("SC2_cruise_gnss", (60.0, 120.0)), ("SC7_hover_vib", None),
                    ("SC7_hi_vib", None), ("SC8_bias_jump", (30.0, 60.0))],
        "C_q05": [("SC2_cruise_gnss", (60.0, 120.0)), ("SC7_hover_vib", None),
                  ("SC7_hi_vib", None), ("SC8_bias_jump", (30.0, 60.0))],
    },
    "P0": {
        "P0_cur": [("SC9_zupt_false_static", None), ("SC9b_zupt_lowspeed", None)],
        "P0_guard": [("SC9_zupt_false_static", None), ("SC9b_zupt_lowspeed", None)],
        "P0_guard08": [("SC9_zupt_false_static", None), ("SC9b_zupt_lowspeed", None)],
        "P0_noadapt": [("SC9_zupt_false_static", None),
                       ("SC9b_zupt_lowspeed", None)],
        "P0_pre_p1": [("SC9_zupt_false_static", None),
                      ("SC9b_zupt_lowspeed", None)],
    },
}


# ===========================================================================
# 6. 运行 / 聚合 / 输出
# ===========================================================================
def build_tasks(suite, seeds):
    """展开套件任务清单：[(变体, 场景, 统计窗口, 种子)]，供进程池并行调度。"""
    tasks = []
    for vname, lst in SUITE_EVAL[suite].items():
        for scn, win in lst:
            for sd in seeds:
                tasks.append((suite, vname, scn, sd, win))
    return tasks


def aggregate(runs):
    """[(variant, scenario, seed, metrics)] -> 嵌套统计"""
    out = {}
    for r in runs:
        v, s = r["variant"], r["scenario"]
        out.setdefault(v, {}).setdefault(s, []).append(r["metrics"])
    agg = {}
    for v, scn_map in out.items():
        agg[v] = {}
        for s, rows in scn_map.items():
            keys = sorted({k for r in rows for k in r})
            st = {}
            for k in keys:
                vals = np.array([r[k] for r in rows if k in r], float)
                vals = vals[~np.isnan(vals)]
                if len(vals) == 0:
                    continue
                st[k] = dict(mean=float(vals.mean()),
                             std=float(vals.std(ddof=1)) if len(vals) > 1
                             else 0.0,
                             min=float(vals.min()), max=float(vals.max()),
                             n=int(len(vals)))
            agg[v][s] = st
    return agg


KEY_PRINT = ["pos_ne_rmse", "pos_d_rmse", "vel_ne_rmse", "yaw_rmse_deg",
             "yaw_max_err_deg", "yaw_kick_max_deg", "gnss_accept",
             "mag_skip_frac", "vel_unlock_longest_s", "false_iso_v_rate",
             "reanchors", "outage_speed_est_min", "outage_zupt_fused",
             "accel_std_mean"]


def print_suite(agg, suite):
    """把聚合结果（变体→场景→均值±标准差）打印成人读表。"""
    print(f"\n===== suite {suite} =====")
    for v, scn_map in agg.items():
        print(f"-- {v}: {VARIANTS[suite][v]['desc']}")
        for s, st in scn_map.items():
            parts = []
            for k in KEY_PRINT:
                if k in st:
                    parts.append(f"{k}={st[k]['mean']:.3g}"
                                 f"±{st[k]['std']:.2g}")
            print(f"   {s:22s} " + " ".join(parts))


def write_json(obj, path=None):
    """把结果写到 `sim-data/opt/upgrade_candidates.json`（默认路径），UTF-8 缩进 2。"""
    path = path or OUT_JSON
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=2)
    print(f"[完成] 写入 {path}")


# ===========================================================================
# 7. 自检
# ===========================================================================
def selftest():
    """4 项自检：磁方位符号、运行器等价性（空升级 vs 基线 <1e-9）、H 索引证据、场景可复现。

    @note 自检失败即退出，不产生结果 JSON；用于确认候选实现没有悄悄改变基线语义。
    """
    print("== 自检 1：磁方位误差符号 ==")
    q_true = quat_from_yp(0.5, 0.3, 0.1)
    m_b = quat_to_dcm(q_true).T @ MAG_B_NED
    for dpsi_deg in (+2.0, -3.0, 0.0):
        q_est = quat_from_yp(0.5 + math.radians(dpsi_deg), 0.3, 0.1)
        y, az, _ = mag_azimuth_error(m_b, q_est, MAG_DEC)
        print(f"  估计航向偏移 {dpsi_deg:+.1f}° → y={y * R2D:+.3f}° "
              f"(应 ≈ {-dpsi_deg:+.1f}°)")
        assert abs(y * R2D + dpsi_deg) < 1e-6, "磁方位误差符号错误"
    print("  OK")

    print("== 自检 2：UpgradeScenario 与 baseline Scenario 行为一致 ==")
    spec = deep_merge(scenario_spec("SC5_outage_reacq"), CUR_SPEC)
    cfgd = build_cfg(CUR_CFG)
    a = Scenario(copy.deepcopy(spec), lambda: OptEkf(cfgd))
    a.run(seed=1)
    b = UpgradeScenario(copy.deepcopy(spec),
                        lambda: UpgradeEkf(cfgd, {}), {"mag": None})
    b.run(seed=1)
    ma, mb = a.metrics(), b.metrics()
    diffs = {k: abs(ma[k] - mb[k]) for k in ma
             if isinstance(ma[k], float) and not math.isnan(ma[k])
             and k in mb}
    worst = sorted(diffs.items(), key=lambda kv: -kv[1])[:5]
    print("  最大差异:", [(k, f"{d:.3e}") for k, d in worst])
    assert worst[0][1] < 1e-9, "复刻运行器与 baseline 不等价"
    print("  OK（逐项 < 1e-9）")

    print("== 自检 3：候选 B 子集更新在干净通道下不改变状态 ==")
    ekf = UpgradeEkf(build_cfg(CUR_CFG), {})
    ekf.home_lla = HOME_LLA.copy()
    ekf.initialize(np.array([0.0, 0.0, -G_MPS2]), np.zeros(3), HOME_LLA)
    P0, v0 = ekf.P.copy(), ekf.vel.copy()
    H = np.zeros((3, 15))
    H[0:3, 3:6] = np.eye(3)
    ekf._subset_update(ekf, H, np.diag([0.15 ** 2] * 2 + [0.25 ** 2]),
                       "v", ekf.vel.copy())
    print(f"  零新息速度子集更新: |Δv|={np.linalg.norm(ekf.vel - v0):.2e} "
          f"|ΔP|={np.linalg.norm(ekf.P - P0):.2e}")
    assert np.linalg.norm(ekf.vel - v0) < 1e-12
    print("  OK")

    print("== 自检 4：H2 航向索引（固件 6..8；历史 7:10 bug 见 notes/05）==")

    def _fresh():
        e = UpgradeEkf(build_cfg(CUR_CFG), {})
        e.home_lla = HOME_LLA.copy()
        e.initialize(np.array([0.0, 0.0, -G_MPS2]), np.zeros(3), HOME_LLA)
        e.P[8, 8] = math.radians(20.0) ** 2      # 航向 1σ=20°
        return e

    e1 = _fresh()
    ba0 = e1.ba.copy()
    e1.measurement_update_yaw_h2(math.radians(10.0), math.radians(1.0))
    yaw1 = math.degrees(quat_to_euler(e1.quat)[0])
    print(f"  修正后 H2(6:9) +10° 航向观测 → 航向={yaw1:+.3f}° "
          f"Δba_x={e1.ba[0] - ba0[0]:+.3f} m/s²")
    assert abs(yaw1 - 10.0) < 0.5 and abs(e1.ba[0] - ba0[0]) < 1e-9, \
        "H2 修正未按固件 6..8 工作（航向未修正或污染 ba_x）"
    print("  → 证据成立：按固件 6..8 修正航向，不再污染加计零偏 x")
    print("  （历史 7:10 bug 的单点注入证据见 notes/05 §2/自检记录）")
    print("  OK")


def main():
    """程序入口：解析 --selftest/--suite/--seeds/--workers 并调度运行器。"""
    ap = argparse.ArgumentParser()
    ap.add_argument("--suite", default="all",
                    choices=["A", "B", "C", "P0", "all"])
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--workers", type=int, default=10)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--out", default=OUT_JSON)
    args = ap.parse_args()
    if args.selftest:
        selftest()
        return
    suites = ["A", "B", "C", "P0"] if args.suite == "all" else [args.suite]
    seeds = list(range(1, args.seeds + 1))
    t_all = time.time()
    payload = dict(
        meta=dict(
            generated=time.strftime("%Y-%m-%d %H:%M:%S"),
            python=sys.version.split()[0], numpy=np.__version__,
            seeds=seeds, workers=args.workers,
            baseline="固件现值 = 静态门控(F1) + P0 航向 0.2(F2) + ZUPT NIS 16.3(P1)",
            cur_spec=CUR_SPEC, cur_cfg=CUR_CFG,
            mag_model=dict(B_NEU_ut=MAG_B_NED.tolist(),
                           B_total_ut=MAG_B_TOTAL,
                           dip_deg=MAG_DIP * R2D,
                           dec_deg=MAG_DEC * R2D),
            note="所有数值为仿真结果；仿真无控制闭环、磁模型为简化硬/软磁+正弦干扰"),
        suites={})
    for suite in suites:
        tasks = build_tasks(suite, seeds)
        print(f"[{suite}] {len(tasks)} 次运行 ...")
        t0 = time.time()
        runs = []
        with ProcessPoolExecutor(max_workers=args.workers) as ex:
            for i, r in enumerate(ex.map(run_one, tasks, chunksize=1)):
                runs.append(r)
                if (i + 1) % 20 == 0 or i + 1 == len(tasks):
                    print(f"   {i + 1}/{len(tasks)}  {time.time() - t0:.0f}s")
        agg = aggregate(runs)
        payload["suites"][suite] = dict(
            eval={v: [dict(scenario=s, window=w) for s, w in lst]
                  for v, lst in SUITE_EVAL[suite].items()},
            desc={v: VARIANTS[suite][v]["desc"]
                  for v in SUITE_EVAL[suite]},
            variants=agg,
            runs=[dict(variant=r["variant"], scenario=r["scenario"],
                       seed=r["seed"], metrics=r["metrics"]) for r in runs],
            elapsed_s=round(time.time() - t0, 1))
        print_suite(agg, suite)
    payload["meta"]["elapsed_s"] = round(time.time() - t_all, 1)
    write_json(payload, args.out)


if __name__ == "__main__":
    main()
