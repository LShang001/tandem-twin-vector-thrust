# -*- coding: utf-8 -*-
"""
15 状态误差状态 EKF 数值仿真核心
================================
逐行对照固件 `lib/navigation-main/src/ekf_15_state.h` 与
`src/navigation_task.cpp` 的算法语义实现，用于生成分析报告的数值素材。

状态定义（与固件完全一致）:
    x = [δp_N, δp_E, δp_D, δv_N, δv_E, δv_D, δβ_N, δβ_E, δβ_D,
         δba_x, δba_y, δba_z, δbg_x, δbg_y, δbg_z]^T
误差姿态为体坐标系右乘等效旋转向量（q = q_nom ⊗ Exp(δβ^b)）。

关键数值参数直接取自固件代码（见下方 CONFIG）:
  - IMU:  gyro_std=0.0015 rad/s, gyro_markov_bias_std=0.00524 rad/s,
          gyro_tau=180 s, accel_std=0.25 m/s^2, accel_tau=100 s
  - GNSS R 地板: pos_ne 2m / pos_d 3m / vel_ne 0.15 / vel_d 0.25
  - NIS 门限: GNSS=30, ZUPT=1000, Gravity=18, StaticGyro=24, Yaw=12
  - 静止辅助: ZUPT 每 4 帧, Gravity 每 8 帧, StaticGyro 每 8 帧 (200Hz 基)
  - 传播: 双子样圆锥/划摇补偿, 中点重力, 地球自转+传输速率, 二阶 Φ, Simpson Qd
  - 更新: Joseph 形式协方差, NIS 门控, GNSS 重捕获膨胀 R
"""
import json
import math
import os

import numpy as np

# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------
D2R = math.pi / 180.0
R2D = 180.0 / math.pi
G_MPS2 = 9.80665
WE_RADPS = 7.2921151467e-5
SEMI_MAJOR = 6378137.0
ECC2 = 6.69437999014e-3


def skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def rotvec_to_quat(theta):
    """旋转向量 -> 四元数 (w,x,y,z)，与固件 BuildDeltaQuatFromRotationVector 一致"""
    a = np.linalg.norm(theta)
    q = np.zeros(4)
    if a > 1e-6:
        half = 0.5 * a
        s = math.sin(half) / a
        q[0] = math.cos(half)
        q[1:] = s * theta
    else:
        q[0] = 1.0
        q[1:] = 0.5 * theta
    return q / np.linalg.norm(q)


def quat_mult(q, p):
    """q ⊗ p, (w,x,y,z)"""
    w1, x1, y1, z1 = q
    w2, x2, y2, z2 = p
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2])


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_to_dcm(q):
    """四元数 -> 机体系到导航系方向余弦 C^b_n 的转置 C^n_b（行=导航, 列=机体）
    返回矩阵 M 满足 v_ned = M @ v_body"""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]])


def dcm_to_quat(C):
    """C^n_b -> 四元数"""
    tr = C[0, 0] + C[1, 1] + C[2, 2]
    q = np.zeros(4)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q[0] = 0.25 * s
        q[1] = (C[2, 1] - C[1, 2]) / s
        q[2] = (C[0, 2] - C[2, 0]) / s
        q[3] = (C[1, 0] - C[0, 1]) / s
    else:
        if C[0, 0] > C[1, 1] and C[0, 0] > C[2, 2]:
            s = math.sqrt(1.0 + C[0, 0] - C[1, 1] - C[2, 2]) * 2
            q[0] = (C[2, 1] - C[1, 2]) / s
            q[1] = 0.25 * s
            q[2] = (C[0, 1] + C[1, 0]) / s
            q[3] = (C[0, 2] + C[2, 0]) / s
        elif C[1, 1] > C[2, 2]:
            s = math.sqrt(1.0 + C[1, 1] - C[0, 0] - C[2, 2]) * 2
            q[0] = (C[0, 2] - C[2, 0]) / s
            q[1] = (C[0, 1] + C[1, 0]) / s
            q[2] = 0.25 * s
            q[3] = (C[1, 2] + C[2, 1]) / s
        else:
            s = math.sqrt(1.0 + C[2, 2] - C[0, 0] - C[1, 1]) * 2
            q[0] = (C[1, 0] - C[0, 1]) / s
            q[1] = (C[0, 2] + C[2, 0]) / s
            q[2] = (C[1, 2] + C[2, 1]) / s
            q[3] = 0.25 * s
    return q / np.linalg.norm(q)


def quat_to_euler(q):
    """ZYX (yaw, pitch, roll)，与固件 quat2angle 一致"""
    C = quat_to_dcm(q)
    pitch = -math.asin(max(-1.0, min(1.0, C[2, 0])))
    roll = math.atan2(C[2, 1], C[2, 2])
    yaw = math.atan2(C[1, 0], C[0, 0])
    return np.array([yaw, pitch, roll])


def wrap_pi(a):
    a = math.fmod(a + math.pi, 2 * math.pi)
    if a < 0:
        a += 2 * math.pi
    return a - math.pi


# ---------------------------------------------------------------------------
# 地球模型（与固件 ComputeEarthModelTerms 一致）
# ---------------------------------------------------------------------------
class EarthModel:
    def __init__(self, lat_rad, alt_m):
        self.lat = lat_rad
        self.alt = alt_m
        self.sin_lat = math.sin(lat_rad)
        self.cos_lat = math.cos(lat_rad)
        om_e2s2 = max(1e-12, 1.0 - ECC2 * self.sin_lat * self.sin_lat)
        sqrt_t = math.sqrt(om_e2s2)
        self.rn = SEMI_MAJOR / sqrt_t
        self.rm = SEMI_MAJOR * (1.0 - ECC2) / (om_e2s2 * sqrt_t)
        self.mean_r = math.sqrt(self.rn * self.rm)
        sin_2lat = 2.0 * self.sin_lat * self.cos_lat
        g0 = 9.780318 * (1.0 + 5.3024e-3 * self.sin_lat**2 -
                         5.898e-6 * sin_2lat**2)
        scale = 1.0 + alt_m / self.mean_r
        self.g = g0 / (scale * scale)
        self.omega_ned = np.array([WE_RADPS * self.cos_lat, 0.0,
                                   -WE_RADPS * self.sin_lat])

    def nav_rate(self, v_ned):
        rnh = self.rn + self.alt
        rmh = self.rm + self.alt
        tan = self.sin_lat / max(self.cos_lat, 1e-9)
        return np.array([v_ned[1] / rnh,
                         -v_ned[0] / rmh,
                         -v_ned[1] * tan / rnh])

    def lla_rate(self, v_ned):
        rmh = self.rm + self.alt
        rnh = self.rn + self.alt
        return np.array([v_ned[0] / rmh,
                         v_ned[1] / (rnh * max(self.cos_lat, 1e-9)),
                         -v_ned[2]])


# ---------------------------------------------------------------------------
# 15 状态 EKF（按固件语义）
# ---------------------------------------------------------------------------
class Ekf15:
    def __init__(self, cfg):
        self.cfg = cfg
        self.dt = cfg["dt"]
        self.n = 15
        # 状态
        self.lla = np.zeros(3)          # lat, lon, alt (rad, rad, m)
        self.vel = np.zeros(3)          # NED
        self.quat = np.array([1.0, 0, 0, 0])
        self.ba = np.zeros(3)           # accel bias
        self.bg = np.zeros(3)           # gyro bias
        self.P = np.zeros((15, 15))
        # 过程噪声连续谱密度
        a_std = cfg["accel_std"]
        g_std = cfg["gyro_std"]
        a_b = cfg["accel_bias_std"]
        g_b = cfg["gyro_bias_std"]
        a_tau = cfg["accel_tau"]
        g_tau = cfg["gyro_tau"]
        self.rw = np.zeros((12, 12))
        self.rw[0:3, 0:3] = a_std**2 * np.eye(3)
        self.rw[3:6, 3:6] = g_std**2 * np.eye(3)
        self.rw[6:9, 6:9] = 2.0 * a_b**2 / a_tau * np.eye(3)
        self.rw[9:12, 9:12] = 2.0 * g_b**2 / g_tau * np.eye(3)
        self.accel_markov = -1.0 / a_tau * np.eye(3)
        self.gyro_markov = -1.0 / g_tau * np.eye(3)
        self.prev_dtheta = np.zeros(3)
        self.prev_dv = np.zeros(3)
        self.Cnb = np.eye(3)
        self.gravity = G_MPS2
        self.earth = None
        self.time_since_gnss = 0.0
        self.gnss_reacquire_timer = 0.0
        self.hist = []                  # 快照历史（延迟回放）

    # ---- 初始化 ----
    def initialize(self, accel, gyro, lla, vel=None):
        if vel is None:
            vel = np.zeros(3)
        self.lla = np.array(lla, dtype=float)
        self.vel = np.array(vel, dtype=float)
        # 静止调平：比力方向为重力反方向
        f = np.array(accel, dtype=float)
        f_n = f / np.linalg.norm(f)
        # 重力在 NED 下为 (0,0,g)，机体系比力应为 C^b_n @ (0,0,-g)
        Cbn = np.eye(3)
        # 用最小旋转把 (0,0,1) 对齐到 -f_n
        z_n = np.array([0.0, 0.0, 1.0])
        target = -f_n
        v = np.cross(z_n, target)
        s = np.linalg.norm(v)
        c = np.dot(z_n, target)
        if s > 1e-9:
            Vx = skew(v)
            Cbn = np.eye(3) + Vx + Vx @ Vx * (1 - c) / (s * s)
        else:
            Cbn = np.eye(3) if c > 0 else -np.diag([1, -1, 1]) @ np.eye(3)
        self.Cnb = Cbn.T
        self.quat = dcm_to_quat(self.Cnb)
        # 地球自转 + 传输速率初值
        self.earth = EarthModel(self.lla[0], self.lla[2])
        nav_rate = self.earth.omega_ned + self.earth.nav_rate(self.vel)
        self.bg = np.array(gyro, dtype=float) - self.Cnb.T @ nav_rate
        self.gravity = self.earth.g
        # 初始协方差
        p0 = np.zeros((15, 15))
        p0[0:3, 0:3] = cfg_p0["pos"]**2 * np.eye(3)
        p0[3:6, 3:6] = cfg_p0["vel"]**2 * np.eye(3)
        p0[6:8, 6:8] = cfg_p0["att"]**2 * np.eye(2)
        p0[8, 8] = cfg_p0["hdg"]**2
        p0[9:12, 9:12] = cfg_p0["ba"]**2 * np.eye(3)
        p0[12:15, 12:15] = cfg_p0["bg"]**2 * np.eye(3)
        self.P = p0
        self.hist = []

    # ---- 双子样传播 ----
    def time_update(self, dtheta1, dtheta2, dv1, dv2, dt, record=True):
        # 记录本步 IMU 增量供延迟回放重传播
        self._last_inc = (dtheta1.copy(), dtheta2.copy(), dv1.copy(), dv2.copy(),
                          dt)
        # ★ 对齐固件 TimeUpdateTwoSample：先扣当前零偏估计再传播
        half = 0.5 * dt
        dtheta1 = dtheta1 - self.bg * half
        dtheta2 = dtheta2 - self.bg * half
        dv1 = dv1 - self.ba * half
        dv2 = dv2 - self.ba * half
        dtheta = dtheta1 + dtheta2
        dv = dv1 + dv2
        coning = dtheta + (2.0 / 3.0) * np.cross(dtheta1, dtheta2)
        scull = (dv + 0.5 * np.cross(dtheta, dv) +
                 (2.0 / 3.0) * (np.cross(dtheta1, dv2) + np.cross(dv1, dtheta2)))
        # 中点四元数（用于比力旋转）
        dq_mid = rotvec_to_quat(coning * 0.5)
        q_mid = quat_mult(self.quat, dq_mid)
        q_mid /= np.linalg.norm(q_mid)
        Cnb_mid = quat_to_dcm(q_mid)
        # 位置中点重力/地球项
        em_prev = EarthModel(self.lla[0], self.lla[2])
        lla_rate_prev = em_prev.lla_rate(self.vel)
        lla_mid = self.lla + 0.5 * dt * lla_rate_prev
        em = EarthModel(lla_mid[0], lla_mid[2])
        self.gravity = em.g
        g_ned = np.array([0.0, 0.0, em.g])
        omega_n = em.omega_ned + em.nav_rate(self.vel)
        coriolis = -np.cross(2.0 * omega_n, self.vel)
        nav_dtheta = -omega_n * dt
        # 姿态推进（导航系转动 + 圆锥补偿 + 机体系角增量）
        dq_nav = rotvec_to_quat(nav_dtheta)
        dq_imu = rotvec_to_quat(coning)
        self.quat = quat_mult(dq_nav, quat_mult(self.quat, dq_imu))
        self.quat /= np.linalg.norm(self.quat)
        if self.quat[0] < 0:
            self.quat = -self.quat
        self.Cnb = quat_to_dcm(self.quat)
        # 速度推进（★ 对齐固件 MED-14 修复：绕中点姿态旋转，与 F 线性化点一致）
        dv_ned = Cnb_mid @ scull + dt * (g_ned + coriolis)
        self.vel = self.vel + dv_ned
        # 位置推进（速度中点）
        v_mid = self.vel - 0.5 * dv_ned
        lla_rate_mid = em.lla_rate(v_mid)
        self.lla = self.lla + dt * lla_rate_mid
        # ---- 误差状态线性化 ----
        a_est = self.Cnb.T @ (self.vel  # (中间量, 见下) 实际直接用比力
                              * 0)      # 占位
        # 真实机体系比力（去偏后）
        f_body = (dv1 + dv2) / dt
        # 连续 F 矩阵
        F = np.zeros((15, 15))
        F[0:3, 3:6] = np.eye(3)                     # 位置 <- 速度
        F[3:6, 3:6] = self.fvv(em, self.vel)        # 哥氏/传输速率自耦合
        F[0:3, 0:3] = self.fpp(em, self.vel)        # 位置自耦合
        F[3:6, 6:9] = -skew(self.Cnb @ f_body)      # 速度 <- 姿态误差
        F[3:6, 9:12] = -self.Cnb                    # 速度 <- 加速度零偏
        F[6:9, 3:6] = self.fphiv(em)                # 姿态误差 <- 速度 (传输速率)
        F[6:9, 6:9] = -skew(self.Cnb @ f_body * 0 + self.bg_gyro_est())  # 姿态自旋
        F[6:9, 12:15] = -np.eye(3)                  # 姿态 <- 陀螺零偏
        F[5, 2] = 2.0 * em.g / em.mean_r            # 高度通道重力梯度
        F[9:12, 9:12] = self.accel_markov
        F[12:15, 12:15] = self.gyro_markov
        self.F = F
        # 姿态自旋项用真实去偏角速度
        g_body = (dtheta1 + dtheta2) / dt
        F[6:9, 6:9] = -skew(g_body)
        # 离散化
        dt_f = dt
        Phi = np.eye(15) + F * dt_f + 0.5 * (F * dt_f) @ (F * dt_f)
        # Gs
        Gs = np.zeros((15, 12))
        Gs[3:6, 0:3] = -self.Cnb
        Gs[6:9, 3:6] = -np.eye(3)
        Gs[9:12, 6:9] = np.eye(3)
        Gs[12:15, 9:12] = np.eye(3)
        # Simpson Qd
        Qc = Gs @ self.rw @ Gs.T
        Phi_half = np.eye(15) + F * 0.5 * dt_f + 0.5 * (F * 0.5 * dt_f) @ (F * 0.5 * dt_f)
        Qd = (dt_f / 6.0) * (Qc + 4.0 * Phi_half @ Qc @ Phi_half.T +
                             Phi @ Qc @ Phi.T)
        Qd = 0.5 * (Qd + Qd.T)
        self.P = Phi @ self.P @ Phi.T + Qd
        self.P = self.stabilize(self.P)
        # 快照（延迟回放）
        if record:
            self.hist.append(self.snapshot())
            if len(self.hist) > 64:
                self.hist.pop(0)
        # 失联膨胀
        self.time_since_gnss += dt
        if self.time_since_gnss >= 2.0:
            self.grow_outage_cov(dt)
        self.prev_dtheta = dtheta
        self.prev_dv = dv

    def bg_gyro_est(self):
        return self.bg

    def fvv(self, em, v):
        """F(V,V) 哥氏/传输速率 3x3（对照固件 BFS_NAVIGATION_EMBEDDED_FULL_F_MATRIX）"""
        vn, ve, vd = v
        sin_lat, cos_lat = em.sin_lat, em.cos_lat
        tan_lat = sin_lat / max(cos_lat, 1e-9)
        rmh = em.rm + em.alt
        rnh = em.rn + em.alt
        w = WE_RADPS
        F = np.zeros((3, 3))
        F[0, 0] = vd / rmh
        F[0, 1] = -2.0 * (w * sin_lat + ve * tan_lat / rnh)
        F[0, 2] = vn / rmh
        F[1, 0] = 2.0 * w * sin_lat + ve * tan_lat / rnh
        F[1, 1] = (vd + vn * tan_lat) / rnh
        F[1, 2] = 2.0 * w * cos_lat + ve / rnh
        F[2, 0] = -2.0 * vn / rmh
        F[2, 1] = -2.0 * (w * cos_lat + ve / rnh)
        return F

    def fpp(self, em, v):
        """F(P,P) 位置自耦合 3x3"""
        vn, ve, vd = v
        tan_lat = em.sin_lat / max(em.cos_lat, 1e-9)
        rmh = em.rm + em.alt
        rnh = em.rn + em.alt
        F = np.zeros((3, 3))
        F[0, 0] = -vd / rmh
        F[0, 2] = vn / rmh
        F[1, 0] = ve * tan_lat / rnh
        F[1, 1] = -(vd + vn * tan_lat) / rnh
        F[1, 2] = ve / rnh
        return F

    def fphiv(self, em):
        """F(Φ,V) = -C^b_n M（体误差，对照固件 2397-2401 行）"""
        rmh = em.rm + em.alt
        rnh = em.rn + em.alt
        tan_lat = em.sin_lat / max(em.cos_lat, 1e-9)
        M = np.zeros((3, 3))
        M[0, 1] = 1.0 / rnh
        M[1, 0] = -1.0 / rmh
        M[2, 1] = -tan_lat / rnh
        return -self.Cnb.T @ M

    def grow_outage_cov(self, dt):
        """GNSS 失联协方差膨胀（对照固件 GrowGnssOutageCovariance）"""
        scale = min(self.time_since_gnss / 2.0, 4.0)
        pos_ne = (1.2**2) * dt * scale
        pos_d = (1.8**2) * dt * scale
        vel_ne = (0.35**2) * dt * scale
        vel_d = (0.50**2) * dt * scale
        for i in (0, 1):
            self.P[i, i] += pos_ne
        self.P[2, 2] += pos_d
        for i in (3, 4):
            self.P[i, i] += vel_ne
        self.P[5, 5] += vel_d
        self.P = self.stabilize(self.P)

    def snapshot(self):
        s = dict(lla=self.lla.copy(), vel=self.vel.copy(), quat=self.quat.copy(),
                 ba=self.ba.copy(), bg=self.bg.copy(), P=self.P.copy(),
                 prev_dtheta=self.prev_dtheta.copy(), prev_dv=self.prev_dv.copy(),
                 dt=self.dt)
        if hasattr(self, "_last_inc"):
            s["inc"] = self._last_inc
        return s

    def restore(self, s):
        self.lla = s["lla"].copy()
        self.vel = s["vel"].copy()
        self.quat = s["quat"].copy()
        self.ba = s["ba"].copy()
        self.bg = s["bg"].copy()
        self.P = s["P"].copy()
        self.prev_dtheta = s["prev_dtheta"].copy()
        self.prev_dv = s["prev_dv"].copy()
        self.Cnb = quat_to_dcm(self.quat)

    def stabilize(self, P):
        """对称化 + Gershgorin + LDLT/特征值投影（对照固件 StabilizeCovariance）"""
        P = 0.5 * (P + P.T)
        # Gershgorin 下界
        lb = np.inf
        for i in range(15):
            off = np.abs(P[i]).sum() - abs(P[i, i])
            lb = min(lb, P[i, i] - off)
        if lb >= -1e-5:
            return P
        # LDLT 检查（正定即返回）
        try:
            np.linalg.cholesky(P)
            return P
        except np.linalg.LinAlgError:
            pass
        # 特征值投影
        evals, evecs = np.linalg.eigh(P)
        floor = max(1e-9, evals.max() * 1e-10)
        evals = np.maximum(evals, floor)
        P = evecs @ np.diag(evals) @ evecs.T
        P = 0.5 * (P + P.T)
        return P

    # ---- GNSS 6 维量测更新（含延迟回放 + NIS + 重捕获）----
    def measurement_update_gnss(self, z_pos_ned, z_vel_ned, R_std,
                                age_s=0.0, hist=None):
        """z_pos_ned: 相对当前位置的 NED 位置观测(由外部 lla2ned 提供)。
        简化实现：直接以 NED 位置/速度残差做更新，延迟回放通过历史状态重放。"""
        # 恢复历史状态（延迟回放）
        restored = False
        best = -1
        if age_s > 0 and hist is not None:
            # 找最接近 age 的快照
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
        # 新息
        y = np.concatenate([z_pos_ned - self.ned_from_lla(self.lla, self.home_lla),
                            z_vel_ned - self.vel])
        H = np.zeros((6, 15))
        H[0:6, 0:6] = np.eye(6)
        R = np.diag([R_std[0]**2, R_std[0]**2, R_std[1]**2,
                     R_std[2]**2, R_std[2]**2, R_std[3]**2])
        S = H @ self.P @ H.T + R
        # NIS 门控（固件 GNSS_NIS_REJECT_THRESHOLD=30）
        nis = y @ np.linalg.solve(S, y)
        use_conservative = False
        if nis > 30.0:
            # 重捕获：残差物理合理时用膨胀 R 低增益
            if self.time_since_gnss >= 2.0 or self.gnss_reacquire_timer > 0:
                R2 = R.copy()
                R2[0, 0] += 2.0**2
                R2[1, 1] += 2.0**2
                R2[2, 2] += 4.0**2
                R2[3, 3] += 0.5**2
                R2[4, 4] += 0.5**2
                R2[5, 5] += 0.7**2
                S2 = H @ self.P @ H.T + R2
                nis2 = y @ np.linalg.solve(S2, y)
                if nis2 <= 80.0:
                    S = S2
                    R = R2
                    use_conservative = True
                else:
                    return {"fused": False, "nis": nis, "rejected": "nis"}
            else:
                return {"fused": False, "nis": nis, "rejected": "nis"}
        K = self.P @ H.T @ np.linalg.solve(S, np.eye(6))
        I15 = np.eye(15)
        self.P = (I15 - K @ H) @ self.P @ (I15 - K @ H).T + K @ R @ K.T
        self.P = self.stabilize(self.P)
        dx = K @ y
        self.inject(dx)
        # 重传播（延迟回放）：从历史时刻重放后续 IMU 增量到当前时刻
        if restored:
            for j in range(best + 1, len(hist)):
                inc = hist[j].get("inc")
                if inc is None:
                    continue
                dth1, dth2, dv1, dv2, ddt = inc
                self.time_update(dth1, dth2, dv1, dv2, ddt, record=False)
            self.time_since_gnss = 0.0
        self.time_since_gnss = 0.0
        self.gnss_reacquire_timer = 6.0 if use_conservative else 0.0
        return {"fused": True, "nis": nis, "conservative": use_conservative}

    def repropagate(self, hist, from_idx):
        """从历史点重放后续 IMU 输入（简化：直接记录当前状态为最终）"""
        pass

    def ned_from_lla(self, lla, home):
        """简化 WGS-84 lla→NED（局部小范围近似足够，用于仿真显示）"""
        dlat = lla[0] - home[0]
        dlon = lla[1] - home[1]
        em = EarthModel(home[0], home[2])
        n = dlat * (em.rm + home[2])
        e = dlon * (em.rn + home[2]) * math.cos(home[0])
        d = -(lla[2] - home[2])
        return np.array([n, e, d])

    # ---- ZUPT 速度量测 ----
    def measurement_update_velocity(self, z_vel, std_ne, std_d):
        H = np.zeros((3, 15))
        H[0:3, 3:6] = np.eye(3)
        R = np.diag([std_ne**2, std_ne**2, std_d**2])
        y = z_vel - self.vel
        # ★ 对齐固件 ApplyVelocityUpdateDetailed 物理残差门控：
        #   零速观测（|z|<0.30）时残差 >5.5 m/s 拒绝——防误静止确认把高速运动强拉为零；
        #   通用残差 >8.0 m/s 拒绝——数米每秒级残差交给 GNSS 主更新而非辅助接口。
        zero_vel_obs = np.linalg.norm(z_vel) < 0.30
        if zero_vel_obs and np.linalg.norm(y) > 5.5:
            return {"fused": False, "nis": float("nan"), "rejected": "zero_vel_residual"}
        if np.linalg.norm(y) > 8.0:
            return {"fused": False, "nis": float("nan"), "rejected": "velocity_residual"}
        S = H @ self.P @ H.T + R
        nis = y @ np.linalg.solve(S, y)
        if nis > 1000.0:
            return {"fused": False, "nis": nis}
        K = self.P @ H.T @ np.linalg.solve(S, np.eye(3))
        I15 = np.eye(15)
        self.P = (I15 - K @ H) @ self.P @ (I15 - K @ H).T + K @ R @ K.T
        self.P = self.stabilize(self.P)
        dx = K @ y
        self.inject(dx)
        return {"fused": True, "nis": nis}

    # ---- 重力方向量测 ----
    def measurement_update_gravity(self, accel_body, noise_rad):
        f = accel_body - self.ba
        if np.linalg.norm(f) < 1.0:
            return {"fused": False}
        z = f / np.linalg.norm(f)
        g_sf_ned = np.array([0.0, 0.0, -1.0])
        Cbn = quat_to_dcm(self.quat).T       # C^b_n
        pred = Cbn @ g_sf_ned
        y = z - pred
        H = np.zeros((3, 15))
        H[0:3, 6:9] = skew(pred)
        R = noise_rad**2 * np.eye(3)
        S = H @ self.P @ H.T + R
        nis = y @ np.linalg.solve(S, y)
        if nis > 18.0:
            return {"fused": False, "nis": nis}
        K = self.P @ H.T @ np.linalg.solve(S, np.eye(3))
        I15 = np.eye(15)
        self.P = (I15 - K @ H) @ self.P @ (I15 - K @ H).T + K @ R @ K.T
        self.P = self.stabilize(self.P)
        dx = K @ y
        self.inject(dx)
        return {"fused": True, "nis": nis}

    # ---- 静止陀螺零偏量测 ----
    def measurement_update_static_gyro(self, gyro_body, noise):
        nav_rate = self.earth.omega_ned + self.earth.nav_rate(self.vel)
        expected = self.Cnb.T @ nav_rate
        pred = expected + self.bg
        y = gyro_body - pred
        H = np.zeros((3, 15))
        H[0:3, 12:15] = np.eye(3)
        R = np.diag(noise**2)
        S = H @ self.P @ H.T + R
        nis = y @ np.linalg.solve(S, y)
        if nis > 24.0:
            return {"fused": False, "nis": nis}
        # 仅更新零偏子块（对照固件 2989-2998 行）
        Pbg = self.P[12:15, 12:15]
        Kbg = Pbg @ np.linalg.solve(S, np.eye(3))
        self.bg = self.bg + Kbg @ y
        Pbg_new = (np.eye(3) - Kbg) @ Pbg @ (np.eye(3) - Kbg).T + Kbg @ R @ Kbg.T
        self.P[0:12, 12:15] = self.P[0:12, 12:15] @ (np.eye(3) - Kbg).T
        self.P[12:15, 0:12] = self.P[0:12, 12:15].T
        self.P[12:15, 12:15] = Pbg_new
        self.P = self.stabilize(self.P)
        return {"fused": True, "nis": nis}

    # ---- 标量航向量测 ----
    def measurement_update_yaw(self, z_yaw, noise_rad):
        yaw = quat_to_euler(self.quat)[0]
        y = wrap_pi(z_yaw - yaw)
        if abs(y) > 1.0:
            return {"fused": False}
        H = np.zeros((1, 15))
        H[0, 8] = 1.0
        R = np.array([[noise_rad**2]])
        S = self.P[8, 8] + R[0, 0]
        nis = y * y / S
        if nis > 12.0:
            return {"fused": False, "nis": nis}
        K = self.P @ H.T / S
        I15 = np.eye(15)
        self.P = (I15 - K @ H) @ self.P @ (I15 - K @ H).T + K @ R @ K.T
        self.P = self.stabilize(self.P)
        dx = K.ravel() * y
        self.inject(dx)
        return {"fused": True, "nis": nis}

    # ---- 误差状态注入 ----
    def inject(self, dx):
        em = EarthModel(self.lla[0], self.lla[2])
        dlat = dx[0] / (em.rm + self.lla[2])
        dlon = dx[1] / ((em.rn + self.lla[2]) * max(math.cos(self.lla[0]), 1e-9))
        self.lla[0] += dlat
        self.lla[1] += dlon
        self.lla[2] -= dx[2]
        self.vel += dx[3:6]
        dq = rotvec_to_quat(dx[6:9])
        self.quat = quat_mult(self.quat, dq)
        self.quat /= np.linalg.norm(self.quat)
        if self.quat[0] < 0:
            self.quat = -self.quat
        self.Cnb = quat_to_dcm(self.quat)
        self.ba += dx[9:12]
        self.bg += dx[12:15]


# ---------------------------------------------------------------------------
# 配置（固件实机参数，navigation_task.cpp 初始化分支）
# ---------------------------------------------------------------------------
cfg = {
    "dt": 0.005,
    "accel_std": 0.25,
    "accel_bias_std": 0.01,
    "accel_tau": 100.0,
    "gyro_std": 0.0015,
    "gyro_bias_std": 0.00524,
    "gyro_tau": 180.0,
}
cfg_p0 = {
    "pos": 2.0, "vel": 0.08, "att": 0.08,
    "hdg": math.pi, "ba": 0.196, "bg": 0.00698,
}
# 静止辅助分频（200Hz）
ZUPT_DIV = 4      # 50 Hz
GRAV_DIV = 8      # 25 Hz
SG_DIV = 8        # 25 Hz
HOME_LLA = np.array([28.2 * D2R, 112.9 * D2R, 50.0])

OUT = os.path.join(os.path.dirname(__file__), "..", "sim-data")
FIG = os.path.join(os.path.dirname(__file__), "..", "fig")
os.makedirs(OUT, exist_ok=True)
os.makedirs(FIG, exist_ok=True)
