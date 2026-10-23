# -*- coding: utf-8 -*-
"""ekf_core.py — 15 状态误差状态 EKF 数值仿真核心（逐项对照固件语义）。

@file    ekf_core.py
@module  analysis/EKF 研究仿真（组合导航滤波器）
@brief   按固件 ekf_15_state.h / navigation_task.cpp 语义复现 15 态 ESKF。
@details 输入：双子样 IMU 增量 (dθ1,dθ2,dv1,dv2)、GNSS 位置/速度、静止辅助量测；
         处理：扣零偏 → 圆锥/划摇补偿 → 中点姿态旋转 → 一阶+二阶 Φ 离散 → P 传播
               → Joseph 量测更新 + NIS 门控 → 误差状态注入；
         输出：状态 [lla, v_ned, q, ba, bg]、协方差 P 与每次融合的 NIS/接受标志。
         误差姿态为体轴右乘：q = q_nom ⊗ Exp(δβ^b)，δβ 为导航系分量近似（φ≈δβ）。
@theory  比力方程与速度更新 @ref REF-120 §7.4 (7.4.6) p.189、§9.6 (9.6.9) p.298；
         等效旋转矢量与圆锥补偿 (9.3.30) p.274 @ref REF-120 §9.3；
         划摇补偿 (9.6.25) p.301 @ref REF-120 §9.6；
         地球模型与正常重力 (7.2.17)(7.2.18) @ref REF-120 §7.2.4 p.178；
         位置速率方程 @ref REF-120 §8.1；
         KF 五方程与 P 递推 @ref REF-121 §2.2.1 (2.2.4a–e) p.33；
         Joseph 形式 @ref REF-121 §2.2.4 p.42–43；
         残差 χ²(NIS) 门控 @ref REF-121 §7.3.1 p.255–259；
         延迟回放＝局部重处理而非 RTS 平滑 @ref REF-121 §6.1.3 p.194。
@frame  NED 导航系 + FRD 机体系；四元数 (w,x,y,z)。误差状态下标 6-8 为失准角。
@units   SI：位置 m（lla 为 rad/rad/m）、速度 m/s、角度 rad、加计零偏 m/s²、陀螺零偏 rad/s。
@note    本文件是**研究仿真**，与固件同构但不等同（口径差异单列在核查记录里）。
         固件取值照抄于此（见 CONFIG 常量与 cfg/cfg_p0）：
           - IMU：gyro_std=0.0015 rad/s、gyro_markov_bias_std=0.00524 rad/s、gyro_tau=180 s、
             accel_std=0.25 m/s²、accel_tau=100 s（加计零偏游走 std=0.01）
           - GNSS R 地板：pos_ne 2 m / pos_d 3 m / vel_ne 0.15 / vel_d 0.25（不可收紧）
           - NIS 门限：GNSS=30、ZUPT=1000、Gravity=18、StaticGyro=24、Yaw=12
           - 静止辅助分频（200 Hz 基）：ZUPT 每 4 帧、Gravity 每 8 帧、StaticGyro 每 8 帧
           - 传播：双子样圆锥/划摇、中点重力、地球自转+传输速率、二阶 Φ、Simpson Qd
           - 更新：Joseph 协方差、NIS 门控、GNSS 重捕获膨胀 R
         P0 航向方差已按固件 2026-10-21 修复取 0.2 rad（cfg_p0["hdg"]）；
         若要复现旧失效链（航向 π），必须由调用方显式覆盖该项，不要改默认值。
         静止检测/光流重锚/子集 FDI 仅在 opt/upgrade_candidates.py 一侧实现。
@see     run_scenarios.py、verify.py、hifi/hifi_core.py、opt/opt_ekf.py、
         @ref TandemVec-EKF-Report/优化研究-算法与参数优化研究报告.md
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
    """反对称矩阵 [v]×（叉乘算子）。

    @brief   构造 3×3 反对称阵，使 [v]×·u = v×u。
    @param[in] v 三维矢量（角速度/位置/比力等）
    @return  3×3 反对称矩阵
    @formula [v]× = [[0,−v3,v2],[v3,0,−v1],[−v2,v1,0]]
    @ref     REF-120 §9.6 p.298（误差方程中的反对称阵形式）、@ref THY-001 §2
    @note    F(v,β)=−C^n_b·[f_b]× 与 F(β,v) 由该算子生成，符号翻转敏感，
             改符号会直接翻转姿态-速度耦合方向。
    """
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
    """四元数共轭 (w,−x,−y,−z)，单位四元数下即逆。

    @brief   返回 q*，用于 C^n_b = C^b_n^T 的装配与姿态取逆。
    @param[in] q 四元数 (w,x,y,z)
    @return  共轭四元数
    @ref     MOD-002 §3.3
    """
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
    """把角度折算到 (−π, π]。

    @brief   航向/方位角缠绕保护，避免 ±π 跳变造成虚假大残差。
    @param[in] a 角度 [rad]
    @return  等价角度 [rad]，区间 (−π, π]
    @note    航向量测残差 y=wrap_pi(z−yaw) 必须先缠绕再算 NIS，
             否则 179°/−179° 边界会误判为野值。
    """
    a = math.fmod(a + math.pi, 2 * math.pi)
    if a < 0:
        a += 2 * math.pi
    return a - math.pi


def two_sample_delta_v(dtheta1, dtheta2, dv1, dv2):
    """双子样速度增量（与固件 2026-10 P0-0 修复后同构）。

    @brief   合并两个子样的速度增量并加划摇补偿。
    @param[in] dtheta1/dtheta2 两个等分子样的角增量 [rad]
    @param[in] dv1/dv2         两个等分子样的速度增量 [m/s]
    @return  机体系等效速度增量 [m/s]
    @formula ΔV = ΔV1+ΔV2 + (2/3)·(Δθ1×ΔV2 + ΔV1×Δθ2)
    @ref     REF-120 §9.6 (9.6.25) p.301（划摇补偿，注意 ΔV×Δθ = −Δθ×ΔV）
    @note    ★ 不含 ½Δθ×ΔV。该项只在用**区间起始姿态**旋转时需要；本实现用
             **中点姿态** Cnb_mid 旋转，半程旋转已包含在内，再加会双计
             （固件 P0-0，2026-10-21；本仿真 A-01，2026-10-23 对齐）。
    """
    dtheta = dtheta1 + dtheta2
    dv = dv1 + dv2
    return dv + (2.0 / 3.0) * (
        np.cross(dtheta1, dv2) + np.cross(dv1, dtheta2)
    )


# ---------------------------------------------------------------------------
# 地球模型（与固件 ComputeEarthModelTerms 一致）
# ---------------------------------------------------------------------------
class EarthModel:
    """WGS-84 地球模型量（与固件 ComputeEarthModelTerms 同构）。

    提供子午圈/卯酉圈曲率半径 rm/rn、正常重力 g、地球自转在 NED 的投影
    ω_ie^n，以及导航系相对地球的传输速率 ω_en^n 与经纬高变化率。

    @formula rn = a/√(1−e²sin²L)，rm = a(1−e²)/(1−e²sin²L)^{3/2}
             g0 = 9.780318(1+5.3024e-3 sin²L−5.898e-6 sin²2L)，g = g0/(1+h/√(rm·rn))²
             ω_ie^n = [ΩcosL, 0, −ΩsinL]；ω_en^n = [V_E/(rn+h), −V_N/(rm+h), −V_E·tanL/(rn+h)]
             ḣ = −V_D
    @ref     REF-120 §7.2.4 (7.2.17)(7.2.18) p.178（正常重力）、§8.1（位置速率）
    @note    alt 用**椭球高**近似；重力用 Somigliana 展开式而非完整级数
             （@deviation 与教材 (7.2.17) 相差 ~1e-5 m/s² 量级，随时间累积极小）。
    """

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
    """15 状态误差状态卡尔曼滤波器（固件语义复现）。

    状态：x = [δp(3), δv(3), δβ(3), δba(3), δbg(3)]，误差为“估计−真值”的
    机体系右乘失准角；名义状态为 [lla, v_ned, q, ba, bg]。

    主要接口：
      initialize()                    静基座调平初始化 + P0
      time_update()                   双子样传播 + Φ/Qd + 协方差传播
      measurement_update_gnss()       6 维位置/速度融合（NIS + 重捕获 + 延迟回放）
      measurement_update_velocity()   ZUPT/速度辅助（含物理残差门控）
      measurement_update_gravity()    重力方向（约束 roll/pitch）
      measurement_update_static_gyro() 静止陀螺零偏（仅零偏子块）
      measurement_update_yaw()        标量航向
      inject()                        误差状态注入名义状态并清零

    @note  R 为对角阵（固件 UpdateGnssNoiseMatrix 只填对角），故这里不做序贯分解；
           6 维联合 NIS 会把“速度坏”判成“整帧坏”，是 F1/F3 失效链的成因
           （见 @ref docs/08-参考资料/notes/02-卡尔曼滤波理论.md §2、§3 P0-1）。
    @test   verify.py V1–V4（F 矩阵数值微分、NIS 分布、协方差一致性、双子样一致性）
    """

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
        # ---- 可调门限/系数（默认=固件现值，行为保持；opt 研究用于扫描）----
        self.nis_gnss = cfg.get("nis_gnss", 30.0)
        self.nis_zupt = cfg.get("nis_zupt", 1000.0)
        self.nis_grav = cfg.get("nis_grav", 18.0)
        self.nis_sg = cfg.get("nis_sg", 24.0)
        self.nis_yaw = cfg.get("nis_yaw", 12.0)
        self.zupt_res_rej = cfg.get("zupt_res_rej", 5.5)
        self.vel_res_rej = cfg.get("vel_res_rej", 8.0)
        self.reacq_r_infl = cfg.get("reacq_r_infl",
                                     [4.0, 4.0, 16.0, 0.25, 0.25, 0.49])
        self.reacq_nis2 = cfg.get("reacq_nis2", 80.0)
        self.outage_start = cfg.get("outage_start", 2.0)
        self.outage_cap = cfg.get("outage_cap", 4.0)
        self.outage_rate = cfg.get("outage_rate",
                                   [1.44, 1.44, 3.24, 0.1225, 0.1225, 0.25])

    # ---- 初始化 ----
    def initialize(self, accel, gyro, lla, vel=None):
        """静基座调平初始化 + P0 装配。

        @param[in] accel 静止时加计比力 [m/s²]（机体系）
        @param[in] gyro  静止时陀螺读数 [rad/s]（机体系，含零偏）
        @param[in] lla   [lat, lon, alt]（rad, rad, m）
        @param[in] vel   初始速度 [m/s]（None 取 0）
        @formula 用最小旋转把 NED 的 (0,0,1) 对齐到 −f_n，得 C^b_n；零偏初值
                 bg = gyro − C^b_nᵀ·(ω_ie^n+ω_en^n)
        @ref     REF-120 §7.4 p.189（比力与重力关系）、§8.1（对准）
        @note    ★ P0 直接取自模块级全局 cfg_p0（未走 self.cfg），因此外部改 cfg
                 不会改变 P0；opt/opt_ekf.py::build_p0 通过覆盖 cfg_p0 生效。
                 ★ 2026-10-23（A-03）修正：本函数把 `self.home_lla` 初始化为
                 传入 lla 的副本，`measurement_update_gnss()` 可直接使用；外部
                 run_*.py 若需不同起飞点，仍可在 initialize 后覆盖。
        """
        if vel is None:
            vel = np.zeros(3)
        self.lla = np.array(lla, dtype=float)
        # 起飞点 = 初始 LLA（GNSS 位置观测的 NED 参考；外部可覆盖）
        self.home_lla = self.lla.copy()
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
        """双子样时间更新：补偿 → 传播 → Φ/Qd → 协方差。

        @param[in] dtheta1/2 两个等分子样的角增量 [rad]
        @param[in] dv1/dv2   两个等分子样的速度增量 [m/s]
        @param[in] dt        导航周期 [s]（固件 200 Hz 基 = 0.005 s）
        @param[in] record    是否写延迟回放快照
        @formula 圆锥 Δθ=Δθ1+Δθ2+(2/3)Δθ1×Δθ2
                 划摇 ΔV=ΔV1+ΔV2+(2/3)(Δθ1×ΔV2+ΔV1×Δθ2)
                 ΔV^n = C^n_b(mid)·ΔV + dt·(g^n − (2ω_ie+ω_en)×V^n)
        @ref     REF-120 §9.3 (9.3.30) p.274、§9.6.1 (9.6.9) p.298、§9.6.2 (9.6.25) p.301
        @deviation 无：2026-10-23（A-01）已删除旧实现的 `½Δθ×ΔV`。该旋转项只
                 配合**区间起始姿态**使用；本实现用**中点姿态** `Cnb_mid` 旋转，
                 半程旋转已包含在内，重复相加会多算 ~1e-2–5e-2 m/s²（与固件
                 P0-0 同源）。双子样增量统一由 `two_sample_delta_v()` 计算。
        @note    协方差用二阶 Φ + Simpson Qd（与固件飞行构建的 ZOH Qd 不同，量级 1%）。
        """
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
        scull = two_sample_delta_v(dtheta1, dtheta2, dv1, dv2)
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
        # 可选：切断航向-速度耦合（航向由双矢量/磁量测负责，
        # 弱可观耦合在 VTOL 过渡期会把 GNSS 速度噪声放大为航向踢动）
        if self.cfg.get("decouple_yaw_vel", False):
            F[3:6, 8] = 0.0
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
        if self.time_since_gnss >= self.outage_start:
            self.grow_outage_cov(dt)
        # 重捕获窗口计时（固件 TimeUpdate 内递减）
        self.gnss_reacquire_timer = max(0.0, self.gnss_reacquire_timer - dt)
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
        scale = min(self.time_since_gnss / 2.0, self.outage_cap)
        pos_ne = self.outage_rate[0] * dt * scale
        pos_d = self.outage_rate[2] * dt * scale
        vel_ne = self.outage_rate[3] * dt * scale
        vel_d = self.outage_rate[5] * dt * scale
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
        if nis > self.nis_gnss:
            # 重捕获：残差物理合理时用膨胀 R 低增益
            if self.time_since_gnss >= self.outage_start or self.gnss_reacquire_timer > 0:
                R2 = R.copy()
                R2[0, 0] += self.reacq_r_infl[0]
                R2[1, 1] += self.reacq_r_infl[1]
                R2[2, 2] += self.reacq_r_infl[2]
                R2[3, 3] += self.reacq_r_infl[3]
                R2[4, 4] += self.reacq_r_infl[4]
                R2[5, 5] += self.reacq_r_infl[5]
                S2 = H @ self.P @ H.T + R2
                nis2 = y @ np.linalg.solve(S2, y)
                if nis2 <= self.reacq_nis2:
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
        if zero_vel_obs and np.linalg.norm(y) > self.zupt_res_rej:
            return {"fused": False, "nis": float("nan"), "rejected": "zero_vel_residual"}
        if np.linalg.norm(y) > self.vel_res_rej:
            return {"fused": False, "nis": float("nan"), "rejected": "velocity_residual"}
        S = H @ self.P @ H.T + R
        nis = y @ np.linalg.solve(S, y)
        if nis > self.nis_zupt:
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
        if nis > self.nis_grav:
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
        if nis > self.nis_sg:
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
        if nis > self.nis_yaw:
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
    # 航向方差：0.2 rad = 固件 2026-10-21 修复后的值（旧 π 仅用于复现历史失效链；
    # 需要时由调用方显式覆盖，不要改回默认值）。
    "hdg": 0.2, "ba": 0.196, "bg": 0.00698,
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
