# -*- coding: utf-8 -*-
"""
高保真飞控固件闭环仿真 —— 核心框架
====================================
纵列双发矢量推力飞行器固件级全链路仿真：

  6DOF 动力学(真实物理) → 传感器模型(IMU/GPS/光流/气压, 含噪声/震动/零偏漂移/故障)
    → EKF 导航(修复后 15 态误差状态滤波 + 静止辅助 + GNSS 融合 + 双矢量航向)
    → 固件控制律(误差四元数外环 + 角速率内环 + 惯量逆解 + B 矩阵分配 + 差速)
    → 执行器(摆座伺服动态 + 电机 τm 一阶滞后 + 限幅) → 6DOF 动力学(闭环)

物理参数逐项取自固件 TandemVec_Config.h / FlightCtrlParams.h（唯一事实源）。
控制律语义逐行复刻固件 flight_control.cpp（误差四元数外环、角速率内环、
惯量逆解、BTRUE 工作点分配、差速 √(1+Δω) 公式、τm 观测器）。
"""
import math

import numpy as np

# =====================================================================
# 固件物理参数（TandemVec_Config.h kDefaultTandemVecParams）
# =====================================================================
P = {
    "kT": 1.04e-5,      # N·s² 推力系数
    "kQ": 2.8e-7,       # N·m·s² 反扭系数
    "Jp": 2.0e-4,       # kg·m² 桨+转子惯量
    "wMax": 1150.0,     # rad/s 最大转速
    "a": 0.315,         # m 前电机力臂
    "b": 0.315,         # m 尾电机力臂
    "tauM": 0.28,       # s 电机一阶滞后
    "Ix": 0.0021,       # kg·m²
    "Iy": 0.022,        # kg·m²
    "Iz": 0.022,        # kg·m²
    "dMax": 0.2618,     # rad (15°) 摆角限幅
    "dwMax": 0.7,       # 差速限幅
    "m": 0.7,           # kg
    "g": 9.79,          # m/s²
}

# 悬停转速
w_hover = math.sqrt(0.5 * P["m"] * P["g"] / P["kT"])
P["w_hover"] = w_hover          # ≈574 rad/s
P["w0_floor"] = 0.6 * w_hover   # ≈344 rad/s 分配器工作点下限

# =====================================================================
# 固件控制参数（FlightCtrlParams.h kFlightCtrlParamsDefaults）
# =====================================================================
CTRL = {
    # 姿态外环 kp (deg 域)
    "att_kp": 2.8,
    "att_kp_yaw": 5.0,
    "att_out_max": 600.0,       # kMaxTargetRate (deg/s)
    # 角速率内环 (deg/s → deg/s²)
    "rate_kp_rp": 16.042818,
    "rate_ki_rp": 5.729578,
    "rate_kp_yaw": 11.459156,
    "rate_ki_yaw": 4.297184,
    "rate_out_max": 5729.578,   # deg/s²
    "rate_int_max": 1145.9156,
    "rate_int_sep": 60.0,       # 积分分离阈值 deg/s
    # 滤波
    "speed_alpha": 0.4,
    "speed_alpha2": 0.99,
    "angle_out_alpha": 0.85,
    "output_alpha": 0.9,
    # 垂直串级
    "alt_pos_kp": 1.0,
    "alt_vel_kp": 5.0,
    "alt_vel_ki": 1.25,
    "alt_vel_out_max": 12.5,    # m/s²(油门百分制 ×2.6)
    # 惯量逆解
    "inertia_mask": 0x03,
}

R2D = 180.0 / math.pi
D2R = math.pi / 180.0


# =====================================================================
# 四元数工具（与 ekf_core 一致）
# =====================================================================
def quat_mult(q, p):
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
    """四元数 -> 机体系到导航系 C^b_n（行=导航, 列=机体）"""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]])


def rotvec_to_quat(theta):
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


def quat_from_euler(yaw, pitch, roll):
    """ZYX (yaw, pitch, roll) -> quat"""
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    return np.array([
        cy*cp*cr + sy*sp*sr,
        cy*cp*sr - sy*sp*cr,
        cy*sp*cr + sy*cp*sr,
        sy*cp*cr - cy*sp*sr])


def quat_to_euler(q):
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


# =====================================================================
# 六自由度动力学（真实物理，RK2 足够稳定；子步 0.004s 符合行为红线）
# =====================================================================
class Dynamics:
    """真实刚体动力学：机体坐标，含推力矢量/反扭/转子陀螺/重力"""

    def __init__(self, seed=1):
        self.rng = np.random.default_rng(seed)
        self.pos = np.zeros(3)            # NED
        self.vel = np.zeros(3)
        self.quat = np.array([1.0, 0, 0, 0])  # 悬停：机头朝天 q_hover
        self.omega = np.zeros(3)          # 机体系角速度
        # 执行器状态
        self.wf = w_hover                 # 前电机转速
        self.wt = w_hover
        self.df = 0.0                     # 前摆(上摆/roll)
        self.dt = 0.0                     # 尾摆(下摆/pitch)
        self.thr = 0.5                    # 油门 0-1
        self.w_est = w_hover              # τm 观测器
        self.gyro_bias_true = np.array([0.003, -0.002, 0.004])  # 真实零偏漂移
        self.accel_bias_true = np.array([0.05, -0.03, 0.08])
        self.disturb = np.zeros(3)        # 外部扰动（阵风/推力扰动）
        self._prev_vel = np.zeros(3)
        self._dt_prev = 0.005
        self._prev_wf_act = w_hover
        self._prev_wt_act = w_hover

    def step(self, act, dt):
        """act: (df_cmd, dt_cmd, dw_cmd, thr_cmd, w0_cmd)——执行器指令"""
        # ---- 执行器动态 ----
        # 摆座伺服一阶（τs=0.05s）+ 限幅
        tau_s = 0.05
        k = min(dt / tau_s, 1.0)
        self.df += k * (act["df"] - self.df)
        self.dt += k * (act["dt"] - self.dt)
        self.df = max(-P["dMax"], min(P["dMax"], self.df))
        self.dt = max(-P["dMax"], min(P["dMax"], self.dt))
        # 电机 τm 一阶
        km = min(dt / P["tauM"], 1.0)
        self.wf += km * (act["wf"] - self.wf)
        self.wt += km * (act["wt"] - self.wt)
        self.wf = max(0.0, min(P["wMax"], self.wf))
        self.wt = max(0.0, min(P["wMax"], self.wt))

        # ---- 推力/力矩（机体系）----
        Tf = P["kT"] * self.wf**2
        Tt = P["kT"] * self.wt**2
        # ★ 实际转速差分用动力学自身状态（上一拍实际值），不用控制器的指令 prev——
        #   指令/实际时序错位会产生虚假巨大 dWf，Jp·dW 反扭瞬态项爆炸（仿真发散根因）。
        dWf = (self.wf - self._prev_wf_act) / max(dt, 1e-4)
        dWt = (self.wt - self._prev_wt_act) / max(dt, 1e-4)
        self._prev_wf_act = self.wf
        self._prev_wt_act = self.wt
        Qf = P["kQ"] * self.wf**2 + P["Jp"] * dWf
        Qt = P["kQ"] * self.wt**2 + P["Jp"] * dWt
        cf, sf = math.cos(self.df), math.sin(self.df)
        ct, st = math.cos(self.dt), math.sin(self.dt)
        # 悬停构型机体系：x_b 沿机体轴（推力轴），机头朝天
        # 俯仰力矩 My（下摆）、侧倾力矩 Mz'（上摆）、滚转力矩 Mx'（差速）
        # 与固件 mix 一致：上摆=roll 主控(Mz')、下摆=pitch 主控(My)、差速=yaw 主控(Mx')
        F_body = np.array([Tf*cf + Tt*ct, Tf*sf, -Tt*st])  # x=推力轴
        M_body = np.array([
            -Qf*cf + Qt*ct,                    # Mx：反扭差 → 差速 yaw
            -P["b"]*Tt*st - Qf*sf,             # My：尾摆 → pitch
            P["a"]*Tf*sf - Qt*st               # Mz：前摆 → roll
        ])
        # 转子角动量（陀螺耦合）
        hv = np.array([
            P["Jp"]*(self.wf*cf - self.wt*ct),
            P["Jp"]*self.wf*sf,
            P["Jp"]*self.wt*st
        ])

        # ---- 刚体动力学（机体坐标）----
        I = np.diag([P["Ix"], P["Iy"], P["Iz"]])
        I_inv = np.diag([1/P["Ix"], 1/P["Iy"], 1/P["Iz"]])
        # 重力在机体系
        g_body = quat_to_dcm(self.quat).T @ np.array([0, 0, P["g"]])
        # 平动（含外部扰动）
        a_body = F_body / P["m"] + g_body - np.cross(self.omega, self.vel) + self.disturb
        # 转动：I·ω̇ = M - ω×(I·ω) - ω×h
        domega = I_inv @ (M_body - np.cross(self.omega, I @ self.omega)
                          - np.cross(self.omega, hv))
        # 姿态：q̇ = ½ q ⊗ [0, ω]
        dq = 0.5 * quat_mult(self.quat, np.array([0, *self.omega]))

        # RK2 半步
        dt2 = dt / 2
        self.vel += a_body * dt
        self.omega += domega * dt
        self.quat = quat_mult(self.quat, rotvec_to_quat(self.omega * dt))
        self.quat /= np.linalg.norm(self.quat)
        # NED 位置
        v_ned = quat_to_dcm(self.quat) @ self.vel
        self.pos += v_ned * dt

        return Tf, Tt, M_body

    # ---- 传感器输出（含噪声/震动/零偏漂移/故障注入）----
    def imu_measure(self, t, fault=None):
        """返回原始 IMU 读数（机体系比力 + 角速度）"""
        # ★ 比力 = -(重力在机体) + 运动加速度在机体（加速度计测比力=反重力）
        #   f_b = -C_n^b @ g^n + C_n^b @ a_ned
        # 静止悬停（x 指上）：f_b = (g,0,0)；水平静止：f_b = (0,0,-g)
        # 运动加速度（NED）转机体
        a_ned_motion = (self.vel - self._prev_vel) / max(getattr(self, "_dt_prev", 0.005), 1e-4)
        self._prev_vel = self.vel.copy()
        self._dt_prev = 0.005
        f_b = -quat_to_dcm(self.quat).T @ np.array([0, 0, P["g"]])             + quat_to_dcm(self.quat).T @ a_ned_motion
        # 震动（共轴双桨 80Hz）
        vib = 0.05 * np.sin(2 * np.pi * 80 * t) * np.array([1, 1, 1])
        accel = f_b + self.accel_bias_true + self.rng.normal(0, 0.05, 3) + vib * 0.3
        gyro = self.omega + self.gyro_bias_true + self.rng.normal(0, 0.0015, 3)
        # 故障注入
        if fault:
            if "accel_bias_jump" in fault and t > fault["accel_bias_jump_t"]:
                accel = accel + np.asarray(fault["accel_bias_jump"], dtype=float)
            if "gyro_bias_jump" in fault and t > fault["gyro_bias_jump_t"]:
                gyro = gyro + np.asarray(fault["gyro_bias_jump"], dtype=float)
            if "gyro_sat" in fault and t > fault["gyro_sat_t"]:
                gyro = np.clip(gyro, -fault["gyro_sat"], fault["gyro_sat"])
        return accel, gyro

    def gps_measure(self, t, fault=None):
        """GPS NED 位置/速度（10Hz），含噪声/丢星/延迟/野值"""
        pos_ned = self.pos.copy()
        vel_ned = quat_to_dcm(self.quat) @ self.vel
        if fault:
            if fault.get("gps_lost") and fault["gps_lost"][0] < t < fault["gps_lost"][1]:
                return None
            if fault.get("gps_jump") and t > fault["gps_jump_t"]:
                pos_ned += fault["gps_jump"]
        pos_ned += self.rng.normal(0, 1.5, 3)
        vel_ned += self.rng.normal(0, 0.2, 3)
        return pos_ned, vel_ned

    def baro_measure(self, t):
        """气压高度（向上为正）"""
        return -self.pos[2] + self.rng.normal(0, 0.5)


# =====================================================================
# 固件控制律（复刻 flight_control.cpp）
# =====================================================================
class FirmwareControl:
    """误差四元数姿态外环 + 角速率内环 + 惯量逆解 + B 矩阵分配 + 差速"""

    def __init__(self):
        # 滤波器状态
        self.rate_filt = np.zeros(3)
        self.rate_filt2 = np.zeros(3)
        self.att_out_filt = np.zeros(2)
        self.out_filt = np.zeros(3)
        self.rate_int = np.zeros(3)
        self.prev_w0 = w_hover
        self.prev_wf = w_hover
        self.prev_wt = w_hover

    def _lowpass(self, prev, x, alpha):
        return alpha * x + (1 - alpha) * prev

    def step(self, nav, ref, dt):
        """
        nav: {quat, omega_body, gyro_bias, alt, vel_up, pos}
        ref: {q_target, thr_cmd, alt_hold, mode}
        """
        # ---- 角速率测量（含 EKF 零偏补偿，复刻固件）----
        omega_raw = nav["omega_body"].copy()
        if nav.get("gyro_bias") is not None:
            omega_raw = omega_raw - nav["gyro_bias"]
        self.rate_filt = self._lowpass(self.rate_filt, omega_raw * R2D, CTRL["speed_alpha"])
        self.rate_filt2 = self._lowpass(self.rate_filt2, self.rate_filt, CTRL["speed_alpha2"])
        omega_dps = self.rate_filt2

        # ---- 姿态外环：误差四元数 -> 目标角速度 ----
        q_cur = nav["quat"]
        q_err = quat_mult(quat_conj(q_cur), ref["q_target"])
        if q_err[0] < 0:
            q_err = -q_err
        # 体轴误差（deg）：roll=q_err.x, pitch=q_err.y（悬停构型轴置换后）
        # 固件：roll 取 x、pitch 取 y；yaw 由航向通道处理
        err_deg = np.array([q_err[1], q_err[2], q_err[3]]) * 2 * R2D
        # 外环 P：目标角速度（yaw 用 att_yaw kp，无积分）
        omega_ref = np.array([
            CTRL["att_kp"] * err_deg[0],
            CTRL["att_kp"] * err_deg[1],
            CTRL["att_kp_yaw"] * err_deg[2],
        ])
        # 航向参考：若启用航向锁，目标 yaw 由 ref 给出（简化：用 ref yaw 误差）
        if ref.get("yaw_lock"):
            dy = wrap_pi(ref["yaw_lock"] - quat_to_euler(q_cur)[0])
            omega_ref[2] = CTRL["att_kp_yaw"] * dy * R2D
        omega_ref = np.clip(omega_ref, -CTRL["att_out_max"], CTRL["att_out_max"])
        self.att_out_filt = np.array([
            self._lowpass(self.att_out_filt[0], omega_ref[0], CTRL["angle_out_alpha"]),
            self._lowpass(self.att_out_filt[1], omega_ref[1], CTRL["angle_out_alpha"]),
        ])
        omega_ref[0], omega_ref[1] = self.att_out_filt

        # ---- 角速率内环：ω 误差 -> 角加速度指令（deg/s²）----
        # 积分分离：|误差| < 60 deg/s 才积分
        alpha_cmd = np.zeros(3)
        for i in range(3):
            kp = CTRL["rate_kp_rp"] if i < 2 else CTRL["rate_kp_yaw"]
            ki = CTRL["rate_ki_rp"] if i < 2 else CTRL["rate_ki_yaw"]
            err_rate = omega_ref[i] - omega_dps[i]
            if abs(err_rate) < CTRL["rate_int_sep"]:
                self.rate_int[i] += ki * err_rate * dt
                self.rate_int[i] = max(-CTRL["rate_int_max"], min(CTRL["rate_int_max"], self.rate_int[i]))
            else:
                self.rate_int[i] = 0
            alpha_cmd[i] = kp * err_rate + self.rate_int[i]
        alpha_cmd = np.clip(alpha_cmd, -CTRL["rate_out_max"], CTRL["rate_out_max"])
        # 内环输出滤波
        self.out_filt = np.array([self._lowpass(self.out_filt[i], alpha_cmd[i], CTRL["output_alpha"])
                                  for i in range(3)])
        alpha_cmd = self.out_filt

        # ---- 惯量逆解 -> 力矩指令（rad/s² → N·m）----
        # ★ 模型系自洽闭环（轴阻尼测试验证三轴收敛）：
        #   本仿真采用模型系（x'=推力轴朝天、y'=下摆、z'=上摆），
        #   动力学 M_body、EKF 姿态、控制 M 全部在同一参考系，
        #   M = +I·alpha（无需固件的档案系↔模型系轴置换）。
        M_cmd = np.array([
            P["Ix"] * alpha_cmd[0] * D2R,   # Mx'(差速)
            P["Iy"] * alpha_cmd[1] * D2R,   # My'(下摆)
            P["Iz"] * alpha_cmd[2] * D2R,   # Mz'(上摆)
        ])
        # 陀螺耦合前馈（inertia_comp_mask & 0x03，悬停 ω≈0 时≈0）
        if CTRL["inertia_mask"] & 0x01:
            I = np.diag([P["Ix"], P["Iy"], P["Iz"]])
            M_cmd += np.cross(omega_raw, I @ omega_raw)
        if CTRL["inertia_mask"] & 0x02:
            hv = np.array([P["Jp"]*(self.prev_wf - self.prev_wt), 0.0, 0.0])
            M_cmd += np.cross(omega_raw, hv)

        # ---- B 矩阵分配（工作点 w0）----
        # 油门：垂直控制（定高模式）或手动
        if ref.get("alt_hold"):
            # 垂直串级（★ 对齐固件语义：alt_vel 输出 = 油门百分比，限幅 ±12.5）
            #   alt_pos: 高度误差 -> 垂直速度指令（±1.5 m/s）
            #   alt_vel: 垂直速度误差 -> 油门百分比增量（叠加悬停 50%）
            # 50% 油门 = w0=575 = w_hover → 双机推力 6.88N ≈ m·g（物理平衡已验证）
            alt_err = ref["alt_hold"] - nav["alt_up"]
            vel_cmd = np.clip(CTRL["alt_pos_kp"] * alt_err, -1.5, 1.5)
            vel_err = vel_cmd - nav["vel_up"]
            thr_delta = CTRL["alt_vel_kp"] * vel_err
            self.alt_int = getattr(self, "alt_int", 0.0) + CTRL["alt_vel_ki"] * vel_err * dt
            self.alt_int = max(-10.0, min(10.0, self.alt_int))
            thr_delta += self.alt_int
            thr_delta = np.clip(thr_delta, -CTRL["alt_vel_out_max"], CTRL["alt_vel_out_max"])
            thr = 50.0 + thr_delta
            thr = max(20.0, min(80.0, thr))
        else:
            thr = ref.get("thr", 50.0)
        w0 = (thr / 100.0) * P["wMax"]
        w0 = max(P["w0_floor"], min(P["wMax"], w0))

        # 分配：摆角/差速（对照固件 TandemVec_Propulsion.h B 矩阵对角线：
        #   ∂Mx/∂Δω = -2kQ·w0² < 0；∂My/∂δt = -b·T0 < 0；∂Mz/∂δf = +a·T0 > 0）
        T0 = P["kT"] * w0 * w0
        df_cmd = M_cmd[2] / (P["a"] * T0)                   # Mz'(上摆) → δf
        dt_cmd = -M_cmd[1] / (P["b"] * T0)                  # My'(下摆) → δt
        dw_cmd = -M_cmd[0] / (2 * P["kQ"] * w0 * w0)        # Mx'(差速) → Δω
        df_cmd = max(-P["dMax"], min(P["dMax"], df_cmd))
        dt_cmd = max(-P["dMax"], min(P["dMax"], dt_cmd))
        dw_cmd = max(-P["dwMax"], min(P["dwMax"], dw_cmd))

        # ---- 差速分配（固件公式 ωf=ω0√(1+Δω)）----
        wf_target = min(w0 * math.sqrt(max(0, 1 + dw_cmd)), P["wMax"])
        wt_target = min(w0 * math.sqrt(max(0, 1 - dw_cmd)), P["wMax"])

        act = {
            "df": df_cmd, "dt": dt_cmd,
            "wf": wf_target, "wt": wt_target,
            "thr": thr / 100.0, "w0": w0,
            "wf_prev": self.prev_wf, "wt_prev": self.prev_wt,
        }
        self.prev_wf = wf_target
        self.prev_wt = wt_target
        return act
