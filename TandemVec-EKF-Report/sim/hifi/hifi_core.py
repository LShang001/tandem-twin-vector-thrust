# -*- coding: utf-8 -*-
"""hifi_core.py — 高保真固件闭环仿真核心（6DOF 动力学 + 传感器 + 固件控制律）。

@file    hifi_core.py
@module  analysis/闭环仿真核心（动力学 + 传感器模型 + 固件控制律）
@brief   提供真值 6DOF 动力学、带噪/带故障传感器与逐行复刻固件的控制律。
@details 输入：控制器指令 (δf, δt, Δω, 油门/转速)；处理：执行器一阶（舵机 0.05 s、
         电机 τm=0.28 s）→ 推力/反扭矩/转子角动量 → 受扰刚体动力学 → 位置/姿态积分，
         同时输出 IMU（比力+角速度）、GNSS、磁、气压量测；
         输出：真值状态、传感器读数与执行器指令，供 hifi/run_*.py 组闭环。
         全链：6DOF → 传感器 → EKF → 固件控制律 → 执行器 → 6DOF。
@theory  推力矢量六维映射与差速滚转 @ref MOD-001 §2–§4；
         刚体平动/转动方程（含 ω×Iω 与 ω×h）@ref MOD-001 §5.1–§5.2；
         推力 T=kT·ω²、反扭 τ=kQ·ω²+Jp·ω̇ @ref THY-002 §2；
         控制分配 B 矩阵工作点 @ref THY-003 §3、@ref CTL-001 §5；
         误差四元数外环 + 角速率内环 + 惯量逆解 @ref CTL-001 §4；
         差速分配 ωf=ω0√(1+Δω) @ref CTL-001 §5。
@frame  NED 真值位置/速度 + FRD 机体系；悬停时 x_b 朝天（推力轴），
         控制通道 [roll,pitch,yaw] 映射到力矩 [Mz',My',Mx']。
@units  SI：力 N、力矩 N·m、转速 rad/s、加速度 m/s²、角度 rad；控制增益在 deg 域
         （rate_kp 等为 1/s、1/s²，见 @ref AGENTS.md 速率环量纲条目）。
@note    ★ 口径：本文件 P["dMax"]=0.2618 rad（**15°，固件实机值**），
         而 simulations/high-fidelity-analysis 用 25°（模型 JSON）——两者不可混用结论。
         P["g"]=9.79 取 @ref models/aircraft-model.json 的 MODEL-DEFAULT；
         EKF 侧用 9.80665（G_MPS2），闭环内存在 0.17% 的重力口径差。
         本文件不是 C++ 固件 HIL；固件正确性由 test_host 与 `pio run` 验证。
@see     hifi/run_scenes.py、hifi/run_full_envelope.py、hifi/run_nav_envelope.py、
         @ref docs/08-参考资料/notes/05-仿真验证.md §1.1（基线口径）
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

# 理想地磁矢量（NED，µT；长沙附近量级，仅仿真概念验证）
MAG_NED_UT = np.array([22.0, 0.0, -42.0])

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
    """Hamilton 四元数乘积 q⊗p（标量在前）。

    @brief   姿态更新与误差四元数计算的基础算子。
    @param[in] q 左乘四元数 (w,x,y,z)
    @param[in] p 右乘四元数 (w,x,y,z)
    @return  q⊗p (w,x,y,z)
    @formula (q⊗p)_w=q_w p_w−q_v·p_v；(q⊗p)_v=q_w p_v+p_w q_v+q_v×p_v
    @ref     REF-120 §9.2 p.267–270、@ref MOD-002 §3.4
    """
    w1, x1, y1, z1 = q
    w2, x2, y2, z2 = p
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2])


def quat_conj(q):
    """四元数共轭 (w,−x,−y,−z)。

    @brief   单位四元数下等价于逆，用于 q_err=q_cur*⊗q_target。
    @param[in] q 四元数 (w,x,y,z)
    @return  共轭四元数
    @ref     MOD-002 §3.3
    """
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_to_dcm(q):
    """四元数 -> 机体系到导航系 C^b_n（行=导航, 列=机体）"""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]])


def rotvec_to_quat(theta):
    """旋转矢量 → 四元数（指数映射，小角度自动降阶）。

    @brief   把等效旋转矢量 φ 映射为 q=Exp(φ/2)。
    @param[in] theta 等效旋转矢量 (3,) [rad]
    @return  单位四元数 (w,x,y,z)
    @formula q0=cos(|φ|/2)、q_v=sin(|φ|/2)·φ/|φ|；|φ|<1e-6 时 q_v≈φ/2
    @ref     REF-120 §9.3 (9.3.30) p.274（等效旋转矢量）
    @note    小角度分支必须存在：|φ|→0 时 sin(half)/|φ| 为 0/0 型。
    """
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
    """四元数 → ZYX 欧拉角（固件 quat2angle 语义）。

    @brief   由 C^n_b 元素提取 (yaw, pitch, roll)。
    @param[in] q 四元数 (w,x,y,z)
    @return  [yaw, pitch, roll] [rad]
    @formula pitch=−asin(C20)、roll=atan2(C21,C22)、yaw=atan2(C10,C00)
    @ref     MOD-002 §4.1（本项目 theta=−asin(R13) 锁定约定）
    @note    C20 需钳到 [−1,1]，避免浮点越界使 asin 返回 NaN。
    """
    C = quat_to_dcm(q)
    pitch = -math.asin(max(-1.0, min(1.0, C[2, 0])))
    roll = math.atan2(C[2, 1], C[2, 2])
    yaw = math.atan2(C[1, 0], C[0, 0])
    return np.array([yaw, pitch, roll])


def wrap_pi(a):
    """角度折算到 (−π, π]。

    @brief   航向误差缠绕保护。
    @param[in] a 角度 [rad]
    @return  等价角度 [rad]
    @note    航向锁 dy=wrap_pi(yaw_ref−yaw) 必须在乘 kp 前缠绕。
    """
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
        self.t = 0.0                      # 仿真时间（执行器/故障用）
        self.fault = {}                   # 执行器故障（thrust_loss/servo_jam/wind_shear）
        self._prev_vel = np.zeros(3)
        self._dt_prev = 0.005
        self._prev_wf_act = w_hover
        self._prev_wt_act = w_hover

    def step(self, act, dt):
        """推进一个真值仿真步（执行器动态 + 刚体动力学）。

        @param[in] act dict：df/dt 摆角指令 [rad]、wf/wt 目标转速 [rad/s]、thr/w0 油门与工作点
        @param[in] dt  步长 [s]（调用方用 0.005 s = 200 Hz，与固件 tick 同量级）
        @return (Tf, Tt, M_body)：本步推力 [N] 与机体系力矩 [N·m]，供记录/诊断
        @formula F_body=[Tf·cf+Tt·ct, Tf·sf, −Tt·st]；
                 M_body=[−Qf·cf+Qt·ct, −b·Tt·st−Qf·sf, a·Tf·sf−Qt·st]；
                 I·ω̇ = M − ω×Iω − ω×h；q ← q⊗Exp(ω·dt)
        @ref     MOD-001 §2–§5（推力映射与刚体动力学）
        @deviation 本实现是**一阶半隐式欧拉**（速度/角速度先用本步导数更新，四元数再用新 ω
                 右乘），不是二阶 RK2；函数内 `dt2 = dt/2` 与 `dq` 为早期遗留变量，未参与计算。
                 在 dt=5 ms、τm=0.28 s 的悬停闭环下数值稳定，但定步长收敛阶为一阶，
                 做精度对比（如与 core.py 的 RK4 对比）时必须先声明这一差异。
        @note    摆角伺服用 τs=0.05 s 一阶，电机用 τm=0.28 s 一阶，均按 dt/τ 比例更新；
                 ω̇ 只用于反扭矩的 Jp·ω̇ 项。
        """
        self.t += dt
        # ---- 执行器动态 ----
        # 摆座伺服一阶（τs=0.05s）+ 限幅
        tau_s = 0.05
        k = min(dt / tau_s, 1.0)
        self.df += k * (act["df"] - self.df)
        self.dt += k * (act["dt"] - self.dt)
        # 执行器故障：舵机卡死（在伺服一阶之后覆盖，模拟机械卡死）
        sj = self.fault.get("servo_jam")
        if sj and self.t >= float(sj.get("t", 0.0)):
            if sj.get("axis", "df") == "df":
                self.df = float(sj.get("value", 0.0))
            else:
                self.dt = float(sj.get("value", 0.0))
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
        # 执行器故障：单发推力损失（ESC/电机/桨效率下降）
        tl = self.fault.get("thrust_loss")
        if tl and self.t >= float(tl.get("t", 0.0)):
            if tl.get("motor", "front") == "front":
                Tf *= float(tl.get("scale", 1.0))
            else:
                Tt *= float(tl.get("scale", 1.0))
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
        # 震动（共轴双桨 80Hz）；vib_amp 可被故障场景放大
        vib_amp = float(self.fault.get("vib_amp", 0.05))
        vib = vib_amp * np.sin(2 * np.pi * 80 * t) * np.array([1, 1, 1])
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
        """GPS NED 位置/速度（10Hz），含噪声/丢星/延迟/野值/慢漂"""
        pos_ned = self.pos.copy()
        vel_ned = quat_to_dcm(self.quat) @ self.vel
        if fault:
            if fault.get("gps_lost") and fault["gps_lost"][0] < t < fault["gps_lost"][1]:
                return None
            if "gps_jump" in fault and t > fault["gps_jump_t"]:
                pos_ned = pos_ned + fault["gps_jump"]
            # 慢漂/欺骗：从 t0 起按速率累积（不是单帧跳变）
            gd = fault.get("gps_drift")
            if gd and t > gd.get("t", 0.0):
                pos_ned = pos_ned + np.asarray(gd.get("rate", [0, 0, 0]), dtype=float) * (t - gd.get("t", 0.0))
        pos_ned += self.rng.normal(0, 1.5, 3)
        vel_ned += self.rng.normal(0, 0.2, 3)
        return pos_ned, vel_ned

    def mag_measure(self, t, fault=None):
        """理想三轴磁力计（机体系 µT）+ 噪声/软磁干扰；仅 P0-2 概念验证。"""
        m_ned = MAG_NED_UT.copy()
        if fault:
            m_ned = m_ned + np.asarray(fault.get("mag_disturb_ut", [0, 0, 0]),
                                       dtype=float)
        m_body = quat_to_dcm(self.quat).T @ m_ned
        m_body = m_body + self.rng.normal(0, 1.0, 3)
        return m_body

    def baro_measure(self, t, fault=None):
        """气压高度（向上为正），支持慢漂/偏置故障"""
        alt = -self.pos[2]
        if fault:
            bd = fault.get("baro_drift")
            if bd and t > bd.get("t", 0.0):
                alt += float(bd.get("rate_mps", 0.0)) * (t - bd.get("t", 0.0))
            alt += float(fault.get("baro_bias_m", 0.0))
        return alt + self.rng.normal(0, 0.5)


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
        # 舵机卡死/姿态持续超差的简化 failsafe（仿真验证用，不是固件实现）
        self.failsafe_active = False
        self.fault_count = 0

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
        # ★ 2026-10 轴映射修复：Dynamics 的 omega 是机体系 [x,y,z]
        #   （x=推力轴/偏航、y=下摆/俯仰、z=上摆/滚转）；控制通道是
        #   [roll,pitch,yaw]，必须重排为 [ωz, ωy, ωx]。
        #   旧实现直接把 [ωx,ωy,ωz] 当 [roll,pitch,yaw]，速率环正反馈发散。
        omega_ctrl = np.array([omega_raw[2], omega_raw[1], omega_raw[0]])
        self.rate_filt = self._lowpass(self.rate_filt, omega_ctrl * R2D, CTRL["speed_alpha"])
        self.rate_filt2 = self._lowpass(self.rate_filt2, self.rate_filt, CTRL["speed_alpha2"])
        omega_dps = self.rate_filt2

        # ---- 姿态外环：误差四元数 -> 目标角速度 ----
        q_cur = nav["quat"]
        q_err = quat_mult(quat_conj(q_cur), ref["q_target"])
        if q_err[0] < 0:
            q_err = -q_err
        # 体轴误差（deg）：机体系 x=推力轴/yaw、y=pitch、z=roll；
        # 控制通道顺序 [roll,pitch,yaw] = [q_err.z,q_err.y,q_err.x]。
        err_deg = np.array([q_err[3], q_err[2], q_err[1]]) * 2 * R2D
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
        #   本仿真采用模型系（x'=差速偏航、y'=下摆俯仰、z'=上摆滚转）。
        #   控制通道顺序是 [roll, pitch, yaw] = q_err.x/y/z；必须交叉映射：
        #   yaw→Mx'(差速)、pitch→My'(下摆)、roll→Mz'(上摆)。
        #   2026-10 修复：旧实现把 roll 接到 Mx、yaw 接到 Mz，起飞即滚转/偏航
        #   交叉耦合，是 hifi 闭环发散的根因之一。
        M_cmd = np.array([
            P["Ix"] * alpha_cmd[2] * D2R,   # Mx'(差速) ← yaw
            P["Iy"] * alpha_cmd[1] * D2R,   # My'(下摆) ← pitch
            P["Iz"] * alpha_cmd[0] * D2R,   # Mz'(上摆) ← roll
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
            alt_ki = ref.get("alt_vel_ki")
            if alt_ki is None:
                alt_ki = CTRL["alt_vel_ki"]
            self.alt_int = getattr(self, "alt_int", 0.0) + float(alt_ki) * vel_err * dt
            self.alt_int = max(-10.0, min(10.0, self.alt_int))
            thr_delta += self.alt_int
            thr_delta = np.clip(thr_delta, -CTRL["alt_vel_out_max"], CTRL["alt_vel_out_max"])
            thr = 50.0 + thr_delta
            thr = max(20.0, min(80.0, thr))
        else:
            # 悬停配平油门：w_hover 对应 100*w_hover/wMax ≈49.91%，不是恰好 50%。
            # 旧默认 50% 会多 0.17% 推力，60 s 爬升约 60 m；A0 静止场景会误判。
            thr = ref.get("thr", 100.0 * w_hover / P["wMax"])
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

        # ---- 简化执行器故障 failsafe（仅仿真验证）----
        # 舵机卡死/姿态持续超差且控制量饱和 -> 切油门，避免继续积分发散。
        if ref.get("failsafe", False):
            # 姿态持续超差即认为执行器失效（不依赖舵机反馈/饱和判断）。
            severe = max(abs(err_deg[0]), abs(err_deg[1])) > 10.0
            self.fault_count = self.fault_count + 1 if severe else 0
            if self.fault_count >= 40:  # 0.2 s @200Hz
                self.failsafe_active = True
        if self.failsafe_active:
            thr = 0.0
            df_cmd = 0.0
            dt_cmd = 0.0
            dw_cmd = 0.0

        # ---- 差速分配（固件公式 ωf=ω0√(1+Δω)）----
        if self.failsafe_active:
            wf_target = 0.0
            wt_target = 0.0
        else:
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
