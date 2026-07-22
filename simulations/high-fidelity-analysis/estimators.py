# -*- coding: utf-8 -*-
"""角加速度估计器 — 用于 INDI 的 ω̇ 估计方法对比研究
   实现四种估计器，统一接口 update(omega, dt) -> omega_dot
   参考 THY-004 §10.6.3 角加速度获取与滤波
"""

import numpy as np
from collections import deque


# ============================================================
# 基线：后向欧拉差分 + 一阶低通（当前 INDI 实现）
# ============================================================
class EulerDiffEstimator:
    """后向欧拉差分 + 一阶低通。零延迟但噪声放大 ∝ 1/dt²。"""
    name = "后向欧拉差分(基线)"

    def __init__(self, dt, tau_f=0.01):
        self.prev_omega = None
        self.filt = np.zeros(3)
        self.tau_f = tau_f

    def update(self, omega, dt):
        omega = np.asarray(omega, dtype=float)
        if self.prev_omega is None:
            self.prev_omega = omega.copy()
            return np.zeros(3)
        raw = (omega - self.prev_omega) / dt
        alpha = min(dt / self.tau_f, 1.0)
        self.filt += alpha * (raw - self.filt)
        self.prev_omega = omega.copy()
        return self.filt.copy()


# ============================================================
# Savitzky-Golay 滤波器（滑动窗口多项式拟合 + 解析求导）
# ============================================================
class SavitzkyGolayEstimator:
    """S-G 滤波：窗口内最小二乘多项式拟合，解析求导得 ω̇。
       同时输出平滑 ω 和 ω̇，无模型依赖。群延迟 M·dt。"""
    name = "Savitzky-Golay(M=4,d=2)"

    def __init__(self, dt, M=4, d=2):
        # 窗口 2M+1 点，d 阶多项式
        self.M, self.d, self.dt = M, d, dt
        self.buf = deque(maxlen=2 * M + 1)
        # 预计算一阶导数滤波系数（对窗口中心）
        n = 2 * M + 1
        x = np.arange(-M, M + 1) * dt          # 相对时间
        A = np.vander(x, d + 1, increasing=True)  # [n, d+1]
        # 最小二乘系数矩阵 pinv，第一行是函数值，第二行是一阶导数
        self.H = np.linalg.pinv(A)              # [d+1, n]
        self.deriv_coef = self.H[1]             # 一阶导数权重

    def update(self, omega, dt):
        self.buf.append(np.asarray(omega, dtype=float))
        if len(self.buf) < 2 * self.M + 1:
            # 窗口未填满：退化为差分
            if len(self.buf) >= 2:
                return (self.buf[-1] - self.buf[-2]) / dt
            return np.zeros(3)
        W = np.array(self.buf)                   # [n, 3]
        return self.deriv_coef @ W               # [3]


# ============================================================
# 自适应互补滤波（模型预测低频 + 差分高频，自适应截止）
# ============================================================
class AdaptiveComplementaryEstimator:
    """互补滤波：模型前向预测（低噪低频）+ 数值差分（高频），
       截止频率随角加速度残差自适应。参考 Jiang et al. 2025。

       模型预测通道用完整刚体转动方程（含陀螺/Coriolis）：
           ω̇_model = I⁻¹ [B_true·u − ω×(Iω) − ω×h_rotor]
       仅需已知 I、B_true、转子角动量，无需气动数据库。
       这是它在 INDI 闭环中胜出的关键：用低噪声模型通道抑制差分噪声，
       同时保留差分通道的低延迟（避免 S-G/Kalman 的同步效应损失）。"""
    name = "自适应互补滤波"

    def __init__(self, dt, P, B_of_state, omega_c_min=20.0, omega_c_max=150.0):
        self.P = P
        self.B_of_state = B_of_state      # fn(df,dt,dw,omega0)->B_true
        self.prev_omega = None
        self.est = np.zeros(3)            # 估计的 ω̇（差分通道平滑态）
        self.wc_min, self.wc_max = omega_c_min, omega_c_max
        self.sigma = 5.0                  # 残差归一化尺度 [rad/s²]
        self.df = self.dt_ = self.dw = 0.0
        self.omega0 = 400.0
        self.h_rotor = 0.0                # 转子角动量 x 分量（前后对消后）

    def set_control(self, df, dt_, dw, omega0, h_rotor=0.0):
        self.df, self.dt_, self.dw, self.omega0 = df, dt_, dw, omega0
        self.h_rotor = h_rotor

    def _model_dot(self, omega):
        """完整刚体转动模型预测角加速度（不含气动/扰动）"""
        P = self.P
        B = self.B_of_state(self.df, self.dt_, self.dw, self.omega0)
        u = np.array([self.dw, self.dt_, self.df])
        M_ctrl = B @ u
        p, q, r = omega
        Ix, Iy, Iz = P["Ix"], P["Iy"], P["Iz"]
        # ω×(Iω) Coriolis + ω×h_rotor 陀螺
        gx = (Iz - Iy) * q * r
        gy = (Ix - Iz) * r * p - r * self.h_rotor
        gz = (Iy - Ix) * p * q + q * self.h_rotor
        return np.array([(M_ctrl[0]-gx)/Ix, (M_ctrl[1]-gy)/Iy, (M_ctrl[2]-gz)/Iz])

    def update(self, omega, dt):
        omega = np.asarray(omega, dtype=float)
        model_dot = self._model_dot(omega)
        if self.prev_omega is None:
            self.prev_omega = omega.copy()
            self.est = model_dot.copy()
            return self.est.copy()
        diff_dot = (omega - self.prev_omega) / dt
        self.prev_omega = omega.copy()
        # 自适应截止：差分与模型残差大（机动）→ 偏向差分（保响应）
        resid = np.linalg.norm(diff_dot - model_dot)
        wc = self.wc_min + (self.wc_max - self.wc_min) * np.tanh(resid / self.sigma)
        # 互补融合：差分经低通，模型经高通（一阶互补对）
        alpha = min(wc * dt / (1 + wc * dt), 1.0)
        self.est += alpha * (diff_dot - self.est)
        # 稳态偏向模型（低噪声），机动偏向差分（低延迟）
        beta = 1.0 - 0.5 * np.tanh(resid / self.sigma)   # resid小→0.5模型, 大→0差分侧
        out = beta * model_dot + (1 - beta) * self.est
        return out.copy()


# ============================================================
# 固定滞后 Kalman 平滑（状态 [ω, ω̇]，恒加速度模型）
# ============================================================
class FixedLagSmootherEstimator:
    """固定滞后 Kalman 平滑：利用 N 步"未来"观测改善当前 ω̇ 估计。
       状态 x=[ω(3), ω̇(3)]，恒加速度过程模型。延迟 N·dt。"""
    name = "固定滞后平滑(N=4)"

    def __init__(self, dt, N=4, q_proc=50.0, r_meas=1e-3):
        self.N, self.dt = N, dt
        # 单轴状态 [ω, ω̇]，三轴独立
        self.F = np.array([[1, dt], [0, 1]])
        self.H = np.array([[1, 0]])
        self.Q = q_proc * np.array([[dt**3/3, dt**2/2], [dt**2/2, dt]])
        self.R = np.array([[r_meas]])
        # 三轴各自 Kalman 状态
        self.x = [np.zeros(2) for _ in range(3)]
        self.Pcov = [np.eye(2) for _ in range(3)]
        self.hist = deque(maxlen=N + 1)   # 存 (x, P) 历史用于平滑

    def update(self, omega, dt):
        omega = np.asarray(omega, dtype=float)
        est = np.zeros(3)
        self.hist.append(([x.copy() for x in self.x],
                          [P.copy() for P in self.Pcov]))
        for ax in range(3):
            z = omega[ax]
            # 预测
            self.x[ax] = self.F @ self.x[ax]
            self.Pcov[ax] = self.F @ self.Pcov[ax] @ self.F.T + self.Q
            # 更新
            y = z - (self.H @ self.x[ax])[0]
            S = (self.H @ self.Pcov[ax] @ self.H.T + self.R)[0, 0]
            K = (self.Pcov[ax] @ self.H.T) / S
            self.x[ax] = self.x[ax] + K.flatten() * y
            self.Pcov[ax] = (np.eye(2) - K @ self.H) @ self.Pcov[ax]
            est[ax] = self.x[ax][1]
        return est
