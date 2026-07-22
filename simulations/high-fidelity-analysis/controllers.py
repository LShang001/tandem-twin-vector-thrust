# -*- coding: utf-8 -*-
"""四元数域控制器：QuatSAS + QuatINDI + VTOL 仿真"""

import numpy as np
from core import (
    quat_multiply, quat_conj, quat_norm, quat_rotate, euler_from_quat,
    control_effectiveness, aero_forces, rk4_step, Propulsion
)

# ============================================================
#  四元数 SAS 控制器
# ============================================================
class QuatSASController:
    def __init__(self, P, trim=None, omega0=None):
        self.P = P
        self.omega0 = omega0 if omega0 else 400.0  # 允许外部传入
        self.int_eps = np.zeros(3)

    def update(self, q, q_des, omega, dt, omega_ref=None):
        """q: 当前姿态, q_des: 期望姿态, omega: [p,q,r],
           omega_ref: 参考角速率前馈（slerp 过渡段非零），阻尼作用于速率误差"""
        P = self.P
        if omega_ref is None:
            omega_ref = np.zeros(3)
        # 误差四元数: q_err = q_des* ⊗ q
        q_err = quat_multiply(quat_conj(q_des), q)
        eps = np.array([q_err[1], q_err[2], q_err[3]])  # ε
        # 积分（条件积分抗饱和：大误差机动期间冻结，避免 slerp 段饱和后长期放电偏置）
        if np.linalg.norm(eps) < 0.2:
            self.int_eps += eps * dt
            self.int_eps = np.clip(self.int_eps, -P["intThMax"], P["intThMax"])
        # 四元数 PD 力矩
        K_q = np.array([0.8, 0.6, 0.8])   # 比例增益 (调至与欧拉角 SAS 等价)
        K_w = np.array([0.18, 0.14, 0.14]) # 角速率阻尼增益
        # 阻尼增益调度：大姿态误差时增强阻尼，抑制大角度机动的欠阻尼振荡
        # (eps=0 时 g=1 恢复基准律; eps>=0.15 (~17°) 时 g=4, 俯仰通道 ζ: 0.22→0.88)
        g_damp = 1.0 + 3.0 * min(np.linalg.norm(eps) / 0.15, 1.0)
        tau = (-K_q * eps - g_damp * K_w * (omega - omega_ref)
               - np.array([0.15, 0.1, 0.0]) * self.int_eps)
        # 效能矩阵逆映射 (简化: 取对角元素)
        omega0 = self.omega0
        tau0 = P["kQ"] * omega0*omega0
        T0 = P["kT"] * omega0*omega0
        dw   = np.clip(tau[0] / (-2*tau0) if abs(tau0) > 1e-10 else 0, -P["dwMax"], P["dwMax"])
        dt_c = np.clip(tau[1] / (-P["b"]*T0) if abs(T0) > 1e-10 else 0, -P["dMax"], P["dMax"])
        df_c = np.clip(tau[2] / ( P["a"]*T0) if abs(T0) > 1e-10 else 0, -P["dMax"], P["dMax"])
        return df_c, dt_c, dw

# ============================================================
#  四元数 LQR 控制器（悬停点线性化 + CARE）
# ============================================================
class QuatLQRController:
    """状态 x=[ε(3), ω(3)]，输入 u=[Δω, δt, δf]。
       线性化误差动力学：ε̇≈½(ω-ω_ref)，ω̇=I⁻¹B·u（悬停点 B 满秩）。
       权重按 Bryson 规则：Q_ii=1/x_max²，R_ii=1/u_max²。"""

    def __init__(self, P, omega0):
        from scipy.linalg import solve_continuous_are
        self.P = P
        self.omega0 = omega0
        B, T0, tau0 = control_effectiveness(omega0, 0.0, 0.0, 0.0, P)
        Iinv = np.diag([1.0/P["Ix"], 1.0/P["Iy"], 1.0/P["Iz"]])
        A = np.zeros((6, 6)); A[:3, 3:] = 0.5*np.eye(3)
        Bm = np.zeros((6, 3)); Bm[3:, :] = Iinv @ B
        x_max = np.array([0.5, 0.5, 0.5, 1.0, 1.0, 1.0])   # ε~0.5(≈60°), ω~1 rad/s
        u_max = np.array([P["dwMax"], P["dMax"], P["dMax"]])
        Q = np.diag(1.0/x_max**2); R = np.diag(1.0/u_max**2)
        S = solve_continuous_are(A, Bm, Q, R)
        self.K = np.linalg.solve(R, Bm.T @ S)
        self.A, self.Bm, self.Q, self.R = A, Bm, Q, R  # 供论文导出

    def update(self, q, q_des, omega, dt, omega_ref=None):
        P = self.P
        if omega_ref is None:
            omega_ref = np.zeros(3)
        q_err = quat_multiply(quat_conj(q_des), q)
        eps = np.array([q_err[1], q_err[2], q_err[3]])
        x = np.concatenate([eps, np.asarray(omega) - omega_ref])
        u = -self.K @ x
        dw   = np.clip(u[0], -P["dwMax"], P["dwMax"])
        dt_c = np.clip(u[1], -P["dMax"], P["dMax"])
        df_c = np.clip(u[2], -P["dMax"], P["dMax"])
        return df_c, dt_c, dw

# ============================================================
#  四元数 INDI 控制器（v2: 参考速率前馈 + 带宽提升）
# ============================================================
class QuatINDIController:
    def __init__(self, P, trim=None):
        self.P = P
        self.prev_df = 0.0; self.prev_dt = 0.0; self.prev_dw = 0.0
        self.prev_omega = np.zeros(3)
        self.omega_dot_filt = np.zeros(3)
        self.first = True

    def update(self, q, q_des, omega, omega0, dt, omega_ref=None):
        P = self.P
        if omega_ref is None:
            omega_ref = np.zeros(3)
        # 误差四元数
        q_err = quat_multiply(quat_conj(q_des), q)
        eps = np.array([q_err[1], q_err[2], q_err[3]])
        # 外环: 四元数比例 → 目标角速率（叠加参考速率前馈）
        w_ref = -1.0 * eps + omega_ref
        # 内环: 角速率误差 → 虚拟角加速度（带宽 0.8→3.0）
        K_rate = 3.0
        nu = K_rate * (w_ref - omega)
        # 角加速度 (混合估计)
        if self.first:
            self.omega_dot_filt = np.zeros(3); self.first = False
        else:
            raw_dot = (omega - self.prev_omega) / dt
            alpha = min(dt/0.01, 1.0)
            self.omega_dot_filt += alpha * (raw_dot - self.omega_dot_filt)
        self.prev_omega = omega.copy()
        # 在线 Jacobian + 增量
        B, T0, tau0 = control_effectiveness(omega0, self.prev_df, self.prev_dt, self.prev_dw, P)
        try:
            du = np.linalg.solve(B, nu - self.omega_dot_filt)
            if np.any(~np.isfinite(du)):
                du = np.zeros(3)
            du = np.clip(du, -0.2, 0.2)
        except (np.linalg.LinAlgError, ValueError):
            du = np.zeros(3)
        self.prev_dw += du[0]; self.prev_dt += du[1]; self.prev_df += du[2]
        self.prev_dw = np.clip(self.prev_dw, -P["dwMax"], P["dwMax"])
        self.prev_dt = np.clip(self.prev_dt, -P["dMax"], P["dMax"])
        self.prev_df = np.clip(self.prev_df, -P["dMax"], P["dMax"])
        return self.prev_df, self.prev_dt, self.prev_dw

# ============================================================
#  VTOL / Tail-sitter 仿真
# ============================================================
def simulate_vtol(controller, P, q_des_target, omega0, T_total=10, dt=0.004,
                  use_aero=False, slerp_duration=0.0, q_des_initial=None):
    """VTOL tail-sitter 仿真。电机预旋转到 omega0。"""
    N = int(T_total / dt)
    v = np.array([0.0, 0.0, 0.0])
    w = np.zeros(3)
    # tail-sitter 初始已在竖直姿态 (机头朝天)
    q = np.array(q_des_target, copy=True) if q_des_initial is None else np.array(q_des_initial, copy=True)
    prop = Propulsion(P)
    prop.wf = prop.wt = omega0
    prop.prev_wf = prop.prev_wt = omega0

    t_arr = np.zeros(N); h_arr = np.zeros(N)
    phi_a = np.zeros(N); theta_a = np.zeros(N)
    p_a = np.zeros(N); q_a = np.zeros(N); r_a = np.zeros(N)
    df_a = np.zeros(N); dt_a = np.zeros(N); dw_a = np.zeros(N)
    eps_norm = np.zeros(N)
    vel_x = np.zeros(N); vel_z = np.zeros(N)

    df = 0.0; dt_cmd = 0.0; dw = 0.0  # dt_cmd = 尾电机摆角指令 δ_t (勿遮蔽时间步 dt)
    pos_z = 0.0  # NED: z 向下, 初始在地面

    # slerp 参考角速率前馈：定轴旋转 => 机体系参考角速率为常值
    if slerp_duration > 0 and q_des_initial is not None:
        dq0 = quat_multiply(quat_conj(quat_norm(np.array(q_des_initial, dtype=float))),
                            quat_norm(np.array(q_des_target, dtype=float)))
        if dq0[0] < 0: dq0 = -dq0  # 最短路径
        ang0 = 2.0 * np.arccos(np.clip(dq0[0], -1, 1))
        axis0 = dq0[1:4] / max(np.sin(ang0 / 2), 1e-9)
        omega_ref_slerp = axis0 * ang0 / slerp_duration
    else:
        omega_ref_slerp = np.zeros(3)

    for i in range(N):
        t = i*dt; t_arr[i] = t
        # slerp 过渡
        if slerp_duration > 0 and t < slerp_duration:
            tau_s = min(t / slerp_duration, 1.0)
            q_des = slerp_quat(q_des_initial, q_des_target, tau_s)
            omega_ref = omega_ref_slerp
        else:
            q_des = q_des_target
            omega_ref = np.zeros(3)
        # 推进 (先算力，供控制器用)
        Fx, Fy, Fz, Mx, My, Mz = prop.forces(df, dt_cmd)
        # 气动 (可选关闭)
        if use_aero:
            aero = aero_forces(v, w, P)
        else:
            aero = (0,0,0,0,0,0)
        # 控制器
        if isinstance(controller, QuatINDIController):
            df, dt_cmd, dw = controller.update(q, q_des, w, omega0, dt, omega_ref)
        else:  # QuatSASController / QuatLQRController
            df, dt_cmd, dw = controller.update(q, q_des, w, dt, omega_ref)
        # 推进更新
        prop.update(omega0, dw, dt)
        Fx, Fy, Fz, Mx, My, Mz = prop.forces(df, dt_cmd)
        # RK4 积分
        v, w, q = rk4_step(v, w, q, prop, P, Fx,Fy,Fz,Mx,My,Mz, dt, use_aero=use_aero)
        # 位置 (惯性系, NED: z 向下为正; pos_z = 相对起点的深度)
        vi = quat_rotate(v, q)
        pos_z += vi[2] * dt
        if pos_z > 0 and vi[2] > 0:   # 触地且仍在下降: 钳制并清零下降速度
            pos_z = 0.0
            vi[2] = 0.0
            vb = quat_rotate(vi, quat_conj(q))
            v[0] = vb[0]; v[1] = vb[1]; v[2] = vb[2]
        # 记录
        phi, theta, psi = euler_from_quat(q)
        phi_a[i] = phi; theta_a[i] = theta
        p_a[i] = w[0]; q_a[i] = w[1]; r_a[i] = w[2]
        df_a[i] = df; dt_a[i] = dt_cmd; dw_a[i] = dw
        q_err = quat_multiply(quat_conj(q_des), q)
        eps_norm[i] = np.linalg.norm(q_err[1:4])
        vel_x[i] = v[0]; vel_z[i] = v[2]
        h_arr[i] = -pos_z  # 高度 (NED: z 向下, 高度 = -z)

    return {
        "t": t_arr, "h": h_arr, "phi": phi_a, "theta": theta_a,
        "p": p_a, "q": q_a, "r": r_a,
        "delta_f": df_a, "delta_t": dt_a, "dw": dw_a,
        "eps_norm": eps_norm, "vx": vel_x, "vz": vel_z,
    }

def slerp_quat(q0, q1, t):
    """球面线性插值"""
    q0 = quat_norm(q0.copy()); q1 = quat_norm(q1.copy())
    dot = np.dot(q0, q1)
    if dot < 0: q1 = -q1; dot = -dot
    if dot > 0.9995:
        result = q0 + t*(q1 - q0)
        return quat_norm(result)
    theta = np.arccos(np.clip(dot, -1, 1))
    s = np.sin(theta)
    return (np.sin((1-t)*theta)/s)*q0 + (np.sin(t*theta)/s)*q1
