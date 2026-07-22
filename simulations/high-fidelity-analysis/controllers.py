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

    def update(self, q, q_des, omega, dt):
        """q: 当前姿态, q_des: 期望姿态, omega: [p,q,r]"""
        P = self.P
        # 误差四元数: q_err = q_des* ⊗ q
        q_err = quat_multiply(quat_conj(q_des), q)
        eps = np.array([q_err[1], q_err[2], q_err[3]])  # ε
        # 积分（限幅）
        self.int_eps += eps * dt
        self.int_eps = np.clip(self.int_eps, -P["intThMax"], P["intThMax"])
        # 四元数 PD 力矩
        K_q = np.array([0.8, 0.6, 0.8])   # 比例增益 (调至与欧拉角 SAS 等价)
        K_w = np.array([0.18, 0.14, 0.14]) # 角速率阻尼增益
        tau = -K_q * eps - K_w * omega - np.array([0.15, 0.1, 0.0]) * self.int_eps
        # 效能矩阵逆映射 (简化: 取对角元素)
        omega0 = self.omega0
        tau0 = P["kQ"] * omega0*omega0
        T0 = P["kT"] * omega0*omega0
        dw   = np.clip(tau[0] / (-2*tau0) if abs(tau0) > 1e-10 else 0, -P["dwMax"], P["dwMax"])
        dt_c = np.clip(tau[1] / (-P["b"]*T0) if abs(T0) > 1e-10 else 0, -P["dMax"], P["dMax"])
        df_c = np.clip(tau[2] / ( P["a"]*T0) if abs(T0) > 1e-10 else 0, -P["dMax"], P["dMax"])
        return df_c, dt_c, dw

# ============================================================
#  四元数 INDI 控制器
# ============================================================
class QuatINDIController:
    def __init__(self, P, trim=None):
        self.P = P
        self.int_eps = np.zeros(3)
        self.prev_df = 0.0; self.prev_dt = 0.0; self.prev_dw = 0.0
        self.prev_omega = np.zeros(3)
        self.omega_dot_filt = np.zeros(3)
        self.first = True

    def update(self, q, q_des, omega, omega0, dt):
        P = self.P
        # 误差四元数
        q_err = quat_multiply(quat_conj(q_des), q)
        eps = np.array([q_err[1], q_err[2], q_err[3]])
        self.int_eps += eps*dt
        self.int_eps = np.clip(self.int_eps, -P["intThMax"], P["intThMax"])
        # 外环: 四元数比例 → 目标角速率
        w_ref = -0.6 * eps
        # 内环: 角速率误差 → 虚拟角加速度
        K_rate = 0.8
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

    for i in range(N):
        t = i*dt; t_arr[i] = t
        # slerp 过渡
        if slerp_duration > 0 and t < slerp_duration:
            tau = min(t / slerp_duration, 1.0)
            q_des = slerp_quat(q_des_initial, q_des_target, tau)
        else:
            q_des = q_des_target
        # 推进 (先算力，供控制器用)
        Fx, Fy, Fz, Mx, My, Mz = prop.forces(df, dt_cmd)
        # 气动 (可选关闭)
        if use_aero:
            aero = aero_forces(v, w, P)
        else:
            aero = (0,0,0,0,0,0)
        # 控制器
        if isinstance(controller, QuatSASController):
            df, dt_cmd, dw = controller.update(q, q_des, w, dt)
        else:
            df, dt_cmd, dw = controller.update(q, q_des, w, omega0, dt)
        # 推进更新
        prop.update(omega0, dw, dt)
        Fx, Fy, Fz, Mx, My, Mz = prop.forces(df, dt_cmd)
        # RK4 积分
        v, w, q = rk4_step(v, w, q, prop, P, Fx,Fy,Fz,Mx,My,Mz, dt, use_aero=use_aero)
        # 位置 (惯性系)
        vi = quat_rotate(v, q)
        pos_z -= vi[2] * dt   # vi[2] 为 NED 速度, z 向上为负, 高度=-pos_z
        if pos_z > 0 and -vi[2] > 0:
            pos_z = 0; vi[2] = 0
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
