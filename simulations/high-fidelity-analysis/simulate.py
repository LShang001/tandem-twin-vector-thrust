# -*- coding: utf-8 -*-
"""时域仿真：SAS 和 INDI 控制器 + 对比分析"""

import numpy as np
from core import *  # noqa: F403

# ============================================================
#  SAS 控制器 (对标 control.mjs)
# ============================================================
class SASController:
    def __init__(self, P, trim):
        self.P = P; self.trim = trim
        self.int_th = 0.0; self.int_phi = 0.0

    def update(self, phi, theta, p, q, r, dt):
        P = self.P
        dtheta = theta - (-self.trim["alpha"])
        dphi = phi - 0.0
        # 俯仰
        self.int_th += dtheta * dt
        self.int_th = np.clip(self.int_th, -P["intThMax"], P["intThMax"])
        delta_t = self.trim["delta_t"] + P["sasQ"]*q - P["sasTh"]*dtheta - P["sasI"]*self.int_th
        # 偏航
        delta_f = -P["sasR"] * r
        # 滚转
        self.int_phi += dphi * dt
        self.int_phi = np.clip(self.int_phi, -P["intPhiMax"], P["intPhiMax"])
        dw = P["sasP"]*p - P["sasPhi"]*dphi - P["sasIPhi"]*self.int_phi
        # 限幅
        delta_t = np.clip(delta_t, -P["dMax"], P["dMax"])
        delta_f = np.clip(delta_f, -P["dMax"], P["dMax"])
        dw = np.clip(dw, -P["dwMax"], P["dwMax"])
        return delta_f, delta_t, dw

# ============================================================
#  INDI 控制器
# ============================================================
class INDIController:
    def __init__(self, P, trim):
        self.P = P; self.trim = trim
        self.int_th = 0.0; self.int_phi = 0.0
        self.prev_delta_f = 0.0; self.prev_delta_t = trim["delta_t"]; self.prev_dw = 0.0
        self.prev_omega = np.zeros(3)
        self.omega_dot_filt = np.zeros(3)
        self.first = True

    def update(self, phi, theta, p, q, r, omega0, dt, aero, M_prop):
        """INDI 增量控制律: 使用混合角加速度（模型预测 + 传感器微分融合）"""
        P = self.P
        # 外环: 姿态 → 目标角速率
        # JS 欧拉约定 phi_dot=-p, theta_dot=-q, 故 q_ref=+k·dtheta (负反馈)
        dtheta = theta - (-self.trim["alpha"]); dphi = phi
        self.int_th += dtheta*dt; self.int_th = np.clip(self.int_th, -P["intThMax"], P["intThMax"])
        self.int_phi += dphi*dt; self.int_phi = np.clip(self.int_phi, -P["intPhiMax"], P["intPhiMax"])
        p_ref = 3.0*(P["sasPhi"]*dphi + P["sasIPhi"]*self.int_phi)
        q_ref = 3.0*(P["sasTh"]*dtheta + P["sasI"]*self.int_th)
        r_ref = 0.0
        # 内环: 角速率误差 → 虚拟角加速度（带宽 0.8→3.0, 与 VTOL 版一致）
        K_rate = 3.0
        nu_p = K_rate*(p_ref - p); nu_q = K_rate*(q_ref - q); nu_r = K_rate*(r_ref - r)
        # 混合角加速度: 模型预测 (低频) + 差分 (高频)
        omega = np.array([p, q, r])
        if self.first:
            self.omega_dot_filt = np.zeros(3); self.first = False
        else:
            # 模型预测: 从当前力矩和状态计算 ω̇
            Ix, Iy, Iz = P["Ix"], P["Iy"], P["Iz"]
            gx = (Iz-Iy)*omega[1]*omega[2]; gy = (Ix-Iz)*omega[2]*omega[0]; gz = (Iy-Ix)*omega[0]*omega[1]
            model_dot = np.array([
                (M_prop[0] + aero[0] - gx) / Ix,
                (M_prop[1] + aero[1] - gy) / Iy,
                (M_prop[2] + aero[2] - gz) / Iz,
            ])
            # 差分: 传感器角加速度
            raw_dot = (omega - self.prev_omega) / dt
            # 互补融合: 70% 模型 + 30% 差分
            alpha_fusion = 0.3
            fused_dot = (1-alpha_fusion)*model_dot + alpha_fusion*raw_dot
            alpha_lp = min(dt / 0.01, 1.0)
            self.omega_dot_filt += alpha_lp * (fused_dot - self.omega_dot_filt)
        self.prev_omega = omega.copy()
        # 在线 Jacobian
        B, _, _ = control_effectiveness(omega0, self.prev_delta_f, self.prev_delta_t,
                                         self.prev_dw, P)
        nu = np.array([nu_p, nu_q, nu_r])
        delta_u = np.zeros(3)
        try:
            err = nu - self.omega_dot_filt
            delta_u = np.linalg.solve(B, err)
            delta_u = np.clip(delta_u, -0.2, 0.2)
        except np.linalg.LinAlgError:
            pass
        self.prev_dw += delta_u[0]
        self.prev_delta_t += delta_u[1]
        self.prev_delta_f += delta_u[2]
        self.prev_dw = np.clip(self.prev_dw, -P["dwMax"], P["dwMax"])
        self.prev_delta_t = np.clip(self.prev_delta_t, -P["dMax"], P["dMax"])
        self.prev_delta_f = np.clip(self.prev_delta_f, -P["dMax"], P["dMax"])
        return self.prev_delta_f, self.prev_delta_t, self.prev_dw

# ============================================================
#  单次仿真运行
# ============================================================
def simulate(controller, P, trim, T_total=10.0, dt=0.004, disturbance=None):
    """运行一次时域仿真，返回时间序列数据."""
    alpha0 = trim["alpha"]; omega0 = trim["omega0"]; delta_t0 = trim["delta_t"]
    V0 = trim["V"]
    # 姿态四元数: 绕 +y 转 +α (物理抬头), JS 欧拉读数 theta=-α
    c=np.cos(alpha0/2); s=np.sin(alpha0/2); q0=np.array([c,0,s,0])
    u0=V0*np.cos(alpha0); w0=V0*np.sin(alpha0)
    v = np.array([u0, 0.0, w0])
    w = np.zeros(3)
    q = q0.copy()
    prop = Propulsion(P)
    # 初始化电机到配平转速
    prop.wf = omega0; prop.wt = omega0; prop.prev_wf = omega0; prop.prev_wt = omega0

    N = int(T_total / dt)
    t_arr = np.zeros(N); phi_arr=np.zeros(N); theta_arr=np.zeros(N); psi_arr=np.zeros(N)
    p_arr=np.zeros(N); q_arr=np.zeros(N); r_arr=np.zeros(N)
    vx_arr=np.zeros(N); vy_arr=np.zeros(N); vz_arr=np.zeros(N)
    delta_f_arr=np.zeros(N); delta_t_arr=np.zeros(N); dw_arr=np.zeros(N)

    delta_f = 0.0; delta_t = delta_t0; dw = 0.0
    for i in range(N):
        t = i*dt; t_arr[i] = i*dt
        # 扰动注入：在指定时刻给本体角速率施加脉冲（作用于被控对象）
        if disturbance and abs(t - disturbance[0]) < dt:
            if disturbance[2] == "pitch":
                w[1] += np.radians(disturbance[3])  # q 脉冲
            elif disturbance[2] == "roll":
                w[0] += disturbance[3]  # p 脉冲
            elif disturbance[2] == "yaw":
                w[2] += np.radians(disturbance[3])
        w_actual = w
        # 推进力/力矩（使用上一步指令）
        Fx, Fy, Fz, Mx, My, Mz = prop.forces(delta_f, delta_t)
        # 控制器
        phi, theta, psi_ = euler_from_quat(q)
        if isinstance(controller, SASController):
            delta_f, delta_t, dw = controller.update(phi, theta, w_actual[0], w_actual[1], w_actual[2], dt)
        else:
            aero_now = aero_forces(v, w_actual, P)
            M_prop = np.array([Mx, My, Mz])
            aero_M_vals = np.array([aero_now[3], aero_now[4], aero_now[5]])
            delta_f, delta_t, dw = controller.update(phi, theta, w_actual[0], w_actual[1], w_actual[2], omega0, dt, aero_M_vals, M_prop)
        # 推进（用新指令更新电机状态）
        prop.update(omega0, dw, dt)
        # 重新计算推进力（使用新指令，供 RK4 用）
        Fx, Fy, Fz, Mx, My, Mz = prop.forces(delta_f, delta_t)
        # 气动
        aero = aero_forces(v, w, P)
        # RK4 积分
        v, w, q = rk4_step(v, w, q, prop, P, Fx,Fy,Fz,Mx,My,Mz, dt)
        # 记录
        phi_arr[i], theta_arr[i], psi_arr[i] = phi, theta, psi_
        p_arr[i], q_arr[i], r_arr[i] = w[0], w[1], w[2]
        vx_arr[i], vy_arr[i], vz_arr[i] = v[0], v[1], v[2]
        delta_f_arr[i], delta_t_arr[i], dw_arr[i] = delta_f, delta_t, dw

    return {
        "t": t_arr, "phi": phi_arr, "theta": theta_arr, "psi": psi_arr,
        "p": p_arr, "q": q_arr, "r": r_arr,
        "u": vx_arr, "v": vy_arr, "w": vz_arr,
        "delta_f": delta_f_arr, "delta_t": delta_t_arr, "dw": dw_arr,
    }

# ============================================================
#  对比分析
# ============================================================
def compare_controllers(P, trim, T_total=10, disturbance=None):
    """运行 SAS 和 INDI 控制器，返回对比数据."""
    sas = SASController(P, trim)
    indi = INDIController(P, trim)
    data_sas = simulate(sas, P, trim, T_total, disturbance=disturbance)
    data_indi = simulate(indi, P, trim, T_total, disturbance=disturbance)
    return data_sas, data_indi
