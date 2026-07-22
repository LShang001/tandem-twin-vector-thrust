# -*- coding: utf-8 -*-
"""纵列双发矢量推力飞行器 — 高精度多维仿真分析
   核心动力学模块：参数加载、四元数运算、推进、气动、牛-欧方程、RK4积分
"""

import json, numpy as np
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent

# ============================================================
#  参数加载
# ============================================================
def load_params():
    path = ROOT / "models" / "aircraft-model.json"
    data = json.loads(path.read_text("utf-8"))
    P = {}
    for sec in data["sections"]:
        for p in sec["parameters"]:
            P[p["name"]] = p["value"]
    # 派生参数
    P["g"] = abs(P["g"])   # NED: z 向下, 重力加速度为正
    return P

# ============================================================
#  四元数运算（标量在前: q = [w, x, y, z]）
# ============================================================
def quat_norm(q):
    return q / np.linalg.norm(q)

def quat_multiply(p, q):
    pw, px, py, pz = p; qw, qx, qy, qz = q
    return np.array([
        pw*qw - px*qx - py*qy - pz*qz,
        pw*qx + px*qw + py*qz - pz*qy,
        pw*qy - px*qz + py*qw + pz*qx,
        pw*qz + px*qy - py*qx + pz*qw,
    ])

def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quat_rotate(v, q):
    """将向量 v 用四元数 q 旋转 (q⊗v⊗q*)"""
    qv = np.array([0, v[0], v[1], v[2]])
    r = quat_multiply(quat_multiply(q, qv), quat_conj(q))
    return r[1:4]

def euler_from_quat(q):
    """3-2-1 Euler 角: phi=roll, theta=pitch, psi=yaw.
       与 JS math.mjs 一致: R13 = 2(q1q3 + q0q2), theta = -arcsin(R13)。
       注意: 绕 +y 转 +a 的四元数读数为 theta=-a（项目锁定约定, 见 MOD-002）。"""
    q0, q1, q2, q3 = q
    R13 = 2*(q1*q3 + q0*q2)
    R23 = 2*(q2*q3 - q0*q1)
    R33 = 1 - 2*(q1*q1 + q2*q2)
    R12 = 2*(q1*q2 - q0*q3)
    R11 = 1 - 2*(q2*q2 + q3*q3)
    phi = np.arctan2(R23, R33)
    theta = -np.arcsin(np.clip(R13, -1, 1))
    psi = np.arctan2(R12, R11)
    return phi, theta, psi

# ============================================================
#  推进系统（对标 propulsion.mjs）
# ============================================================
class Propulsion:
    def __init__(self, P):
        self.P = P
        self.wf = 0.0   # 前电机转速
        self.wt = 0.0   # 尾电机转速
        self.prev_wf = 0.0
        self.prev_wt = 0.0

    def update(self, omega0, dw_cmd, dt):
        """omega0: 基准转速, dw_cmd: 差速指令 [-1,1]"""
        P = self.P
        wfT = omega0 * np.sqrt(1 + dw_cmd)
        wtT = omega0 * np.sqrt(1 - dw_cmd)
        alpha = min(dt / P["tauM"], 1.0)
        self.wf += (wfT - self.wf) * alpha
        self.wt += (wtT - self.wt) * alpha
        dwf = (self.wf - self.prev_wf) / max(dt, 1e-4)
        dwt = (self.wt - self.prev_wt) / max(dt, 1e-4)
        self.prev_wf = self.wf; self.prev_wt = self.wt
        return self.wf, self.wt, dwf, dwt

    def forces(self, delta_f, delta_t):
        """返回推进力/力矩的机体系分量"""
        P = self.P
        wf, wt = self.wf, self.wt
        dwf = (wf - self.prev_wf) / 0.004  # 近似
        dwt = (wt - self.prev_wt) / 0.004
        Tf = P["kT"] * wf*wf; Tt = P["kT"] * wt*wt
        Qf = P["kQ"] * wf*wf + P["Jp"] * dwf
        Qt = P["kQ"] * wt*wt + P["Jp"] * dwt
        cf, sf = np.cos(delta_f), np.sin(delta_f)
        ct, st = np.cos(delta_t), np.sin(delta_t)
        Fx = Tf*cf + Tt*ct
        Fy = Tf*sf
        Fz = -Tt*st
        Mx = -Qf*cf + Qt*ct
        My = -P["b"]*Tt*st - Qf*sf
        Mz =  P["a"]*Tf*sf - Qt*st
        return Fx, Fy, Fz, Mx, My, Mz

# ============================================================
#  气动力
# ============================================================
def aero_forces(vel, omega, P):
    u, v, w = vel; p, q, r = omega
    V = np.sqrt(u*u + v*v + w*w)
    if V < 0.5: V = 0.5
    alpha = np.arctan2(w, u)
    beta = np.arcsin(np.clip(v / V, -1, 1))
    qbar = 0.5 * P["rho"] * V*V
    CL = P["CLa"] * alpha
    CD = P["CD0"] + P["CDk"] * CL*CL
    CY = P["CYb"] * beta
    Cm = P["Cm0"] + P["Cma"] * alpha + P["Cmq"] * q * P["cbar"] / (2*V)
    Cl = P["Clb"] * beta + P["Clp"] * p * P["bspan"] / (2*V)
    Cn = P["Cnb"] * beta + P["Cnr"] * r * P["bspan"] / (2*V)
    Sw = P["Sw"]; c = P["cbar"]; b = P["bspan"]
    aX = -CD * qbar * Sw
    aY =  CY * qbar * Sw
    aZ = -CL * qbar * Sw
    La = Cl * qbar * Sw * b
    Ma = Cm * qbar * Sw * c
    Na = Cn * qbar * Sw * b
    return aX, aY, aZ, La, Ma, Na

# ============================================================
#  B_true — 在线控制效能 Jacobian
# ============================================================
def control_effectiveness(omega0, delta_f, delta_t, dw_cmd, P):
    T0 = P["kT"] * omega0*omega0
    tau0 = P["kQ"] * omega0*omega0
    Tf = T0 * (1 + dw_cmd); Tt = T0 * (1 - dw_cmd)
    tauf = tau0 * (1 + dw_cmd); taut = tau0 * (1 - dw_cmd)
    sf, cf = np.sin(delta_f), np.cos(delta_f)
    st, ct = np.sin(delta_t), np.cos(delta_t)
    B = np.zeros((3,3))
    B[0,0] = -tau0 * (cf + ct)
    B[0,1] = +taut * st
    B[0,2] = +tauf * sf
    B[1,0] = +P["b"]*T0*st - tau0*sf
    B[1,1] = -P["b"]*Tt*ct
    B[1,2] = -tauf*cf
    B[2,0] = +P["a"]*T0*sf + tau0*st
    B[2,1] = -taut*ct
    B[2,2] = +P["a"]*Tf*cf
    return B, T0, tau0

# ============================================================
#  刚体动力学导数: 返回 [v̇, ω̇, q̇]
# ============================================================
def dynamics_derivatives(v, w, q, prop, P, Fx, Fy, Fz, Mx, My, Mz, aero):
    """返回速度、角速度、四元数的导数（对标 dynamics.mjs）"""
    aX, aY, aZ, La, Ma, Na = aero
    # 重力在机体系的分量 (NED: 惯性系重力为 +z 向下, 与 JS dynamics.mjs 一致)
    gb = quat_rotate(np.array([0, 0, P["g"]]), quat_conj(q))
    # 平动: v̇ = F/m − ω×v
    m = P["m"]
    vdot = np.array([
        (Fx + aX) / m + gb[0] - (w[1]*v[2] - w[2]*v[1]),
        (Fy + aY) / m + gb[1] - (w[2]*v[0] - w[0]*v[2]),
        (Fz + aZ) / m + gb[2] - (w[0]*v[1] - w[1]*v[0]),
    ])
    # 转动: I·ω̇ = M − ω×Iω − ω×h转子 (前CW沿+x_b, 尾CCW沿-x_b)
    Ix, Iy, Iz = P["Ix"], P["Iy"], P["Iz"]
    gx = (Iz - Iy) * w[1]*w[2]
    gy = (Ix - Iz) * w[2]*w[0]
    gz = (Iy - Ix) * w[0]*w[1]
    hx = P["Jp"] * (prop.wf - prop.wt)  # 转子角动量 (沿 x_b)
    wdot = np.array([
        (Mx + La - gx) / Ix,
        (My + Ma - gy - w[2]*hx) / Iy,
        (Mz + Na - gz + w[1]*hx) / Iz,
    ])
    # 四元数运动学: q̇ = ½ q ⊗ [0, ω]
    qdot = 0.5 * quat_multiply(q, np.array([0, w[0], w[1], w[2]]))
    return vdot, wdot, qdot

# ============================================================
#  RK4 一步积分
# ============================================================
def rk4_step(v, w, q, prop, P, Fx, Fy, Fz, Mx, My, Mz, h, use_aero=True):
    def f(vv, ww, qq):
        if use_aero:
            aX, aY, aZ, La, Ma, Na = aero_forces(vv, ww, P)
        else:
            aX = aY = aZ = La = Ma = Na = 0.0
        return dynamics_derivatives(vv, ww, qq, prop, P, Fx, Fy, Fz, Mx, My, Mz,
                                    (aX, aY, aZ, La, Ma, Na))
    v0, w0, q0 = v.copy(), w.copy(), q.copy()
    # k1
    d1v, d1w, d1q = f(v0, w0, q0)
    # k2
    v1 = v0 + 0.5*h*d1v; w1 = w0 + 0.5*h*d1w
    q1 = quat_norm(q0 + 0.5*h*d1q)
    d2v, d2w, d2q = f(v1, w1, q1)
    # k3
    v2 = v0 + 0.5*h*d2v; w2 = w0 + 0.5*h*d2w
    q2 = quat_norm(q0 + 0.5*h*d2q)
    d3v, d3w, d3q = f(v2, w2, q2)
    # k4
    v3 = v0 + h*d3v; w3 = w0 + h*d3w
    q3 = quat_norm(q0 + h*d3q)
    d4v, d4w, d4q = f(v3, w3, q3)
    # 加权
    h6 = h / 6
    v_new = v0 + h6*(d1v+2*d2v+2*d3v+d4v)
    w_new = w0 + h6*(d1w+2*d2w+2*d3w+d4w)
    q_new = quat_norm(q0 + h6*(d1q+2*d2q+2*d3q+d4q))
    return v_new, w_new, q_new
