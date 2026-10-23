# -*- coding: utf-8 -*-
"""core.py — 高保真分析仿真核心：参数、四元数、推进/气动、六自由度 RK4。

@file    core.py
@module  analysis/高保真仿真核心（六自由度动力学）
@brief   提供参数加载、四元数运算、推力矢量/气动模型与刚体动力学 RK4 积分。
@details 输入：models/aircraft-model.json 参数 + 状态 [v, ω, q] + 指令 [δf, δt, Δω, ω0]；
         处理：电机一阶滞后 → 推力/反扭矩/转子角动量 → 气动力 → 牛顿-欧拉方程 → RK4；
         输出：机体系线加速度/角加速度/姿态导数，供 simulate/controllers/trim_analysis 复用。
         不变量：四元数每阶段保范投影；全部动力学在机体系积分；函数不隐式改全局状态。
@theory  推力 T=kT·ω²、反扭 τ=kQ·ω²+J_p·ω̇ @ref THY-002 §2；
         推力矢量六维映射与差速滚转 @ref MOD-001 §2–§4；
         刚体平动/转动方程（含转子角动量项）@ref MOD-001 §5.1–§5.2；
         气动小迎角线性模型 @ref MOD-001 §6；
         四元数运动学 q̇=½·q⊗[0,ω_b] @ref REF-120 §9.2 (9.2.14) p.268、@ref MOD-002 §3.4；
         欧拉角 theta=-asin(R13) 锁定约定 @ref MOD-002 §4.1。
@frame  NED（惯性/导航系）+ FRD（机体系）；推力沿 +x_b，前摆 δf 绕 z_b、尾摆 δt 绕 y_b。
@units  SI：力 N、力矩 N·m、转速 rad/s、角速度 rad/s、角度 rad、时间 s、质量 kg。
@note   本文件是 simulations/vector-thrust-lab/src/core 的 Python 对标实现，数值语义须逐项一致；
         摆角限幅仿真 ±25°（dMax）/±0.7（dwMax），固件实机已收紧 ±15°，不可外推实机
         （见 @ref MOD-001 §10、@ref AGENTS.md 2026-10-21 低速悬停仿真口径）。
@redline 四元数 RK4 积分 + 每阶段保范投影；theta=-asin(R13)；差速 ωf=ω0√(1+Δω)。
@see     controllers.py、simulate.py、trim_analysis.py、docs/04-数学建模/MOD-001-六自由度仿真模型规范.md
"""

import json, numpy as np
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent

# ============================================================
#  参数加载
# ============================================================
def load_params():
    """读取唯一参数源并展平为 name→value 字典。

    @brief   加载 models/aircraft-model.json 的 59 参数为扁平 dict。
    @return  dict，键为参数名（kT/kQ/Jp/wMax/Ix…g），值为 SI 数值。
    @formula g = |g_json|（NED 下 z 轴向下，重力加速度取正）
    @pre     必须在仓库根目录存在 models/aircraft-model.json。
    @note    参数唯一事实源是该 JSON；此处只读不写，禁止在代码内硬编码参数值
             （见 @ref AGENTS.md 关键路径）。
    @test    被 simulate.py / trim_analysis.py / run_all.py 共同调用。
    """
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
    """四元数保范投影。

    @brief   归一化四元数，抵消 RK4 各阶段的舍入漂移。
    @param[in] q 四元数 [w,x,y,z]（标量在前）
    @return  单位四元数
    @formula q ← q/|q|
    @ref     MOD-002 §3.4（每阶段保范投影属于行为红线）
    @pre     |q| > 0
    """
    return q / np.linalg.norm(q)

def quat_multiply(p, q):
    """Hamilton 四元数乘积 p ⊗ q（标量在前）。

    @brief   计算 p⊗q，用于姿态更新与误差四元数 q_err=q_des*⊗q。
    @param[in] p 左乘四元数 [w,x,y,z]
    @param[in] q 右乘四元数 [w,x,y,z]
    @return  p⊗q，[w,x,y,z]
    @formula (p⊗q)_w = pw·qw − pv·qv； (p⊗q)_v = pw·qv + qw·pv + pv×qv
    @ref     REF-120 §9.2 p.267–270（四元数代数）
    @note    机体系角速度用右乘更新 q̇=½q⊗[0,ω]，次序不可互换（@ref MOD-002 §3.4）。
    """
    pw, px, py, pz = p; qw, qx, qy, qz = q
    return np.array([
        pw*qw - px*qx - py*qy - pz*qz,
        pw*qx + px*qw + py*qz - pz*qy,
        pw*qy - px*qz + py*qw + pz*qx,
        pw*qz + px*qy - py*qx + pz*qw,
    ])

def quat_conj(q):
    """单位四元数共轭（= 逆旋转）。

    @brief   返回 q* = [w,−x,−y,−z]，用于把惯性系矢量旋回机体系。
    @param[in] q 单位四元数 [w,x,y,z]
    @return  共轭四元数 [w,−x,−y,−z]
    @ref     MOD-002 §3.3（单位四元数 q⁻¹=q*）
    """
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
    """双电机推进模型（对标 vector-thrust-lab propulsion.mjs）。

    状态：前/尾电机转速 wf, wt [rad/s] 及其上一步 prev_wf, prev_wt、
    本步转子角加速度 dwf, dwt 缓存、双转子角动量 hv [N·m·s]。
    电机为一阶滞后（时间常数 τm=P.tauM=0.28 s），差速按 ωf=ω0√(1+Δω)、
    ωt=ω0√(1−Δω) 分配（@ref CTL-001 §5，差速公式为行为红线）。
    反扭矩含转子惯性项 Jp·ω̇（@ref MOD-001 §3.1）。
    """

    def __init__(self, P):
        self.P = P
        self.wf = 0.0   # 前电机转速
        self.wt = 0.0   # 尾电机转速
        self.prev_wf = 0.0
        self.prev_wt = 0.0
        self.dwf = 0.0  # 本步转子角加速度（update 缓存，forces 消费）
        self.dwt = 0.0
        self.hv = np.zeros(3)  # 转子角动量（三轴，forces 缓存，对齐 dynamics.mjs hv / MOD-001）

    def update(self, omega0, dw_cmd, dt):
        """omega0: 基准转速, dw_cmd: 差速指令 [-1,1]"""
        P = self.P
        # 与 JS propulsion.mjs 一致：目标转速钳制到 wMax（差速指令可使 sqrt(1±dw) 超 1）
        wfT = min(omega0 * np.sqrt(1 + dw_cmd), P["wMax"])
        wtT = min(omega0 * np.sqrt(1 - dw_cmd), P["wMax"])
        alpha = min(dt / P["tauM"], 1.0)
        self.wf += (wfT - self.wf) * alpha
        self.wt += (wtT - self.wt) * alpha
        dwf = (self.wf - self.prev_wf) / max(dt, 1e-4)
        dwt = (self.wt - self.prev_wt) / max(dt, 1e-4)
        self.prev_wf = self.wf; self.prev_wt = self.wt
        self.dwf = dwf; self.dwt = dwt   # 缓存供 forces() 使用（否则 forces 重算恒为 0）
        return self.wf, self.wt, dwf, dwt

    def forces(self, delta_f, delta_t):
        """返回推进力/力矩的机体系分量"""
        P = self.P
        wf, wt = self.wf, self.wt
        # 惯性反扭 Jp·ω̇ 用 update() 缓存的角加速度；
        # 不能用 (wf - prev_wf)/dt 重算——update() 已把 prev_wf 置为 wf，重算恒为 0。
        dwf, dwt = self.dwf, self.dwt
        Tf = P["kT"] * wf*wf; Tt = P["kT"] * wt*wt
        Qf = P["kQ"] * wf*wf + P["Jp"] * dwf
        Qt = P["kQ"] * wt*wt + P["Jp"] * dwt
        cf, sf = np.cos(delta_f), np.sin(delta_f)
        ct, st = np.cos(delta_t), np.sin(delta_t)
        # 转子角动量（含摆角投影，对齐 dynamics.mjs hv / MOD-001 §5；
        # 前电机绕 z_b 摆 δf，尾电机绕 y_b 摆 δt，前后转向相反）
        self.hv = np.array([
            P["Jp"] * (wf*cf - wt*ct),
            P["Jp"] * wf*sf,
            P["Jp"] * wt*st,
        ])
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
    """气动力/力矩（机体系，风轴投影）。

    @brief   由机体系速度与角速度计算线性气动六分量。
    @param[in] vel   机体系速度 [u,v,w] [m/s]
    @param[in] omega 机体系角速度 [p,q,r] [rad/s]
    @param[in] P     参数字典（rho/Sw/cbar/bspan/CLa/CD0/CDk/…）
    @return  (aX,aY,aZ) 机体系气动力 [N]，(La,Ma,Na) 机体系气动力矩 [N·m]
    @formula α=atan2(w,u)、β=asin(v/V)、q̄=½ρV²、L=q̄·S·CLa·α、D=q̄·S·(CD0+CDk·CL²)
             aX=L·sinα−D·cosα、aZ=−L·cosα−D·sinα；Ma=q̄·S·c̄·(Cm0+Cma·α+Cmq·q·c̄/2V)
    @ref     MOD-001 §6（气动模型与阻尼导数）
    @note    V<0.5 m/s 时钳到 0.5 m/s——空速下限保护，避免 q̄ 与阻尼导数分母归零
             （行为红线「空速下限 0.5m/s」）。α/β 用 atan2/asin 而不是小角度近似。
    @deviation 未含大迎角失速、非定常效应与螺旋桨滑流干扰（@ref MOD-001 §11）。
    """
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
    L  = CL * qbar * Sw
    D  = CD * qbar * Sw
    aX = L * np.sin(alpha) - D * np.cos(alpha)   # 风轴 → 机体系（对标 aerodynamics.mjs）
    aY = CY * qbar * Sw
    aZ = -L * np.cos(alpha) - D * np.sin(alpha)
    La = Cl * qbar * Sw * b
    Ma = Cm * qbar * Sw * c
    Na = Cn * qbar * Sw * b
    return aX, aY, aZ, La, Ma, Na

# ============================================================
#  B_true — 在线控制效能 Jacobian
# ============================================================
def control_effectiveness(omega0, delta_f, delta_t, dw_cmd, P):
    """工作点处的控制效能矩阵 B_true（3×3，用于 INDI/LQR/自适应辨识）。

    @brief   对 [Δω, δt, δf] → [Mx, My, Mz] 在当前工作点求解析 Jacobian。
    @param[in] omega0  基准转速 [rad/s]
    @param[in] delta_f 前摆角（上摆，绕 z_b）[rad]
    @param[in] delta_t 尾摆角（下摆，绕 y_b）[rad]
    @param[in] dw_cmd  差速指令 [-1,1]（无量纲）
    @param[in] P       参数字典
    @return  (B, T0, tau0)：B 为 3×3 效能矩阵，T0=kT·ω0²[N]，tau0=kQ·ω0²[N·m]
    @formula ∂Mx/∂Δω≈−2kQ·ω0²、∂My/∂δt=−b·Tt·cosδt、∂Mz/∂δf=+a·Tf·cosδf；
             非对角项来自反扭随摆座旋转的投影（−Qt·sinδt、−Qf·sinδf 等）
    @ref     MOD-001 §4（控制效率矩阵）、THY-003 §3（分配与效能）
    @note    列顺序固定为 [Δω, δt, δf]——controllers.py 与 scripts/ident_probe.py
             均按此顺序组装 u，改顺序会静默改变分配结果。
    @note    差速列用 ωf² = ω0²(1+Δω) 线性化近似（一阶），大 Δω 时偏差为 O(Δω²)。
    """
    T0 = P["kT"] * omega0*omega0
    tau0 = P["kQ"] * omega0*omega0
    Tf = T0 * (1 + dw_cmd); Tt = T0 * (1 - dw_cmd)
    tauf = tau0 * (1 + dw_cmd); taut = tau0 * (1 - dw_cmd)
    sf, cf = np.sin(delta_f), np.cos(delta_f)
    st, ct = np.sin(delta_t), np.cos(delta_t)
    B = np.zeros((3,3))
    B[0,0] = -tau0 * (cf + ct)
    B[0,1] = -taut * st          # ∂Mx/∂δ_t = -Qt·st（反扭耦合，负号）
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
    # 转动: I·ω̇ = M − ω×Iω − ω×h转子（前CW沿+x_b, 尾CCW沿-x_b，含摆角投影；
    # h_rotor 由 Propulsion.forces 缓存，对齐 dynamics.mjs hv / MOD-001 §5）
    Ix, Iy, Iz = P["Ix"], P["Iy"], P["Iz"]
    hx, hy, hz = prop.hv
    gx = (Iz - Iy) * w[1]*w[2] + (w[1]*hz - w[2]*hy)
    gy = (Ix - Iz) * w[2]*w[0] + (w[2]*hx - w[0]*hz)
    gz = (Iy - Ix) * w[0]*w[1] + (w[0]*hy - w[1]*hx)
    wdot = np.array([
        (Mx + La - gx) / Ix,
        (My + Ma - gy) / Iy,
        (Mz + Na - gz) / Iz,
    ])
    # 四元数运动学: q̇ = ½ q ⊗ [0, ω]
    qdot = 0.5 * quat_multiply(q, np.array([0, w[0], w[1], w[2]]))
    return vdot, wdot, qdot

# ============================================================
#  RK4 一步积分
# ============================================================
def rk4_step(v, w, q, prop, P, Fx, Fy, Fz, Mx, My, Mz, h, use_aero=True):
    """四元数 RK4 一步积分（v, ω, q 联合，每阶段保范）。

    @brief   用一个子步长 h 对 [v, ω, q] 做四阶 Runge-Kutta 积分。
    @param[in] v,w,q  当前机体系速度 [m/s]、角速度 [rad/s]、姿态四元数 [w,x,y,z]
    @param[in] prop   Propulsion 实例（提供缓存的转子角动量 hv 与 ω̇）
    @param[in] Fx..Mz 本步推力/力矩（机体系）[N]/[N·m]，步内视为常值
    @param[in] h      积分子步长 [s]；红线要求 ≤0.004 s
    @param[in] use_aero  是否叠加气动力（False 用于纯推进/零空速工况）
    @return  (v_new, w_new, q_new)
    @formula y_{n+1}=y_n+(h/6)(k1+2k2+2k3+k4)；q 每个阶段后归一化
    @ref     MOD-001 §9（数值实现：四元数 RK4、子步 ≤4 ms、每阶段保范）
    @pre     h ≤ 0.004 s（行为红线）；q 已归一化
    @note    推进力/力矩在步内冻结（prop.forces 在循环外算一次），
             气动力则在每个 RK4 阶段按中间状态重算——这是有意的半隐式取舍。
    @test    与 vector-thrust-lab dynamics.mjs::physicsStep 对标（回归基线锁定）。
    """
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
