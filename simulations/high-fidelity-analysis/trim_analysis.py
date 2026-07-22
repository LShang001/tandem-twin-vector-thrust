# -*- coding: utf-8 -*-
"""配平求解、数值线性化、特征值分析"""

import numpy as np
from core import *  # noqa: F403

def trim_longitudinal(V_target, P, verbose=False):
    """纵向配平: 求解 α, ω₀, δ_t 使 Fx=0, Fz=0, My=0.
       假设 δ_f=0, Δω=0, β=0, 水平飞行。
       姿态四元数为绕 +y 转 +α (物理抬头), 按 JS 约定欧拉读数 theta=-α."""
    def residual(x):
        alpha, omega0, delta_t = x
        # 状态
        u = V_target * np.cos(alpha)
        w = V_target * np.sin(alpha)
        v = np.array([u, 0.0, w])
        wb = np.zeros(3)
        # 四元数 (绕 +y 转 +α = 物理抬头 α)
        c = np.cos(alpha/2); s = np.sin(alpha/2)
        q = np.array([c, 0, s, 0])
        # 推进
        wf = omega0; wt = omega0
        Tf = P["kT"]*wf*wf; Tt = P["kT"]*wt*wt
        Qf = P["kQ"]*wf*wf; Qt = P["kQ"]*wt*wt
        cf = 1.0; sf = 0.0   # δ_f=0
        ct = np.cos(delta_t); st = np.sin(delta_t)
        Fx_p = Tf*cf + Tt*ct
        Fz_p = -Tt*st
        My_p = -P["b"]*Tt*st
        # 气动
        Vs = np.linalg.norm(v); alpha_a = np.arctan2(w, max(u, 0.1))
        qbar = 0.5*P["rho"]*Vs*Vs
        CL = P["CLa"]*alpha_a
        CD = P["CD0"] + P["CDk"]*CL*CL
        Cm = P["Cm0"] + P["Cma"]*alpha_a
        aX = -CD*qbar*P["Sw"]; aZ = -CL*qbar*P["Sw"]
        Ma = Cm*qbar*P["Sw"]*P["cbar"]
        # 重力在机体系 (NED: +z 向下)
        gb = quat_rotate(np.array([0,0,P["g"]]), quat_conj(q))
        # 残差
        r_Fx = Fx_p + aX + P["m"]*gb[0]
        r_Fz = Fz_p + aZ + P["m"]*gb[2]
        r_My = My_p + Ma
        return np.array([r_Fx, r_Fz, r_My])

    # Newton 法
    x = np.array([0.05, 600.0, -0.01])
    for it in range(50):
        r = residual(x)
        if np.max(np.abs(r)) < 1e-3: break
        J = np.zeros((3,3))
        eps = 1e-6
        for j in range(3):
            xp = x.copy(); xp[j] += eps
            J[:,j] = (residual(xp) - r) / eps
        dx = np.linalg.solve(J, -r)
        x += dx
        # 限幅
        x[0] = np.clip(x[0], -0.3, 0.3)
        x[1] = np.clip(x[1], 50, P["wMax"])
        x[2] = np.clip(x[2], -P["dMax"], P["dMax"])
    alpha, omega0, delta_t = x
    if verbose:
        print(f"  配平: α={np.degrees(alpha):.3f}°, ω₀={omega0:.1f} rad/s, "
              f"δ_t={np.degrees(delta_t):.3f}°, thr={omega0/P['wMax']*100:.1f}%")
    return {"alpha": alpha, "omega0": omega0, "delta_t": delta_t, "V": V_target}

def linearize_at_trim(trim, P):
    """在配平点处对六自由度系统做数值线性化，返回 A(12×12) 矩阵."""
    alpha = trim["alpha"]; omega0 = trim["omega0"]; delta_t = trim["delta_t"]
    Vt = trim["V"]
    u = Vt*np.cos(alpha); w = Vt*np.sin(alpha)
    c = np.cos(alpha/2); s = np.sin(alpha/2)  # 绕 +y 转 +α (物理抬头)
    q0 = np.array([c, 0, s, 0])
    x0 = np.array([u, 0, w, 0, 0, 0, 0, 0, 0, q0[0], q0[1], q0[2], q0[3]])
    # 状态编号: 0-u,1-v,2-w,3-p,4-q,5-r,6-x,7-y,8-z,9-q0,10-q1,11-q2,12-q3

    def f_12(x):
        vv = np.array([x[0], x[1], x[2]])
        ww = np.array([x[3], x[4], x[5]])
        qq = np.array([x[9], x[10], x[11], x[12]])
        qq = qq / np.linalg.norm(qq)
        # 推力 (constant at trim)
        wf = omega0; wt = omega0
        Tf = P["kT"]*wf*wf; Tt = P["kT"]*wt*wt
        Qf = P["kQ"]*wf*wf; Qt = P["kQ"]*wt*wt
        ct = np.cos(delta_t); st = np.sin(delta_t)
        Fx_p = Tf + Tt*ct; Fz_p = -Tt*st
        My_p = -P["b"]*Tt*st
        # 气动
        V = np.linalg.norm(vv)
        if V < 0.5: V = 0.5
        alpha_a = np.arctan2(vv[2], max(vv[0], 0.1))
        beta_a = np.arcsin(np.clip(vv[1]/V, -1, 1))
        qbar = 0.5*P["rho"]*V*V
        CL = P["CLa"]*alpha_a; CD = P["CD0"]+P["CDk"]*CL*CL
        CY = P["CYb"]*beta_a
        Cm = P["Cm0"]+P["Cma"]*alpha_a+P["Cmq"]*ww[1]*P["cbar"]/(2*V)
        Cl = P["Clb"]*beta_a+P["Clp"]*ww[0]*P["bspan"]/(2*V)
        Cn = P["Cnb"]*beta_a+P["Cnr"]*ww[2]*P["bspan"]/(2*V)
        aX=-CD*qbar*P["Sw"]; aY=CY*qbar*P["Sw"]; aZ=-CL*qbar*P["Sw"]
        La=Cl*qbar*P["Sw"]*P["bspan"]; Ma=Cm*qbar*P["Sw"]*P["cbar"]; Na=Cn*qbar*P["Sw"]*P["bspan"]
        gb = quat_rotate(np.array([0,0,P["g"]]), quat_conj(qq))
        m=P["m"]; Ix=P["Ix"]; Iy=P["Iy"]; Iz=P["Iz"]
        vdot = np.array([
            (Fx_p+aX)/m+gb[0]-(ww[1]*vv[2]-ww[2]*vv[1]),
            (0+aY)/m+gb[1]-(ww[2]*vv[0]-ww[0]*vv[2]),
            (Fz_p+aZ)/m+gb[2]-(ww[0]*vv[1]-ww[1]*vv[0]),
        ])
        gx=(Iz-Iy)*ww[1]*ww[2]; gy=(Ix-Iz)*ww[2]*ww[0]; gz=(Iy-Ix)*ww[0]*ww[1]
        wdot = np.array([
            (0+La-gx)/Ix, (My_p+Ma-gy)/Iy, (0+Na-gz)/Iz,
        ])
        qdot = 0.5*quat_multiply(qq, np.array([0,ww[0],ww[1],ww[2]]))
        # 位置运动学 (惯性速度)
        vi = quat_rotate(vv, qq)
        return np.concatenate([vdot, wdot, vi, qdot])

    A = np.zeros((13,13))
    eps = 1e-4
    f0 = f_12(x0)
    for j in range(12):
        xp = x0.copy(); xp[j] += eps
        if j >= 9: xp[9:13] /= np.linalg.norm(xp[9:13])
        A[:,j] = (f_12(xp) - f0) / eps

    # 提取有用子块: 纵向 [u,w,q,θ] 和横航向 [v,p,r,ϕ]
    # θ = -arcsin(R13), ϕ = atan2(R23,R33)
    A_long = np.zeros((4,4))
    idx_long = [0,2,4,11]  # u,w,q,q2→θ
    for i in range(4):
        for j in range(4):
            A_long[i,j] = A[idx_long[i], idx_long[j]]
    A_lat = np.zeros((4,4))
    idx_lat = [1,3,5,10]  # v,p,r,q1→ϕ
    for i in range(4):
        for j in range(4):
            A_lat[i,j] = A[idx_lat[i], idx_lat[j]]
    return A, A_long, A_lat

def analyze_modes(A_long, A_lat, P):
    """分析纵向和横航向模态特征值"""
    results = {}
    for name, A in [("longitudinal", A_long), ("lateral", A_lat)]:
        eigs = np.linalg.eigvals(A)
        results[name] = {"eigenvalues": eigs}
    # 纵向模态分类
    e = results["longitudinal"]["eigenvalues"]
    real_parts = np.real(e)
    # 找短周期和长周期
    for ev in e:
        freq = np.abs(ev)/(2*np.pi)
        damp = -np.real(ev)/np.abs(ev)
        results.setdefault("modes", []).append({
            "eigenvalue": ev, "freq_hz": freq, "damping": damp
        })
    return results
