# -*- coding: utf-8 -*-
"""LMPC 可行性原型：VTOL 悬停姿态控制（悬停点线性化 + 滚动时域 + 显式约束）
对比 QuatSASController，验证 MPC 在该构型的约束价值与实时性量级"""
import sys
sys.path.insert(0, 'simulations/high-fidelity-analysis')
import time
import numpy as np
from scipy.optimize import minimize
from core import (load_params, control_effectiveness, quat_multiply,
                  quat_conj, rk4_step, Propulsion, quat_norm)
from controllers import QuatSASController, slerp_quat

P = load_params()
qh = np.array([np.cos(np.pi/4), 0, np.sin(np.pi/4), 0])   # 机头朝天
q_level = np.array([1.0, 0, 0, 0])
W0_H = np.sqrt(P['m']*P['g']/(2*P['kT']))
DT = 0.004

# ============================================================
#  线性模型（悬停点）：x = [ε(3), ω(3)]，u = [Δω, δt, δf]
#  ε̇ = ½ω，ω̇ = I⁻¹·B·u（忽略陀螺/耦合，线性近似）
# ============================================================
B0, _, _ = control_effectiveness(W0_H, 0.0, 0.0, 0.0, P)
Iinv = np.diag([1/P['Ix'], 1/P['Iy'], 1/P['Iz']])
Ac = np.zeros((6, 6))
Ac[:3, 3:] = 0.5 * np.eye(3)
Bc = np.vstack([np.zeros((3, 3)), Iinv @ B0])
A = np.eye(6) + Ac * DT
Bd = Bc * DT
# —— 增广执行器滞后模型（一阶滞后进状态）——
# x_aug = [ε(3), ω(3), a(3)]，a = 实际角加速度（滞后实现）
#   ε̇ = ½ω，ω̇ = a，ȧ = (Bc·u − a)/τ
# 预测步长 dt_p 远大于控制周期（0.004）：时域必须覆盖滞后 τ=0.28s，
# 否则 MPC 窗口内控制效果被滞后吞噬 → 认为控制无效 → 输出≈0
DT_PRED = 0.02   # 预测步长（MPC 标准做法：粗步长覆盖主导时间常数）
N = 30           # 预测时域 30×0.02 = 0.6s > 2τ
A6 = np.eye(6) + Ac * DT_PRED
B6 = Bc * DT_PRED
A_aug = np.zeros((9, 9))
A_aug[:6, :6] = A6
A_aug[3:6, 6:9] = np.eye(3) * DT_PRED       # ω ← a
A_aug[6:9, 6:9] = np.eye(3) * (1.0 - DT_PRED / P['tauM'])  # a 一阶滞后
B_aug = np.zeros((9, 3))
B_aug[6:9, :] = Bc[3:6, :] * (DT_PRED / P['tauM'])   # 转动块（I⁻¹B）经滞后进入 a
Q = np.diag([4.0, 4.0, 4.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0])  # 姿态/角速度惩罚，执行器状态不惩罚
R = np.diag([0.1, 0.1, 0.1])              # 控制权重（偏小：更积极，原型整定）
u_max = np.array([P['dwMax'], P['dMax'], P['dMax']])

class LMPCAttitude:
    """滚动时域 LMPC：展开式 QP（condensed）+ 梯度投影法（PGD，嵌入式一阶方法）
    min ½z'Hz + f'z s.t. |u_k|≤u_max，z=[u_0..u_{N-1}]"""
    def __init__(self, n_iter=400):
        self.n_iter = n_iter
        self.solve_us = 0.0   # 求解耗时统计
        self._z_warm = np.zeros(3*N)   # warm start（滚动时域连续性）
        # 预计算展开系数：x_k = A^k x0 + Σ_{j<k} A^{k-1-j}B u_j（增广 9 维模型）
        A_ = A_aug; Bd_ = B_aug
        self.Ap = [np.eye(9)]
        for _ in range(N):
            self.Ap.append(A_ @ self.Ap[-1])
        self.Ab = [[None]*N for _ in range(N+1)]   # 行索引 k=0..N（含终端步）
        for k in range(1, N+1):
            for j in range(k):
                self.Ab[k][j] = self.Ap[k-1-j] @ Bd_
        # Hessian H (3N×3N) 与线性项系数
        self.H = np.zeros((3*N, 3*N))
        self.F = np.zeros((3*N, 6))
        for i in range(N):
            for j in range(N):
                Hij = np.zeros((3, 3))
                for k in range(max(i, j) + 1, N + 1):
                    Mi = self.Ab[k][i] if k > i else np.zeros((9, 3))
                    Mj = self.Ab[k][j] if k > j else np.zeros((9, 3))
                    Hij += Mi.T @ Q @ Mj
                if i == j:
                    Hij += R
                self.H[3*i:3*i+3, 3*j:3*j+3] = Hij
        self.H = 2 * self.H
        # 线性项 f(z) = 2·Σ_k (A^k x0)'Q·(∂x_k/∂z)：f_i = 2 Σ_{k>i} (A^{k-1-i}B)'Q A^k x0
        self.L = np.zeros((3*N, 9))
        for i in range(N):
            Li = np.zeros((3, 9))
            for k in range(i+1, N+1):
                Li += (self.Ab[k][i] if k > i else np.zeros((9,3))).T @ Q @ self.Ap[k]
            self.L[3*i:3*i+3] = 2 * Li
        # PGD 步长：1/λmax(H)（保守取大）
        self.alpha = 1.0 / (np.linalg.eigvalsh(self.H).max() + 1e-12)

    def update(self, q, q_des, omega, dt, omega_ref=None):
        q_err = quat_multiply(quat_conj(q_des), q)
        if q_err[0] < 0: q_err = -q_err
        x0 = np.array([q_err[1], q_err[2], q_err[3],
                       omega[0], omega[1], omega[2],
                       0.0, 0.0, 0.0])   # 执行器状态 a 初值（稳态 ω̇≈0）
        f = self.L @ x0
        lo = np.tile(-u_max, N)
        hi = np.tile(u_max, N)
        z = np.roll(self._z_warm, -3)      # warm start：上一时刻解前移
        z[-3:] = 0.0
        t0 = time.perf_counter()
        for _ in range(self.n_iter):
            grad = self.H @ z + f
            z = np.clip(z - self.alpha * grad, lo, hi)
        self._z_warm = z.copy()
        self.solve_us += time.perf_counter() - t0
        u0 = z[0:3]
        return u0[2], u0[1], u0[0]   # (df, dt, dw)

def simulate_vtol_6dof(controller, q0, omega0, T_total=5.0, q_des=None):
    """6DOF VTOL 仿真（复用 core 的 rk4_step + Propulsion）"""
    v = np.zeros(3); w = np.zeros(3)
    q = np.array(q0, dtype=float)
    prop = Propulsion(P); prop.wf = prop.wt = omega0
    prop.prev_wf = prop.prev_wt = omega0
    if q_des is None: q_des = qh
    N = int(T_total / DT)
    eps_hist = np.zeros(N)
    for i in range(N):
        q_err = quat_multiply(quat_conj(q_des), q)
        eps_hist[i] = np.linalg.norm(q_err[1:4])
        df, dt_c, dw = controller.update(q, q_des, w, omega0, DT)
        prop.update(omega0, dw, DT)
        Fx, Fy, Fz, Mx, My, Mz = prop.forces(df, dt_c)
        v, w, q = rk4_step(v, w, q, prop, P, Fx, Fy, Fz, Mx, My, Mz, DT)
    return eps_hist

def run(name, controller, q0, q_des=None):
    eps = simulate_vtol_6dof(controller, q0, W0_H, T_total=5.0, q_des=q_des)
    print('%-14s eps峰值=%.4f 末值=%.4f' % (name, eps.max(), eps[-1]))
    if hasattr(controller, 'solve_us'):
        print('%-14s 单步求解均耗: %.3f ms' % ('', controller.solve_us / len(eps) * 1000))

print('悬停配平转速 w0 = %.1f rad/s（油门 %.1f%%）' % (W0_H, W0_H / P['wMax'] * 100))
a5 = np.radians(5)
a60 = np.radians(60)

if __name__ == '__main__':
    # —— 场景 A：悬停小扰动 5°（绕 y） ——
    print('\n[场景 A] 悬停 5° 扰动（绕 y）')
    q0A = quat_multiply(qh, np.array([np.cos(a5/2), 0, np.sin(a5/2), 0]))
    run('SAS', QuatSASController(P, omega0=W0_H), q0A)
    run('LMPC(N=5)', LMPCAttitude(), q0A)

    # —— 场景 B：大扰动 60°（约束活跃） ——
    print('\n[场景 B] 悬停 60° 扰动（约束活跃）')
    q0B = quat_multiply(qh, np.array([np.cos(a60/2), 0, np.sin(a60/2), 0]))
    run('SAS', QuatSASController(P, omega0=W0_H), q0B)
    run('LMPC(N=5)', LMPCAttitude(), q0B)

    # —— 场景 C：航向指令（绕 x_b 转 20°） ——
    print('\n[场景 C] 航向指令 20°（绕 x_b）')
    q_desC = quat_multiply(qh, np.array([np.cos(np.radians(20)/2), np.sin(np.radians(20)/2), 0, 0]))
    run('SAS', QuatSASController(P, omega0=W0_H), qh, q_desC)
    run('LMPC(N=5)', LMPCAttitude(), qh, q_desC)
