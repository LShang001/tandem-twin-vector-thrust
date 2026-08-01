# -*- coding: utf-8 -*-
"""高级 MPC 原型：Offset-free LMPC + NMPC（非线性预测）对比 SAS/LMPC
场景：VTOL 悬停扰动（5°/60°）+ slerp 过渡（水平→悬停）
Offset-free：增广常数扰动 d 估计（消除模型误差稳态残差）
NMPC：完整非线性模型（四元数运动学 + 陀螺 ω×(Iω)+ω×h + 推进一阶滞后 + 推进非线性）
"""
import sys
sys.path.insert(0, 'simulations/high-fidelity-analysis')
import time
import numpy as np
from scipy.optimize import minimize
from core import (load_params, control_effectiveness, quat_multiply, quat_conj,
                  rk4_step, Propulsion, quat_norm)

P = load_params()
qh = np.array([np.cos(np.pi/4), 0, np.sin(np.pi/4), 0])   # 机头朝天
q_level = np.array([1.0, 0, 0, 0])
W0_H = np.sqrt(P['m']*P['g']/(2*P['kT']))
DT = 0.004
u_max = np.array([P['dwMax'], P['dMax'], P['dMax']])

# ============================================================
#  共享：仿真与指标
# ============================================================
def simulate_6dof(controller, q0, T_total=5.0, q_des_fn=None):
    v = np.zeros(3); w = np.zeros(3)
    q = np.array(q0, dtype=float)
    prop = Propulsion(P); prop.wf = prop.wt = W0_H
    prop.prev_wf = prop.prev_wt = W0_H
    N = int(T_total / DT)
    eps_hist = np.zeros(N)
    for i in range(N):
        q_des = q_des_fn(i * DT) if q_des_fn else qh
        q_err = quat_multiply(quat_conj(q_des), q)
        eps_hist[i] = np.linalg.norm(q_err[1:4])
        df, dt_c, dw = controller.update(q, q_des, w, DT)
        prop.update(W0_H, dw, DT)
        Fx, Fy, Fz, Mx, My, Mz = prop.forces(df, dt_c)
        v, w, q = rk4_step(v, w, q, prop, P, Fx, Fy, Fz, Mx, My, Mz, DT)
    return eps_hist

def slerp_traj(t, T_slerp=3.0):
    """水平→悬停 slerp 参考轨迹（3s）"""
    if t >= T_slerp: return qh
    s = t / T_slerp
    q0, q1 = q_level, qh
    dot = np.dot(q0, q1)
    if dot < 0: q0 = -q0; dot = -dot
    theta = np.arccos(np.clip(dot, -1, 1))
    if theta < 1e-6: return q1
    return (np.sin((1-s)*theta)/np.sin(theta))*q0 + (np.sin(s*theta)/np.sin(theta))*q1

# ============================================================
#  Offset-free LMPC（增广常数扰动 d：ω̇ = a + d，d 在线估计）
# ============================================================
class OffsetFreeLMPC:
    def __init__(self, n_iter=400):
        self.n_iter = n_iter
        from mpc_proto import (A_aug, B_aug, Q, R, N, u_max as UM)
        self.A = A_aug; self.B = B_aug; self.Q = Q; self.R = R
        self.N = N; self.u_max = UM
        # 增广 d ∈ R³（作用于 ω̇）：x9 = [ε, ω, a] → x12 = [ε, ω, a, d]
        self.Ad = np.eye(12)
        self.Ad[:9, :9] = A_aug
        self.Ad[3:6, 9:12] = np.eye(3) * 0.02    # ω ← d
        self.Bd = np.zeros((12, 3)); self.Bd[:9, :] = B_aug
        Qd = np.zeros((12, 12)); Qd[:9, :9] = Q
        self.Qd = Qd
        self.alpha = 1.0 / (np.linalg.eigvalsh(self.Qd).max() + 1.0)  # 占位
        # 展开系数（12 维）
        self.Ap = [np.eye(12)]
        for _ in range(N): self.Ap.append(self.Ad @ self.Ap[-1])
        self.Ab = [[None]*N for _ in range(N+1)]
        for k in range(1, N+1):
            for j in range(k): self.Ab[k][j] = self.Ap[k-1-j] @ self.Bd
        self.H = np.zeros((3*N, 3*N))
        for i in range(N):
            for j in range(N):
                Hij = np.zeros((3, 3))
                for k in range(max(i, j) + 1, N + 1):
                    Mi = self.Ab[k][i] if k > i else np.zeros((12, 3))
                    Mj = self.Ab[k][j] if k > j else np.zeros((12, 3))
                    Hij += Mi.T @ self.Qd @ Mj
                if i == j: Hij += self.R
                self.H[3*i:3*i+3, 3*j:3*j+3] = Hij
        self.H = 2 * self.H
        self.L = np.zeros((3*N, 12))
        for i in range(N):
            Li = np.zeros((3, 12))
            for k in range(i+1, N+1):
                Li += (self.Ab[k][i] if k > i else np.zeros((12,3))).T @ self.Qd @ self.Ap[k]
            self.L[3*i:3*i+3] = 2 * Li
        self.alpha = 1.0 / (np.linalg.eigvalsh(self.H).max() + 1e-12)
        self.d = np.zeros(3)          # 扰动估计
        self.w_prev = None
        self.z_warm = np.zeros(3*N)
        self.solve_us = 0.0

    def update(self, q, q_des, w, dt, t=None):
        # —— 扰动估计：ω̇_meas(差分) − ω̇_model(滞后 a 部分) ——
        if self.w_prev is not None:
            wdot_meas = (np.array(w) - self.w_prev) / dt
            self.d += 0.2 * (wdot_meas - self.d)   # 扰动低通（增益 0.2）
        self.w_prev = np.array(w)
        q_err = quat_multiply(quat_conj(q_des), q)
        if q_err[0] < 0: q_err = -q_err
        x0 = np.array([q_err[1], q_err[2], q_err[3],
                       w[0], w[1], w[2], 0, 0, 0,
                       self.d[0], self.d[1], self.d[2]])
        f = self.L @ x0
        lo = np.tile(-self.u_max, self.N); hi = np.tile(self.u_max, self.N)
        z = np.roll(self.z_warm, -3); z[-3:] = 0.0
        t0 = time.perf_counter()
        for _ in range(self.n_iter):
            z = np.clip(z - self.alpha * (self.H @ z + f), lo, hi)
        self.z_warm = z.copy()
        self.solve_us += time.perf_counter() - t0
        return z[2], z[1], z[0]

# ============================================================
#  NMPC：完整非线性预测模型 + SLSQP
#  预测状态 x = [q(4), ω(3), wf, wt]（9 维），决策 u_0..u_{N-1}
# ============================================================
class NMPC:
    def __init__(self, n_horizon=8, dt_pred=0.04, n_iter=25):
        self.N = n_horizon
        self.dt_p = dt_pred
        self.n_iter = n_iter
        self.solve_us = 0.0
        self.z_warm = None

    def _predict(self, x, u):
        """非线性一步预测（欧拉）"""
        q = x[0:4]; w = x[4:7]; wf = x[7]; wt = x[8]
        # 推进一阶滞后
        wfT = W0_H * np.sqrt(max(0.0, 1.0 + u[0])); wtT = W0_H * np.sqrt(max(0.0, 1.0 - u[0]))
        a = min(self.dt_p / P['tauM'], 1.0)
        wf1 = wf + (wfT - wf) * a; wt1 = wt + (wtT - wt) * a
        Tf = P['kT'] * wf1 * wf1; Tt = P['kT'] * wt1 * wt1
        Qf = P['kQ'] * wf1 * wf1; Qt = P['kQ'] * wt1 * wt1
        cf = np.cos(u[2]); sf = np.sin(u[2]); ct = np.cos(u[1]); st = np.sin(u[1])
        M = np.array([-Qf*cf + Qt*ct, -P['b']*Tt*st - Qf*sf, P['a']*Tf*sf - Qt*st])
        # 陀螺耦合
        hx = P['Jp'] * (wf1 - wt1)
        g = np.array([(P['Iz']-P['Iy'])*w[1]*w[2],
                      (P['Ix']-P['Iz'])*w[2]*w[0] - w[2]*hx,
                      (P['Iy']-P['Ix'])*w[0]*w[1] + w[1]*hx])
        wdot = (M - g) / np.array([P['Ix'], P['Iy'], P['Iz']])
        w1 = w + wdot * self.dt_p
        # 四元数积分（小旋转四元数 w 分量必须≈1，否则模≈|ω|dt/2 归一化后翻转）
        qw = np.array([1, w1[0]*0.5*self.dt_p, w1[1]*0.5*self.dt_p, w1[2]*0.5*self.dt_p])
        q1 = quat_multiply(q, qw)
        q1 = quat_norm(q1)
        return np.concatenate([q1, w1, [wf1, wt1]])

    def _cost(self, z, x0, q_des):
        x = x0.copy(); J = 0.0
        for k in range(self.N):
            u = z[k*3:(k+1)*3]
            J += 0.5 * u @ np.diag([0.5, 0.5, 0.5]) @ u
            x = self._predict(x, u)
            q_err = quat_multiply(quat_conj(q_des), x[0:4])
            eps = q_err[1:4]
            J += 4.0 * eps @ eps + 0.5 * x[4:7] @ x[4:7]
        q_err = quat_multiply(quat_conj(q_des), x[0:4])
        J += 10.0 * (q_err[1:4] @ q_err[1:4])   # 终端
        return J

    def update(self, q, q_des, w, dt, t=None):
        x0 = np.concatenate([q, w, [W0_H, W0_H]])
        z0 = np.zeros(3*self.N) if self.z_warm is None else np.roll(self.z_warm, -3)
        t0 = time.perf_counter()
        res = minimize(lambda z: self._cost(z, x0, q_des), z0, method='SLSQP',
                       bounds=[(-u_max[i%3], u_max[i%3]) for i in range(3*self.N)],
                       options={'maxiter': self.n_iter, 'ftol': 1e-6})
        self.solve_us += time.perf_counter() - t0
        self.z_warm = res.x.copy()
        u0 = res.x[0:3]
        return u0[2], u0[1], u0[0]

# ============================================================
#  场景与对比
# ============================================================
def run(name, controller, q0, T_total=5.0, q_des_fn=None):
    eps = simulate_6dof(controller, q0, T_total, q_des_fn)
    extra = ''
    if hasattr(controller, 'solve_us'):
        extra = '  [求解均耗 %.2f ms]' % (controller.solve_us / len(eps) * 1000)
    print('%-16s eps峰值=%.4f 末值=%.4f%s' % (name, eps.max(), eps[-1], extra))
    return eps

if __name__ == '__main__':
    from controllers import QuatSASController
    a5 = np.radians(5); a60 = np.radians(60)
    print('w0 = %.1f rad/s（油门 %.1f%%）' % (W0_H, W0_H / P['wMax'] * 100))

    # 场景 A：5° 扰动
    print('\n[场景 A] 5° 扰动（绕 y，3s）')
    qA = quat_multiply(qh, np.array([np.cos(a5/2), 0, np.sin(a5/2), 0]))
    run('SAS', QuatSASController(P, omega0=W0_H), qA, T_total=3.0)
    run('LMPC', __import__('mpc_proto').LMPCAttitude(), qA, T_total=3.0)
    run('OF-LMPC', OffsetFreeLMPC(), qA, T_total=3.0)
    run('NMPC(N=6)', NMPC(6, 0.05, 20), qA, T_total=3.0)

    # 场景 B：60° 大扰动（约束活跃）
    print('\n[场景 B] 60° 扰动（约束活跃，4s）')
    qB = quat_multiply(qh, np.array([np.cos(a60/2), 0, np.sin(a60/2), 0]))
    run('SAS', QuatSASController(P, omega0=W0_H), qB, T_total=4.0)
    run('OF-LMPC', OffsetFreeLMPC(), qB, T_total=4.0)
    run('NMPC(N=8)', NMPC(), qB, T_total=4.0)

    # 场景 D：slerp 过渡（水平→悬停 3s，NMPC 招牌场景）
    print('\n[场景 D] slerp 过渡 3s（水平→悬停，6s）')
    run('SAS', QuatSASController(P, omega0=W0_H), q_level, T_total=6.0, q_des_fn=slerp_traj)
    run('OF-LMPC', OffsetFreeLMPC(), q_level, T_total=6.0, q_des_fn=slerp_traj)
    run('NMPC(N=8)', NMPC(), q_level, T_total=6.0, q_des_fn=slerp_traj)
