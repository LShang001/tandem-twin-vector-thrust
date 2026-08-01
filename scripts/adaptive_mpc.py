# -*- coding: utf-8 -*-
"""自适应 MPC 原型：模型失配下 RLS 在线辨识 B_eff → 周期更新 LMPC 预测模型
真实动力学用失配参数（kT×0.8、kQ×0.8、Iy×1.3、Ix×1.2），控制器只用名义参数。
RLS：ω̇ ≈ B_eff·u（3 通道独立回归，遗忘因子 λ），u = [Δω, δt, δf]
对比：名义 LMPC（无辨识）vs 自适应 LMPC vs SAS
"""
import sys
sys.path.insert(0, 'simulations/high-fidelity-analysis')
import time
import numpy as np
from core import (load_params, control_effectiveness, quat_multiply, quat_conj,
                  rk4_step, Propulsion, quat_norm)

P = load_params()
P_true = dict(P)
P_true['kT'] *= 0.8; P_true['kQ'] *= 0.8
P_true['Iy'] *= 1.3; P_true['Ix'] *= 1.2   # 失配：真实 vs 名义
qh = np.array([np.cos(np.pi/4), 0, np.sin(np.pi/4), 0])
W0_H = np.sqrt(P['m']*P['g']/(2*P['kT']))
DT = 0.004
u_max = np.array([P['dwMax'], P['dMax'], P['dMax']])

# ============================================================
#  RLS 逐通道辨识（3×3 B_eff）
# ============================================================
class RLS:
    def __init__(self, n=3, lam=0.97):
        self.P = np.eye(n) * 200.0
        self.theta = np.zeros(n)
        self.lam = lam
    def update(self, phi, y):
        Pphi = self.P @ phi
        g = Pphi / (self.lam + phi @ Pphi)
        self.theta += g * (y - phi @ self.theta)
        self.P = (self.P - np.outer(g, Pphi)) / self.lam
        return self.theta.copy()

# ============================================================
#  可更新模型的 LMPC（B_eff → B_aug 重算展开系数）
# ============================================================
class AdaptiveLMPC:
    def __init__(self, n_iter=400, update_every=300, excite_until=4.0):
        self.n_iter = n_iter
        self.update_every = update_every
        self.excite_until = excite_until
        self.rls = [RLS() for _ in range(3)]
        self.step = 0
        self.u_prev = None; self.w_prev = None
        self.wf = W0_H; self.wt = W0_H   # 控制器侧滞后转速（与 prop 同步演化）
        self.solve_us = 0.0
        self.z_warm = np.zeros(3 * 30)
        self.B_eff = None
        self._rebuild(np.zeros(9))   # 初始用名义 B（占位，随后真算）
        self._rebuild_from_nominal()

    def _rebuild_from_nominal(self):
        B0, _, _ = control_effectiveness(W0_H, 0, 0, 0, P)
        Beff_nom = np.diag([1/P['Ix'], 1/P['Iy'], 1/P['Iz']]) @ B0
        self._rebuild(Beff_nom)

    def _rebuild(self, Beff_flat):
        """B_eff 9 参数 → B_aug → 展开系数 H/L"""
        Beff = Beff_flat.reshape(3, 3)
        DT_PRED, N = 0.02, 30
        Ac = np.zeros((6, 6)); Ac[:3, 3:] = 0.5 * np.eye(3)
        A6 = np.eye(6) + Ac * DT_PRED
        A_aug = np.zeros((9, 9)); A_aug[:6, :6] = A6
        A_aug[3:6, 6:9] = np.eye(3) * DT_PRED
        A_aug[6:9, 6:9] = np.eye(3) * (1.0 - DT_PRED / P['tauM'])
        B_aug = np.zeros((9, 3))
        B_aug[6:9, :] = Beff * (DT_PRED / P['tauM'])
        Q = np.diag([4, 4, 4, 1, 1, 1, 0, 0, 0])
        R = np.diag([0.1, 0.1, 0.1])
        Ap = [np.eye(9)]
        for _ in range(N): Ap.append(A_aug @ Ap[-1])
        Ab = [[None]*N for _ in range(N+1)]
        for k in range(1, N+1):
            for j in range(k): Ab[k][j] = Ap[k-1-j] @ B_aug
        H = np.zeros((3*N, 3*N))
        for i in range(N):
            for j in range(N):
                Hij = np.zeros((3, 3))
                for k in range(max(i, j) + 1, N + 1):
                    Mi = Ab[k][i] if k > i else np.zeros((9, 3))
                    Mj = Ab[k][j] if k > j else np.zeros((9, 3))
                    Hij += Mi.T @ Q @ Mj
                if i == j: Hij += R
                H[3*i:3*i+3, 3*j:3*j+3] = Hij
        H = 2 * H
        L = np.zeros((3*N, 9))
        for i in range(N):
            Li = np.zeros((3, 9))
            for k in range(i+1, N+1):
                Li += (Ab[k][i] if k > i else np.zeros((9,3))).T @ Q @ Ap[k]
            L[3*i:3*i+3] = 2 * Li
        self.A_aug, self.B_aug = A_aug, B_aug
        self.H, self.L, self.N = H, L, N
        self.alpha = 1.0 / (np.linalg.eigvalsh(H).max() + 1e-12)
        self.B_eff = Beff.copy()

    def update(self, q, q_des, w, dt, t=None):
        # —— RLS 辨识：ω̇_meas(差分) ≈ B_eff·u_prev ——
        if self.u_prev is not None and self.w_prev is not None:
            wdot = (np.array(w) - self.w_prev) / dt
            for i in range(3):
                self.rls[i].update(np.array(self.u_prev), wdot[i])
        self.w_prev = np.array(w)
        # —— RLS 更新门控：激励充分（|u| 大）才更新，防收敛后协方差漂移 ——
        # 回归量用实际滞后实现（u_eff）：ω̇ ≈ B_eff·u_eff（u_prev 未含滞后会错位）
        self.step += 1
        if self.u_prev is not None and np.linalg.norm(self.u_prev) > 0.03:
            dw_eff = (self.wf / W0_H) ** 2 - 1.0     # 实际差速（滞后实现后）
            u_eff = np.array([dw_eff, self.u_prev[1], self.u_prev[2]])
            wdot = (np.array(w) - self.w_prev) / dt
            # 减陀螺补偿（名义 I）：ω̇_corr ≈ B_eff·u_eff（悬停差速激励下陀螺项显著）
            ww = np.array(w)
            hx = P['Jp'] * (self.wf - self.wt)
            g = np.array([(P['Iz']-P['Iy'])*ww[1]*ww[2],
                          (P['Ix']-P['Iz'])*ww[2]*ww[0] - ww[2]*hx,
                          (P['Iy']-P['Ix'])*ww[0]*ww[1] + ww[1]*hx])
            wdot_c = wdot - g / np.array([P['Ix'], P['Iy'], P['Iz']])
            for i in range(3):
                self.rls[i].update(u_eff, wdot_c[i])
        # —— 滞后转速演化（与 prop.update 一致）——
        if self.u_prev is not None:
            wfT = W0_H * np.sqrt(max(0.0, 1.0 + self.u_prev[0]))
            wtT = W0_H * np.sqrt(max(0.0, 1.0 - self.u_prev[0]))
            a = min(dt / P['tauM'], 1.0)
            self.wf += (wfT - self.wf) * a
            self.wt += (wtT - self.wt) * a
        # —— 周期更新模型 ——
        if self.step % self.update_every == 0:
            Beff_hat = np.array([self.rls[i].theta for i in range(3)])
            self._rebuild(Beff_hat)

        # —— MPC 求解 ——
        q_err = quat_multiply(quat_conj(q_des), q)
        if q_err[0] < 0: q_err = -q_err
        x0 = np.array([q_err[1], q_err[2], q_err[3],
                       w[0], w[1], w[2], 0, 0, 0])
        f = self.L @ x0
        lo = np.tile(-u_max, self.N); hi = np.tile(u_max, self.N)
        z = np.roll(self.z_warm, -3); z[-3:] = 0.0
        t0 = time.perf_counter()
        for _ in range(self.n_iter):
            z = np.clip(z - self.alpha * (self.H @ z + f), lo, hi)
        self.z_warm = z.copy()
        self.solve_us += time.perf_counter() - t0
        u0 = z[0:3]
        # 辨识激励：初期三通道轮流方波（保证持续激励覆盖 B_eff 全部 9 参数）
        if t is not None and t < self.excite_until:
            phase = 1.0 if (int(t / 0.25) % 2 == 0) else -1.0
            ch = int(t / 0.5) % 3          # 轮流激励 dw/dt/df 通道
            excite = [0.08, 0.04, 0.04][ch] * phase
            u0[ch] = np.clip(u0[ch] + excite, -u_max[ch], u_max[ch])
        u0 = np.clip(u0, -u_max, u_max)
        self.u_prev = u0
        return u0[2], u0[1], u0[0]

# ============================================================
#  仿真（真实参数 P_true 动力学）
# ============================================================
def simulate(controller, q0, T_total=8.0):
    v = np.zeros(3); w = np.zeros(3)
    q = np.array(q0, dtype=float)
    prop = Propulsion(P_true); prop.wf = prop.wt = W0_H
    prop.prev_wf = prop.prev_wt = W0_H
    N = int(T_total / DT)
    eps_hist = np.zeros(N)
    for i in range(N):
        q_err = quat_multiply(quat_conj(qh), q)
        eps_hist[i] = np.linalg.norm(q_err[1:4])
        df, dt_c, dw = controller.update(q, qh, w, DT, i * DT)
        prop.update(W0_H, dw, DT)
        Fx, Fy, Fz, Mx, My, Mz = prop.forces(df, dt_c)
        v, w, q = rk4_step(v, w, q, prop, P_true, Fx, Fy, Fz, Mx, My, Mz, DT)
    return eps_hist

if __name__ == '__main__':
    from controllers import QuatSASController
    from mpc_proto import LMPCAttitude
    # 真实 B_eff（供对照）
    B0_true, _, _ = control_effectiveness(W0_H, 0, 0, 0, P_true)
    Beff_true = np.diag([1/P_true['Ix'], 1/P_true['Iy'], 1/P_true['Iz']]) @ B0_true

    a5 = np.radians(5)
    qA = quat_multiply(qh, np.array([np.cos(a5/2), 0, np.sin(a5/2), 0]))
    print('真实 Beff（对照）:\n', np.round(Beff_true, 2))
    print('\n[场景] 悬停 5° 扰动，模型失配（kT×0.8、Iy×1.3），8s')

    eps_sas = simulate(QuatSASController(P, omega0=W0_H), qA)
    print('SAS(名义P)        eps峰值=%.4f 末值=%.4f' % (eps_sas.max(), eps_sas[-1]))

    ctrl_l = LMPCAttitude()
    eps_l = simulate(ctrl_l, qA)
    print('LMPC(名义模型)    eps峰值=%.4f 末值=%.4f' % (eps_l.max(), eps_l[-1]))

    ctrl_a = AdaptiveLMPC()
    eps_a = simulate(ctrl_a, qA)
    print('自适应LMPC(RLS)   eps峰值=%.4f 末值=%.4f' % (eps_a.max(), eps_a[-1]))
    print('  辨识 Beff 收敛值:\n', np.round(ctrl_a.B_eff, 2))
    err = np.linalg.norm(ctrl_a.B_eff - Beff_true) / np.linalg.norm(Beff_true)
    print('  辨识相对误差: %.1f%%' % (err * 100))
