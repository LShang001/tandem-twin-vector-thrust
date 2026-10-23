# -*- coding: utf-8 -*-
"""adaptive_nmpc.py — 自适应 NMPC：物理非线性模型 + 辨识线性校正。

@file    adaptive_nmpc.py
@module  analysis/控制算法原型（自适应 NMPC）
@brief   预测模型取"物理非线性项 + 辨识 B 偏差修正"，在失配仿真下与其他方案对比。
@details 预测式：ω̇_pred = 非线性(θ) + (B̂ − B̄)·u；失配参数（kT×0.8、Iy×1.3、Ix×1.2）下
         对比 SAS / LMPC(名义|辨识) / NMPC(名义|辨识)。
@theory  非线性刚体转动方程 @ref MOD-001 §5.2；
         非线性预测控制 @ref TandemVec-Paper/ch17；在线辨识 @ref TandemVec-Paper/ch16。
@frame  FRD 机体系。
@units   SI。
@note    该结构把"模型可信度"集中在 B 上：物理非线性项保留、参数误差用辨识修正，
         因此对气动/推进参数误差的鲁棒性优于纯 LMPC，代价是每步非线性求解耗时。
@see     adaptive_mpc.py、mpc_adv.py、@ref TandemVec-Paper/ch17
"""
import sys
sys.path.insert(0, 'scripts')
sys.path.insert(0, 'simulations/high-fidelity-analysis')
import numpy as np
from core import (load_params, control_effectiveness, quat_multiply, quat_conj,
                  rk4_step, Propulsion)
from controllers import QuatSASController
import mpc_adv as MA
import ident_probe as ID
import mpc_proto as MP

P = load_params()
P_true = ID.P_true
qh = ID.qh; W0_H = ID.W0_H; DT = 0.004

# —— 辨识段：6 脉冲 → B_eff（9 参数 <3%）——
U, Y = [], []
for ch, amp in [(0, 0.10), (0, -0.10), (1, 0.06), (1, -0.06), (2, 0.06), (2, -0.06)]:
    u, y = ID.pulse_segment(ch, amp)
    U.append(u); Y.append(y)
U = np.array(U); Y = np.array(Y)
Beff_hat = np.zeros((3, 3))
for i in range(3):
    Beff_hat[i] = np.linalg.lstsq(U, Y[:, i], rcond=None)[0]
B0_nom, _, _ = control_effectiveness(W0_H, 0, 0, 0, P)
Beff_nom = np.diag([1/P['Ix'], 1/P['Iy'], 1/P['Iz']]) @ B0_nom
dBeff = Beff_hat - Beff_nom

# —— 可更新 LMPC（B_eff → 展开系数）——
def make_ctrl(Beff):
    """按效能矩阵构造控制器：None=名义 NMPC，B̂=辨识校正后的 NMPC。"""
    DT_PRED, N = 0.02, 30
    Ac = np.zeros((6, 6)); Ac[:3, 3:] = 0.5 * np.eye(3)
    A_aug = np.zeros((9, 9)); A_aug[:6, :6] = np.eye(6) + Ac * DT_PRED
    A_aug[3:6, 6:9] = np.eye(3) * DT_PRED
    A_aug[6:9, 6:9] = np.eye(3) * (1.0 - DT_PRED / P['tauM'])
    B_aug = np.zeros((9, 3)); B_aug[6:9, :] = Beff * (DT_PRED / P['tauM'])
    Q = np.diag([4, 4, 4, 1, 1, 1, 0, 0, 0]); R = np.diag([0.1, 0.1, 0.1])
    Ap = [np.eye(9)]
    for _ in range(N): Ap.append(A_aug @ Ap[-1])
    Ab = [[None]*N for _ in range(N+1)]
    for k in range(1, N+1):
        for j in range(k): Ab[k][j] = Ap[k-1-j] @ B_aug
    H = np.zeros((3*N, 3*N))
    for i in range(N):
        for j in range(N):
            Hij = np.zeros((3, 3))
            for k in range(max(i, j)+1, N+1):
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
    ctrl = MP.LMPCAttitude()
    ctrl.H, ctrl.L, ctrl.N, ctrl.alpha = H, L, N, 1.0/(np.linalg.eigvalsh(H).max()+1e-12)
    return ctrl

# —— 自适应 NMPC（物理 + 辨识线性校正）——
class AdaptiveNMPC(MA.NMPC):
    """在物理非线性预测上叠加 (B̂−B̄)·u 线性校正项的自适应 NMPC。

    @ref TandemVec-Paper/ch16、ch17
    """
    def _predict(self, x, u):
        x1 = super()._predict(x, u)
        x1[4:7] += (dBeff @ np.array(u)) * self.dt_p
        return x1

# —— 闭环仿真（失配动力学，无气动）——
def simulate(controller, q0, T_total):
    """闭环跑一次（子步 DT=0.004 s），返回时间序列 dict。"""
    v = np.zeros(3); w = np.zeros(3); q = np.array(q0, dtype=float)
    prop = Propulsion(P_true); prop.wf = prop.wt = W0_H
    prop.prev_wf = prop.prev_wt = W0_H
    N = int(T_total / DT)
    eps_hist = np.zeros(N)
    for i in range(N):
        q_err = quat_multiply(quat_conj(qh), q)
        eps_hist[i] = np.linalg.norm(q_err[1:4])
        df, dt_c, dw = controller.update(q, qh, w, DT)
        prop.update(W0_H, dw, DT)
        Fx, Fy, Fz, Mx, My, Mz = prop.forces(df, dt_c)
        v, w, q = rk4_step(v, w, q, prop, P_true, Fx, Fy, Fz, Mx, My, Mz, DT, use_aero=False)
    return eps_hist

if __name__ == '__main__':
    a5 = np.radians(5); a60 = np.radians(60)
    qA = quat_multiply(qh, np.array([np.cos(a5/2), 0, np.sin(a5/2), 0]))
    qB = quat_multiply(qh, np.array([np.cos(a60/2), 0, np.sin(a60/2), 0]))
    print('辨识 B_eff（真值对照）:\n', np.round(Beff_hat, 2), '\n', np.round(ID.Beff_true, 2))
    print('dBeff 校正项:\n', np.round(dBeff, 2))
    for tag, q0 in [('[A] 5° 扰动 6s', qA), ('[B] 60° 扰动 6s（约束活跃）', qB)]:
        print('\n%s' % tag)
        e = simulate(QuatSASController(P, omega0=W0_H), q0, 6.0)
        print('SAS(名义)          eps末值=%.4f' % e[-1])
        e = simulate(make_ctrl(Beff_nom), q0, 6.0)
        print('LMPC(名义B)        eps末值=%.4f' % e[-1])
        e = simulate(make_ctrl(Beff_hat), q0, 6.0)
        print('自适应LMPC(辨识B)  eps末值=%.4f' % e[-1])
        e = simulate(MA.NMPC(6, 0.05, 20), q0, 6.0)
        print('NMPC(名义)         eps末值=%.4f' % e[-1])
        e = simulate(AdaptiveNMPC(6, 0.05, 20), q0, 6.0)
        print('自适应NMPC(辨识)   eps末值=%.4f' % e[-1])
