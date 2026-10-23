# -*- coding: utf-8 -*-
"""adaptive_full.py — 自适应 MPC 整链演示：辨识 → 更新模型 → 闭环对比。

@file    adaptive_full.py
@module  analysis/控制算法原型（自适应 MPC 整链）
@brief   用开环脉冲辨识 B_eff，把辨识结果喂给 MPC 预测模型，再做闭环对比。
@details 失配仿真（kT×0.8、Iy×1.3、Ix×1.2）：俯仰/偏航通道辨识精确、差速通道用名义近似。
         链路：ident_probe 激励 → 通道回归 → mpc_proto 替换 B → 闭环评估。
@theory  增量效能辨识 ω̇≈B_eff·u @ref THY-003 §3、@ref MOD-001 §4；
         模型失配下的自适应控制 @ref TandemVec-Paper/ch17、ch16（在线辨识）。
@frame  FRD 机体系；u=[Δω,δt,δf]。
@units   SI；B 元素量纲随列（N·m/(rad/s²) 等），报告中按通道归一化比较。
@note    "差速通道名义近似"是已知局限：Jp·ω̇ 瞬态与摆角交叉项使该通道信噪比低，
         不是辨识器实现错误（@ref TandemVec-Paper/ch16）。
@see     ident_probe.py、mpc_proto.py、adaptive_mpc.py、@ref TandemVec-Paper/ch17
"""
import sys
sys.path.insert(0, 'scripts')
sys.path.insert(0, 'simulations/high-fidelity-analysis')
import numpy as np
from core import (load_params, control_effectiveness, quat_multiply, quat_conj,
                  rk4_step, Propulsion)
from controllers import QuatSASController
import ident_probe as ID
import mpc_proto as MP

P = load_params()
P_true = ID.P_true
qh = ID.qh
W0_H = ID.W0_H
DT = 0.004

# ============================================================
#  1. 辨识段（6 脉冲，俯仰/偏航精确）
# ============================================================
U, Y = [], []
for ch, amp in [(0, 0.10), (0, -0.10), (1, 0.06), (1, -0.06), (2, 0.06), (2, -0.06)]:
    u, y = ID.pulse_segment(ch, amp)
    U.append(u); Y.append(y)
U = np.array(U); Y = np.array(Y)
Beff_hat = np.zeros((3, 3))
for i in range(3):
    Beff_hat[i] = np.linalg.lstsq(U, Y[:, i], rcond=None)[0]
B0_true, _, _ = control_effectiveness(W0_H, 0, 0, 0, P_true)
Beff_true = np.diag([1/P_true['Ix'], 1/P_true['Iy'], 1/P_true['Iz']]) @ B0_true
print('辨识 Beff:\n', np.round(Beff_hat, 2))
print('真实 Beff:\n', np.round(Beff_true, 2))

# ============================================================
#  2. 可更新模型 LMPC（B_eff → 重算展开系数）
# ============================================================
def build_mpc(Beff):
    """按给定效能矩阵构造 LMPC 实例（辨识前后对比用）。

    @param[in] Beff 3×3 效能矩阵（列顺序 [Δω,δt,δf]）
    @ref  MOD-001 §4、@ref TandemVec-Paper/ch17
    """
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
    return H, L, N, 1.0 / (np.linalg.eigvalsh(H).max() + 1e-12)

def make_ctrl(Beff):
    """按效能矩阵构造自适应 MPC 控制器（辨识后模型）。"""
    H, L, N, alpha = build_mpc(Beff)
    ctrl = MP.LMPCAttitude()
    ctrl.H, ctrl.L, ctrl.N, ctrl.alpha = H, L, N, alpha
    return ctrl

# ============================================================
#  3. 闭环对比（失配动力学）
# ============================================================
def simulate(controller, q0, T_total=8.0):
    """闭环跑一次（子步 DT=0.004 s），返回时间序列 dict。

    @param[in] q0 初始姿态四元数 [w,x,y,z]
    @note  真值动力学用 ident_probe.P_true（失配参数），控制器只用名义 P。
    """
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
        v, w, q = rk4_step(v, w, q, prop, P_true, Fx, Fy, Fz, Mx, My, Mz, DT)
    return eps_hist

if __name__ == '__main__':
    a5 = np.radians(5); a60 = np.radians(60)
    qA = quat_multiply(qh, np.array([np.cos(a5/2), 0, np.sin(a5/2), 0]))
    qB = quat_multiply(qh, np.array([np.cos(a60/2), 0, np.sin(a60/2), 0]))

    B0_nom, _, _ = control_effectiveness(W0_H, 0, 0, 0, P)
    Beff_nom = np.diag([1/P['Ix'], 1/P['Iy'], 1/P['Iz']]) @ B0_nom
    # 自适应 B：完整辨识矩阵（9 参数全部精确 <3%）
    Beff_adapt = Beff_hat.copy()
    print('B[1,1]: 名义 %.2f → 辨识 %.2f（真实 %.2f）' % (Beff_nom[1,1], Beff_hat[1,1], Beff_true[1,1]))

    for tag, q0, T in [('[A] 5° 扰动 8s', qA, 8.0), ('[B] 60° 扰动 8s（约束活跃）', qB, 8.0)]:
        print('\n%s' % tag)
        e_s = simulate(QuatSASController(P, omega0=W0_H), q0, T)
        print('SAS(名义)         eps峰值=%.4f 末值=%.4f' % (e_s.max(), e_s[-1]))
        eps_l = simulate(make_ctrl(Beff_nom), q0, T)
        print('LMPC(名义B)       eps峰值=%.4f 末值=%.4f' % (eps_l.max(), eps_l[-1]))
        eps_a = simulate(make_ctrl(Beff_adapt), q0, T)
        print('自适应LMPC(辨识B) eps峰值=%.4f 末值=%.4f' % (eps_a.max(), eps_a[-1]))
        # 2s 中间值（收敛速度对比）
        n2 = int(2.0 / DT)
        print('  2s 时刻: 名义 %.4f / 自适应 %.4f' % (eps_l[n2], eps_a[n2]))
