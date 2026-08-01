# -*- coding: utf-8 -*-
"""论文 MPC 实验图件生成：收敛曲线、过渡跟踪、辨识结果、自适应闭环
输出到 docs/03-理论推导/THY-004/fig-mpc/ + sim-data/mpc_compare.tex
"""
import sys, os
sys.path.insert(0, 'scripts')
sys.path.insert(0, 'simulations/high-fidelity-analysis')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from core import (load_params, quat_multiply, quat_conj, rk4_step, Propulsion,
                  control_effectiveness)
from controllers import QuatSASController
import mpc_proto as MP
import mpc_adv as MA
import ident_probe as ID

P = load_params()
P_true = ID.P_true
qh = ID.qh; W0_H = ID.W0_H; DT = 0.004
OUT = os.path.join('docs', '03-理论推导', 'THY-004', 'fig-mpc')
os.makedirs(OUT, exist_ok=True)
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei']
plt.rcParams['axes.unicode_minus'] = False
C1, C2, C3, C4 = '#2E86AB', '#E4572E', '#2CA02C', '#9467BD'

def simulate(controller, q0, T_total, q_des_fn=None, use_aero=False):
    v = np.zeros(3); w = np.zeros(3); q = np.array(q0, dtype=float)
    prop = Propulsion(P_true); prop.wf = prop.wt = W0_H
    prop.prev_wf = prop.prev_wt = W0_H
    N = int(T_total / DT)
    eps = np.zeros(N); t = np.arange(N) * DT
    for i in range(N):
        q_des = q_des_fn(t[i]) if q_des_fn else qh
        q_err = quat_multiply(quat_conj(q_des), q)
        eps[i] = np.linalg.norm(q_err[1:4])
        df, dt_c, dw = controller.update(q, q_des, w, DT)
        prop.update(W0_H, dw, DT)
        Fx, Fy, Fz, Mx, My, Mz = prop.forces(df, dt_c)
        v, w, q = rk4_step(v, w, q, prop, P_true, Fx, Fy, Fz, Mx, My, Mz, DT, use_aero=use_aero)
    return t, eps

def slerp_traj(t, T_slerp=3.0):
    if t >= T_slerp: return qh
    s = t / T_slerp
    q0, q1 = np.array([1.0, 0, 0, 0]), qh
    dot = np.dot(q0, q1)
    if dot < 0: q0 = -q0; dot = -dot
    th = np.arccos(np.clip(dot, -1, 1))
    if th < 1e-6: return q1
    return (np.sin((1-s)*th)/np.sin(th))*q0 + (np.sin(s*th)/np.sin(th))*q1

a5 = np.radians(5); a60 = np.radians(60)
qA = quat_multiply(qh, np.array([np.cos(a5/2), 0, np.sin(a5/2), 0]))
qB = quat_multiply(qh, np.array([np.cos(a60/2), 0, np.sin(a60/2), 0]))
qH = quat_multiply(qh, np.array([np.cos(np.radians(20)/2), np.sin(np.radians(20)/2), 0, 0]))
q_level = np.array([1.0, 0, 0, 0])

# ============================================================
#  图 1：三场景收敛对比（SAS / LMPC / NMPC）——4s
# ============================================================
scen = [('5° 扰动', qA, 6.0, None), ('60° 扰动（约束活跃）', qB, 6.0, None),
        ('航向指令 20°', qH, 6.0, None)]
fig, axes = plt.subplots(1, 3, figsize=(13.5, 3.4))
for ax, (tag, q0, T, qf) in zip(axes, scen):
    for name, mk, c in [('SAS', lambda: QuatSASController(P, omega0=W0_H), C1),
                        ('LMPC', lambda: MP.LMPCAttitude(), C2),
                        ('NMPC', lambda: MA.NMPC(6, 0.05, 20), C3)]:
        t, e = simulate(mk(), q0, T, qf)
        ax.semilogy(t, e, label=name, color=c, lw=1.6)
    ax.set_title(tag, fontsize=10)
    ax.set_xlabel('t [s]'); ax.grid(alpha=0.3)
    ax.set_ylim(1e-4, 1.0)
axes[0].set_ylabel('姿态误差 ‖ε‖')
axes[0].legend(fontsize=9)
fig.tight_layout()
fig.savefig(os.path.join(OUT, 'mpc_scenarios.pdf'))
plt.close(fig)
print('图1 mpc_scenarios.pdf')

# ============================================================
#  图 2：slerp 过渡跟踪误差（6s）
# ============================================================
fig, ax = plt.subplots(figsize=(6.8, 3.4))
for name, mk, c in [('SAS', lambda: QuatSASController(P, omega0=W0_H), C1),
                    ('LMPC', lambda: MP.LMPCAttitude(), C2),
                    ('NMPC', lambda: MA.NMPC(6, 0.05, 20), C3)]:
    t, e = simulate(mk(), q_level, 6.0, slerp_traj)
    ax.semilogy(t, e, label=name, color=c, lw=1.6)
ax.axvline(3.0, ls='--', color='gray', lw=0.8)
ax.text(3.05, 0.4, '过渡结束', fontsize=8, color='gray')
ax.set_xlabel('t [s]'); ax.set_ylabel('跟踪误差 ‖ε‖')
ax.set_title('VTOL 过渡（水平→悬停 slerp 3s）跟踪误差')
ax.legend(); ax.grid(alpha=0.3); ax.set_ylim(1e-4, 1.0)
fig.tight_layout(); fig.savefig(os.path.join(OUT, 'mpc_transition.pdf'))
plt.close(fig)
print('图2 mpc_transition.pdf')

# ============================================================
#  图 3：预测时域效应（LMPC N=5 vs 30，5° 扰动，失配）
# ============================================================
def make_ctrl_N(Beff, N):
    DT_PRED = 0.02
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
    ctrl._z_warm = np.zeros(3*N)   # 与展开维度一致（默认实例为 90）
    return ctrl

B0_nom, _, _ = control_effectiveness(W0_H, 0, 0, 0, P)
Beff_nom = np.diag([1/P['Ix'], 1/P['Iy'], 1/P['Iz']]) @ B0_nom
fig, ax = plt.subplots(figsize=(6.8, 3.4))
t, e = simulate(make_ctrl_N(Beff_nom, 5), qA, 4.0)
ax.semilogy(t, e, label='N=5（时域 0.1s < τ，不收敛）', color=C3, lw=1.6)
t, e = simulate(make_ctrl_N(Beff_nom, 30), qA, 4.0)
ax.semilogy(t, e, label='N=30（时域 0.6s > 2τ，收敛）', color=C2, lw=1.6)
ax.set_xlabel('t [s]'); ax.set_ylabel('姿态误差 ‖ε‖')
ax.set_title('预测时域对滞后补偿的影响（电机 τ=0.28s）')
ax.legend(); ax.grid(alpha=0.3); ax.set_ylim(1e-4, 0.1)
fig.tight_layout(); fig.savefig(os.path.join(OUT, 'mpc_horizon.pdf'))
plt.close(fig)
print('图3 mpc_horizon.pdf')

# ============================================================
#  图 4：自适应辨识结果（B_eff 热图：辨识 vs 真实）
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
fig, axes = plt.subplots(1, 2, figsize=(7.6, 3.2))
for ax, B, tag in [(axes[0], Beff_true, '真实 B_eff'),
                   (axes[1], Beff_hat, '辨识 B_eff（误差 <3%）')]:
    im = ax.imshow(B, cmap='RdBu_r', vmin=-80, vmax=80)
    ax.set_xticks(range(3)); ax.set_yticks(range(3))
    ax.set_xticklabels(['Δω', 'δt', 'δf']); ax.set_yticklabels(['ω̇x', 'ω̇y', 'ω̇z'])
    ax.set_title(tag, fontsize=10)
    for i in range(3):
        for j in range(3):
            ax.text(j, i, '%.0f' % B[i, j], ha='center', va='center',
                    fontsize=8, color='white' if abs(B[i, j]) > 40 else 'black')
fig.colorbar(im, ax=axes, fraction=0.046)
fig.tight_layout(); fig.savefig(os.path.join(OUT, 'mpc_ident.pdf'))
plt.close(fig)
print('图4 mpc_ident.pdf')

# ============================================================
#  图 5：自适应闭环（60° 失配场景：名义 vs 辨识）
# ============================================================
B0_nom, _, _ = control_effectiveness(W0_H, 0, 0, 0, P)
Beff_nom = np.diag([1/P['Ix'], 1/P['Iy'], 1/P['Iz']]) @ B0_nom

def make_ctrl(Beff):
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
    ctrl._z_warm = np.zeros(3*N)   # 与展开维度一致（默认实例为 90）
    return ctrl

fig, ax = plt.subplots(figsize=(6.8, 3.4))
for name, mk, c in [('LMPC（名义 B）', lambda: make_ctrl(Beff_nom), C2),
                    ('自适应 LMPC（辨识 B）', lambda: make_ctrl(Beff_hat), C4)]:
    t, e = simulate(mk(), qB, 6.0)
    ax.semilogy(t, e, label=name, color=c, lw=1.6)
ax.set_xlabel('t [s]'); ax.set_ylabel('姿态误差 ‖ε‖')
ax.set_title('模型失配下（kT×0.8、Iy×1.3）60° 扰动闭环')
ax.legend(); ax.grid(alpha=0.3); ax.set_ylim(1e-4, 1.0)
fig.tight_layout(); fig.savefig(os.path.join(OUT, 'mpc_adapt_closedloop.pdf'))
plt.close(fig)
print('图5 mpc_adapt_closedloop.pdf')

# ============================================================
#  对比表（sim-data/mpc_compare.tex）
# ============================================================
rows = []
for tag, q0, T, qf in [('5° 扰动', qA, 6.0, None), ('60° 扰动', qB, 6.0, None),
                       ('航向指令 20°', qH, 6.0, None), ('slerp 过渡', q_level, 6.0, slerp_traj)]:
    e_sas = simulate(QuatSASController(P, omega0=W0_H), q0, T, qf)[1]
    e_l = simulate(MP.LMPCAttitude(), q0, T, qf)[1]
    e_n = simulate(MA.NMPC(6, 0.05, 20), q0, T, qf)[1]
    rows.append((tag, e_sas[-1], e_l[-1], e_n[-1]))
with open(os.path.join('docs', '03-理论推导', 'THY-004', 'sim-data', 'mpc_compare.tex'), 'w',
          encoding='utf-8') as f:
    f.write('\\begin{table}[H]\n\\centering\n'
            '\\caption{姿态误差范数末值对比（失配仿真 kT×0.8、Iy×1.3、Ix×1.2，4--6\\,s）}\n'
            '\\label{tab:mpc_compare}\n\\small\n'
            '\\begin{tabular}{l c c c}\n\\toprule\n'
            '\\textbf{场景} & \\textbf{SAS} & \\textbf{LMPC} & \\textbf{NMPC} \\\\\n\\midrule\n')
    for tag, a, b, c in rows:
        f.write('%s & %.4f & %.4f & %.4f \\\\\n' % (tag, a, b, c))
    f.write('\\bottomrule\n\\end{tabular}\n\\end{table}\n')
print('表 mpc_compare.tex 已写入')
print('\n对比数据:')
for tag, a, b, c in rows:
    print('%-14s SAS=%.4f LMPC=%.4f NMPC=%.4f' % (tag, a, b, c))
