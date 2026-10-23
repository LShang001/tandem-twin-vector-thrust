# -*- coding: utf-8 -*-
"""paper_fgm_fig.py — 论文图：FGM（快速梯度法）vs PGD 的 QP 收敛轨迹。

@file    paper_fgm_fig.py
@module  analysis/图件生成（QP 求解器对比）
@brief   在同一个二次型上对比 FGM 与 PGD 的迭代收敛，画收敛曲线图。
@details 目标 f(z)=½z'Hz+f'z（N=30 展开式），最优值 f* 用 5000 次 PGD 迭代近似；
         初态取 5° 姿态误差对应的 QP。
         运行：仓库根目录 `py -3.12 scripts/paper_fgm_fig.py`。
@theory  二次规划与梯度类求解 @ref TandemVec-Paper/ch17（MPC 的 QP 子问题）；
         MPC 在线求解的实时性约束同上。
@frame  N/A（纯数值优化）
@units   目标函数无量纲化；迭代次数为整数。
@note    f* 是**数值近似**（5000 次 PGD），不是解析最优；比较收敛率时可以，
         但不能把该 f* 当精确下界用于绝对最优性证明。
@see     mpc_proto.py、@ref TandemVec-Paper/ch17
"""
import sys, os
sys.path.insert(0, 'scripts')
sys.path.insert(0, 'simulations/high-fidelity-analysis')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei']
plt.rcParams['axes.unicode_minus'] = False
from core import load_params, control_effectiveness, quat_multiply, quat_conj
import mpc_proto as MP

P = load_params()
qh = np.array([np.cos(np.pi/4), 0, np.sin(np.pi/4), 0])
W0_H = MP.W0_H
DT = 0.004

# 构建 N=30 模型（复用 mpc_proto 的全局 H/L——LMPCAttitude 已构建）
mpc = MP.LMPCAttitude()
H = mpc.H
# 5° 绕 y 误差状态
a5 = np.radians(5)
q0 = quat_multiply(qh, np.array([np.cos(a5/2), 0, np.sin(a5/2), 0]))
qe = quat_multiply(quat_conj(qh), q0)
x0 = np.array([qe[1], qe[2], qe[3], 0, 0, 0, 0, 0, 0])
f = mpc.L @ x0
u_max = MP.u_max
lo = np.tile(-u_max, MP.N); hi = np.tile(u_max, MP.N)
alpha = mpc.alpha

def cost(z):
    """二次目标 f(z)=½zᵀHz+fᵀz（无量纲），收敛曲线纵轴的取值函数。"""
    return 0.5 * z @ H @ z + f @ z

# f*：5000 迭代 PGD
z = np.zeros(3 * MP.N)
for _ in range(5000):
    z = np.clip(z - alpha * (H @ z + f), lo, hi)
fstar = cost(z)

def pgd_traj(n_iter, warm=False):
    """投影梯度下降（PGD）迭代轨迹：返回每次迭代的目标值列表，用于作图与 f* 参考。"""
    z = np.zeros(3 * MP.N)
    traj = []
    for it in range(n_iter):
        z = np.clip(z - alpha * (H @ z + f), lo, hi)
        traj.append(cost(z) - fstar)
    return np.array(traj)

def fgm_traj(n_iter):
    """快速梯度法（FGM/Nesterov）迭代轨迹：返回每次迭代目标值列表，与 PGD 对比收敛率。"""
    z = np.zeros(3 * MP.N)
    t = 1.0
    traj = []
    for it in range(n_iter):
        grad = H @ z + f
        zn = np.clip(z - alpha * grad, lo, hi)   # 梯度步
        tn = 0.5 * (1 + np.sqrt(1 + 4 * t * t))  # Nesterov 动量系数
        beta = (t - 1) / tn
        z = zn + beta * (zn - z)                 # 动量外推
        t = tn
        traj.append(cost(zn) - fstar)
    return np.array(traj)

# 收敛轨迹
n = 400
tp = pgd_traj(n)
tf = fgm_traj(n)
OUT = os.path.join('TandemVec-Paper', 'fig-mpc')
os.makedirs(OUT, exist_ok=True)

fig, ax = plt.subplots(figsize=(6.8, 3.6))
ax.semilogy(np.arange(1, n + 1), tp, color='#2E86AB', lw=1.6, label='PGD（O(1/k)）')
ax.semilogy(np.arange(1, n + 1), tf, color='#E4572E', lw=1.6, label='FGM（Nesterov，O(1/k²)）')
# 标注嵌入式关键点
ax.axhline(cost(np.clip(-alpha*(H@np.zeros(3*MP.N)+f), lo, hi)) - fstar if False else 1e-4,
           ls=':', color='gray', lw=0.8)
ax.axvline(10, ls='--', color='#E4572E', lw=0.8, alpha=0.6)
ax.text(10.5, 2e-2, 'FGM-10\n(0.026 ms)', fontsize=8, color='#E4572E')
ax.axvline(400, ls='--', color='#2E86AB', lw=0.8, alpha=0.6)
ax.text(240, 2e-5, 'PGD-400\n(1.07 ms)', fontsize=8, color='#2E86AB')
ax.set_xlabel('迭代数 k'); ax.set_ylabel('目标函数余量 f(z) − f*')
ax.set_title('快速梯度法 vs 投影梯度法收敛（N=30 展开式 QP，STM32H743 同构）')
ax.legend(); ax.grid(alpha=0.3); ax.set_xlim(0, 410)
fig.tight_layout()
fig.savefig(os.path.join(OUT, 'mpc_fgm.pdf'))
plt.close(fig)
print('mpc_fgm.pdf 已生成')
print('FGM-10 余量: %.3e  PGD-400 余量: %.3e' % (tf[9], tp[399]))
print('FGM-25 余量: %.3e  PGD-25 余量: %.3e' % (tf[24], tp[24]))
