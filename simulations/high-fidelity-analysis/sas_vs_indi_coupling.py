# -*- coding: utf-8 -*-
"""耦合抑制对比：SAS(对角) vs INDI(全矩阵) vs LQR(冻结B_full)
   场景：24 m/s 巡航 + 前摆 δ_f 阶跃 → 俯仰耦合（-τ_f·sinδ_f）
   扫摆角幅度，量化 LQR 冻结 B(δ=0) 的大摆角退化
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from core import (load_params, rk4_step, Propulsion, quat_norm, euler_from_quat)
from controllers import QuatSASController, QuatINDIController, QuatLQRController
from estimators import EulerDiffEstimator

P = load_params()
DT = 0.004
W0 = 0.441 * P["wMax"]

def simulate(ctrl, kind, noise_std=0.002, T=6.0, seed=0, df_step=10.0):
    rng = np.random.default_rng(seed)
    N = int(T/DT)
    v = np.array([24.,0,0]); w = np.zeros(3); q = quat_norm(np.array([1.,0,0,0]))
    prop = Propulsion(P); prop.wf=prop.wt=W0; prop.prev_wf=prop.prev_wt=W0
    th = np.zeros(N); q_des = quat_norm(np.array([1.,0,0,0]))
    for i in range(N):
        t = i*DT
        df_d = np.radians(df_step) if t>=2.0 else 0.0
        w_meas = w + rng.normal(0, noise_std, 3)
        if kind=="SAS":
            df_c, dt_c, dw = ctrl.update(q, q_des, w_meas, DT)
        elif kind=="LQR":
            df_c, dt_c, dw = ctrl.update(q, q_des, w_meas, DT)
        else:
            df_c, dt_c, dw = ctrl.update(q, q_des, w_meas, W0, DT)
        Fx,Fy,Fz,Mx,My,Mz = prop.forces(df_d+df_c, dt_c)
        v,w,q = rk4_step(v,w,q,prop,P,Fx,Fy,Fz,Mx,My,Mz,DT,use_aero=True)
        prop.update(W0, dw, DT)
        th[i]=np.degrees(euler_from_quat(q)[1])
    return np.arange(N)*DT, th

def make(kind):
    if kind=="SAS": return QuatSASController(P, omega0=W0)
    if kind=="LQR": return QuatLQRController(P, omega0=W0)
    return QuatINDIController(P, estimator=EulerDiffEstimator(DT))

def coupling_peak(kind, df_step, ns=0.002, NSEED=5):
    peaks=[]
    for s in range(NSEED):
        t, th = simulate(make(kind), kind, noise_std=ns, seed=s, df_step=df_step)
        seg = th[t>=2.0]
        peaks.append(np.max(np.abs(seg - seg[:5].mean())))
    return np.mean(peaks), np.std(peaks)

# ====== 扫摆角幅度：量化 LQR 冻结 B 的退化 ======
print("俯仰耦合峰值(°) vs 前摆阶跃幅度（5次蒙特卡洛均值）")
print(f"{'δ_f阶跃':<10}{'SAS(对角)':<16}{'LQR(冻结B)':<16}{'INDI(在线B)':<16}")
steps = [5, 10, 15, 20, 25]
rows = {k: [] for k in ["SAS","LQR","INDI"]}
for st in steps:
    line = f"{st}°      "
    for kind in ["SAS","LQR","INDI"]:
        m, sd = coupling_peak(kind, st)
        rows[kind].append(m)
        line += f"{m:.2f}±{sd:.2f}    "
    print(line)

# ====== 绘图：耦合峰值 vs 摆角幅度 ======
fig, ax = plt.subplots(figsize=(8,4.5))
colors = {"SAS":"#1f77b4","LQR":"#2ca02c","INDI":"#d62728"}
labels = {"SAS":"SAS (diagonal alloc)","LQR":"LQR (frozen B at δ=0)","INDI":"INDI (online B_true)"}
for kind in ["SAS","LQR","INDI"]:
    ax.plot(steps, rows[kind], 'o-', color=colors[kind], label=labels[kind], lw=1.5)
ax.set_xlabel("yaw-gimbal step δ_f (deg)")
ax.set_ylabel("pitch coupling peak (deg)")
ax.set_title("Pitch coupling rejection vs gimbal amplitude: SAS / LQR / INDI")
ax.legend(); ax.grid(alpha=0.3)
plt.tight_layout()
plt.savefig("../../scripts/output/coupling_vs_amplitude.png", dpi=130)
print("\nsaved ../../scripts/output/coupling_vs_amplitude.png")
