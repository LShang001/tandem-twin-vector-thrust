# -*- coding: utf-8 -*-
"""sas_vs_indi_coupling.py — 前摆阶跃引起的俯仰耦合抑制对比。

@file    sas_vs_indi_coupling.py
@module  analysis/实验脚本（耦合抑制）
@brief   扫前摆 δf 幅度，对比 SAS(对角) / INDI(全矩阵) / LQR(冻结 B) 的耦合残差。
@details 输入：24 m/s 配平点 + δf 阶跃（默认 10°）+ 陀螺噪声；处理：三种控制器闭环，
         记录俯仰/滚转耦合峰值；输出：fig-sim PDF 与退化曲线。
@theory  前摆反扭在 y 轴的耦合 M_y=−τ_f·sinδ_f @ref MOD-001 §3.3；
         控制效率矩阵非对角项 @ref MOD-001 §4；
         LQR 冻结 B(δ=0) 在大摆角下的退化 @ref TandemVec-Paper/ch14。
@frame  FRD 机体系；δf 绕 z_b（偏航主控），其反扭投影到 y_b 即俯仰耦合。
@units   摆角 deg、角速度 rad/s、力矩 N·m、时间 s。
@note    seed 固定（默认 0）保证可复现；耦合量级随油门（T0）变化，结论只在同工作点成立。
@see     controllers.py、lqr_online_b.py、@ref MOD-001
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
    """跑一次闭环：24 m/s 初速 + 前摆阶跃 + 陀螺噪声。

    @param[in] ctrl      控制器（update(q,q_des,ω,dt) 或 INDI 五参版）
    @param[in] noise_std 陀螺噪声 std [rad/s]
    @param[in] T         时长 [s]，dt=DT=0.004 s
    @param[in] seed      随机种子（固定以保证可复现）
    @param[in] df_step   前摆阶跃 [deg]（t=2 s 起）
    @return (t, theta)：时间 [s] 与锁定显示俯仰角 [rad]
    """
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
    """控制器工厂：按名字新建实例（SAS / LQR / INDI），避免跨种子共享状态。"""
    if kind=="SAS": return QuatSASController(P, omega0=W0)
    if kind=="LQR": return QuatLQRController(P, omega0=W0)
    return QuatINDIController(P, estimator=EulerDiffEstimator(DT))

def coupling_peak(kind, df_step, ns=0.002, NSEED=5):
    """多种子统计俯仰耦合峰值。

    @return 平均耦合峰值 [rad]
    @note   峰值以阶跃后前 5 点均值为基准，避免把初值偏置算成耦合。
    """
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
