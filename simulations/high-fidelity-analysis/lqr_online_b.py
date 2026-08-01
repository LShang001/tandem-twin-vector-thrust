# -*- coding: utf-8 -*-
"""级联控制：外环姿态P→角速度指令，内环角速度环+交叉项补偿
   用户架构：外环纯比例（姿态），内环角速度控制（带在线B_true交叉项补偿）
   对比: 纯LQR / 纯INDI / 级联P+B_true分配 / 级联P+增益调度分配
"""
import numpy as np
from scipy.linalg import solve_continuous_are
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from core import (load_params, control_effectiveness, rk4_step, Propulsion,
                  quat_norm, quat_multiply, quat_conj, euler_from_quat)
from controllers import QuatLQRController, QuatINDIController
from estimators import EulerDiffEstimator

P = load_params(); DT=0.004; W0=0.441*P["wMax"]
U_MAX = np.array([P["dwMax"], P["dMax"], P["dMax"]])

def eps_of(q, q_des):
    qe = quat_multiply(quat_conj(q_des), q)
    return np.array([qe[1], qe[2], qe[3]])

# ============================================================
# 级联：外环姿态P → 角速度指令 ω_ref = -K_p·ε
#       内环角速度环: ν = K_ω(ω_ref - ω)，u = B_true(u_k)⁻¹ ν  （交叉项补偿）
# ============================================================
class CascadeOnlineB:
    name = "级联P+在线B_true"
    def __init__(self, Kp=2.0, Kw=8.0, du_max=0.5):
        self.Kp=Kp; self.Kw=Kw; self.du_max=du_max
        self.u = np.zeros(3)
    def update(self, q, q_des, omega, omega_ref=None):
        if omega_ref is None: omega_ref=np.zeros(3)
        # 外环：姿态误差 → 角速度指令（纯比例）
        w_ref = -self.Kp * eps_of(q, q_des) + omega_ref
        # 内环：角速度误差 → 虚拟角加速度
        nu = self.Kw * (w_ref - omega)
        # 在线 B_true 分配（交叉项补偿）
        B,_,_ = control_effectiveness(W0, self.u[2], self.u[1], self.u[0], P)
        try:
            # ν 为角加速度指令：×惯量得目标力矩增量，再经 B⁻¹ 分配
            du = np.linalg.solve(B, np.array([P["Ix"], P["Iy"], P["Iz"]]) * nu)
            du = np.clip(du, -self.du_max, self.du_max)
        except np.linalg.LinAlgError:
            du = np.zeros(3)
        self.u = np.clip(self.u + du, -U_MAX, U_MAX)
        return self.u

# 级联 + 冻结B（无交叉项补偿，作对照）
class CascadeFrozenB:
    name = "级联P+冻结B(无补偿)"
    def __init__(self, Kp=2.0, Kw=8.0, du_max=0.5):
        self.Kp=Kp; self.Kw=Kw; self.du_max=du_max
        self.u = np.zeros(3)
        self.B0,_,_ = control_effectiveness(W0, 0,0,0, P)
    def update(self, q, q_des, omega, omega_ref=None):
        if omega_ref is None: omega_ref=np.zeros(3)
        w_ref = -self.Kp * eps_of(q, q_des) + omega_ref
        nu = self.Kw * (w_ref - omega)
        try:
            du = np.linalg.solve(self.B0, nu)
            du = np.clip(du, -self.du_max, self.du_max)
        except np.linalg.LinAlgError:
            du = np.zeros(3)
        self.u = np.clip(self.u + du, -U_MAX, U_MAX)
        return self.u

# ============================================================
def simulate(ctrl, kind, noise_std=0.002, T=6.0, seed=0, df_step=10.0):
    rng=np.random.default_rng(seed); N=int(T/DT)
    v=np.array([24.,0,0]); w=np.zeros(3); q=quat_norm(np.array([1.,0,0,0]))
    prop=Propulsion(P); prop.wf=prop.wt=W0; prop.prev_wf=prop.prev_wt=W0
    th=np.zeros(N); q_des=quat_norm(np.array([1.,0,0,0]))
    for i in range(N):
        t=i*DT; df_d=np.radians(df_step) if t>=2.0 else 0.0
        w_meas=w+rng.normal(0,noise_std,3)
        if kind=="LQR":
            df_c,dt_c,dw=ctrl.update(q,q_des,w_meas,DT)
        elif kind=="INDI":
            df_c,dt_c,dw=ctrl.update(q,q_des,w_meas,W0,DT); df_c=0.0
        else:
            u=ctrl.update(q,q_des,w_meas); dw,dt_c,df_c=u
        Fx,Fy,Fz,Mx,My,Mz=prop.forces(df_d+df_c,dt_c)
        v,w,q=rk4_step(v,w,q,prop,P,Fx,Fy,Fz,Mx,My,Mz,DT,use_aero=True)
        prop.update(W0,dw,DT)
        th[i]=np.degrees(euler_from_quat(q)[1])
    return np.arange(N)*DT, th

def peak(make, kind, df_step, NSEED=5):
    peaks=[]
    for s in range(NSEED):
        t,th=simulate(make(), kind, seed=s, df_step=df_step)
        seg=th[t>=2.0]; peaks.append(np.max(np.abs(seg-seg[:5].mean())))
    return np.mean(peaks)

cases = [
    ("纯LQR(冻结B)", lambda: QuatLQRController(P,omega0=W0), "LQR"),
    ("纯INDI(在线B)", lambda: QuatINDIController(P,estimator=EulerDiffEstimator(DT)), "INDI"),
    ("级联P+冻结B", lambda: CascadeFrozenB(), "CAS_F"),
    ("级联P+在线B", lambda: CascadeOnlineB(), "CAS_O"),
]
print("俯仰耦合峰值(°)，5次蒙特卡洛")
print(f"{'架构':<22}{'δ_f=10°':<12}{'δ_f=25°':<12}")
res={}
for label, mk, k in cases:
    m10=peak(mk,k,10); m25=peak(mk,k,25); res[label]=(m10,m25)
    print(f"{label:<22}{m10:<12.2f}{m25:<12.2f}")

fig,ax=plt.subplots(figsize=(8.5,4.5))
x=np.arange(len(cases)); wd=0.35
ax.bar(x-wd/2,[res[l][0] for l,_,_ in cases],wd,label="δ_f=10°",color="#1f77b4")
ax.bar(x+wd/2,[res[l][1] for l,_,_ in cases],wd,label="δ_f=25°",color="#d62728")
ax.set_xticks(x); ax.set_xticklabels([l for l,_,_ in cases],rotation=12,ha='right',fontsize=8)
ax.set_ylabel("pitch coupling peak (deg)")
ax.set_title("Cascade (P outer + B inner) vs LQR vs INDI")
ax.legend(); ax.grid(alpha=0.3,axis='y'); plt.tight_layout()
plt.savefig("../../scripts/output/cascade_coupling.png", dpi=130)
print("saved ../../scripts/output/cascade_coupling.png")
