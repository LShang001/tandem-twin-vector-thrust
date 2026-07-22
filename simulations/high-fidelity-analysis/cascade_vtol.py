# -*- coding: utf-8 -*-
"""级联架构 VTOL tail-sitter 大姿态角验证
   验证误差四元数外环在万向锁附近（θ=-90°）的有效性
   场景：水平→悬停 slerp 过渡 + 悬停抗扰
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from core import (load_params, control_effectiveness, Propulsion, rk4_step,
                  quat_norm, quat_multiply, quat_conj, euler_from_quat)
from controllers import (QuatSASController, QuatINDIController,
                         QuatLQRController, simulate_vtol, slerp_quat)
from estimators import EulerDiffEstimator

P = load_params(); DT=0.004
qh = np.array([np.cos(np.pi/4),0,np.sin(np.pi/4),0])   # 悬停（机头朝天）
q_level = np.array([1.0,0,0,0])
W0_H = np.sqrt(P["m"]*9.81/(2*P["kT"]))
U_MAX = np.array([P["dwMax"], P["dMax"], P["dMax"]])

def eps_of(q,q_des):
    qe=quat_multiply(quat_conj(q_des),q); return np.array([qe[1],qe[2],qe[3]])

class CascadePI:
    """级联：误差四元数外环PI → 角速度指令，内环 B_true 分配"""
    def __init__(self, omega0, Kp=2.0, Ki=0.5, Kw=8.0, du_max=0.5):
        self.omega0=omega0; self.Kp=Kp; self.Ki=Ki; self.Kw=Kw; self.du_max=du_max
        self.u=np.zeros(3); self.ie=np.zeros(3)
    def update(self, q, q_des, omega, dt, omega_ref=None):
        if omega_ref is None: omega_ref=np.zeros(3)
        e=eps_of(q,q_des)
        if np.linalg.norm(e)<0.3: self.ie=np.clip(self.ie+e*dt,-0.3,0.3)
        w_ref=-self.Kp*e-self.Ki*self.ie+omega_ref
        nu=self.Kw*(w_ref-omega)
        B,_,_=control_effectiveness(self.omega0,self.u[2],self.u[1],self.u[0],P)
        try: du=np.clip(np.linalg.solve(B,nu),-self.du_max,self.du_max)
        except np.linalg.LinAlgError: du=np.zeros(3)
        self.u=np.clip(self.u+du,-U_MAX,U_MAX)
        df,dt_c,dw=self.u[2],self.u[1],self.u[0]
        return df,dt_c,dw

T_SLERP=3.0; T_TO=10.0
def run(ctrl):
    return simulate_vtol(ctrl, P, qh, W0_H*1.05, T_total=T_TO,
                         slerp_duration=T_SLERP, q_des_initial=q_level)

print("VTOL tail-sitter 垂直起飞（3s slerp）：")
runs={}
for name,mk in [("SAS",lambda:QuatSASController(P,omega0=W0_H*1.05)),
                ("INDI",lambda:QuatINDIController(P,estimator=EulerDiffEstimator(DT))),
                ("LQR",lambda:QuatLQRController(P,omega0=W0_H*1.05)),
                ("级联",lambda:CascadePI(W0_H*1.05))]:
    d=run(mk()); runs[name]=d
    eps_max=np.max(d["eps_norm"]); eps_end=d["eps_norm"][-1]
    # 收敛时间：eps<0.035(~2°) 且保持
    idx=np.where(d["eps_norm"]>0.035)[0]
    settle=d["t"][idx[-1]] if len(idx)>0 else 0.0
    print(f"  {name:<6} eps峰值={eps_max:.4f}  末值={eps_end:.2e}  收敛t≈{settle:.2f}s")

# 绘图
from plot_style import apply_style, finish, log_sci_ticks, C_BLUE, C_VERM, C_GREEN, C_GRAY
apply_style()
fig,(ax1,ax2)=plt.subplots(2,1,figsize=(5.6,4.4),sharex=True)
colors={"SAS":C_BLUE,"INDI":C_VERM,"LQR":C_GREEN,"级联":"#9467bd"}
ax1.plot(runs["LQR"]["t"],np.where(runs["LQR"]["t"]<T_SLERP,-90*runs["LQR"]["t"]/T_SLERP,-90),
         color=C_GRAY,ls='--',lw=0.8,label='参考')
for name,d in runs.items():
    ax1.plot(d["t"],np.degrees(d["theta"]),color=colors[name],label=name,lw=1.1)
ax1.set_ylabel(r'$\theta$ [°]'); ax1.set_ylim(-100,5); ax1.legend(loc='lower right',ncol=5,fontsize=7)
ax1.set_title('级联架构 vs SAS/INDI/LQR：tail-sitter 垂直起飞姿态跟踪')
for name,d in runs.items():
    ax2.semilogy(d["t"],np.maximum(d["eps_norm"],1e-10),color=colors[name],label=name,lw=1.1)
log_sci_ticks(ax2); ax2.set_ylabel(r'$\Vert\varepsilon_{\mathrm{err}}\Vert$'); ax2.set_xlabel('t [s]')
finish(fig,"../../scripts/output/cascade_vtol.png")
print("saved ../../scripts/output/cascade_vtol.png")
