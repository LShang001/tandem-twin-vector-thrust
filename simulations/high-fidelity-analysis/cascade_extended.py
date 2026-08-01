# -*- coding: utf-8 -*-
"""级联架构扩展仿真：增益扫描/噪声鲁棒/饱和/时域细节，生成论文图件"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from core import (load_params, control_effectiveness, rk4_step, Propulsion,
                  quat_norm, quat_multiply, quat_conj, euler_from_quat)
from controllers import QuatLQRController, QuatINDIController
from estimators import EulerDiffEstimator
from plot_style import apply_style, finish, log_sci_ticks, C_BLUE, C_VERM, C_GREEN, C_GRAY

P = load_params(); DT=0.004; W0=0.441*P["wMax"]
U_MAX=np.array([P["dwMax"],P["dMax"],P["dMax"]])
apply_style()

def eps_of(q,q_des):
    qe=quat_multiply(quat_conj(q_des),q); return np.array([qe[1],qe[2],qe[3]])

class CascadePI:
    def __init__(self,Kp=2.0,Ki=0.5,Kw=8.0,du_max=0.5):
        self.Kp=Kp;self.Ki=Ki;self.Kw=Kw;self.du_max=du_max
        self.u=np.zeros(3);self.ie=np.zeros(3)
    def update(self,q,q_des,omega,dt,omega_ref=None):
        if omega_ref is None: omega_ref=np.zeros(3)
        e=eps_of(q,q_des)
        if np.linalg.norm(e)<0.3: self.ie=np.clip(self.ie+e*dt,-0.3,0.3)
        w_ref=-self.Kp*e-self.Ki*self.ie+omega_ref
        nu=self.Kw*(w_ref-omega)
        B,_,_=control_effectiveness(W0,self.u[2],self.u[1],self.u[0],P)
        # ν 为角加速度指令：×惯量得目标力矩增量，再经 B⁻¹ 分配（缺 I 各向异性发散）
        try: du=np.clip(np.linalg.solve(B,np.array([P["Ix"],P["Iy"],P["Iz"]])*nu),-self.du_max,self.du_max)
        except np.linalg.LinAlgError: du=np.zeros(3)
        self.u=np.clip(self.u+du,-U_MAX,U_MAX)
        return self.u

def simulate(ctrl,kind,ns=0.002,T=6.0,seed=0,df_step=10.0):
    rng=np.random.default_rng(seed);N=int(T/DT)
    v=np.array([24.,0,0]);w=np.zeros(3);q=quat_norm(np.array([1.,0,0,0]))
    prop=Propulsion(P);prop.wf=prop.wt=W0;prop.prev_wf=prop.prev_wt=W0
    th=np.zeros(N);wy=np.zeros(N);dtc=np.zeros(N)
    q_des=quat_norm(np.array([1.,0,0,0]))
    for i in range(N):
        t=i*DT;df_d=np.radians(df_step) if t>=2.0 else 0.0
        w_meas=w+rng.normal(0,ns,3)
        if kind=="LQR": df_c,dt_c,dw=ctrl.update(q,q_des,w_meas,DT)
        elif kind=="INDI": _,dt_c,dw=ctrl.update(q,q_des,w_meas,W0,DT); df_c=0.0
        else: u=ctrl.update(q,q_des,w_meas,DT); dw,dt_c,df_c=u
        Fx,Fy,Fz,Mx,My,Mz=prop.forces(df_d+df_c,dt_c)
        v,w,q=rk4_step(v,w,q,prop,P,Fx,Fy,Fz,Mx,My,Mz,DT,use_aero=True)
        prop.update(W0,dw,DT)
        th[i]=np.degrees(euler_from_quat(q)[1]); wy[i]=w[1]; dtc[i]=np.degrees(dt_c)
    return np.arange(N)*DT,th,wy,dtc

def peak(make,kind,df_step,ns=0.002,NSEED=5):
    ps=[]
    for s in range(NSEED):
        t,th,_,_=simulate(make(),kind,ns=ns,seed=s,df_step=df_step)
        seg=th[t>=2.0]; ps.append(np.max(np.abs(seg-seg[:5].mean())))
    return np.mean(ps),np.std(ps)

OUT="../../scripts/output"

# ========== 图1: 时域响应细节（θ/q/δ_t 三通道）==========
print("图1: 时域响应细节...")
runs={}
for name,mk,k,c in [("LQR",lambda:QuatLQRController(P,omega0=W0),"LQR",C_BLUE),
                    ("INDI",lambda:QuatINDIController(P,estimator=EulerDiffEstimator(DT)),"INDI",C_VERM),
                    ("级联",lambda:CascadePI(),"CAS",C_GREEN)]:
    t,th,wy,dtc=simulate(mk(),k,df_step=25,seed=0); runs[name]=(t,th,wy,dtc,c)
fig,(a1,a2,a3)=plt.subplots(3,1,figsize=(5.6,5.4),sharex=True)
for name,(t,th,wy,dtc,c) in runs.items():
    a1.plot(t,th,color=c,label=name,lw=1.1)
    a2.plot(t,np.degrees(wy),color=c,lw=1.1)
    a3.plot(t,dtc,color=c,lw=1.1)
for a in (a1,a2,a3): a.axvline(2.0,ls='--',color=C_GRAY,lw=0.7)
a1.set_ylabel(r'$\theta$ [°]'); a1.legend(loc='upper right',ncol=3,fontsize=7)
a1.set_title(r'交叉耦合时域响应（$\delta_f=25°$ 阶跃 @2s）')
a2.set_ylabel(r'$q$ [°/s]'); a3.set_ylabel(r'$\delta_t$ [°]'); a3.set_xlabel('t [s]')
finish(fig,f'{OUT}/cascade_time_response.pdf')

# ========== 图2: 增益扫描（Kp×Kw 耦合峰值曲面）==========
print("图2: 增益扫描...")
Kps=[1.0,1.5,2.0,2.5,3.0]; Kws=[4.0,6.0,8.0,10.0,12.0]
Z=np.zeros((len(Kps),len(Kws)))
for i,kp in enumerate(Kps):
    for j,kw in enumerate(Kws):
        m,_=peak(lambda kp=kp,kw=kw: CascadePI(Kp=kp,Kw=kw),"CAS",10,NSEED=3)
        Z[i,j]=m
fig,ax=plt.subplots(figsize=(5.6,3.4))
X,Y=np.meshgrid(Kws,Kps)
cs=ax.contourf(X,Y,Z,levels=12,cmap='viridis')
plt.colorbar(cs,ax=ax,label=r'耦合峰值 [°]')
ax.set_xlabel(r'内环增益 $K_\omega$ [1/s]'); ax.set_ylabel(r'外环增益 $K_p$ [1/s]')
ax.set_title('级联增益整定：耦合峰值随 $(K_p,K_\omega)$ 变化')
finish(fig,f'{OUT}/cascade_gain_surface.pdf')
print("  Kp×Kw 曲面:\n",np.round(Z,3))

# ========== 图3: 噪声鲁棒性 ==========
print("图3: 噪声鲁棒性...")
nss=[0.0,0.005,0.01,0.02,0.04]
series={}
for name,mk,k,c in [("LQR",lambda:QuatLQRController(P,omega0=W0),"LQR",C_BLUE),
                    ("INDI",lambda:QuatINDIController(P,estimator=EulerDiffEstimator(DT)),"INDI",C_VERM),
                    ("级联",lambda:CascadePI(),"CAS",C_GREEN)]:
    ms=[peak(mk,k,10,ns=ns,NSEED=4)[0] for ns in nss]; series[name]=(ms,c)
    print(f"  {name}: {np.round(ms,3)}")
fig,ax=plt.subplots(figsize=(5.6,3.2))
for name,(ms,c) in series.items():
    ax.plot(nss,ms,'o-',color=c,label=name,lw=1.3)
ax.set_xlabel(r'陀螺噪声标准差 [rad/s]'); ax.set_ylabel(r'耦合峰值 [°]')
ax.set_yscale('log'); ax.legend(); ax.grid(alpha=0.3,which='both')
ax.set_title('噪声鲁棒性：耦合峰值随陀螺噪声变化')
finish(fig,f'{OUT}/cascade_noise_robust.pdf')

# ========== 图4: 执行器饱和影响 ==========
print("图4: 执行器饱和...")
dums=[0.1,0.2,0.3,0.5,0.8,1.2]
ms=[peak(lambda dm=dm: CascadePI(du_max=dm),"CAS",25,NSEED=3)[0] for dm in dums]
print("  du_max→耦合峰值:",np.round(ms,3))
fig,ax=plt.subplots(figsize=(5.6,3.0))
ax.plot(dums,ms,'s-',color=C_GREEN,lw=1.4)
ax.set_xlabel(r'内环增量限幅 $\Delta u_{\max}$'); ax.set_ylabel(r'耦合峰值 [°]')
ax.grid(alpha=0.3); ax.set_title(r'执行器速率饱和对耦合抑制的影响（$\delta_f=25°$）')
finish(fig,f'{OUT}/cascade_saturation.pdf')

print("\n全部扩展图件完成")
