# -*- coding: utf-8 -*-
"""cascade_param_opt.py — 级联控制参数离线网格搜索（多目标代价）。

@file    cascade_param_opt.py
@module  analysis/参数优化脚本
@brief   在 (Kp, Ki, Kw) 网格上按复合代价 J 选最优级联增益。
@details 输入：参数 + 固定 DT=0.004 与 W0=0.441·wMax 工作点；处理：每个增益组合跑
         固定时长闭环，统计耦合峰值/调节时间/控制能量；输出：寻优表与 fig-sim 图件。
         运行：仓库根目录 `py -3.12 simulations/high-fidelity-analysis/cascade_param_opt.py`。
@theory  级联结构（外环姿态 P、内环角速率）@ref TandemVec-Paper/ch15；
         效能矩阵分配 @ref MOD-001 §4、@ref THY-003 §3。
@frame  FRD 机体系三轴。
@units   增益无量纲/1/s、时间 s、控制能量无量纲。
@note    代价权重 (w1,w2,w3) 是研究者设定，不是物理量——换权重会换"最优"增益；
         结果不可直接写回固件参数（仿真工作点 W0 与固件调度不同）。
@see     cascade_extended.py、@ref TandemVec-Paper/ch15
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from core import (load_params, control_effectiveness, rk4_step, Propulsion,
                  quat_norm, quat_multiply, quat_conj, euler_from_quat)
from cascade_extended import CascadePI, eps_of, P, DT, W0, U_MAX
from plot_style import apply_style, finish, C_BLUE, C_VERM, C_GREEN

apply_style()

def simulate_full(ctrl, ns=0.002, T=6.0, seed=0, df_step=10.0):
    """跑一次闭环（24 m/s 初速 + t=2 s 前摆阶跃 + 陀螺噪声）。

    @param[in] ctrl     控制器实例
    @param[in] ns       陀螺噪声 std [rad/s]
    @param[in] T        时长 [s]，dt=DT=0.004 s
    @param[in] seed     随机种子（固定以保证可复现）
    @param[in] df_step  前摆阶跃 [deg]
    @return (t, theta, u_energy)
    """
    rng=np.random.default_rng(seed);N=int(T/DT)
    v=np.array([24.,0,0]);w=np.zeros(3);q=quat_norm(np.array([1.,0,0,0]))
    prop=Propulsion(P);prop.wf=prop.wt=W0;prop.prev_wf=prop.prev_wt=W0
    th=np.zeros(N);u_eng=np.zeros(N)
    q_des=quat_norm(np.array([1.,0,0,0]))
    for i in range(N):
        t=i*DT;df_d=np.radians(df_step) if t>=2.0 else 0.0
        u=ctrl.update(q,q_des,w+rng.normal(0,ns,3),DT); dw,dt_c,df_c=u
        Fx,Fy,Fz,Mx,My,Mz=prop.forces(df_d+df_c,dt_c)
        v,w,q=rk4_step(v,w,q,prop,P,Fx,Fy,Fz,Mx,My,Mz,DT,use_aero=True)
        prop.update(W0,dw,DT)
        th[i]=np.degrees(euler_from_quat(q)[1]); u_eng[i]=np.sum(np.abs(u))
    return np.arange(N)*DT,th,u_eng

def cost(params, NSEED=1):
    """网格搜索的复合代价 J = w1·耦合峰值 + w2·调节时间 + w3·控制能量。

    @param[in] params (Kp, Ki, Kw)
    @param[in] NSEED  种子数（默认 1：网格搜索为控时，最终候选应增加种子复核）
    @return 标量代价（越小越好）
    @note    权重是研究者设定而非物理量，换权重会换"最优"增益。
    """
    Kp,Ki,Kw=params
    pk=[];ts=[];ue=[]
    for s in range(NSEED):
        t,th,u_eng=simulate_full(CascadePI(Kp=Kp,Ki=Ki,Kw=Kw),seed=s,T=4.0)
        seg=th[t>=2.0]; base=seg[:5].mean()
        dev=np.abs(seg-base)
        pk.append(np.max(dev))
        thr=0.1*np.max(dev)+1e-6
        idx=np.where(dev>thr)[0]
        ts.append(t[2:][idx[-1]]-2.0 if len(idx) else 0.0)
        ue.append(np.mean(u_eng))
    return 1.0*np.mean(pk) + 0.3*np.mean(ts) + 0.05*np.mean(ue), np.mean(pk), np.mean(ts)

# ====== 粗网格搜索 ======
print("粗网格搜索最优 (Kp, Ki, Kw)...")
Kps=[1.0,2.0,3.0]; Kis=[0.0,0.5]; Kws=[4.0,8.0]
best=(1e9,None)
records=[]
for kp in Kps:
    for ki in Kis:
        for kw in Kws:
            J,pk,ts=cost((kp,ki,kw))
            records.append((kp,ki,kw,J,pk,ts))
            if J<best[0]: best=(J,(kp,ki,kw,pk,ts))
print(f"  最优: Kp={best[1][0]} Ki={best[1][1]} Kw={best[1][2]}  J={best[0]:.3f} 耦合峰值={best[1][3]:.3f} 调节={best[1][4]:.2f}s")

# ====== 精化（在最优附近细分）======
kp0,ki0,kw0=best[1][0],best[1][1],best[1][2]
for kp in np.arange(kp0-0.5,kp0+0.6,0.25):
    for kw in np.arange(kw0-2,kw0+2.1,1.0):
        if kp<=0 or kw<=0: continue
        J,pk,ts=cost((kp,ki0,kw))
        if J<best[0]: best=(J,(kp,ki0,kw,pk,ts))
print(f"  精化后: Kp={best[1][0]} Ki={best[1][1]} Kw={best[1][2]}  J={best[0]:.3f}")

# ====== 可视化：Ki=最优下 Kp-Kw 代价曲面 ======
Kps2=np.linspace(1.0,3.0,6); Kws2=np.linspace(4.0,11.0,5)
ZJ=np.zeros((len(Kps2),len(Kws2))); Zpk=np.zeros_like(ZJ)
for i,kp in enumerate(Kps2):
    for j,kw in enumerate(Kws2):
        J,pk,ts=cost((kp,ki0,kw),NSEED=2); ZJ[i,j]=J; Zpk[i,j]=pk
fig,(a1,a2)=plt.subplots(1,2,figsize=(9.5,3.4))
X,Y=np.meshgrid(Kws2,Kps2)
c1=a1.contourf(X,Y,ZJ,12,cmap='viridis'); plt.colorbar(c1,ax=a1,label='代价 $J$')
a1.plot(best[1][2],best[1][0],'r*',ms=16,label='最优')
a1.set_xlabel(r'$K_\omega$'); a1.set_ylabel(r'$K_p$'); a1.legend(); a1.set_title('复合代价曲面')
c2=a2.contourf(X,Y,Zpk,12,cmap='magma'); plt.colorbar(c2,ax=a2,label='耦合峰值 [°]')
a2.set_xlabel(r'$K_\omega$'); a2.set_ylabel(r'$K_p$'); a2.set_title('耦合峰值曲面')
finish(fig,"../../scripts/output/cascade_param_opt.pdf")
print("saved cascade_param_opt.pdf")

# ====== 不同 Ki 的代价对比（积分增益权衡）======
print("\nKi 权衡（Kp,Kw 固定最优）:")
for ki in [0.0,0.5,1.0]:
    J,pk,ts=cost((best[1][0],ki,best[1][2]))
    print(f"  Ki={ki}: J={J:.3f} 耦合峰值={pk:.3f} 调节={ts:.2f}s")
