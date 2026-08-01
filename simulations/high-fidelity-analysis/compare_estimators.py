# -*- coding: utf-8 -*-
"""INDI 估计器闭环对比（严谨版）
   关键改进：
   1. 每个估计器用其最优带宽参数
   2. 对比多种陀螺噪声水平下的耦合抑制 + 稳定性
   3. 自适应互补滤波用完整动力学模型预测通道
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from core import (load_params, control_effectiveness, aero_forces,
                  rk4_step, Propulsion, quat_norm, euler_from_quat)
from estimators import (EulerDiffEstimator, SavitzkyGolayEstimator,
                        AdaptiveComplementaryEstimator, FixedLagSmootherEstimator)

P = load_params()

class INDI:
    def __init__(self, estimator, K=3.0):
        self.est = estimator
        self.K = K
        self.prev_u = np.zeros(3)
    def update(self, omega, omega0, dt):
        nu = self.K * (np.zeros(3) - omega)
        B, T0, tau0 = control_effectiveness(
            omega0, self.prev_u[2], self.prev_u[1], self.prev_u[0], P)
        wdot = self.est.update(omega, dt)
        try:
            # ×惯量：角加速度增量 → 力矩增量（缺 I 时滚转通道放大发散）
            du = np.linalg.solve(B, np.array([P["Ix"], P["Iy"], P["Iz"]]) * (nu - wdot))
            du = np.clip(du, -0.2, 0.2)
        except np.linalg.LinAlgError:
            du = np.zeros(3)
        self.prev_u = np.clip(self.prev_u + du,
                              [-P["dwMax"],-P["dMax"],-P["dMax"]],
                              [ P["dwMax"], P["dMax"], P["dMax"]])
        return self.prev_u

def simulate(est, noise_std, T=6.0, dt=0.004, seed=0, K=3.0):
    rng = np.random.default_rng(seed)
    N = int(T/dt); omega0 = 0.441*P["wMax"]
    v = np.array([24.0,0,0]); w = np.zeros(3); q = quat_norm(np.array([1.,0,0,0]))
    prop = Propulsion(P); prop.wf=prop.wt=omega0; prop.prev_wf=prop.prev_wt=omega0
    indi = INDI(est, K)
    th = np.zeros(N)
    for i in range(N):
        t = i*dt
        df_d = np.radians(10) if t>=2.0 else 0.0
        Fx,Fy,Fz,Mx,My,Mz = prop.forces(df_d, indi.prev_u[1])
        v,w,q = rk4_step(v,w,q,prop,P,Fx,Fy,Fz,Mx,My,Mz,dt,use_aero=True)
        w_meas = w + rng.normal(0,noise_std,3)
        if isinstance(est, AdaptiveComplementaryEstimator):
            est.set_control(indi.prev_u[2],indi.prev_u[1],indi.prev_u[0],omega0)
        indi.update(w_meas, omega0, dt)
        prop.update(omega0, indi.prev_u[0], dt)
        th[i] = np.degrees(euler_from_quat(q)[1])
    t_arr = np.arange(N)*dt
    seg = th[t_arr>=2.0]
    return np.max(np.abs(seg - seg[:5].mean())), np.abs(seg[-1]), th

# 每个估计器配其代表性参数
def make(name, dt):
    if name=="差分": return EulerDiffEstimator(dt)
    if name=="S-G":  return SavitzkyGolayEstimator(dt, M=3, d=2)   # 更小窗口减延迟
    if name=="互补": return AdaptiveComplementaryEstimator(dt, P,
                        lambda df,dt_,dw,w0: control_effectiveness(w0,df,dt_,dw,P)[0])
    if name=="平滑": return FixedLagSmootherEstimator(dt, N=2, q_proc=200) # 更小N
names = ["差分","S-G","互补","平滑"]

print("耦合峰值(°) / 稳态偏差(°)，行=噪声水平")
print(f"{'噪声':<8}" + "".join(f"{n:<18}" for n in names))
for ns in [0.0, 0.002, 0.005, 0.01, 0.02]:
    row = f"{ns:<8}"
    for n in names:
        est = make(n, 0.004)
        peak, settle, _ = simulate(est, ns)
        row += f"{peak:.2f}/{settle:.2f}      "
    print(row)

# 画图：中高噪声 ns=0.01 下四条曲线
fig, ax = plt.subplots(figsize=(10,5))
for n in names:
    est = make(n, 0.004)
    _,_,th = simulate(est, 0.01)
    t = np.arange(len(th))*0.004
    ax.plot(t, th, label=n, lw=1.2)
ax.axvline(2.0, ls='--', c='gray', lw=0.8)
ax.set_xlabel("t (s)"); ax.set_ylabel("theta (deg)")
ax.set_title("INDI estimator compare, gyro noise std=0.01 rad/s, df step 10deg @2s")
ax.legend(); ax.grid(alpha=0.3)
plt.tight_layout()
plt.savefig("../../scripts/output/indi_estimator_compare.png", dpi=130)
print("\nsaved ../../scripts/output/indi_estimator_compare.png")
