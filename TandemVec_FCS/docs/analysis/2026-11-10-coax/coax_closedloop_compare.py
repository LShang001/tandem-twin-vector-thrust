# -*- coding: utf-8 -*-
"""
闭环对比：原版（误差→舵角） vs 新版（误差→α→分配器→舵角）
机体用简化一阶： I_real * dω/dt = l*T*sin(delta) - b*ω
关键变量：真实惯量 I_real 与固件名义惯量 I_nom 的比值。
"""
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DT = 0.005
FS = 1.0 / DT
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

# 机体/几何
L = 0.21
T = 0.84 * 9.79
I_NOM = 0.003          # 固件名义惯量
B_DAMP = 0.02          # 气动/结构阻尼（N·m·s/rad）
TAU_SERVO = 0.020      # 舵机一阶响应 20 ms

# 控制律
ATT_KP = 4.25
ATT_LPF = 0.85
RATE_LPF_OLD = 0.25
RATE_LPF_NEW = 0.5
FB_LPF_OLD = 0.3
FB_LPF_NEW = 0.8
KP_OLD = 0.25          # 舵角域 deg/(deg/s)
KP_NEW = 100.0         # α 域 deg/s² per deg/s
INT_OLD = 0.08         # 连续域，舵角域
INT_NEW = 26.0

class F:
    def __init__(self, a): self.a = a; self.y = 0.0
    def __call__(self, x):
        self.y = self.a * x + (1 - self.a) * self.y
        return self.y

def sim(mode, i_real, target_dps=20.0, t_end=1.5):
    n = int(t_end / DT)
    omega = 0.0            # deg/s 真实角速率
    delta_act = 0.0        # deg 实际舵角
    fb_old = F(FB_LPF_OLD); fb_new = F(FB_LPF_NEW)
    att_old = F(ATT_LPF); att_new = F(ATT_LPF)
    out_old = F(RATE_LPF_OLD); out_new = F(RATE_LPF_NEW)
    integ_old = 0.0; integ_new = 0.0
    hist = []
    for k in range(n):
        # ---- 反馈 ----
        w_old = fb_old(omega)
        w_new = fb_new(omega)
        # ---- 内环 ----
        if mode == "old":
            err = target_dps - w_old
            p = KP_OLD * err
            integ_old += err * DT / INT_OLD if INT_OLD else 0.0
            delta_cmd = p
            delta_cmd = max(-10.0, min(10.0, delta_cmd))
            delta_cmd = out_old(delta_cmd)
        else:
            err = target_dps - w_new
            alpha = KP_NEW * err                      # deg/s²
            alpha_rad = alpha * DEG2RAD
            kk = I_NOM / (L * T)
            s = max(-1.0, min(1.0, kk * alpha_rad))
            delta_cmd = math.asin(s) * RAD2DEG
            delta_cmd = out_new(delta_cmd)
        # ---- 舵机一阶 ----
        delta_act += (delta_cmd - delta_act) * DT / TAU_SERVO
        # ---- 机体 ----
        torque = L * T * math.sin(delta_act * DEG2RAD) - B_DAMP * (omega * DEG2RAD)
        alpha_real = torque / i_real            # rad/s²
        omega += alpha_real * RAD2DEG * DT
        hist.append((omega, delta_cmd, delta_act))
    return np.array(hist)

def metrics(y, target):
    n = len(y)
    tail = y[int(n * 0.7):]
    ss = tail.mean()
    peak = y.max() if target > 0 else y.min()
    overshoot = (peak - target) / target * 100
    # 振荡次数：尾部穿越稳态次数
    cross = np.sum(np.diff(np.sign(tail - ss)) != 0)
    return ss, overshoot, cross

print("=== 闭环对比（目标速率阶跃 20 deg/s，机体真实惯量变化）===")
print(" I_real  模式   稳态(deg/s)  超调(%)  尾部振荡次数")
ratios = [0.5, 1.0, 2.0, 4.0]
rows = {}
for r in ratios:
    i_real = I_NOM * r
    for mode in ("old", "new"):
        h = sim(mode, i_real)
        ss, os_, cr = metrics(h[:, 0], 20.0)
        rows[(mode, r)] = h
        print("  %.1f×    %-4s %10.2f %10.1f %10d" % (r, mode, ss, os_, cr))

# 图
fig, axes = plt.subplots(2, 2, figsize=(11, 6.5))
for ax, r in zip(axes.ravel(), ratios):
    ax.plot(rows[("old", r)][:, 0], label="old (err->delta)", lw=1.6)
    ax.plot(rows[("new", r)][:, 0], label="new (err->alpha->delta)", lw=1.6, ls="--")
    ax.axhline(20, color="gray", lw=0.8, ls=":")
    ax.set_title("I_real = %.1f x I_nom" % r)
    ax.set_xlabel("t [ms]")
    ax.set_ylabel("rate [deg/s]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
plt.suptitle("Closed-loop rate step 20 deg/s: old direct-delta vs new alpha+allocator")
plt.tight_layout()
out = r"D:\纵列双发矢量推力飞行器\TandemVec_FCS\tmp\coax_closedloop_compare.png"
plt.savefig(out, dpi=125)
print("\n图:", out)
