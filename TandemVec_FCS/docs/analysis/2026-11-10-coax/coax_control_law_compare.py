# -*- coding: utf-8 -*-
"""
共轴控制律对比：旧固件（误差→舵角） vs 新固件（误差→α→分配器→舵角）
纯控制律，不含机体动力学、不含传感器。
200 Hz 离散，参数取当前固件实际值。
"""
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DT = 0.005
FS = 1.0 / DT

# ---------- 旧固件（原始共轴 main.cpp）----------
OLD = dict(att_kp=4.25, att_out=80.0, att_lpf=0.85,
           rate_kp=0.25, rate_ki=0.08, rate_out=10.0, rate_lpf=0.25,
           int_limit=5.0, deriv_lpf=0.2)

# ---------- 新固件（统一链路）----------
# 速率环 deg/s² 域；分配器 delta = asin(I*alpha_rad/(l*T))
NEW = dict(att_kp=4.25, att_out=80.0, att_lpf=0.85,
           rate_kp=144.0, rate_ki=46.0, rate_out=5756.0, rate_lpf=0.9,
           int_limit=2878.0, deriv_lpf=0.2,
           I=0.003, l=0.21, T=0.84 * 9.79)
RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0


class Lpf:
    def __init__(self, a, init=0.0):
        self.a = a
        self.y = init

    def __call__(self, x):
        self.y = self.a * x + (1.0 - self.a) * self.y
        return self.y


def pid_step(st, sp, meas, kp, ki, out_lim, int_lim, dt, d_lpf, kd=0.0):
    """位置式 PID，微分作用于测量（与固件 computeDerivativeOnMeasurement 同）"""
    err = sp - meas
    d_raw = -((meas - st["prev"]) if st["init"] else 0.0)
    st["prev"] = meas
    st["init"] = True
    st["d"] = d_lpf(st["d"]) if False else (d_lpf.a * d_raw + (1 - d_lpf.a) * st["d"] if hasattr(d_lpf, "a") else d_raw)
    st["d"] = d_lpf.a * d_raw + (1.0 - d_lpf.a) * st["d"]
    st["i"] += err * dt
    bound = int_lim / ki if ki > 0 else 0.0
    st["i"] = max(-bound, min(bound, st["i"])) if ki > 0 else 0.0
    iout = max(-int_lim, min(int_lim, ki * st["i"])) if ki > 0 else 0.0
    pout = kp * err
    out = pout + iout + kd * st["d"]
    return max(-out_lim, min(out_lim, out))


def run_old(err_seq):
    """旧：误差→目标速率→速率PID→舵角（两级滤波）"""
    att_f = Lpf(OLD["att_lpf"])
    rate_f = Lpf(OLD["rate_lpf"])
    st = dict(i=0.0, prev=0.0, init=False, d=0.0)
    out = []
    for e in err_seq:
        sp = max(-OLD["att_out"], min(OLD["att_out"], OLD["att_kp"] * e))
        sp = att_f(sp)
        delta = pid_step(st, sp, 0.0, OLD["rate_kp"], OLD["rate_ki"],
                         OLD["rate_out"], OLD["int_limit"], DT, Lpf(OLD["deriv_lpf"]))
        out.append(rate_f(delta))
    return np.array(out)


def run_new(err_seq):
    """新：误差→目标速率→速率PID(α)→分配器逆解→舵角"""
    att_f = Lpf(NEW["att_lpf"])
    rate_f = Lpf(NEW["rate_lpf"])
    st = dict(i=0.0, prev=0.0, init=False, d=0.0)
    k = NEW["I"] / (NEW["l"] * NEW["T"])  # rad/s² -> rad
    out = []
    for e in err_seq:
        sp = max(-NEW["att_out"], min(NEW["att_out"], NEW["att_kp"] * e))
        sp = att_f(sp)
        alpha_deg = pid_step(st, sp, 0.0, NEW["rate_kp"], NEW["rate_ki"],
                             NEW["rate_out"], NEW["int_limit"], DT, Lpf(NEW["deriv_lpf"]))
        alpha_rad = alpha_deg * DEG2RAD
        s = max(-1.0, min(1.0, k * alpha_rad))
        delta = math.asin(s) * RAD2DEG
        out.append(rate_f(delta))
    return np.array(out)


# ---------- 1. 静态增益（P 项，无滤波稳态）----------
print("=== 静态增益（纯 P，稳态）===")
print(" err(deg/s)   旧 delta(deg)   新 delta(deg)   比值")
for e in [0.5, 1, 2, 5, 10, 20, 40, 80]:
    d_old = min(OLD["att_out"], OLD["att_kp"] * e)
    d_old = min(OLD["rate_out"], OLD["rate_kp"] * d_old)
    sp = min(NEW["att_out"], NEW["att_kp"] * e)
    a = min(NEW["rate_out"], NEW["rate_kp"] * sp) * DEG2RAD
    k = NEW["I"] / (NEW["l"] * NEW["T"])
    d_new = math.asin(max(-1.0, min(1.0, k * a))) * RAD2DEG
    print("%9.1f %14.4f %14.4f   %6.3f" % (e, d_old, d_new, (d_new / d_old) if d_old else float("nan")))

# ---------- 2. 阶跃响应 ----------
n = int(0.6 * FS)
step = np.full(n, 10.0)  # 10 deg/s 速率误差阶跃
d_old = run_old(step)
d_new = run_new(step)
t = np.arange(n) * DT
print("\n=== 阶跃(10 deg/s)响应 ===")
print("  t(ms)   旧(deg)   新(deg)")
for i in [0, 1, 2, 5, 10, 20, 40, 80, n-1]:
    print("%7.0f %9.4f %9.4f" % (t[i] * 1000, d_old[i], d_new[i]))

# 10%-90% 上升时间
def rise_time(y):
    yf = y[-1]
    if abs(yf) < 1e-9:
        return float("nan")
    lo, hi = 0.1 * yf, 0.9 * yf
    i_lo = next((i for i, v in enumerate(y) if v >= lo), None)
    i_hi = next((i for i, v in enumerate(y) if v >= hi), None)
    return (i_hi - i_lo) * DT * 1000 if i_lo is not None and i_hi is not None else float("nan")

print("上升时间(10-90%%): 旧 %.1f ms, 新 %.1f ms" % (rise_time(d_old), rise_time(d_new)))

# ---------- 3. 正弦跟踪（相位滞后）----------
print("\n=== 正弦跟踪（误差 5 deg/s 幅值）===")
for f in [1.0, 2.0, 5.0, 10.0]:
    n2 = int(1.0 * FS)
    tt = np.arange(n2) * DT
    sig = 5.0 * np.sin(2 * math.pi * f * tt)
    a_old = run_old(sig)
    a_new = run_new(sig)
    tail = slice(int(0.6 * n2), None)
    ref = OLD["rate_kp"] * OLD["att_kp"] * sig  # 理想比例输出
    ref = np.clip(ref, -OLD["rate_out"], OLD["rate_out"])

    def amp_phase(y):
        y = y[tail]
        r = ref[tail]
        # 最小二乘拟合正弦
        A = np.vstack([np.sin(2 * math.pi * f * tt[tail]), np.cos(2 * math.pi * f * tt[tail])]).T
        coef, *_ = np.linalg.lstsq(A, y, rcond=None)
        amp = math.hypot(*coef)
        ph = math.degrees(math.atan2(coef[1], coef[0]))
        return amp, ph

    ao, po = amp_phase(a_old)
    an, pn = amp_phase(a_new)
    print("f=%4.1fHz 旧: amp=%.3f phase=%6.1f° | 新: amp=%.3f phase=%6.1f° | 相位差 %+.1f°"
          % (f, ao, po, an, pn, pn - po))

# ---------- 4. 画图 ----------
fig, ax = plt.subplots(1, 2, figsize=(12, 4.2))
ax[0].plot(t * 1000, d_old, label="old firmware (err -> delta)", lw=1.8)
ax[0].plot(t * 1000, d_new, label="new firmware (err -> alpha -> allocator)", lw=1.8, ls="--")
ax[0].set_xlabel("t [ms]")
ax[0].set_ylabel("servo delta [deg]")
ax[0].set_title("Step response: rate error 10 deg/s")
ax[0].grid(alpha=0.3)
ax[0].legend()

n3 = int(1.0 * FS)
t3 = np.arange(n3) * DT
sig3 = 5.0 * np.sin(2 * math.pi * 5.0 * t3)
ax[1].plot(t3 * 1000, sig3 * OLD["rate_kp"] * 1.0, label="ideal (0.25 gain)", lw=1.2, color="gray")
ax[1].plot(t3 * 1000, run_old(sig3), label="old firmware", lw=1.6)
ax[1].plot(t3 * 1000, run_new(sig3), label="new firmware", lw=1.6, ls="--")
ax[1].set_xlabel("t [ms]")
ax[1].set_ylabel("servo delta [deg]")
ax[1].set_title("5 Hz sine tracking (5 deg/s error amplitude)")
ax[1].grid(alpha=0.3)
ax[1].legend()
plt.tight_layout()
out_png = r"D:\纵列双发矢量推力飞行器\TandemVec_FCS\tmp\coax_control_law_compare.png"
plt.savefig(out_png, dpi=130)
print("\n图: " + out_png)
