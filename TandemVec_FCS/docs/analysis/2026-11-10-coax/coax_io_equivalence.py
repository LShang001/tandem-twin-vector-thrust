# -*- coding: utf-8 -*-
"""
高精度输入-输出对比（200 Hz 离散，严格按固件 PositionPID 语义）

链 A（原版）：误差 → rate PID（舵角域）→ 限幅 → 输出滤波 → 舵角
链 B（新版）：误差 → rate PID（α 域）→ 限幅 → 分配器 asin(I·α/(l·T)) → 输出滤波 → 舵角

两组实验：
  [1] 纯控制律（同一反馈信号，不含反馈滤波差异）—— 检验 α→δ 换算是否严格等价
  [2] 完整链（各自反馈滤波 0.3 / 0.8）—— 量化滤波带来的实际差异
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

# ---------- 原版 ----------
OLD = dict(kp=0.25, ki=0.08, kd=0.0, out_min=-10.0, out_max=10.0,
           int_limit=5.0, threshold=0.0, falpha=0.2,
           fb_alpha=0.3, out_alpha=0.25)
# ---------- 新版（当前烧录参数）----------
NEW = dict(kp=28.8, ki=9.2, kd=0.0, out_min=-1151.0, out_max=1151.0,
           int_limit=575.6, threshold=60.0, falpha=0.2,
           fb_alpha=0.8, out_alpha=0.5,
           I=0.015, l=0.21, T=0.84 * 9.79)


class LPF:
    def __init__(self, a, y=0.0):
        self.a = a; self.y = y
    def __call__(self, x):
        self.y = self.a * x + (1.0 - self.a) * self.y
        return self.y


class PID:
    """严格复刻固件 PositionPID：微分作用于测量、积分钳位 int_limit/ki、输出端限幅"""
    def __init__(self, p):
        self.p = p
        self.integral = 0.0
        self.prev_input = 0.0
        self.d_filt = 0.0
        self.last = 0.0
        self.dlpf = LPF(p["falpha"])
    def step(self, setpoint, meas, dt):
        err = setpoint - meas
        deriv = self.prev_input - meas
        self.d_filt = self.dlpf(deriv)
        self.prev_input = meas
        ki = self.p["ki"]
        if ki != 0.0:
            if self.p["threshold"] <= 0.0 or abs(err) < self.p["threshold"]:
                self.integral += err * dt
            bound = abs(self.p["int_limit"] / ki)
            self.integral = max(-bound, min(bound, self.integral))
            iout = max(-self.p["int_limit"], min(self.p["int_limit"], ki * self.integral))
        else:
            self.integral = 0.0
            iout = 0.0
        out = self.p["kp"] * err + iout + self.p["kd"] * self.d_filt
        out = max(self.p["out_min"], min(self.p["out_max"], out))
        self.last = out
        return out


def run_old(err_seq, meas_seq=None, use_fb_filter=True):
    pid = PID(OLD); out_f = LPF(OLD["out_alpha"]); fb_f = LPF(OLD["fb_alpha"])
    ys = []
    for i, e in enumerate(err_seq):
        if meas_seq is not None:
            m = fb_f(meas_seq[i]) if use_fb_filter else meas_seq[i]
        else:
            m = fb_f(0.0)      # 纯控制律：反馈固定 0
        u = pid.step(e, m, DT)
        ys.append(out_f(u))
    return np.array(ys)


def run_new(err_seq, meas_seq=None, use_fb_filter=True):
    pid = PID(NEW); out_f = LPF(NEW["out_alpha"]); fb_f = LPF(NEW["fb_alpha"])
    k = NEW["I"] / (NEW["l"] * NEW["T"])
    ys = []
    for i, e in enumerate(err_seq):
        if meas_seq is not None:
            m = fb_f(meas_seq[i]) if use_fb_filter else meas_seq[i]
        else:
            m = fb_f(0.0)
        alpha = pid.step(e, m, DT)        # deg/s²
        alpha_rad = alpha * DEG2RAD
        s = max(-1.0, min(1.0, k * alpha_rad))
        ys.append(out_f(math.asin(s) * RAD2DEG))
    return np.array(ys)


# ================= [1] 纯控制律：静态增益 =================
print("=" * 68)
print("[1] 纯控制律静态增益（P 项，反馈=0）")
print("  误差(deg/s)   旧舵角(deg)   新舵角(deg)   绝对差   相对差")
max_rel = 0.0
for e in [0.1, 0.25, 0.5, 1, 2, 5, 8, 10, 15, 20, 40]:
    d_old = run_old([e] * 400, use_fb_filter=False)[-1]
    d_new = run_new([e] * 400, use_fb_filter=False)[-1]
    rel = abs(d_new - d_old) / abs(d_old) * 100 if abs(d_old) > 1e-9 else 0.0
    max_rel = max(max_rel, rel)
    print("%11.2f %13.5f %13.5f %9.5f %8.3f%%" % (e, d_old, d_new, abs(d_new - d_old), rel))
print("  最大相对差: %.3f%%" % max_rel)

# ================= [2] 纯控制律：阶跃与频响 =================
n = 400
t = np.arange(n) * DT
step = np.full(n, 10.0)
d_old = run_old(step, use_fb_filter=False)
d_new = run_new(step, use_fb_filter=False)
print("\n[2] 纯控制律阶跃(10 deg/s) 关键点")
for i in [0, 1, 2, 5, 10, 20, 40, 79, 199]:
    print("  t=%5.0fms  旧=%.5f  新=%.5f  差=%.2e" % (t[i] * 1000, d_old[i], d_new[i], abs(d_new[i] - d_old[i])))
print("  阶跃全程最大绝对差: %.3e deg" % np.max(np.abs(d_new - d_old)))
print("  阶跃全程最大相对差: %.3f%%" % (np.max(np.abs(d_new - d_old) / np.maximum(np.abs(d_old), 1e-6)) * 100))

# 频响（正弦相关法）
print("\n[2b] 纯控制律频响（幅值比 / 相位差，以旧链为基准）")
freqs = [0.5, 1, 2, 5, 10, 20]
for f in freqs:
    n2 = int(2.0 / DT)
    t2 = np.arange(n2) * DT
    sig = 5.0 * np.sin(2 * math.pi * f * t2)
    a1 = run_old(sig, use_fb_filter=False)
    a2 = run_new(sig, use_fb_filter=False)
    tail = slice(int(n2 * 0.6), None)
    A = np.vstack([np.sin(2 * math.pi * f * t2[tail]), np.cos(2 * math.pi * f * t2[tail])]).T
    c1, *_ = np.linalg.lstsq(A, a1[tail], rcond=None)
    c2, *_ = np.linalg.lstsq(A, a2[tail], rcond=None)
    m1 = math.hypot(*c1); m2 = math.hypot(*c2)
    p1 = math.degrees(math.atan2(c1[1], c1[0])); p2 = math.degrees(math.atan2(c2[1], c2[0]))
    print("  %5.1f Hz  幅值比=%.4f  相位差=%+.2f°" % (f, m2 / m1, p2 - p1))

# ================= [3] 完整链（含反馈滤波差异） =================
print("\n" + "=" * 68)
print("[3] 完整链（含反馈滤波 0.3 vs 0.8 + 输出滤波 0.25 vs 0.5）")
print("  说明：这一组才是真机上实际发生的差异")
n3 = 400
meas = np.zeros(n3)   # 假定反馈恒 0（开环测控制量），滤波状态初值 0
d_old3 = run_old(step, meas, use_fb_filter=True)
d_new3 = run_new(step, meas, use_fb_filter=True)
print("  t=%5.0fms  旧=%.5f  新=%.5f" % (0, d_old3[0], d_new3[0]))
for i in [5, 20, 79, 199]:
    print("  t=%5.0fms  旧=%.5f  新=%.5f" % (t[i] * 1000, d_old3[i], d_new3[i]))

# 图：静态 + Bode
fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
es = np.linspace(0.05, 20, 200)
o1 = [run_old([e] * 300, use_fb_filter=False)[-1] for e in es]
o2 = [run_new([e] * 300, use_fb_filter=False)[-1] for e in es]
ax[0].plot(es, o1, label="old (err->delta)", lw=2)
ax[0].plot(es, o2, "--", label="new (err->alpha->delta)", lw=1.6)
ax[0].set_xlabel("rate error [deg/s]")
ax[0].set_ylabel("servo delta [deg]")
ax[0].set_title("Static gain: P-only, feedback=0")
ax[0].grid(alpha=0.3); ax[0].legend()

freqs2 = np.logspace(-1, 1.3, 18)
mag, ph = [], []
for f in freqs2:
    n4 = int(3.0 / DT)
    t4 = np.arange(n4) * DT
    sig4 = 5.0 * np.sin(2 * math.pi * f * t4)
    a1 = run_old(sig4, use_fb_filter=False)
    a2 = run_new(sig4, use_fb_filter=False)
    tail = slice(int(n4 * 0.6), None)
    A = np.vstack([np.sin(2 * math.pi * f * t4[tail]), np.cos(2 * math.pi * f * t4[tail])]).T
    c1, *_ = np.linalg.lstsq(A, a1[tail], rcond=None)
    c2, *_ = np.linalg.lstsq(A, a2[tail], rcond=None)
    mag.append(math.hypot(*c2) / math.hypot(*c1))
    ph.append(math.degrees(math.atan2(c2[1], c2[0]) - math.atan2(c1[1], c1[0])))
ax[1].semilogx(freqs2, np.array(mag) - 1.0, label="mag ratio - 1")
ax[1].semilogx(freqs2, np.array(ph) / 100.0, label="phase diff / 100 [deg]")
ax[1].set_xlabel("freq [Hz]")
ax[1].set_title("Pure control law: new vs old deviation")
ax[1].grid(alpha=0.3, which="both"); ax[1].legend()
plt.tight_layout()
out = r"D:\纵列双发矢量推力飞行器\TandemVec_FCS\tmp\coax_io_equivalence.png"
plt.savefig(out, dpi=130)
print("\n图:", out)
