# -*- coding: utf-8 -*-
"""
从原版实飞参数反推真实模型参数（惯量）
约束条件：
  A. 原版固件实飞稳定（不能振荡/发散）
  B. 原版实飞手感灵敏（阶跃响应要快，用户明确说"很灵敏"）
  C. 舵角权限 ±10°、目标速率 ±80 dps（原版量程）
"""
import math
import numpy as np

DT = 0.005
L = 0.21          # 用户实测力臂
T = 0.84 * 9.79   # 悬停推力
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

# 原版速率环
KP = 0.25         # 舵角 deg per (deg/s)
INT_LIM = 10.0    # 舵角限幅
FB_ALPHA = 0.3    # 反馈滤波
OUT_ALPHA = 0.25  # 输出滤波
TAU_SERVO = 0.020 # 舵机一阶 20ms

class F:
    def __init__(self, a): self.a = a; self.y = 0.0
    def __call__(self, x):
        self.y = self.a*x + (1-self.a)*self.y
        return self.y

def step_response(I, b_damp=0.01, target=20.0, t_end=1.2):
    n = int(t_end/DT)
    omega = 0.0; delta = 0.0
    fb = F(FB_ALPHA); out = F(OUT_ALPHA)
    integ = 0.0
    ys = []
    for k in range(n):
        w = fb(omega)
        err = target - w
        integ += err*DT
        # 原版积分限 5° 舵角
        iout = max(-5.0, min(5.0, 0.08*integ))
        cmd = KP*err + iout
        cmd = max(-INT_LIM, min(INT_LIM, cmd))
        cmd = out(cmd)
        delta += (cmd - delta)*DT/TAU_SERVO
        torque = L*T*math.sin(delta*DEG2RAD) - b_damp*(omega*DEG2RAD)
        alpha = torque/I
        omega += alpha*RAD2DEG*DT
        ys.append(omega)
    return np.array(ys)

def metrics(y, target):
    n = len(y)
    yf = y[-1]
    peak = y.max()
    os_pct = (peak-target)/target*100
    # 10-90 上升
    i10 = next((i for i,v in enumerate(y) if v >= 0.1*target), None)
    i90 = next((i for i,v in enumerate(y) if v >= 0.9*target), None)
    tr = (i90-i10)*DT*1000 if (i10 is not None and i90 is not None) else float("nan")
    tail = y[int(n*0.7):]
    cross = int(np.sum(np.diff(np.sign(tail-yf)) != 0))
    return yf, os_pct, tr, cross

# 相位裕度解析：L = K/(s) · e^{-jωτ}, ω_c=K, PM = 90° - atan(K·τ)
TAU_D = 0.3*0.005/0.3*0 + (1-0.3)/0.3*0.005 + 0.0025  # 0.3 滤波等效 + 采样 2.5ms
def phase_margin(I):
    K = KP*L*T/I
    pm = 90.0 - math.degrees(math.atan(K*TAU_D))
    return K, pm

print("=== 反推：不同真实惯量下，原版固件的闭环表现 ===")
print("  I(kg·m²)   K(1/s)  相位裕度   上升(ms)  超调(%)  尾部振荡  判定")
for I in [0.005, 0.01, 0.02, 0.03, 0.05, 0.08, 0.12]:
    y = step_response(I)
    yf, os_pct, tr, cr = metrics(y, 20.0)
    K, pm = phase_margin(I)
    if pm < 20 or cr >= 6:
        verdict = "发散/振荡"
    elif pm < 40:
        verdict = "明显振荡"
    elif tr < 60:
        verdict = "稳且灵敏 ←"
    elif tr < 150:
        verdict = "稳、偏慢"
    else:
        verdict = "稳但很钝"
    print("%9.3f %8.1f %8.1f° %9.0f %9.1f %9d  %s" % (I, K, pm, tr, os_pct, cr, verdict))

# 偏航：原版差速权限反推
print("\n=== 偏航：原版差速在 180 µs 全行程下能给出多大角加速度 ===")
print(" I_yaw(kg·m²)   α@180µs(deg/s²)   0→90dps 时间(s)   说明")
for Iy in [0.002, 0.005, 0.01, 0.02, 0.05]:
    alpha_rad = 1.0e-6*180.0/Iy
    alpha_deg = alpha_rad*RAD2DEG
    t90 = 90.0/alpha_deg if alpha_deg > 0 else float("inf")
    print("%12.3f %16.2f %18.2f   %s" % (Iy, alpha_deg, t90,
          "能控" if t90 < 3 else ("偏弱" if t90 < 10 else "几乎无权限")))

print("\n=== 若原版偏航实飞可用，反推 k_yaw 下限 ===")
print(" 假设 I_yaw=0.01、要求 0→45dps 在 1.5s 内（可接受偏航响应）:")
need = 45.0/1.5/RAD2DEG   # rad/s²
k_need = need*0.01/180.0
print("   需要 α=%.3f rad/s²，180µs 对应 k_yaw ≥ %.2e N·m/µs（当前 1.0e-6，差 %.0f 倍）"
      % (need, k_need, k_need/1e-6))
