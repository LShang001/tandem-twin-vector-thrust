# -*- coding: utf-8 -*-
"""数值推链：机头上仰 10° → 固件控制链 → 舵机指令 → 修正力矩方向
复刻固件公式：eulerToQuaternion(Z-Y'-X'') / q_error = q_current^-1 ⊗ q_target
/ 外环 P / 内环 P / My = Iy*alpha / 分配 dt / 舵机 PWM / 物理 My(δt)"""
import math

def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cx0(0)) if False else None

def e2q(r, p, y):
    cr, sr = math.cos(r/2), math.sin(r/2)
    cp, sp = math.cos(p/2), math.sin(p/2)
    cy, sy = math.cos(y/2), math.sin(y/2)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y_ = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return (x, y_, z, w)

def conj(q): return (-q[0], -q[1], -q[2], q[3])

def qmul(a, b):
    return (a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1],
            a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0],
            a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3],
            a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2])

DEG = math.pi/180

# 1. 机头上仰 10°（roll=0, pitch=+10°, yaw=0）
q_cur = e2q(0, 10*DEG, 0)
q_tgt = (0.0, 0.0, 0.0, 1.0)  # 水平
q_err = qmul(conj(q_cur), q_tgt)
sign = 1.0 if q_err[3] >= 0 else -1.0
vn = math.sqrt(q_err[0]**2 + q_err[1]**2 + q_err[2]**2)
scale = 2*math.atan2(vn, abs(q_err[3]))/vn*180/math.pi if vn > 0.25 else 2*180/math.pi
err_pitch = sign * q_err[1] * scale
err_roll = sign * q_err[0] * scale
print(f'q_cur(抬头10°) = {q_cur}')
print(f'q_err = {q_err}  err_pitch={err_pitch:+.2f}°  err_roll={err_roll:+.2f}°')

# 2. 外环 P（Kp_pitchAngle>0）：error 正 → 目标角速度正
KpA = 6.0  # 示意正增益
rate_tgt = KpA * err_pitch
print(f'外环 pitchRateTarget = {rate_tgt:+.3f} °/s')

# 3. 内环 P（Kp_rate>0，ω≈0）：alpha 正
KpR = 3.0
alpha_pitch = KpR * (rate_tgt - 0.0)
print(f'内环 alpha_pitch = {alpha_pitch:+.3f} rad/s²')

# 4. My = Iy * alpha
Iy = 0.14
My = Iy * alpha_pitch
print(f'My = Iy*alpha = {My:+.5f} N·m  → {"抬头力矩(更抬头=正反馈!)" if My > 0 else "低头力矩(修正✓)"}')

# 5. 分配：BTRUE 主通道 dt ≈ My 同号 → 舵机正摆
dt = My * 0.3  # 示意正增益
print(f'分配 dt = {dt:+.5f} rad = {dt*180/math.pi:+.2f}°')

# 6. 舵机 PWM：dt>0 → servo_deg>0 → pct>50 → 脉宽>1500us
print(f'舵机指令脉宽 = {1500 + dt*180/math.pi*1.333/45*500:+.0f} us (中位1500)')

# 7. 物理：δt>0 → My_phys = -0.00233 N·m/deg × dt_deg（负斜率！）
My_phys = -0.00233 * (dt*180/math.pi)
print(f'物理响应 My_phys(δt=+{dt*180/math.pi:.2f}°) = {My_phys:+.5f} N·m → {"低头修正 ✓ 负反馈" if My_phys < 0 else "抬头=正反馈 ✗"}')
