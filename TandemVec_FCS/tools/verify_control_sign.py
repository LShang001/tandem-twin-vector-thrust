# -*- coding: utf-8 -*-
"""数值核对俯仰通道"姿态误差 → 力矩 → 舵机脉宽"符号链。

@file    verify_control_sign.py
@module  工具/固件核对（控制符号）
@brief   从 +10° 俯仰误差出发，逐步算出角加速度、My、δt 与舵机脉宽，断言负反馈
@details 输入：姿态外环 Kp=2.8、速率环 Kp=16.042818、Iy=0.022、力臂 0.315、
         kT、悬停转速；处理：四元数误差 → 角速率目标 → 角加速度 → My → δt → 脉宽；
         输出：stdout 符号链；FAIL 时 exit 1（可作 CI 断言）。
@theory  q_err = q_cur⁻¹⊗q_tgt；α = Kp_rate·Kp_att·e @ref CTL-001 §4
@frame   机体系 FRD、NED 欧拉角
@units   deg、deg/s、deg/s²、rad/s²、N·m、µs（中位 1500）
@ref     CTL-001 §4（SAS 速率环与角度域量纲）、MOD-002 §3
"""

import math
import sys


def e2q(roll, pitch, yaw):
    """@brief 3-2-1 欧拉角 → 四元数 (x,y,z,w)
    @param[in] roll,pitch,yaw [rad]
    @return (x,y,z,w)，与固件差整体符号不影响姿态
    """
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def conj(q):
    """@brief 单位四元数共轭（逆）：q⁻¹ = (-x,-y,-z,w)"""
    return (-q[0], -q[1], -q[2], q[3])


def qmul(a, b):
    """@brief 四元数乘法 a⊗b（分量顺序 (x,y,z,w)，与 e2q/conj 一致）"""
    return (
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    )


DEG = math.pi / 180.0

# Current attitude: pitch +10 deg; target: level.
q_cur = e2q(0.0, 10.0 * DEG, 0.0)
q_tgt = (0.0, 0.0, 0.0, 1.0)
q_err = qmul(conj(q_cur), q_tgt)
sign = 1.0 if q_err[3] >= 0.0 else -1.0
vn = math.sqrt(q_err[0] ** 2 + q_err[1] ** 2 + q_err[2] ** 2)
scale = 2.0 * math.atan2(vn, abs(q_err[3])) / vn / DEG if vn > 0.25 else 2.0 / DEG
err_pitch = sign * q_err[1] * scale
err_roll = sign * q_err[0] * scale
print(f'q_cur(pitch +10 deg) = {q_cur}')
print(f'q_err = {q_err}  err_pitch={err_pitch:+.2f} deg  err_roll={err_roll:+.2f} deg')

# Controller domain: deg, deg/s, deg/s^2.
kp_angle = 2.8
rate_target_dps = kp_angle * err_pitch
print(f'outer pitch rate target = {rate_target_dps:+.3f} deg/s')

kp_rate = 16.042818
alpha_pitch_dps2 = kp_rate * rate_target_dps
alpha_pitch_radps2 = alpha_pitch_dps2 * DEG
print(
    f'inner alpha_pitch = {alpha_pitch_dps2:+.3f} deg/s^2 '
    f'= {alpha_pitch_radps2:+.3f} rad/s^2'
)

# Physical boundary and BTRUE pitch main channel.
iy = 0.022
my_cmd = iy * alpha_pitch_radps2
print(f'My_cmd = Iy*alpha = {my_cmd:+.5f} N*m')

# At zero gimbal angle, dMy/d(delta_t) = -b*T0.
b_arm = 0.315
kt = 1.04e-5
w_hover = 574.0
t0 = kt * w_hover * w_hover
delta_t = my_cmd / (-b_arm * t0)
print(f'delta_t = My/(-b*T0) = {delta_t:+.5f} rad = {delta_t/DEG:+.2f} deg')

# Flight-tested servo direction: dir_pitch=-1.
gear = 40.0 / 30.0
dir_pitch = -1.0
servo_deg = delta_t / DEG * gear * dir_pitch
pulse_us = 1500.0 + servo_deg / 45.0 * 500.0
print(f'servo pulse = {pulse_us:.0f} us (center 1500, dir_pitch=-1)')

my_physical = -b_arm * t0 * delta_t
negative_feedback = (
    err_pitch < 0.0
    and alpha_pitch_radps2 < 0.0
    and my_cmd < 0.0
    and delta_t > 0.0
    and pulse_us < 1500.0
    and my_physical < 0.0
)
print(
    f'My_physical = {my_physical:+.5f} N*m -> '
    f'{"negative feedback PASS" if negative_feedback else "sign chain FAIL"}'
)
if not negative_feedback:
    sys.exit(1)
