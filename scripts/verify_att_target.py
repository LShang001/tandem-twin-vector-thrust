# -*- coding: utf-8 -*-
"""VTOL 悬停构型目标姿态合成推导验证
约定：q = NED→FRD（机体系 x=推力轴=机头方向）
VTOL 悬停基态 q0 = 绕 NED y 转 +90°（机头朝天，x̂_b = NED (0,0,-1)）
世界航向 W：绕 NED z 正转（地图顺时针）
"""
import numpy as np

def qmul(p, q):
    w1, x1, y1, z1 = p; w2, x2, y2, z2 = q
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2,
                     w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2])

def qnorm(q): return q / np.linalg.norm(q)

def rot_vec(v, q):
    qn = qnorm(q); qi = np.array([qn[0], -qn[1], -qn[2], -qn[3]])
    return qmul(qmul(qn, np.array([0, v[0], v[1], v[2]])), qi)[1:]

def Rz(w):  return np.array([np.cos(w/2), 0, 0, np.sin(w/2)])
def Ry(a):  return np.array([np.cos(a/2), 0, np.sin(a/2), 0])
def Rx(a):  return np.array([np.cos(a/2), np.sin(a/2), 0, 0])

# VTOL 悬停基态（机头朝天）
q0 = Ry(np.pi/2)
print('q0 机头方向（应≈(0,0,-1) 上）:', np.round(rot_vec([1,0,0], q0), 4))

# ---- 场景 1：悬停 + 世界航向 90° ----
W = np.pi/2
# 期望：绕 NED z 转 W 后的悬停姿态
q_expect = qmul(Rz(W), q0)
print('\n场景1 期望（q_W ⊗ q0）:', np.round(q_expect, 4))

# 方案：q_target = q_tilt(恒等) ⊗ q0 ⊗ Rx_b(-W)
q_yaw = qmul(q0, Rx(-W))
q_target1 = qmul(np.array([1.,0,0,0]), q_yaw)
print('方案A（q_tilt ⊗ q0 ⊗ Rx(-W)）:', np.round(q_target1, 4), ' 一致:', np.allclose(q_target1, q_expect))

# 原代码方案（q_tilt ⊗ Rz_b(W)，z 基准）作对照
q_old = qmul(np.array([1.,0,0,0]), Rz(W))
print('原代码（q_tilt ⊗ Rz(W)）:', np.round(q_old, 4), ' 不一致（机体 z 水平，非航向）')

# ---- 场景 2：悬停 + 航向 0 ----
q_target2 = qmul(np.array([1.,0,0,0]), qmul(q0, Rx(0)))
print('\n场景2 航向0（应=q0 机头朝天）:', np.round(q_target2, 4))
print('  机头方向:', np.round(rot_vec([1,0,0], q_target2), 4), ' 右:', np.round(rot_vec([0,1,0], q_target2), 4))

# ---- 场景 3：倾斜 10°（d 在 x-z 平面，NED 北-上） + 航向 0 ----
d = np.array([np.sin(np.radians(10)), 0, -np.cos(np.radians(10))])
r = np.array([0., 0, -1])
dot = np.dot(r, d); cr = np.cross(r, d)
q_tilt = qnorm(np.array([1 + dot, cr[0], cr[1], cr[2]]))
print('\n场景3 q_tilt（参考(0,0,-1)→d）:', np.round(q_tilt, 4))
q_target3 = qmul(q_tilt, qmul(q0, Rx(0)))
print('  机头方向（应=d）:', np.round(rot_vec([1,0,0], q_target3), 4), ' d:', np.round(d, 4))

# ---- 场景 4：倾斜 10° + 航向 90° ----
q_target4 = qmul(q_tilt, qmul(q0, Rx(-W)))
print('\n场景4 机头方向（应=d）:', np.round(rot_vec([1,0,0], q_target4), 4))
print('  绕推力轴自转（世界航向近似）:', np.round(rot_vec([0,1,0], q_target4), 4))
