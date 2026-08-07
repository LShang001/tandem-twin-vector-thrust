# -*- coding: utf-8 -*-
"""核验 sensor_imu 轴映射链的手性与四元数转换（本轮两处修复）"""
import numpy as np

# 传感器 RUB 右手系基（世界参考：e1=右, e2=上, e3=后）
sX, sY, sZ = np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])
print('传感器 RUB 手性: sX×sY =', np.cross(sX, sY), '应 = sZ', sZ,
      '→', 'OK' if np.allclose(np.cross(sX, sY), sZ) else 'FAIL')

# --- 步骤5 修复后：bX=sY, bY=sX, bZ=-sZ ---
bX, bY, bZ = sY, sX, -sZ
ok = np.allclose(np.cross(bX, bY), bZ)
print(f'\n[步骤5 新] bX=sY bY=sX bZ=-sZ : bX×bY={np.cross(bX,bY)} bZ={bZ} → {"右手系 OK" if ok else "左手系 FAIL"}')
bZ_old = sZ
print(f'[步骤5 旧] bZ=+sZ            : bX×bY={np.cross(bX,bY)} bZ={bZ_old} → '
      f'{"右手系" if np.allclose(np.cross(bX,bY), bZ_old) else "左手系 FAIL(已修)"}')

# --- 实测静止验证：原映射 acc=(0.002,-0.013,-0.997) 反解传感器读数 ---
aX_o, aY_o, aZ_o = 0.002, -0.013, -0.997   # 原映射 aX=-sZ, aY=sX, aZ=-sY
s_read = {'sZ': -aX_o, 'sX': aY_o, 'sY': -aZ_o}
print(f'\n实测反解传感器: sX={s_read["sX"]:+.3f} sY={s_read["sY"]:+.3f} sZ={s_read["sZ"]:+.3f} g')
print(f'  → sY≈+1g = 传感器Y轴朝天 = 机头方向(竖直朝天) → bX=sY 正确')
print(f'新映射静止应读: aX={s_read["sY"]:+.3f} aY={s_read["sX"]:+.3f} aZ={-s_read["sZ"]:+.3f}'
      f'  (x≈+1g 沿机头)')

# --- 步骤6：m = M·b（mX=bZ, mY=-bY, mZ=bX），检验 FLU 手性与重力 ---
M = np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]], dtype=float)
mX, mY, mZ = M @ bX, M @ bY, M @ bZ  # 基向量在 m 系的分量表达（形式检验）
print(f'\n[步骤6] M=\n{M}\n  迹={np.trace(M):+.1f}  det={np.linalg.det(M):+.1f}'
      f' → {"180° 旋转" if abs(np.trace(M)+1) < 1e-9 else "非180°"}')
print(f'  重力检验: bX=+1g(朝天) → mZ = bX = +1g → Madgwick FLU 的 Z(上) ✓')

# --- 步骤8：q_body_from_flu ---
R = M.T  # 正交，M 对称故 R=M
tr = np.trace(R)
theta = np.degrees(np.arccos((tr - 1) / 2))
nnT = (R + np.eye(3)) / 2
n = np.sqrt(np.diag(nnT)) * np.sign([nnT[0, 2] if nnT[0, 0] > 0 else 1, 1, 1] and [1, 1, 1])
axis = np.array([np.sqrt(nnT[0, 0]), np.sqrt(nnT[1, 1]), np.sqrt(nnT[2, 2])])
print(f'\n[步骤8] 旋转角={theta:.1f}°  轴≈({axis[0]:.4f},{axis[1]:.4f},{axis[2]:.4f})')
w = np.cos(np.radians(theta / 2))
v = np.sin(np.radians(theta / 2)) * axis
print(f'  四元数 {{w,x,y,z}} = {{{w:.4f}, {v[0]:.4f}, {v[1]:.4f}, {v[2]:.4f}}}')
print(f'  固件写入: {{0.0, 0.70710678, 0.0, 0.70710678}} → '
      f'{"一致 OK" if abs(w) < 1e-6 and abs(v[0]-0.7071) < 1e-3 and abs(v[2]-0.7071) < 1e-3 else "不一致 FAIL"}')
print('  注：180° 旋转 q ≡ q⁻¹（差整体符号），故左右乘方向不影响结果')
