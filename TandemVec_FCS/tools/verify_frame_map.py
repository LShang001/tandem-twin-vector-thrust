# -*- coding: utf-8 -*-
"""存档系(z=推力轴朝下) ↔ 推进模型系(x'=推力轴朝机头) 的轴置换推导"""
import numpy as np

# 存档系右手基（e1=前, e2=右, e3=下=推力轴指地）
e1, e2, e3 = np.eye(3)
print('存档系手性 e1×e2 =', np.cross(e1, e2), '= e3 ✓')

# 模型系：x' = 推力轴朝机头 = -e3；取 y' = e2（右）
xp = -e3
yp = e2
zp = np.cross(xp, yp)
print(f"\n模型系基（存档系表达）: x'={xp}  y'={yp}  z'={zp}")
print(f"  z' = x'×y' = {zp} → {'= e1（存档前向）' if np.allclose(zp, e1) else '?'}")
print(f"  手性检验 x'×y'=z' ✓")

# 向量分量变换：v' = R·v，R 的行是模型系基在存档系的分量
R = np.vstack([xp, yp, zp])
print(f'\nR (存档→模型) =\n{R.astype(int)}')
print(f'  det={np.linalg.det(R):+.0f}（+1 = 纯旋转 ✓）')
print("  → ω_x' = -ω_z,  ω_y' = +ω_y,  ω_z' = +ω_x")
print("  → M_x'(差速) = -M_z(存档yaw通道)")
print("     M_y'(尾摆) = +M_y(存档pitch通道)")
print("     M_z'(前摆) = +M_x(存档roll通道)")

# 执行器物理绕轴（模型系）→ 存档系语义
print('\n执行器对应：')
print("  差速 绕 x'(推力轴) = -e3 → 存档 yaw 通道（yaw 摇杆）✓")
print("  尾摆 绕 y'          = e2  → 存档 pitch 通道（pitch 摇杆）✓")
print("  前摆 绕 z'          = e1  → 存档 roll 通道（roll 摇杆）✓")
print('\n结论：与存档 VTVL 打杆语义天然一一对应（roll/pitch 杆管两个摆座，yaw 杆管差速）')
