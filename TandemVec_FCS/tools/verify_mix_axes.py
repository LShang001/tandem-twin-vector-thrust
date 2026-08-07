# -*- coding: utf-8 -*-
"""以"实机已验证正确的直通行为"为锚点，推导恢复存档映射后 mix 层的轴置换与符号。

旧映射(2026-08-07 误改, x_b竖直): bX=+sY, bY=+sX, bZ=-sZ
新映射(存档原版, z_b=推力轴):     bX=-sZ, bY=+sX, bZ=-sY
"""
import numpy as np
import sys
sys.path.insert(0, 'simulations/high-fidelity-analysis')
from core import load_params, control_effectiveness

# ---- 1. 陀螺分量在两套映射下的对应（同一物理角速度）----
# 用传感器基分量表达：旧/新各轴等于哪个 ±sensor 轴
old = {'gx': ('sY', +1), 'gy': ('sX', +1), 'gz': ('sZ', -1)}
new = {'gx': ('sZ', -1), 'gy': ('sX', +1), 'gz': ('sY', -1)}
print('陀螺分量对应（旧 → 新）：')
rel = {}
for ko, (ao, so) in old.items():
    for kn, (an, sn) in new.items():
        if ao == an:
            sign = so * sn
            rel[ko] = (kn, sign)
            print(f'  {ko}_old = {"+" if sign > 0 else "-"}{kn}_new')

# ---- 2. 直通锚点（旧映射下实机三通道全部正确）----
print('\n实机验证锚点（旧映射，K>0）：tail=+K·gy_old  front=+K·gz_old  dw=+K·gx_old')
anchor_old = {'tail': ('gy', +1), 'front': ('gz', +1), 'dw': ('gx', +1)}
anchor_new = {}
for act, (g_old, s) in anchor_old.items():
    g_new, sg = rel[g_old]
    anchor_new[act] = (g_new, s * sg)
    print(f'  {act:5s} = {"+" if s*sg > 0 else "-"}K·{g_new}_new')

# ---- 3. 分配器增益（模型系）----
P = load_params()
w0 = P['thrTrim'] * P['wMax']
B, T0, tau0 = control_effectiveness(w0, P['dfTrim'], P['dtTrim'], 0.0, P)
Binv = np.linalg.inv(B)
g_dw, g_dt, g_df = Binv[0, 0], Binv[1, 1], Binv[2, 2]
print(f"\n分配器主对角增益: Mx'→Δω={g_dw:+.2f}  My'→δt={g_dt:+.2f}  Mz'→δf={g_df:+.2f}")

# ---- 4. 反解 mix 层符号 ----
# 内环负反馈：扰动 ω_i>0（target=0）→ alpha_i = -Kp·ω_i < 0
print('\n反解 mix 层（内环负反馈 alpha_i = -Kp·ω_i）：')
ctrl_axis = {'tail': ('alpha_pitch', 'gy', g_dt, "My'"),
             'front': ('alpha_roll', 'gx', g_df, "Mz'"),
             'dw':    ('alpha_yaw', 'gz', g_dw, "Mx'")}
for act, (alpha, gch, gain, mname) in ctrl_axis.items():
    g_new, want_sign = anchor_new[act]
    assert g_new == gch, f'{act}: 通道不匹配 {g_new} vs {gch}'
    # 需要 executor 符号 = want_sign（当 ω>0）
    # executor = gain · M；alpha = -Kp·ω < 0
    # M = c · alpha  → executor = gain·c·alpha = gain·c·(负)
    # 要 executor 符号 = want_sign → sign(gain·c·(-1)) = want_sign
    c_sign = want_sign * -1 * (1 if gain > 0 else -1)
    print(f'  {act:5s}: 需 {mname} = {"+" if c_sign > 0 else "-"}I·{alpha}'
          f'   (ω{gch[-1]}>0 → {act}{"+" if want_sign > 0 else "-"})')
