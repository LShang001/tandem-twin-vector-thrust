# -*- coding: utf-8 -*-
"""算法/数学审查：本轮改动的自洽性核查
1. 增益调度是否破坏 B_true 的三轴解耦（交叉耦合补偿是否仍正确）
2. TVC 内环增益与 15° 摆角限幅的匹配度（饱和阈值）
3. 差速通道限幅余量
"""
import numpy as np
import sys
sys.path.insert(0, 'simulations/high-fidelity-analysis')
from core import load_params, control_effectiveness

P = load_params()
kQ, kT, wMax = P['kQ'], P['kT'], P['wMax']
Ix, Iy, Iz = P['Ix'], P['Iy'], P['Iz']
dMax, dwMax = 0.2617994, P['dwMax']      # 固件当前 15°
m, g = P['m'], P['g']
w_hover = (0.5 * m * g / kT) ** 0.5

print('=' * 64)
print('【1】增益调度是否破坏 B_true 三轴解耦？')
print('=' * 64)
w0 = 0.19 * wMax
B, T0, tau0 = control_effectiveness(w0, P['dfTrim'], P['dtTrim'], 0.0, P)
Binv = np.linalg.inv(B)
print(f'19% 油门 Binv 主要项: Mx→dt={Binv[1,0]:+.2f}  My→dt={Binv[1,1]:+.2f}')
print('  ↑ Mx 对 dt 的影响比 My 还大（差速改变反扭，需摆座补偿）')

# 期望力矩
M_des = np.array([0.02, 0.05, -0.03])
s = (w0 / w_hover) ** 2
M_sched = M_des * np.array([s, 1.0, 1.0])
u = Binv @ M_sched
M_actual = B @ u
print(f'\n  调度系数 s={s:.3f}')
print(f'  期望 M      = [{M_des[0]:+.4f} {M_des[1]:+.4f} {M_des[2]:+.4f}]')
print(f'  调度后指令  = [{M_sched[0]:+.4f} {M_sched[1]:+.4f} {M_sched[2]:+.4f}]')
print(f'  实际产生 M  = [{M_actual[0]:+.4f} {M_actual[1]:+.4f} {M_actual[2]:+.4f}]')
ok_my = abs(M_actual[1] - M_des[1]) < 1e-9
ok_mz = abs(M_actual[2] - M_des[2]) < 1e-9
print(f'  → My 精确实现: {ok_my}   Mz 精确实现: {ok_mz}')
print(f'  → 结论：{"摆座通道未被污染，耦合补偿自洽 OK" if ok_my and ok_mz else "FAIL"}')
print('     （Mx 被有意缩小 = 设计意图；B⁻¹ 保证另两轴精确）')

print('\n' + '=' * 64)
print('【2】TVC 内环增益 vs 15° 摆角限幅')
print('=' * 64)
KP_TVC, RATE_MAX = 16.042818, 80.0  # ATTITUDE_MODE 外环角速率限幅；非 RATE_MODE 满杆
# 尾摆：My→dt 主通道
g_dt = abs(Binv[1, 1])
alpha_sat = dMax / (g_dt * Iy)              # 触发限幅的 alpha
err_sat = np.degrees(alpha_sat) / KP_TVC     # deg/s² ÷ s⁻¹ = deg/s
print(f'  尾摆 My→dt 增益={g_dt:.2f} rad/(N·m)   Iy={Iy}')
print(f'  触发 15° 限幅: alpha={alpha_sat:.2f} rad/s²  →  角速率误差={err_sat:.1f}°/s')
print(f'  姿态外环限幅 = {RATE_MAX:.0f}°/s  →  饱和倍数 {RATE_MAX/err_sat:.1f}×')
alpha_full = np.radians(KP_TVC * RATE_MAX)
dt_full = g_dt * Iy * alpha_full
print(f'  该参考需摆角 {np.degrees(dt_full):.0f}°（物理仅 15°）')
print(f'  → 姿态外环参考范围的线性区占比 {err_sat/RATE_MAX*100:.0f}%，其余为饱和/bang-bang')
alpha_max = dMax / (g_dt * Iy)
print(f'  15° 摆角可达最大角加速度 = {np.degrees(alpha_max):.0f}°/s²')
print(f'  → 从 0 加速到 {RATE_MAX:.0f}°/s 需 {RATE_MAX/np.degrees(alpha_max):.2f}s（可达，属加速期饱和）')

print('\n' + '=' * 64)
print('【3】差速通道限幅余量（调度后）')
print('=' * 64)
KP_YAW = 11.459156
denom_h = 2.0 * kQ * w_hover ** 2
dw_full = Ix * np.radians(KP_YAW * RATE_MAX) / denom_h
print(f'  80°/s 参考下 Δω={dw_full:.3f}（限幅 {dwMax}）余量 {dwMax/dw_full:.1f}×')
err_sat_yaw = np.degrees(dwMax * denom_h / Ix) / KP_YAW
print(f'  触发 dwMax 需角速率误差 {err_sat_yaw:.0f}°/s → 线性区覆盖'
      f' {min(err_sat_yaw/RATE_MAX*100, 100):.0f}% 姿态外环参考范围 ✓')
