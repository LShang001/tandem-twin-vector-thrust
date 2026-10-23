"""debug_vtol.py — 悬停姿态下的重力投影/推力平衡一次性手算诊断。

@file    debug_vtol.py
@module  analysis/调试脚本
@brief   打印悬停四元数对应的机体系重力分量与推力裕度，核对符号约定。
@details 输入：models/aircraft-model.json；处理：quat_rotate(重力, q*) 得 g_b，
         并与 2·kT·ω² 的平衡推力比较；输出：stdout 数值。
         运行：`py -3.12 simulations/high-fidelity-analysis/debug_vtol.py`。
@theory  重力在机体系投影 g_b=q*g*q⁻¹ @ref MOD-002 §3.2；
         悬停平衡 m·g = 2·kT·ω² @ref MOD-001 §3.2。
@frame  NED + FRD；悬停时机头朝天（x_b 指向 NED −z），g_b≈[−g,0,0]。
@units   重力 m/s²、推力 N、转速 rad/s。
@note    一次性排查脚本，不是回归测试；结果仅用于人工核对符号，不产生数据文件。
@see     core.py、run_vtol.py、@ref MOD-002
"""
import sys; sys.path.insert(0,'simulations/high-fidelity-analysis')
from core import load_params, quat_rotate, quat_conj, quat_multiply, quat_norm
import numpy as np

P=load_params()
qh=np.array([np.cos(-np.pi/4),0,np.sin(-np.pi/4),0])
print('qh:', qh)

# Gravity in body frame
gb = quat_rotate(np.array([0,0,P['g']]), quat_conj(qh))
print('gb (body gravity):', gb)
print(f'Expected: approx [-{P["g"]:.2f}, 0, 0] for nose-up hover')

# Thrust
oh=799.4; T=2*P['kT']*oh*oh
print(f'Thrust at {oh:.0f} rad/s: {T:.2f} N, mg={P["m"]*P["g"]:.2f} N')
print(f'Net acceleration: {(T/P["m"] + gb[0]):.4f} m/s²')
print(f'(Should be near zero for hover balance)')

