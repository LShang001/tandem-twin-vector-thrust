"""test_indi.py — SAS vs INDI 的俯仰脉冲扰动 IAE 快速对比。

@file    test_indi.py
@module  analysis/调试脚本（控制器对比）
@brief   在 24 m/s 配平点施加 +5°/s 俯仰脉冲，比较两种控制器的积分绝对误差。
@details 输入：参数 + trim_longitudinal(24)；处理：simulate() 两次（SAS / INDI）；
         输出：stdout 打印 IAE 与摆角范围。
         运行：`py -3.12 simulations/high-fidelity-analysis/test_indi.py`。
@theory  SAS 律 @ref MOD-001 §8；INDI 增量分配 @ref TandemVec-Paper/ch10。
@frame  NED + FRD；theta 为项目锁定显示值（与几何角反号）。
@units   时间 s、角速度 deg/s（扰动）、摆角 rad、IAE 无量纲。
@note    最小冒烟脚本（非 pytest），用于改控制器后快速回归；不含随机数。
@see     simulate.py、@ref MOD-001
"""
import sys; sys.path.insert(0,'simulations/high-fidelity-analysis')
from core import load_params
from trim_analysis import trim_longitudinal
from simulate import SASController, INDIController, simulate
import numpy as np

P = load_params(); trim = trim_longitudinal(24, P)

sas = SASController(P, trim)
data_s = simulate(sas, P, trim, T_total=10, disturbance=(3,3.1,"pitch",5.0))
s_iae = np.sum(np.abs(data_s["theta"] - data_s["theta"][0]))

indi = INDIController(P, trim)
data_i = simulate(indi, P, trim, T_total=10, disturbance=(3,3.1,"pitch",5.0))
i_iae = np.sum(np.abs(data_i["theta"] - data_i["theta"][0]))

improve = (1 - i_iae/s_iae)*100 if s_iae > 0 else 0
print(f"SAS IAE={s_iae:.4f}, INDI IAE={i_iae:.4f}, Improvement={improve:.1f}%")
print(f"INDI dt range: [{np.min(data_i['delta_t']):.4f}, {np.max(data_i['delta_t']):.4f}]")

# 无扰动稳定运行
data_trim = simulate(indi, P, trim, T_total=12, disturbance=None)
v_drift = (np.mean(data_trim['u'][-50:]) - data_trim['u'][0]) / data_trim['u'][0] * 100
print(f"INDI 12s 配平漂移: {v_drift:.3f}%")
