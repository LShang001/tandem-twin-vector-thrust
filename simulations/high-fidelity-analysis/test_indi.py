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
