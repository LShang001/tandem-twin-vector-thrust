# -*- coding: utf-8 -*-
"""核对直通模式的物理斜率假设：Δω→Mx、δf→Mz（与实机观察对照）"""
import sys
sys.path.insert(0, 'simulations/high-fidelity-analysis')
from core import load_params, Propulsion

P = load_params()
prop = Propulsion(P)
w0 = P['thrTrim'] * P['wMax']

# 差速斜率：Δω>0 → Mx 符号（直通假设：负斜率 → Δω=+K·ωx）
prop.wf = prop.prev_wf = w0
prop.wt = prop.prev_wt = w0
_, _, _, Mx0, _, _ = prop.forces(P['dfTrim'], P['dtTrim'])
prop.wf = prop.prev_wf = w0 * 1.05
prop.wt = prop.prev_wt = w0 * 0.95
_, _, _, Mx1, _, _ = prop.forces(P['dfTrim'], P['dtTrim'])
print(f'Δω=0 → +5% : Mx {Mx0:+.6f} → {Mx1:+.6f}  斜率={Mx1-Mx0:+.6f} (负=阻尼假设成立)')

# 前摆斜率：δf>0 → Mz（直通假设：正斜率 → δf=-K·ωz）
prop.wf = prop.prev_wf = w0
prop.wt = prop.prev_wt = w0
_, _, _, _, _, Mz0 = prop.forces(0.0, P['dtTrim'])
_, _, _, _, _, Mz1 = prop.forces(0.01745, P['dtTrim'])
print(f'δf=0 → +1°: Mz {Mz0:+.6f} → {Mz1:+.6f}  斜率={Mz1-Mz0:+.6f} (正=阻尼假设成立)')

# 尾摆斜率：δt>0 → My（直通假设：负斜率 → δt=+K·ωy，用户实测该通道正确）
_, _, _, _, My0, _ = prop.forces(P['dfTrim'], 0.0)
_, _, _, _, My1, _ = prop.forces(P['dfTrim'], 0.01745)
print(f'δt=0 → +1°: My {My0:+.6f} → {My1:+.6f}  斜率={My1-My0:+.6f} (负=阻尼假设成立)')

print(f'参数: kQ={P["kQ"]:.6f}')
