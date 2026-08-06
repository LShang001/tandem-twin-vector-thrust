# -*- coding: utf-8 -*-
"""B_true 分配器符号验证：My→dt 系数、Mx→dw 系数（决定闭环符号）"""
import sys
sys.path.insert(0, 'simulations/high-fidelity-analysis')
from core import load_params, control_effectiveness
import numpy as np

P = load_params()
w0 = P['thrTrim'] * P['wMax']
B, T0, tau0 = control_effectiveness(w0, P['dfTrim'], P['dtTrim'], 0.0, P)
print('B 矩阵 (行=Mx,My,Mz; 列=[dw,dt,df]):')
print(np.array_str(B, precision=5, suppress_small=True))
Binv = np.linalg.inv(B)
print('Binv (行=[dw,dt,df]; 列=Mx,My,Mz):')
print(np.array_str(Binv, precision=5, suppress_small=True))
print(f'My→dt 系数 Binv[1,1] = {Binv[1,1]:+.5f}')
print(f'My→df 系数 Binv[2,1] = {Binv[2,1]:+.5f}')
print(f'Mx→dw 系数 Binv[0,0] = {Binv[0,0]:+.5f}')
