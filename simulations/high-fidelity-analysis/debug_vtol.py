import sys; sys.path.insert(0,'simulations/high-fidelity-analysis')
from core import load_params, quat_rotate, quat_conj, quat_multiply, quat_norm
import numpy as np

P=load_params()
qh=np.array([np.cos(-np.pi/4),0,np.sin(-np.pi/4),0])
print('qh:', qh)

# Gravity in body frame
gb = quat_rotate(np.array([0,0,P['g']]), quat_conj(qh))
print('gb (body gravity):', gb)
print('Expected: approx [-9.81, 0, 0] for nose-up hover')

# Thrust
oh=799.4; T=2*P['kT']*oh*oh
print(f'Thrust at {oh:.0f} rad/s: {T:.2f} N, mg={P["m"]*P["g"]:.2f} N')
print(f'Net acceleration: {(T/P["m"] + gb[0]):.4f} m/s²')
print(f'(Should be near zero for hover balance)')

