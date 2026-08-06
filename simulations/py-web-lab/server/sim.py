# -*- coding: utf-8 -*-
"""py-web-lab 仿真引擎：状态 + 子步推进（复用 high-fidelity-analysis/core.py）。

状态结构与 vector-thrust-lab 的 sim 完全一致（S/F/dyn/aero 字段同名），
遥测输出四元数为 JS 顺序 (x,y,z,w)；引擎内部用 core.py 的 (w,x,y,z)。
"""
import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[3]   # 仓库根
sys.path.append(str(ROOT / 'simulations/high-fidelity-analysis'))
from core import (  # noqa: E402
    load_params, Propulsion, aero_forces, rk4_step,
    quat_norm, quat_multiply, quat_conj, quat_rotate, euler_from_quat,
)

# 悬停初始姿态：绕 NED y +90°（机头朝天，x_b → NED −z）；(w,x,y,z)
Q_HOVER = np.array([np.sqrt(0.5), 0.0, np.sqrt(0.5), 0.0])


def hover_throttle(P):
    """悬停配平油门（物理派生）：√(m·g / (2·kT·wMax²))"""
    return np.sqrt(P['m'] * P['g'] / (2.0 * P['kT'] * P['wMax'] ** 2))


def _zero_vec3():
    return {'x': 0.0, 'y': 0.0, 'z': 0.0}


class Simulation:
    """仿真引擎：一子步推进 + 状态管理。控制律由 controllers.apply 注入。"""

    def __init__(self, P=None):
        self.P = P if P is not None else load_params()
        self.S = self._init_S()
        self.F = {
            'vel': _zero_vec3(), 'vWorld': _zero_vec3(), 'pos': _zero_vec3(),
            'euler': _zero_vec3(),
        }
        self.dyn = {k: 0.0 for k in ('Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz', 'Tf', 'Tt', 'Qf', 'Qt')}
        self.aero = {k: 0.0 for k in ('V', 'qbar', 'alpha', 'beta', 'Mx', 'My', 'Mz')}
        self.prop = Propulsion(self.P)
        self.prev_wf = 0.0
        self.prev_wt = 0.0
        self._quat = np.array([1.0, 0.0, 0.0, 0.0])   # (w,x,y,z) 内部表达
        self.reset_full()

    # ---------- 状态 ----------
    def _init_S(self):
        return {
            'thr': 0.5, 'dt': 0.0, 'df': 0.0, 'dw': 0.0,
            'dtAct': 0.0, 'dfAct': 0.0, 'dwAct': 0.0,
            'sasMode': 1, '_prevSasMode': 1,
            'aero': True, 'lockXY': False,
            'vtolMode': False, 'useBtrue': False,
            'altHold': False, 'altRef': 5.0, 'intAlt': 0.0,
            'paused': False,
            'intTh': 0.0, 'intPhi': 0.0,
            'wf': 0.0, 'wt': 0.0,
            'omega': _zero_vec3(),
            'quat': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
            'time': 0.0,
        }

    # ---------- 复位 ----------
    def reset_full(self):
        """全复位：巡航配平（对齐 JS resetSimulationState）"""
        P = self.P
        S = self.S
        a0 = P['aTrim']
        S.update({
            'vtolMode': False, 'altHold': False, 'intAlt': 0.0,
            'thr': P['thrTrim'], 'dt': 0.0, 'df': 0.0, 'dw': 0.0,
            'dtAct': P['dtTrim'], 'dfAct': P['dfTrim'], 'dwAct': 0.0,
            'intTh': 0.0, 'intPhi': 0.0, 'time': 0.0,
            'aero': True, 'lockXY': False,
            '_prevSasMode': S['sasMode'],
        })
        self._quat = np.array([np.cos(a0 / 2), 0.0, np.sin(a0 / 2), 0.0])
        S['quat'] = self._quat_js()
        S['omega'] = _zero_vec3()
        v = P['vTrim']
        self.F['vel'] = {'x': v * np.cos(a0), 'y': 0.0, 'z': v * np.sin(a0)}
        self.F['vWorld'] = {'x': v, 'y': 0.0, 'z': 0.0}
        self.F['pos'] = _zero_vec3()
        w0 = P['thrTrim'] * P['wMax']
        S['wf'] = w0; S['wt'] = w0
        self.prop.wf = self.prop.prev_wf = w0
        self.prop.wt = self.prop.prev_wt = w0
        self.prev_wf = w0; self.prev_wt = w0
        self._clear_telemetry()
        self._update_euler()

    def reset_vtol(self):
        """进入悬停：机头朝天 + 无翼 + 悬停配平（对齐 JS resetVtolHoverState）"""
        P = self.P
        S = self.S
        thr = hover_throttle(P)
        w0 = thr * P['wMax']
        S.update({
            'vtolMode': True, 'altHold': False, 'useBtrue': False,
            'intAlt': 0.0, 'intTh': 0.0, 'intPhi': 0.0,
            'thr': thr, 'dt': 0.0, 'df': 0.0, 'dw': 0.0,
            'dtAct': 0.0, 'dfAct': 0.0, 'dwAct': 0.0,
            'aero': False, 'lockXY': False, 'time': 0.0,
        })
        self._quat = Q_HOVER.copy()
        S['quat'] = self._quat_js()
        S['omega'] = _zero_vec3()
        self.F['vel'] = _zero_vec3()
        self.F['vWorld'] = _zero_vec3()
        self.F['pos'] = _zero_vec3()
        S['wf'] = w0; S['wt'] = w0
        self.prop.wf = self.prop.prev_wf = w0
        self.prop.wt = self.prop.prev_wt = w0
        self.prev_wf = w0; self.prev_wt = w0
        self._clear_telemetry()
        self._update_euler()

    def reset_pose_only(self):
        """轻量复位：只复位飞行状态，保留模式与输入（对齐 JS resetPoseOnly）"""
        P = self.P
        S = self.S
        S['intTh'] = 0.0; S['intPhi'] = 0.0; S['intAlt'] = 0.0
        w0 = S['thr'] * P['wMax']
        if S['vtolMode']:
            self._quat = Q_HOVER.copy()
            self.F['vel'] = _zero_vec3()
            self.F['vWorld'] = _zero_vec3()
            self.F['pos'] = _zero_vec3()
            S['omega'] = _zero_vec3()
            S['dtAct'] = 0.0; S['dfAct'] = 0.0; S['dwAct'] = 0.0
        else:
            self.reset_flight_cruise()
            S['dtAct'] = P['dtTrim']; S['dfAct'] = P['dfTrim']; S['dwAct'] = 0.0
        S['wf'] = w0; S['wt'] = w0
        self.prop.wf = self.prop.prev_wf = w0
        self.prop.wt = self.prop.prev_wt = w0
        self.prev_wf = w0; self.prev_wt = w0
        S['_prevSasMode'] = S['sasMode']
        self._clear_telemetry()
        self._update_euler()

    def reset_flight_cruise(self):
        """巡航飞行状态复位（对齐 JS resetFlightState，含 lockXY 分支）"""
        P = self.P
        a0 = P['aTrim']
        v = P['vTrim']
        self._quat = np.array([np.cos(a0 / 2), 0.0, np.sin(a0 / 2), 0.0])
        self.S['quat'] = self._quat_js()
        self.S['omega'] = _zero_vec3()
        self.F['vel'] = {'x': v * np.cos(a0), 'y': 0.0, 'z': v * np.sin(a0)}
        self.F['vWorld'] = {'x': v, 'y': 0.0, 'z': 0.0}
        self.F['pos'] = _zero_vec3()
        if self.S['lockXY']:
            self.F['vel'] = {'x': 0.0, 'y': 0.0, 'z': v * np.sin(a0)}
            self.F['vWorld'] = {'x': 0.0, 'y': 0.0, 'z': 0.0}

    # ---------- 四元数辅助 ----------
    def _quat_js(self):
        w, x, y, z = self._quat
        return {'x': float(x), 'y': float(y), 'z': float(z), 'w': float(w)}

    def _update_euler(self):
        phi, theta, psi = euler_from_quat(self._quat)
        self.F['euler'] = {'x': float(phi), 'y': float(theta), 'z': float(psi)}

    def _clear_telemetry(self):
        for k in self.dyn:
            self.dyn[k] = 0.0
        for k in self.aero:
            self.aero[k] = 0.0

    # ---------- 子步推进 ----------
    def step(self, dt, apply_control):
        """单子步：控制律 → 推进 → 气动 → RK4（对齐 dynamics.mjs physicsStep）"""
        P = self.P
        S = self.S
        # 1) 控制律（更新 dtAct/dfAct/dwAct，可选 thr/altRef 消费；返回 thr 修正）
        thr_out = apply_control(self, dt)
        if thr_out is not None:
            S['thr'] = float(np.clip(thr_out, 0.0, 1.0))
        # 2) 推进（差速分配 + 一阶滞后 + 力/力矩）
        omega0 = S['thr'] * P['wMax']
        self.prop.update(omega0, S['dwAct'], dt)
        Fx, Fy, Fz, Mx, My, Mz = self.prop.forces(S['dfAct'], S['dtAct'])
        S['wf'] = float(self.prop.wf)
        S['wt'] = float(self.prop.wt)
        # 瞬态反扭基准（对齐 propulsion.mjs：sim.prevWf 每子步更新，
        # B_true 分支读"上一子步"的转速差分）
        self.prev_wf = float(self.prop.wf)
        self.prev_wt = float(self.prop.wt)
        self.dyn.update(Fx=Fx, Fy=Fy, Fz=Fz, Mx=Mx, My=My, Mz=Mz,
                        Tf=P['kT'] * self.prop.wf ** 2, Tt=P['kT'] * self.prop.wt ** 2,
                        Qf=P['kQ'] * self.prop.wf ** 2 + P['Jp'] * self.prop.dwf,
                        Qt=P['kQ'] * self.prop.wt ** 2 + P['Jp'] * self.prop.dwt)
        # 3) 气动（遥测/子步初；RK4 内部逐阶段重算）
        v = np.array([self.F['vel'][k] for k in ('x', 'y', 'z')])
        w = np.array([S['omega'][k] for k in ('x', 'y', 'z')])
        aX, aY, aZ, La, Ma, Na = aero_forces(v, w, P)
        self.aero.update(V=float(np.linalg.norm(v)),
                         qbar=float(0.5 * P['rho'] * np.linalg.norm(v) ** 2),
                         alpha=float(np.arctan2(v[2], v[0])),
                         beta=float(np.arcsin(np.clip(v[1] / max(np.linalg.norm(v), P['vMin']), -1, 1))),
                         Mx=La, My=Ma, Mz=Na)
        # 4) RK4 联合积分 [v, ω, q]
        v_new, w_new, q_new = rk4_step(v, w, self._quat, self.prop, P,
                                        Fx, Fy, Fz, Mx, My, Mz, dt,
                                        use_aero=S['aero'])
        # 5) 状态写回
        for k, val in zip(('x', 'y', 'z'), v_new):
            self.F['vel'][k] = float(val)
        for k, val in zip(('x', 'y', 'z'), w_new):
            S['omega'][k] = float(val)
        self._quat = quat_norm(q_new)
        S['quat'] = self._quat_js()
        # 惯性速度 + 位置（对齐 dynamics.mjs：NED 系；lockXY 时水平速度不积分）
        vw = quat_rotate(v_new, self._quat)
        for k, val in zip(('x', 'y', 'z'), vw):
            self.F['vWorld'][k] = float(val)
        if S['lockXY']:
            self.F['vWorld']['x'] = 0.0
            self.F['vWorld']['y'] = 0.0
            vb = quat_rotate(np.array([self.F['vWorld']['x'],
                                       self.F['vWorld']['y'],
                                       self.F['vWorld']['z']]),
                             quat_conj(self._quat))
            for k, val in zip(('x', 'y', 'z'), vb):
                self.F['vel'][k] = float(val)
        else:
            for k in ('x', 'y'):
                self.F['pos'][k] += self.F['vWorld'][k] * dt
        self.F['pos']['z'] += self.F['vWorld']['z'] * dt  # z 积分无条件（同 dynamics.mjs）
        # 地面钳位（对齐 dynamics.mjs：pos.z>groundZ 且下落时，vWorld.z 清零并
        # 反向旋转回机体系同步 F.vel —— 保持 vWorld≡R(q)·vel 不变量）
        if self.F['pos']['z'] > P['groundZ'] and self.F['vWorld']['z'] > 0:
            self.F['pos']['z'] = P['groundZ']
            self.F['vWorld']['z'] = 0.0
            vb = quat_rotate(np.array([self.F['vWorld']['x'],
                                       self.F['vWorld']['y'], 0.0]),
                             quat_conj(self._quat))
            for k, val in zip(('x', 'y', 'z'), vb):
                self.F['vel'][k] = float(val)
        S['time'] += dt
        self._update_euler()
