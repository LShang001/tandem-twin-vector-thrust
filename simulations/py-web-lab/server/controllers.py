# -*- coding: utf-8 -*-
"""py-web-lab 控制律：逐位对齐 vector-thrust-lab/src/core/control.mjs。

覆盖：固定翼 SAS（4 模式）+ VTOL 悬停（四元数级联 + 定高 + B_true 分配）。
四元数内部用 core.py 顺序 (w,x,y,z)；S.quat 为 JS 顺序 (x,y,z,w)。
交叉验证 tests/cross_verify.mjs 保证两实现漂移 < 1e-6。
"""
import numpy as np

from sim import Q_HOVER, hover_throttle


def _clamp(x, lo, hi):
    return float(np.clip(x, lo, hi))


def _quat_mul(a, b):
    """core.py 顺序 (w,x,y,z) 的四元数乘积"""
    from core import quat_multiply
    return quat_multiply(a, b)


def _quat_conj(q):
    from core import quat_conj
    return quat_conj(q)


def _quat_norm(q):
    from core import quat_norm
    return quat_norm(q)


def _rotate(v, q):
    """机体系向量 → NED（对齐 JS rotateVecByQuat / core.quat_rotate）"""
    from core import quat_rotate
    return quat_rotate(np.asarray(v, dtype=float), q)


# ============================================================
#  入口：apply(sim, P, dt) → 更新 S.dtAct/dfAct/dwAct（可选 thr）
# ============================================================
def apply(sim, dt):
    """总控制入口（对齐 JS applySas）"""
    P = sim.P
    S = sim.S
    if S['vtolMode']:
        return _apply_vtol_hover(sim, dt)
    _apply_cruise_sas(sim, dt)
    return None


# ============================================================
#  固定翼 SAS（对齐 control.mjs:applySas 固定翼分支）
# ============================================================
def _apply_cruise_sas(sim, dt):
    P = sim.P
    S = sim.S
    F = sim.F
    theta = F['euler']['y']
    phi = F['euler']['x']
    theta_error = theta + P['aTrim']
    dtC = P['dtTrim'] + S['dt']
    dfC = P['dfTrim'] + S['df']
    dwC = S['dw']
    if S['_prevSasMode'] != S['sasMode']:
        S['intTh'] = 0.0
        S['intPhi'] = 0.0
        S['_prevSasMode'] = S['sasMode']
    if S['sasMode'] == 3:
        # 角速度闭环：滑块 = ω_ref（效率为负通道取 (ω−ω_ref)，正通道取 (ω_ref−ω)）
        q_ref = S['dt']; r_ref = S['df']; p_ref = S['dw']
        dtC = P['dtTrim'] + P['rateKq'] * (S['omega']['y'] - q_ref)
        dfC = _clamp(P['dfTrim'] + P['rateKr'] * (r_ref - S['omega']['z']), -P['dMax'], P['dMax'])
        dwC = _clamp(P['rateKp'] * (S['omega']['x'] - p_ref), -P['dwMax'], P['dwMax'])
    elif S['sasMode'] >= 1:
        # 角速率阻尼（模式 1/2 共用）
        dtC = dtC + P['sasQ'] * S['omega']['y']
        dfC = dfC - P['sasR'] * S['omega']['z']
        dwC = dwC + P['sasP'] * S['omega']['x']
        if S['sasMode'] == 1:
            # 全 SAS：姿态比例 + 积分
            S['intTh'] = _clamp(S['intTh'] + theta_error * dt, -P['intThMax'], P['intThMax'])
            S['intPhi'] = _clamp(S['intPhi'] + phi * dt, -P['intPhiMax'], P['intPhiMax'])
            dtC = dtC - P['sasTh'] * theta_error - P['sasI'] * S['intTh']
            dwC = dwC - P['sasPhi'] * phi - P['sasIPhi'] * S['intPhi']
    S['dtAct'] = _clamp(dtC, -P['dMax'], P['dMax'])
    S['dfAct'] = _clamp(dfC, -P['dMax'], P['dMax'])
    S['dwAct'] = _clamp(dwC, -P['dwMax'], P['dwMax'])


# ============================================================
#  VTOL 悬停（对齐 control.mjs:applyVtolHover）
# ============================================================
def _apply_vtol_hover(sim, dt):
    P = sim.P
    S = sim.S
    F = sim.F
    q = sim._quat  # (w,x,y,z)

    # 滑块 → 目标姿态小角度指令（仅 y/z；航向为角速度指令不入 qCmd）
    h = S['dt'] * 0.5
    f = S['df'] * 0.5
    q_local = _quat_mul(
        np.array([np.cos(h), 0.0, np.sin(h), 0.0]),        # 绕 y_b：俯仰
        np.array([np.cos(f), 0.0, 0.0, np.sin(f)]),        # 绕 z_b：侧倾
    )
    # qCmd 跟随当前航向（ψ_est = π/2 − atan2(yb.y, yb.x)）
    yb = _rotate([0.0, 1.0, 0.0], q)
    psi_est = np.pi / 2 - np.arctan2(yb[1], yb[0])
    q_cmd = _quat_norm(_quat_mul(
        _quat_mul(Q_HOVER, np.array([np.cos(psi_est / 2), np.sin(psi_est / 2), 0.0, 0.0])),
        q_local,
    ))

    # ---- 高度保持（独立于姿态自稳）----
    if S['altHold']:
        hgt = -F['pos']['z']
        vZ = -F['vWorld']['z']
        h_err = S['altRef'] - hgt
        S['intAlt'] = _clamp(S['intAlt'] + h_err * dt, -1.5, 1.5)
        v_zref = _clamp(P['altKpH'] * h_err + P['altKpI'] * S['intAlt'],
                        -P['altVZMax'], P['altVZMax'])
        # cosγ = x̂_b·(−ẑ_NED) = −R31；JS quat(x,y,z,w)：R31 = 2(xz − yw)
        # Python (w,x,y,z)：JS x=q1,y=q2,z=q3,w=q0 → xz−yw = q1q3 − q2q0
        cos_g = -2.0 * (q[1] * q[3] - q[2] * q[0])
        cos_g = max(cos_g, 0.5)
        thr_base = hover_throttle(P) / np.sqrt(cos_g)      # T∝thr² → 1/√cosγ
        S['thr'] = _clamp(thr_base + P['altKpV'] * (v_zref - vZ), 0.0, 1.0)

    if S['sasMode'] == 0:
        # 直通：dt/df 摆角（悬停基准 0）；dw 仍为航向角速度指令
        S['dtAct'] = _clamp(S['dt'], -P['dMax'], P['dMax'])
        S['dfAct'] = _clamp(S['df'], -P['dMax'], P['dMax'])
    else:
        # 机体系误差四元数：qe = qCmd⁻¹ ⊗ q（w<0 取反走最短路径）
        qe = _quat_norm(_quat_mul(_quat_conj(q_cmd), q))
        s = -1.0 if qe[0] < 0 else 1.0
        wdy = -2.0 * P['vtolAttKp'] * s * qe[2]    # qe.y（Python 序：w,x,y,z → y=q[2]）
        wdz = -2.0 * P['vtolAttKp'] * s * qe[3]    # qe.z
        # 调试中间量（cli.py step 打印用；不进协议 STATE_KEYS）
        S['dbg'] = {'qe_x': float(s * qe[1]), 'qe_y': float(s * qe[2]),
                    'qe_z': float(s * qe[3]), 'wdy': float(wdy), 'wdz': float(wdz)}

        if S['useBtrue']:
            _apply_b_true(sim, dt, wdy, wdz)
            return None
        S['dtAct'] = _clamp(P['rateKq'] * (S['omega']['y'] - wdy), -P['dMax'], P['dMax'])
        S['dfAct'] = _clamp(P['rateKr'] * (wdz - S['omega']['z']), -P['dMax'], P['dMax'])
    # 航向通道：纯角速度追踪（∂Mx/∂Δω<0 取 (ω.x − ψ̇_cmd)）
    S['dwAct'] = _clamp(P['rateKp'] * (S['omega']['x'] - S['dw']), -P['dwMax'], P['dwMax'])
    return None


# ============================================================
#  B_true 纯控制分配（对齐 control.mjs B_true 分支）
# ============================================================
def _apply_b_true(sim, dt, wdy, wdz):
    """内环 M_des=diag(I)·btrueK·(ωdes−ω)；分配 Δu = B⁻¹·(M_des−M_cur)，u+=Δu"""
    from core import control_effectiveness
    P = sim.P
    S = sim.S
    kX = P['Ix'] * P['btrueK']
    kY = P['Iy'] * P['btrueK']
    kZ = P['Iz'] * P['btrueK']
    m_des = {
        'x': kX * (S['dw'] - S['omega']['x']),
        'y': kY * (wdy - S['omega']['y']),
        'z': kZ * (wdz - S['omega']['z']),
    }
    # 当前执行器力矩（当前摆角 + 当前转速）。
    # ★ Jp 瞬态项：dWf 用 (S.wf − prev_wf)/dt —— prev_wf 在推进后同步为 S.wf
    #   （sim.py step 末尾），差分恒 0；与 JS 端完全一致（propulsion.mjs 同样
    #   在推进后同步 sim.prevWf），交叉验证保证逐位对齐。保留该结构以对齐 JS。
    cf, sf = np.cos(S['dfAct']), np.sin(S['dfAct'])
    ct, st = np.cos(S['dtAct']), np.sin(S['dtAct'])
    dWf = (S['wf'] - sim.prev_wf) / max(dt, 1e-4)
    dWt = (S['wt'] - sim.prev_wt) / max(dt, 1e-4)
    Tf = P['kT'] * S['wf'] ** 2
    Tt = P['kT'] * S['wt'] ** 2
    Qf = P['kQ'] * S['wf'] ** 2 + P['Jp'] * dWf
    Qt = P['kQ'] * S['wt'] ** 2 + P['Jp'] * dWt
    m_cur = {
        'x': -Qf * cf + Qt * ct,
        'y': -P['b'] * Tt * st - Qf * sf,
        'z': P['a'] * Tf * sf - Qt * st,
    }
    dM = np.array([m_des['x'] - m_cur['x'],
                   m_des['y'] - m_cur['y'],
                   m_des['z'] - m_cur['z']])
    B, _, _ = control_effectiveness(S['thr'] * P['wMax'], S['dfAct'], S['dtAct'], S['dwAct'], P)
    try:
        Binv = np.linalg.inv(B)
    except np.linalg.LinAlgError:
        Binv = None
    if Binv is not None and np.isfinite(Binv).all():
        du = Binv @ dM
        S['dbg']['du'] = [float(du[0]), float(du[1]), float(du[2])]
        S['dwAct'] = _clamp(S['dwAct'] + du[0], -P['dwMax'], P['dwMax'])
        S['dtAct'] = _clamp(S['dtAct'] + du[1], -P['dMax'], P['dMax'])
        S['dfAct'] = _clamp(S['dfAct'] + du[2], -P['dMax'], P['dMax'])
    else:
        # 奇异（低油门）回退：对角映射
        S['dtAct'] = _clamp(P['rateKq'] * (S['omega']['y'] - wdy), -P['dMax'], P['dMax'])
        S['dfAct'] = _clamp(P['rateKr'] * (wdz - S['omega']['z']), -P['dMax'], P['dMax'])
        S['dwAct'] = _clamp(P['rateKp'] * (S['omega']['x'] - S['dw']), -P['dwMax'], P['dwMax'])
