# -*- coding: utf-8 -*-
"""py-web-lab 控制律：逐位对齐 vector-thrust-lab/src/core/control.mjs，
并接入 high-fidelity-analysis / 固件的其他控制算法。

ctrl 模式（S.ctrl）：
  sas   默认——巡航 SAS（4 模式）+ VTOL 悬停（四元数级联 + 定高 + B_true）
  indi  巡航 INDI 增量非线性动态逆（移植 simulate.py:INDIController，论文 ch/11）
  lqr   悬停 LQR 姿态控制（移植 controllers.py:QuatLQRController，论文 ch/14）
  adrc  悬停内环 ADRC（移植固件 TandemVec_ADRC.h：ESO2 + PD，论文 ch/16）

四元数内部用 core.py 顺序 (w,x,y,z)；S.quat 为 JS 顺序 (x,y,z,w)。
交叉验证 tests/cross_verify.mjs 保证 sas 模式与 JS 漂移 < 1e-6。
"""
import numpy as np

from sim import Q_HOVER, hover_throttle


def _clamp(x, lo, hi):
    return float(np.clip(x, lo, hi))


def _quat_mul(a, b):
    from core import quat_multiply
    return quat_multiply(a, b)


def _quat_conj(q):
    from core import quat_conj
    return quat_conj(q)


def _quat_norm(q):
    from core import quat_norm
    return quat_norm(q)


def _rotate(v, q):
    from core import quat_rotate
    return quat_rotate(np.asarray(v, dtype=float), q)


def _ctl_reset(sim, name):
    """控制律切换检测：换模式时重建实例（丢弃旧内部状态）"""
    if sim._ctl_name != name:
        sim._ctl_name = name
        sim._ctl = None


# ============================================================
#  入口：apply(sim, P, dt) → 更新 S.dtAct/dfAct/dwAct（可选 thr）
# ============================================================
def apply(sim, dt):
    """总控制入口（对齐 JS applySas + ctrl 算法分发）"""
    P = sim.P
    S = sim.S
    if S['vtolMode']:
        if S['ctrl'] == 'lqr':
            return _apply_vtol_lqr(sim, dt)
        if S['ctrl'] == 'adrc':
            return _apply_vtol_adrc(sim, dt)
        return _apply_vtol_hover(sim, dt)
    if S['ctrl'] == 'indi':
        return _apply_cruise_indi(sim, dt)
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
#  INDI 巡航增稳（移植 simulate.py:INDIController，论文 ch/11 §INDI）
#   外环：姿态 → ω_ref（滑块：dt→俯仰姿态偏移、df/dw→偏航/滚转角速率指令）
#   内环：K_rate=3 角速率 P → ν；混合角加速度（模型预测 70% + 差分 30%）
#   分配：Δu = B⁻¹·I·(ν − ω̇_filt)，增量累加
# ============================================================
class _INDICruise:
    def __init__(self, P):
        self.P = P
        self.int_th = 0.0
        self.int_phi = 0.0
        self.prev_delta_f = 0.0
        self.prev_delta_t = P['dtTrim']
        self.prev_dw = 0.0
        self.prev_omega = np.zeros(3)
        self.omega_dot_filt = np.zeros(3)
        self.first = True
        self.K_rate = 3.0
        self.alpha_fusion = 0.3          # 差分占比（70% 模型 + 30% 差分）
        self.tau_filt = 0.01             # ω̇ 低通时间常数

    def update(self, sim, dt):
        P = self.P
        S = sim.S
        F = sim.F
        phi = F['euler']['x']
        theta = F['euler']['y']
        omega = np.array([S['omega']['x'], S['omega']['y'], S['omega']['z']])
        # 外环：姿态 → 目标角速率（滑块 dt = 俯仰姿态指令偏移）
        dtheta = theta - (-P['aTrim'] + S['dt'])
        dphi = phi
        self.int_th = _clamp(self.int_th + dtheta * dt, -P['intThMax'], P['intThMax'])
        self.int_phi = _clamp(self.int_phi + dphi * dt, -P['intPhiMax'], P['intPhiMax'])
        p_ref = 3.0 * (P['sasPhi'] * dphi + P['sasIPhi'] * self.int_phi) + S['dw']
        q_ref = 3.0 * (P['sasTh'] * dtheta + P['sasI'] * self.int_th)
        r_ref = S['df']
        # 内环：角速率误差 → 虚拟角加速度
        nu = self.K_rate * np.array([p_ref - omega[0], q_ref - omega[1], r_ref - omega[2]])
        # 混合角加速度（模型预测 + 差分融合）
        if self.first:
            self.omega_dot_filt = np.zeros(3)
            self.first = False
        else:
            Ix, Iy, Iz = P['Ix'], P['Iy'], P['Iz']
            gx = (Iz - Iy) * omega[1] * omega[2]
            gy = (Ix - Iz) * omega[2] * omega[0]
            gz = (Iy - Ix) * omega[0] * omega[1]
            M_prop = np.array([sim.dyn['Mx'], sim.dyn['My'], sim.dyn['Mz']])
            aero = np.array([sim.aero['Mx'], sim.aero['My'], sim.aero['Mz']])
            model_dot = np.array([
                (M_prop[0] + aero[0] - gx) / Ix,
                (M_prop[1] + aero[1] - gy) / Iy,
                (M_prop[2] + aero[2] - gz) / Iz,
            ])
            raw_dot = (omega - self.prev_omega) / max(dt, 1e-4)
            fused = (1 - self.alpha_fusion) * model_dot + self.alpha_fusion * raw_dot
            alpha_lp = min(dt / self.tau_filt, 1.0)
            self.omega_dot_filt += alpha_lp * (fused - self.omega_dot_filt)
        self.prev_omega = omega.copy()
        # 在线 Jacobian + 增量分配
        from core import control_effectiveness
        B, _, _ = control_effectiveness(S['thr'] * P['wMax'], self.prev_delta_f,
                                        self.prev_delta_t, self.prev_dw, P)
        err = np.array([P['Ix'], P['Iy'], P['Iz']]) * (nu - self.omega_dot_filt)
        try:
            delta_u = np.linalg.solve(B, err)
        except np.linalg.LinAlgError:
            delta_u = np.zeros(3)
        self.prev_dw += _clamp(delta_u[0], -P['dMax'], P['dMax'])
        self.prev_delta_t += _clamp(delta_u[1], -P['dMax'], P['dMax'])
        self.prev_delta_f += _clamp(delta_u[2], -P['dMax'], P['dMax'])
        S['dwAct'] = _clamp(self.prev_dw, -P['dwMax'], P['dwMax'])
        S['dtAct'] = _clamp(self.prev_delta_t, -P['dMax'], P['dMax'])
        S['dfAct'] = _clamp(self.prev_delta_f, -P['dMax'], P['dMax'])
        S['dbg'] = {'indi_nu': list(nu), 'wdot_filt': list(self.omega_dot_filt)}


def _apply_cruise_indi(sim, dt):
    _ctl_reset(sim, 'indi')
    if sim._ctl is None:
        sim._ctl = _INDICruise(sim.P)
    sim._ctl.update(sim, dt)


# ============================================================
#  悬停 LQR（移植 controllers.py:QuatLQRController，论文 ch/14）
#   状态 x=[ε(3), ω(3)]，输入 u=[Δω, δt, δf]；
#   K 由 CARE 解出（Bryson 权重），悬停点 B 满秩
# ============================================================
class _LQRHover:
    def __init__(self, P):
        from scipy.linalg import solve_continuous_are
        from core import control_effectiveness
        self.P = P
        omega0 = hover_throttle(P) * P['wMax']
        B, _, _ = control_effectiveness(omega0, 0.0, 0.0, 0.0, P)
        Iinv = np.diag([1.0 / P['Ix'], 1.0 / P['Iy'], 1.0 / P['Iz']])
        A = np.zeros((6, 6))
        A[:3, 3:] = 0.5 * np.eye(3)
        Bm = np.zeros((6, 3))
        Bm[3:, :] = Iinv @ B
        x_max = np.array([0.5, 0.5, 0.5, 1.0, 1.0, 1.0])
        u_max = np.array([P['dwMax'], P['dMax'], P['dMax']])
        Q = np.diag(1.0 / x_max ** 2)
        R = np.diag(1.0 / u_max ** 2)
        S = solve_continuous_are(A, Bm, Q, R)
        self.K = np.linalg.solve(R, Bm.T @ S)

    def update(self, sim, dt):
        P = self.P
        S = sim.S
        q = sim._quat
        # qCmd 跟随航向（与级联分支同构）
        h = S['dt'] * 0.5
        f = S['df'] * 0.5
        q_local = _quat_mul(
            np.array([np.cos(h), 0.0, np.sin(h), 0.0]),
            np.array([np.cos(f), 0.0, 0.0, np.sin(f)]),
        )
        yb = _rotate([0.0, 1.0, 0.0], q)
        psi_est = np.pi / 2 - np.arctan2(yb[1], yb[0])
        q_des = _quat_norm(_quat_mul(
            _quat_mul(Q_HOVER, np.array([np.cos(psi_est / 2), np.sin(psi_est / 2), 0.0, 0.0])),
            q_local,
        ))
        # 误差态 x = [ε, ω − ω_ref]（ε = q_des⁻¹⊗q 的矢量部，表达在 q_des 系）
        q_err = _quat_norm(_quat_mul(_quat_conj(q_des), q))
        eps = np.array([q_err[1], q_err[2], q_err[3]])
        omega = np.array([S['omega']['x'], S['omega']['y'], S['omega']['z']])
        omega_ref = np.array([S['dw'], 0.0, 0.0])
        x = np.concatenate([eps, omega - omega_ref])
        u = -self.K @ x
        S['dwAct'] = _clamp(u[0], -P['dwMax'], P['dwMax'])
        S['dtAct'] = _clamp(u[1], -P['dMax'], P['dMax'])
        S['dfAct'] = _clamp(u[2], -P['dMax'], P['dMax'])
        S['dbg'] = {'lqr_eps': list(eps), 'lqr_u': list(u)}


def _apply_vtol_lqr(sim, dt):
    _ctl_reset(sim, 'lqr')
    if sim._ctl is None:
        sim._ctl = _LQRHover(sim.P)
    _apply_alt_hold(sim, dt)          # 定高环独立于控制律
    sim._ctl.update(sim, dt)


# ============================================================
#  悬停内环 ADRC（移植固件 TandemVec_ADRC.h：ESO2 + PD，论文 ch/16）
#   被控对象：ω̇ = b0·u + f；ESO 估计 z1→ω, z2→f
#   控制律：u = (Kp·(ω_ref − z1) − z2) / b0（ω̇_ref=0 简化）
#   y 通道（俯仰，∂My/∂δt<0 取负号）与 z 通道（侧倾，∂Mz/∂δf>0）各一个 ADRC；
#   x 通道（航向）保留 rateKp 角速度 P
# ============================================================
class _ESO2:
    """二阶扩张状态观测器（rad/s 制；固件为 deg/s，比例无关）"""

    def __init__(self, wo):
        self.z1 = 0.0   # ω 估计
        self.z2 = 0.0   # 总扰动 f 估计
        self.wo = wo

    def update(self, y, u, b0, dt):
        e = self.z1 - y
        self.z1 += (self.z2 - 2 * self.wo * e + b0 * u) * dt
        self.z2 += (-self.wo * self.wo * e) * dt

    def reset(self):
        self.z1 = 0.0
        self.z2 = 0.0


class _ADRCHover:
    """双通道 ADRC（俯仰 y / 侧倾 z）+ 航向 P；外环 qe → ω_ref 与级联分支同构"""

    def __init__(self, P):
        self.P = P
        self.wc = 6.0      # 控制器带宽 ωc（固件默认）
        self.wo = 8.0      # ESO 带宽 ωo（固件默认，≈2×执行器带宽 1/τm≈3.6）
        self.eso_y = _ESO2(self.wo)
        self.eso_z = _ESO2(self.wo)
        self.u_prev_y = 0.0
        self.u_prev_z = 0.0
        # 名义增益 b0 = ∂ω̇/∂u（真实符号：∂ω̇y/∂δt = −b·T0/Iy < 0、∂ω̇z/∂δf = +a·T0/Iz > 0）
        # ⚠ b0 必须带真实符号进 ESO 模型项（z1̇ = z2 − 2ωo·e + b0·u）——
        #   若只在前置负号补偿 u 方向，ESO 模型项与实际对象反号，
        #   扰动估计 z2 错误 → 等效带宽降级（ωc 6→2 rad/s），2026-08-06 review 修复
        T0 = P['kT'] * (hover_throttle(P) * P['wMax']) ** 2
        self.b0_y = -(P['b'] * T0) / P['Iy']     # ∂ω̇y/∂δt = −b·T0/Iy
        self.b0_z = (P['a'] * T0) / P['Iz']      # ∂ω̇z/∂δf = +a·T0/Iz

    def update(self, sim, dt):
        P = self.P
        S = sim.S
        q = sim._quat
        # 目标姿态（与级联分支同构：滑块 dt/df + 航向跟随）
        h = S['dt'] * 0.5
        f = S['df'] * 0.5
        q_local = _quat_mul(
            np.array([np.cos(h), 0.0, np.sin(h), 0.0]),
            np.array([np.cos(f), 0.0, 0.0, np.sin(f)]),
        )
        yb = _rotate([0.0, 1.0, 0.0], q)
        psi_est = np.pi / 2 - np.arctan2(yb[1], yb[0])
        q_cmd = _quat_norm(_quat_mul(
            _quat_mul(Q_HOVER, np.array([np.cos(psi_est / 2), np.sin(psi_est / 2), 0.0, 0.0])),
            q_local,
        ))
        qe = _quat_norm(_quat_mul(_quat_conj(q_cmd), q))
        s = -1.0 if qe[0] < 0 else 1.0
        wdy = -2.0 * P['vtolAttKp'] * s * qe[2]   # 外环（同级联）
        wdz = -2.0 * P['vtolAttKp'] * s * qe[3]
        # 内环 ADRC：先 ESO（用上一拍 u），再算新 u
        om_y = S['omega']['y']
        om_z = S['omega']['z']
        self.eso_y.update(om_y, self.u_prev_y, self.b0_y, dt)
        self.eso_z.update(om_z, self.u_prev_z, self.b0_z, dt)
        # 控制律 u = (Kp·(ω_ref − z1) − z2)/b0（b0 含真实符号，u 方向自动正确）
        u_y = (self.wc * (wdy - self.eso_y.z1) - self.eso_y.z2) / self.b0_y
        u_z = (self.wc * (wdz - self.eso_z.z1) - self.eso_z.z2) / self.b0_z
        self.u_prev_y = _clamp(u_y, -P['dMax'], P['dMax'])
        self.u_prev_z = _clamp(u_z, -P['dMax'], P['dMax'])
        S['dtAct'] = self.u_prev_y
        S['dfAct'] = self.u_prev_z
        S['dwAct'] = _clamp(P['rateKp'] * (S['omega']['x'] - S['dw']), -P['dwMax'], P['dwMax'])
        S['dbg'] = {'adrc_z1y': float(self.eso_y.z1), 'adrc_z2y': float(self.eso_y.z2),
                    'adrc_z1z': float(self.eso_z.z1), 'adrc_z2z': float(self.eso_z.z2)}


def _apply_vtol_adrc(sim, dt):
    _ctl_reset(sim, 'adrc')
    if sim._ctl is None:
        sim._ctl = _ADRCHover(sim.P)
    _apply_alt_hold(sim, dt)          # 定高环独立于控制律
    sim._ctl.update(sim, dt)


# ============================================================
#  悬停级联（现有默认，对齐 control.mjs:applyVtolHover）
# ============================================================
def _apply_alt_hold(sim, dt):
    """高度保持（独立于姿态控制律；thr 直接修正）"""
    P = sim.P
    S = sim.S
    F = sim.F
    q = sim._quat
    if S['altHold']:
        hgt = -F['pos']['z']
        vZ = -F['vWorld']['z']
        h_err = S['altRef'] - hgt
        S['intAlt'] = _clamp(S['intAlt'] + h_err * dt, -1.5, 1.5)
        v_zref = _clamp(P['altKpH'] * h_err + P['altKpI'] * S['intAlt'],
                        -P['altVZMax'], P['altVZMax'])
        cos_g = -2.0 * (q[1] * q[3] - q[2] * q[0])
        cos_g = max(cos_g, 0.5)
        thr_base = hover_throttle(P) / np.sqrt(cos_g)
        S['thr'] = _clamp(thr_base + P['altKpV'] * (v_zref - vZ), 0.0, 1.0)


def _apply_vtol_hover(sim, dt):
    P = sim.P
    S = sim.S
    F = sim.F
    q = sim._quat

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
    _apply_alt_hold(sim, dt)

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
