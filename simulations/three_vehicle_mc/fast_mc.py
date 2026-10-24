# -*- coding: utf-8 -*-
"""fast_mc.py — 三机型快速参数化 6DOF 降阶 MC 内核。

@file    fast_mc.py
@module  仿真/三机型快速 MC（降阶 6DOF + 噪声/误差/故障）
@brief   对 TandemVec/共轴/液体三型等效模型做向量化姿态+高度闭环 MC。
@details 状态：高度/垂速、roll/pitch/yaw 角与角速率；控制：姿态外环→速率内环
         （角度域 PD）+ 高度 PD；执行器：电机/舵机一阶滞后、推力曲线、限幅；
         扰动：质量/惯量/推力/增益/力臂偏差、传感器噪声/偏置/延迟、风、力矩噪声、
         推力损失/舵机卡死。每个 case 独立参数，向量化并行。
         输出每个 case 的 max 角度、最终高度、失败标记；不替代 SIL/实机。
@theory  T=u·Tmax（共轴/液体）或 T=Tmax·u²（TandemVec）；M=I·α；
         姿态/速率串级 PD；高度 PD
         @ref CTL-001 §4–§5、§8.4；CTL-003 §2
@frame   悬停存档系小角度等效；角度 rad、角速率 rad/s
@units   m、m/s、rad、rad/s、N、kg、s
@note    ★ 本内核是“包线/敏感度”工具，不是飞控固件复刻；增益/执行器被降阶为
         等效 PD/一阶滞后。不得用其结果宣称实机通过或飞行响应等价。
@test    run_mega.py --cases 2000 --nominal
@see     models.py、run_mega.py、VER-002
"""
import math
from dataclasses import replace
import numpy as np

from models import VehicleModel

G = 9.80665


def _curve_thrust(model: VehicleModel, throttle, tmax):
    throttle = np.clip(throttle, 0.0, 1.0)
    if model.thrust_curve == "quadratic_throttle":
        return tmax * throttle * throttle
    thrust = tmax * throttle
    # 液体最小稳定推力：只有阀门打开时才抬到 Tmin
    if model.thrust_min_n > 0.0:
        thrust = np.where((throttle > 1e-4) & (thrust < model.thrust_min_n),
                          model.thrust_min_n, thrust)
    return thrust


def sample_params(model: VehicleModel, n: int, rng: np.random.Generator,
                  nominal: bool = False):
    """采样每个 case 的模型/传感器/故障参数；返回 dict[str, ndarray]."""
    if nominal:
        def nrm(scale):
            return np.clip(1.0 + scale * rng.standard_normal(n), 1.0 - 1.5 * scale,
                           1.0 + 1.5 * scale)
        p = dict(
            mass_mult=nrm(0.05), i_roll=nrm(0.10), i_pitch=nrm(0.10),
            i_yaw=nrm(0.10), thrust_mult=nrm(0.03), tau_mult=nrm(0.10),
            att_kp_mult=nrm(0.08), rate_kp_mult=nrm(0.08),
            lever_mult=nrm(0.08), sensor_angle_deg=np.abs(nrm(0.02)) * 0.2,
            sensor_rate_dps=np.abs(nrm(0.02)) * 0.5,
            bias_angle_deg=0.2 * rng.standard_normal(n),
            bias_rate_dps=0.2 * rng.standard_normal(n),
            wind_up=0.2 * rng.standard_normal(n),
            torque_noise=0.005 * np.abs(rng.standard_normal(n)),
            delay_s=np.abs(nrm(0.02)) * 0.005,
            thrust_loss=np.ones(n), servo_jam=np.zeros(n, dtype=bool),
            init_angle_deg=0.3 * rng.standard_normal(n),
            init_rate_dps=0.3 * rng.standard_normal(n),
        )
    else:
        p = dict(
            mass_mult=np.clip(1.0 + 0.18 * rng.standard_normal(n), 0.70, 1.30),
            i_roll=np.clip(1.0 + 0.35 * rng.standard_normal(n), 0.55, 1.70),
            i_pitch=np.clip(1.0 + 0.35 * rng.standard_normal(n), 0.55, 1.70),
            i_yaw=np.clip(1.0 + 0.35 * rng.standard_normal(n), 0.55, 1.70),
            thrust_mult=np.clip(1.0 + 0.15 * rng.standard_normal(n), 0.75, 1.25),
            tau_mult=np.clip(1.0 + 0.60 * rng.standard_normal(n), 0.40, 2.50),
            att_kp_mult=np.clip(1.0 + 0.25 * rng.standard_normal(n), 0.60, 1.40),
            rate_kp_mult=np.clip(1.0 + 0.25 * rng.standard_normal(n), 0.60, 1.40),
            lever_mult=np.clip(1.0 + 0.35 * rng.standard_normal(n), 0.50, 1.60),
            sensor_angle_deg=np.abs(2.0 * rng.standard_normal(n)),
            sensor_rate_dps=np.abs(5.0 * rng.standard_normal(n)),
            bias_angle_deg=2.0 * rng.standard_normal(n),
            bias_rate_dps=2.0 * rng.standard_normal(n),
            wind_up=1.5 * rng.standard_normal(n),
            torque_noise=0.20 * np.abs(rng.standard_normal(n)),
            delay_s=np.abs(0.05 * rng.standard_normal(n)),
            thrust_loss=np.where(rng.random(n) < 0.02, 0.5, 1.0),
            servo_jam=(rng.random(n) < 0.01),
            init_angle_deg=5.0 * rng.standard_normal(n),
            init_rate_dps=5.0 * rng.standard_normal(n),
        )
    return p


def simulate(model: VehicleModel, p: dict, rng: np.random.Generator,
             duration_s: float = 20.0, dt: float = 0.005,
             variant: str = "fixed", osc_window_s: float = 0.25,
             osc_threshold: float = 6.0, osc_decay: float = 0.90,
             osc_recovery: float = 1.01, osc_floor: float = 0.55,
             ref_roll_deg=0.0, ref_pitch_deg=0.0,
             wind_step_mps2: float = 0.0, wind_step_s: float = 0.0,
             wind_sine_amp: float = 0.0, wind_sine_hz: float = 1.0,
             osc_filter_tau_s: float = 0.05,
             osc_min_rate_dps: float = 2.0,
             osc_ref_gate_deg: float = 0.5, gain_scale: float = 1.0,
             adrc_wc: float = 16.0, adrc_wo: float = 50.0,
             adrc_b0: float = 1.0, lead_s: float = 0.0,
             indi_filter_tau_s: float = 0.02, indi_b0: float = 1.0,
             accel_fb_gain: float = 0.8, accel_fb_tau_s: float = 0.02,
             phase_lead_gain: float = 0.3, phase_lead_tau_s: float = 0.02):
    """向量化闭环；返回 metrics dict（每个 case 一行）。

    ref_roll_deg/ref_pitch_deg 可为标量或长度 steps 的数组（瞬态跟踪）；
    wind_step/sine 叠加在 p["wind_up"] 上。默认全零时行为与原版一致。
    """
    n = len(p["mass_mult"])
    mass = model.mass_kg * p["mass_mult"]
    ix = model.ix * p["i_roll"]
    iy = model.iy * p["i_pitch"]
    iz = model.iz * p["i_yaw"]
    tmax = model.thrust_max_n * p["thrust_mult"]
    tau_m = model.tau_motor_s * p["tau_mult"]
    att_kp = model.att_kp * p["att_kp_mult"] * gain_scale
    rate_kp = model.rate_kp * p["rate_kp_mult"] * gain_scale
    lever = model.lever_m * p["lever_mult"]
    if model.thrust_curve == "quadratic_throttle":
        u_hover_nom = math.sqrt(model.mass_kg * G / model.thrust_max_n)
        auth_power = 2.0
    else:
        u_hover_nom = model.mass_kg * G / model.thrust_max_n
        auth_power = 1.0
    u_eff_filt = np.full(n, u_hover_nom)
    gain_osc = np.ones(n)
    osc_count = np.zeros(n)
    osc_timer = np.zeros(n)
    sign_prev = np.zeros(n)
    err_filt = np.zeros(n)
    az1r = np.zeros(n); az2r = np.zeros(n); aur = np.zeros(n)
    az1p = np.zeros(n); az2p = np.zeros(n); aup = np.zeros(n)
    az1y = np.zeros(n); az2y = np.zeros(n); auy = np.zeros(n)
    indi_pr = np.zeros(n); indi_qr = np.zeros(n); indi_rr = np.zeros(n)
    indi_pd = np.zeros(n); indi_qd = np.zeros(n); indi_rd = np.zeros(n)
    indi_ur = np.zeros(n); indi_up = np.zeros(n); indi_uy = np.zeros(n)
    indi_first = True
    aaf_pd = np.zeros(n); aaf_qd = np.zeros(n); aaf_rd = np.zeros(n)
    aaf_pr = np.zeros(n); aaf_qr = np.zeros(n); aaf_rr = np.zeros(n)
    aaf_first = True
    lead_lp_p = np.zeros(n); lead_lp_q = np.zeros(n); lead_lp_r = np.zeros(n)
    lead_first = True

    # 等效角加速度权限：保持机型表给出的控制权限（不再用惯量比外推，
    # 避免把 TandemVec 小滚转惯量放大成不真实的滚转权限）。
    alpha_max = np.full(n, model.alpha_max_radps2)
    alpha_yaw_max = np.full(n, model.yaw_alpha_max_radps2)
    alpha_roll_max = np.full(n, model.alpha_max_radps2)

    # 状态
    h = np.zeros(n)
    v = np.zeros(n)
    phi = np.radians(p["init_angle_deg"])
    theta = np.radians(p["init_angle_deg"] * 0.7)
    psi = np.radians(p["init_angle_deg"] * 0.3)
    pp = np.radians(p["init_rate_dps"])
    qq = np.radians(p["init_rate_dps"] * 0.7)
    rr = np.radians(p["init_rate_dps"] * 0.3)
    phi_lag, theta_lag, psi_lag = phi.copy(), theta.copy(), psi.copy()
    p_lag, q_lag, r_lag = pp.copy(), qq.copy(), rr.copy()
    thrust_act = np.zeros(n)
    alpha_roll_act = np.zeros(n)
    alpha_pitch_act = np.zeros(n)
    alpha_yaw_act = np.zeros(n)

    max_phi = np.zeros(n)
    max_theta = np.zeros(n)
    max_psi = np.zeros(n)
    max_alt = np.zeros(n)
    sat_steps = np.zeros(n)
    track_sq_roll = np.zeros(n)
    track_sq_pitch = np.zeros(n)
    max_track_err = np.zeros(n)
    gain_min = np.ones(n)
    gain_sum = np.zeros(n)
    gain_active = np.zeros(n)

    def ref_value(ref, k: int) -> float:
        if np.ndim(ref) == 0:
            return float(ref)
        return float(ref[min(k, len(ref) - 1)])

    steps = int(duration_s / dt)
    for k in range(steps):
        t = k * dt
        roll_ref = np.radians(ref_value(ref_roll_deg, k))
        pitch_ref = np.radians(ref_value(ref_pitch_deg, k))
        # 传感器：一阶延迟 + 偏置 + 白噪声
        tau_s = np.maximum(p["delay_s"], dt)
        a_s = 1.0 - np.exp(-dt / tau_s)
        phi_lag += (phi - phi_lag) * a_s
        theta_lag += (theta - theta_lag) * a_s
        psi_lag += (psi - psi_lag) * a_s
        p_lag += (pp - p_lag) * a_s
        q_lag += (qq - q_lag) * a_s
        r_lag += (rr - r_lag) * a_s
        # 使用随机噪声（每步独立）
        noise_a = np.radians(p["sensor_angle_deg"]) * rng.standard_normal(n)
        noise_r = np.radians(p["sensor_rate_dps"]) * rng.standard_normal(n)
        phi_m = phi_lag + np.radians(p["bias_angle_deg"]) + noise_a
        theta_m = theta_lag + np.radians(p["bias_angle_deg"] * 0.8) + noise_a
        psi_m = psi_lag + np.radians(p["bias_angle_deg"] * 0.5) + noise_a
        p_m = p_lag + np.radians(p["bias_rate_dps"]) + noise_r
        q_m = q_lag + np.radians(p["bias_rate_dps"] * 0.8) + noise_r
        r_m = r_lag + np.radians(p["bias_rate_dps"] * 0.5) + noise_r

        # 高度 PD → 推力/油门
        thrust_cmd = mass * G + model.vertical_kp * mass * (10.0 - h) \
            - model.vertical_kd * mass * v
        thrust_cmd = np.clip(thrust_cmd, 0.0, tmax)
        if model.thrust_curve == "quadratic_throttle":
            throttle_cmd = np.sqrt(np.clip(thrust_cmd / tmax, 0.0, 1.0))
        else:
            throttle_cmd = np.clip(thrust_cmd / tmax, 0.0, 1.0)
        a_t = 1.0 - np.exp(-dt / np.maximum(tau_m, dt))
        # thrust_loss 是推力效率因子，必须乘在推力上；乘在油门上会被平方律放大
        # （0.5 油门因子 → 实际 0.25 推力），把 50% 损失误算成 75%。
        thrust_act += (_curve_thrust(model, throttle_cmd, tmax) * p["thrust_loss"]
                       - thrust_act) * a_t

        # 控制权限归一化：用实际推力对应的等效油门，而不是指令油门；
        # 推力损失时 u_eff 下降 → 增益上调补偿，高推力/高质量时增益下调。
        u_eff = np.where(model.thrust_curve == "quadratic_throttle",
                         np.sqrt(np.clip(thrust_act / tmax, 0.0, 1.0)),
                         np.clip(thrust_act / tmax, 0.0, 1.0))
        u_eff_filt += (u_eff - u_eff_filt) * (1.0 - np.exp(-dt / 0.5))
        if variant in ("auth", "adaptive"):
            auth_scale = np.clip((u_hover_nom / (u_eff_filt + 1e-3)) ** auth_power,
                                 0.6, 1.6)
        else:
            auth_scale = 1.0
        gain_eff = auth_scale * gain_osc
        if variant in ("adrc", "indi"):
            gain_eff = 1.0

        # 姿态串级 PD（角度域）；参考可为瞬态指令，传感器噪声进入反馈
        rate_ref_phi = np.clip(att_kp * gain_eff * (roll_ref - phi_m),
                               -np.radians(model.rate_max_dps),
                               np.radians(model.rate_max_dps))
        rate_ref_theta = np.clip(att_kp * gain_eff * (pitch_ref - theta_m),
                                 -np.radians(model.rate_max_dps),
                                 np.radians(model.rate_max_dps))
        rate_ref_psi = np.clip(att_kp * gain_eff * (-psi_m),
                               -np.radians(model.rate_max_dps),
                               np.radians(model.rate_max_dps))
        if variant == "adrc":
            e1 = az1r - p_m
            az1r += (az2r + adrc_b0 * aur - adrc_wo * e1) * dt
            az2r += (-adrc_wo * adrc_wo * e1) * dt
            az2r = np.clip(az2r, -1.0e4, 1.0e4)
            alpha_roll = np.clip((adrc_wc * (rate_ref_phi - az1r) - az2r) / adrc_b0,
                                 -alpha_roll_max, alpha_roll_max)
            aur = alpha_roll
            e1 = az1p - q_m
            az1p += (az2p + adrc_b0 * aup - adrc_wo * e1) * dt
            az2p += (-adrc_wo * adrc_wo * e1) * dt
            az2p = np.clip(az2p, -1.0e4, 1.0e4)
            alpha_pitch = np.clip((adrc_wc * (rate_ref_theta - az1p) - az2p) / adrc_b0,
                                  -alpha_max, alpha_max)
            aup = alpha_pitch
            e1 = az1y - r_m
            az1y += (az2y + adrc_b0 * auy - adrc_wo * e1) * dt
            az2y += (-adrc_wo * adrc_wo * e1) * dt
            az2y = np.clip(az2y, -1.0e4, 1.0e4)
            alpha_yaw = np.clip((adrc_wc * (rate_ref_psi - az1y) - az2y) / adrc_b0,
                                -alpha_yaw_max, alpha_yaw_max)
            auy = alpha_yaw
        elif variant == "indi":
            if indi_first:
                indi_pr[:] = p_m; indi_qr[:] = q_m; indi_rr[:] = r_m
                indi_first = False
            a_i = 1.0 - np.exp(-dt / max(indi_filter_tau_s, dt))
            indi_pd += ((p_m - indi_pr) / dt - indi_pd) * a_i
            indi_qd += ((q_m - indi_qr) / dt - indi_qd) * a_i
            indi_rd += ((r_m - indi_rr) / dt - indi_rd) * a_i
            alpha_roll = np.clip(
                indi_ur + (rate_kp * gain_eff * (rate_ref_phi - p_m) - indi_pd) / indi_b0,
                -alpha_roll_max, alpha_roll_max)
            alpha_pitch = np.clip(
                indi_up + (rate_kp * gain_eff * (rate_ref_theta - q_m) - indi_qd) / indi_b0,
                -alpha_max, alpha_max)
            alpha_yaw = np.clip(
                indi_uy + (rate_kp * gain_eff * (rate_ref_psi - r_m) - indi_rd) / indi_b0,
                -alpha_yaw_max, alpha_yaw_max)
            indi_ur[:] = alpha_roll; indi_up[:] = alpha_pitch; indi_uy[:] = alpha_yaw
            indi_pr[:] = p_m; indi_qr[:] = q_m; indi_rr[:] = r_m
        elif variant == "lead":
            p_eff = p_m + lead_s * alpha_roll_act
            q_eff = q_m + lead_s * alpha_pitch_act
            r_eff = r_m + lead_s * alpha_yaw_act
            alpha_roll = np.clip(rate_kp * gain_eff * (rate_ref_phi - p_eff),
                                 -alpha_roll_max, alpha_roll_max)
            alpha_pitch = np.clip(rate_kp * gain_eff * (rate_ref_theta - q_eff),
                                  -alpha_max, alpha_max)
            alpha_yaw = np.clip(rate_kp * gain_eff * (rate_ref_psi - r_eff),
                                -alpha_yaw_max, alpha_yaw_max)
        elif variant == "accel_fb":
            # 实测角加速度反馈（AAF）：只对陀螺差分做阻尼，不反馈指令，
            # 因此不会像指令 alpha_ref 那样经 Ki 形成代数正反馈。
            if aaf_first:
                aaf_pr[:] = p_m; aaf_qr[:] = q_m; aaf_rr[:] = r_m
                aaf_first = False
            a_af = 1.0 - np.exp(-dt / max(accel_fb_tau_s, dt))
            aaf_pd += ((p_m - aaf_pr) / dt - aaf_pd) * a_af
            aaf_qd += ((q_m - aaf_qr) / dt - aaf_qd) * a_af
            aaf_rd += ((r_m - aaf_rr) / dt - aaf_rd) * a_af
            alpha_roll = np.clip(
                rate_kp * gain_eff * (rate_ref_phi - p_m) - accel_fb_gain * aaf_pd,
                -alpha_roll_max, alpha_roll_max)
            alpha_pitch = np.clip(
                rate_kp * gain_eff * (rate_ref_theta - q_m) - accel_fb_gain * aaf_qd,
                -alpha_max, alpha_max)
            alpha_yaw = np.clip(
                rate_kp * gain_eff * (rate_ref_psi - r_m) - accel_fb_gain * aaf_rd,
                -alpha_yaw_max, alpha_yaw_max)
            aaf_pr[:] = p_m; aaf_qr[:] = q_m; aaf_rr[:] = r_m
        elif variant == "phase_lead":
            # 相位超前（高通抬升）：ω_eff = ω + K·(ω − LPF(ω))。
            # 不放大差分噪声，DC 增益仍为 1；K 提供中频相位超前/阻尼。
            if lead_first:
                lead_lp_p[:] = p_m; lead_lp_q[:] = q_m; lead_lp_r[:] = r_m
                lead_first = False
            a_l = 1.0 - np.exp(-dt / max(phase_lead_tau_s, dt))
            lead_lp_p += (p_m - lead_lp_p) * a_l
            lead_lp_q += (q_m - lead_lp_q) * a_l
            lead_lp_r += (r_m - lead_lp_r) * a_l
            p_eff = p_m + phase_lead_gain * (p_m - lead_lp_p)
            q_eff = q_m + phase_lead_gain * (q_m - lead_lp_q)
            r_eff = r_m + phase_lead_gain * (r_m - lead_lp_r)
            alpha_roll = np.clip(rate_kp * gain_eff * (rate_ref_phi - p_eff),
                                 -alpha_roll_max, alpha_roll_max)
            alpha_pitch = np.clip(rate_kp * gain_eff * (rate_ref_theta - q_eff),
                                  -alpha_max, alpha_max)
            alpha_yaw = np.clip(rate_kp * gain_eff * (rate_ref_psi - r_eff),
                                -alpha_yaw_max, alpha_yaw_max)
        else:
            alpha_roll = np.clip(rate_kp * gain_eff * (rate_ref_phi - p_m),
                                 -alpha_roll_max, alpha_roll_max)
            alpha_pitch = np.clip(rate_kp * gain_eff * (rate_ref_theta - q_m),
                                  -alpha_max, alpha_max)
            alpha_yaw = np.clip(rate_kp * gain_eff * (rate_ref_psi - r_m),
                                -alpha_yaw_max, alpha_yaw_max)
        # 自适应振荡抑制：滤波 + 最小幅值门控 + 驾驶杆活动门控。
        # 原始“逐拍符号换向”会被陀螺噪声持续触发（悬停也 98% 时间降增益），
        # 不是振荡选择器；这里只统计滤波后、幅值足够、且驾驶杆回中的换向。
        if variant in ("osc", "adaptive"):
            err = rate_ref_phi - p_m
            a_f = 1.0 - np.exp(-dt / max(osc_filter_tau_s, dt))
            err_filt += (err - err_filt) * a_f
            gate = np.abs(roll_ref) <= np.radians(osc_ref_gate_deg)
            min_rate = np.radians(osc_min_rate_dps)
            err_filt = np.where(gate, err_filt, 0.0)
            sign_now = np.where(gate & (np.abs(err_filt) >= min_rate),
                                np.sign(err_filt), 0.0)
            osc_count += ((sign_now * sign_prev) < 0.0).astype(float)
            sign_prev = sign_now
            osc_timer = np.where(gate, osc_timer + dt, 0.0)
            osc_count = np.where(gate, osc_count, 0.0)
            sign_prev = np.where(gate, sign_prev, 0.0)
            due = gate & (osc_timer >= osc_window_s)
            if np.any(due):
                hot = due & (osc_count > osc_threshold)
                gain_osc = np.where(hot, np.maximum(osc_floor, gain_osc * osc_decay),
                                    np.minimum(1.0, gain_osc * osc_recovery))
                osc_count = np.where(due, 0.0, osc_count)
                osc_timer = np.where(due, 0.0, osc_timer)
        # 力矩噪声 + 舵机卡死（冻结在 0，模拟无控制）
        alpha_roll += p["torque_noise"] * rng.standard_normal(n)
        alpha_pitch += p["torque_noise"] * rng.standard_normal(n)
        alpha_yaw += p["torque_noise"] * rng.standard_normal(n)
        alpha_roll = np.where(p["servo_jam"], 0.0, alpha_roll)
        alpha_pitch = np.where(p["servo_jam"], 0.0, alpha_pitch)
        sat_steps += ((np.abs(alpha_roll) >= alpha_roll_max * 0.98) |
                      (np.abs(alpha_pitch) >= alpha_max * 0.98)).astype(float)

        a_act = 1.0 - np.exp(-dt / 0.05)
        alpha_roll_act += (alpha_roll - alpha_roll_act) * a_act
        alpha_pitch_act += (alpha_pitch - alpha_pitch_act) * a_act
        alpha_yaw_act += (alpha_yaw - alpha_yaw_act) * a_act

        # 刚体角动力学（小角度等效 + 阻尼）
        damp = 0.2
        pp += (alpha_roll_act - damp * pp) * dt
        qq += (alpha_pitch_act - damp * qq) * dt
        rr += (alpha_yaw_act - damp * rr) * dt
        phi += pp * dt
        theta += qq * dt
        psi += rr * dt

        # 垂向：推力投影 + 重力 + 风（可叠加阶跃/正弦扰动）
        wind = (p["wind_up"] + wind_step_mps2 * (t >= wind_step_s)
                + wind_sine_amp * np.sin(2.0 * np.pi * wind_sine_hz * t))
        v += ((thrust_act * np.cos(phi) * np.cos(theta) - mass * G) / mass
              + wind) * dt
        h += v * dt

        max_phi = np.maximum(max_phi, np.abs(phi))
        max_theta = np.maximum(max_theta, np.abs(theta))
        max_psi = np.maximum(max_psi, np.abs(psi))
        max_alt = np.maximum(max_alt, h)
        err_roll = roll_ref - phi_m
        err_pitch = pitch_ref - theta_m
        track_sq_roll += err_roll * err_roll
        track_sq_pitch += err_pitch * err_pitch
        max_track_err = np.maximum(max_track_err,
                                   np.maximum(np.abs(err_roll), np.abs(err_pitch)))
        gain_min = np.minimum(gain_min, gain_osc)
        gain_sum += gain_osc
        gain_active += (gain_osc < 0.999)

    max_ang = np.maximum(max_phi, np.maximum(max_theta, max_psi))
    finite = np.isfinite(h) & np.isfinite(v) & np.isfinite(max_ang)
    fail = (~finite) | (h < -0.5) | (h > 30.0) | (max_ang > np.radians(60.0))
    ok = finite & (~fail) & (h >= 8.0) & (h <= 12.0) & \
         (max_ang <= np.radians(35.0))
    return dict(
        ok=ok.astype(np.int8), fail=fail.astype(np.int8),
        final_h=h, max_alt=max_alt,
        max_roll_deg=np.degrees(max_phi), max_pitch_deg=np.degrees(max_theta),
        max_yaw_deg=np.degrees(max_psi), max_ang_deg=np.degrees(max_ang),
        sat_frac=sat_steps / steps,
        track_rms_roll_deg=np.degrees(np.sqrt(track_sq_roll / steps)),
        track_rms_pitch_deg=np.degrees(np.sqrt(track_sq_pitch / steps)),
        max_track_err_deg=np.degrees(max_track_err),
        gain_min=gain_min, gain_mean=gain_sum / steps,
        gain_active_frac=gain_active / steps,
    )
