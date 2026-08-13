# -*- coding: utf-8 -*-
"""
固件集成层关键发现的可复现数值验证（审查报告 §〇 中 V9-V12）
V9  H1 降级路径 AHRS 航向校正开环积分（HIGH）
V11 M4 静止时双矢量航向缺静止门控 → 噪声污染
V12 M1 IMU 缓冲溢出帧传播 dt 低估
V13 MED-11 AHRS 姿态量测门限过宽 vs 双矢量 20°/s
"""
import json
import math
import os

import numpy as np

OUT = os.path.join(os.path.dirname(__file__), "..", "sim-data")


def wrap_pi(a):
    a = math.fmod(a + math.pi, 2 * math.pi)
    if a < 0:
        a += 2 * math.pi
    return a - math.pi


def v9_ahrs_yaw_correction():
    """H1: 降级路径 yaw_err = wrap(EKF-(backup-corr)) vs 全质量 wrap(EKF-backup)
    由于 backup=raw+corr（sensor_imu.cpp:406），降级路径实际算 EKF-raw（开环）"""
    def simulate(path, T=100.0, dt=0.005):
        raw = 0.0
        ekf_yaw = 30.0 * math.pi / 180
        corr, backup = 0.0, 0.0
        max_step = 0.0025
        traj = []
        for k in range(int(T / dt)):
            if path == "full":
                yerr = wrap_pi(ekf_yaw - backup)
            else:
                yerr = wrap_pi(ekf_yaw - (backup - corr))
            yerr = max(-max_step, min(max_step, yerr))
            corr = wrap_pi(corr + yerr)
            backup = wrap_pi(raw + corr)
            if k % 4000 == 0:
                traj.append((k * dt, corr * 180 / math.pi))
        return traj

    full = simulate("full")
    downg = simulate("downgrade")
    # 评估：全质量应收敛到 30°，降级应发散（|corr| 持续大值）
    full_converged = abs(full[-1][1] - 30.0) < 1.0
    downg_peak = max(abs(c) for _, c in downg)
    downg_diverged = downg_peak > 100.0
    print("[V9/H1] 全质量路径末态 corr=%.1f° (应≈30°) -> %s" %
          (full[-1][1], "确认收敛" if full_converged else "异常"))
    print("[V9/H1] 降级路径 corr 峰值=%.1f° (应发散>100°) -> %s" %
          (downg_peak, "确认发散" if downg_diverged else "异常"))
    return dict(full_converged=full_converged, downg_diverged=downg_diverged,
                full_final_deg=full[-1][1], downg_peak_deg=downg_peak)


def v11_dual_vector_yaw_static():
    """M4: 静止时双矢量航向（缺 is_static_confirmed 门控）"""
    rng = np.random.default_rng(5)
    passed = []
    for _ in range(200000):
        v_n = rng.normal(0, 0.8)
        v_e = rng.normal(0, 0.8)
        v_fx = rng.normal(0, 0.6)
        v_fy = rng.normal(0, 0.6)
        sg, sf = math.hypot(v_n, v_e), math.hypot(v_fx, v_fy)
        if sg < 1.0 or sf < 1.0:
            continue
        if abs(sf / sg - 1.0) > 0.35:
            continue
        passed.append(math.atan2(v_e, v_n) - math.atan2(v_fy, v_fx))
    passed = np.array(passed)
    rate = len(passed) / 200000
    std_deg = float(np.degrees(passed.std())) if len(passed) else float("nan")
    print("[V11/M4] 静止噪声下穿过门控比例=%.2f%%, 航向σ=%.0f°" % (100 * rate, std_deg))
    return dict(pass_rate=float(rate), yaw_std_deg=std_deg, n_passed=len(passed))


def v12_overflow_dt():
    """M1: IMU 缓冲溢出帧传播 dt 低估"""
    res = []
    for total in [64, 80, 100, 200]:
        buffered = min(total, 64)
        ratio = buffered / total
        res.append((total, ratio))
        print("[V12/M1] 样本数=%d: 传播dt/真实dt = %.0f%% (低估 %.0f%%)"
              % (total, 100 * ratio, 100 * (1 - ratio)))
    return dict([("n%d_ratio" % n, r) for n, r in res])


def v13_ahrs_gate():
    """MED-11: AHRS angular_rate_quiet=1.2 rad/s vs 双矢量 20°/s"""
    rng = np.random.default_rng(4)
    w_z = 2.5 * np.sin(2 * np.pi * 1.5 * np.arange(0, 10, 0.005)) + rng.normal(0, 0.3, 2000)
    ahrs = np.mean(np.abs(w_z) < 1.2)
    dv = np.mean(np.abs(w_z * 57.2958) < 20.0)
    print("[V13/MED-11] 剧烈机动: AHRS门限(1.2rad/s)通过=%.0f%%, 双矢量(20°/s)通过=%.0f%%" %
          (100 * ahrs, 100 * dv))
    return dict(ahrs_pass=float(ahrs), dualvec_pass=float(dv))


def main():
    results = {
        "V9_H1_ahrs_yaw": v9_ahrs_yaw_correction(),
        "V11_M4_dualvec_static": v11_dual_vector_yaw_static(),
        "V12_M1_overflow_dt": v12_overflow_dt(),
        "V13_MED11_ahrs_gate": v13_ahrs_gate(),
    }
    with open(os.path.join(OUT, "verify_integration.json"), "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, indent=2, default=float)
    print("\n[完成] 集成层验证写入 sim-data/verify_integration.json")


if __name__ == "__main__":
    main()
