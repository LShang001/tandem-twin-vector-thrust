# -*- coding: utf-8 -*-
"""validation_p0_campaign.py — A-01/A-02/A-03 的大样本仿真 A/B 验证。

@file    validation_p0_campaign.py
@module  analysis/EKF 研究仿真（P0 修复验证）
@brief   在含机动/失锁/恢复的 120 s 轨迹上，对旋转项双计、P0 航向、home_lla
          三项修正做新旧口径对照，输出位置/速度/NIS/航向统计。
@details 输入：20 个随机种子 × 4 个口径（new / legacy_rot / legacy_hdg / legacy_both）；
          处理：生成含加速、滚转/俯仰振荡、转弯的 60 s 真值轨迹（初始 10° 航向误差），
          用 `ekf_core.Ekf15` 跑组合导航；旧口径通过 monkeypatch
          `two_sample_delta_v`（补回 ½Δθ×ΔV）和覆盖 `cfg_p0["hdg"]` 得到；
          另跑 10 s 无 GNSS 开环旋转机动，直接量化 A-01 的累积速度偏差；
          输出：`TandemVec-EKF-Report/sim-data/p0-validation-2026-10-23.json`
          与控制台汇总表。
@theory  划摇补偿 @ref REF-120 §9.6 (9.6.25) p.301；旋转项只配合起始姿态
          @ref REF-120 §9.6.1 (9.6.17) p.298；KF/NIS @ref REF-121 §2.2.1 p.33、
          §7.3.1 p.255–259；P0 航向语义 @ref AGENTS.md（EKF 三风险，2026-08-13）。
@frame  NED + FRD；四元数 (w,x,y,z)，右乘姿态误差。
@units   SI：m、m/s、rad、s；NIS 无量纲。
@note    旧口径仅用于 A/B 对照，不写回生产代码；`legacy_rot` 是 2026-10-23 前的
          双计版本，`legacy_hdg` 是 0.2 rad 修复前的 π。初始航向误差默认 10°
          （30° 会超出本仿真 GNSS 速度可观的收敛域，见调试记录）。
@see     ekf_core.py、verify_p0_fixes.py、DEC-002、MOD-004
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import ekf_core  # noqa: E402
from ekf_core import (  # noqa: E402
    EarthModel,
    Ekf15,
    G_MPS2,
    HOME_LLA,
    cfg,
    cfg_p0,
    quat_mult,
    quat_to_dcm,
    quat_to_euler,
    rotvec_to_quat,
)

OUT = os.path.join(HERE, "..", "sim-data")
# 初始航向误差（deg）：默认 30°，用于放大 A-02 的 P0 口径差异；设 0 可做基线对照。
HEADING_ERR_DEG = float(os.environ.get("P0_VAL_HDG_ERR", "30"))
# 真值侧重力与 EKF 同源（EarthModel 在该纬度的正常重力），避免用 G_MPS2
# 与 EKF 的 em.g（9.7917）差 0.015 m/s² 造成纯惯导漂移，污染 A/B 结论。
G_TRUTH = EarthModel(HOME_LLA[0], HOME_LLA[2]).g


def truth_profile(t: float):
    """返回 (a_ned, omega_body) 真值（分段机动）。"""
    w = np.zeros(3)
    a = np.zeros(3)
    if t < 5.0:
        return a, w
    if t < 15.0:
        # 北向加速到 5 m/s，同时缓慢转弯
        a = np.array([0.5, 0.0, 0.0])
        w = np.array([0.0, 0.0, 0.05])
        return a, w
    if t < 25.0:
        # 温和角速率 + 小比力（稳定包线内）
        w = np.array([0.10 * math.sin(0.5 * t), 0.08 * math.cos(0.6 * t),
                      0.03 * math.sin(0.4 * t)])
        a = np.array([0.05 * math.sin(0.2 * t), 0.05 * math.cos(0.25 * t), 0.0])
        return a, w
    if t < 35.0:
        # 温和转弯（提供航向可观性，但不触发滤波失锁）
        w = np.array([0.02, 0.03, 0.15])
        a = np.array([0.0, 0.3, 0.0])
        return a, w
    # 恢复段：温和机动
    w = np.array([0.04 * math.sin(0.3 * t), 0.03 * math.cos(0.35 * t), 0.02])
    a = np.array([0.05 * math.sin(0.15 * t), 0.05 * math.cos(0.12 * t), 0.0])
    return a, w


def legacy_two_sample(dtheta1, dtheta2, dv1, dv2):
    """旧口径：划摇里再补 ½Δθ×ΔV（旋转项双计，A-01 修复前）。"""
    dtheta = dtheta1 + dtheta2
    dv = dv1 + dv2
    return (dv + (2.0 / 3.0) * (np.cross(dtheta1, dv2) + np.cross(dv1, dtheta2))
            + 0.5 * np.cross(dtheta, dv))


def run_one(variant: str, seed: int, duration: float = 60.0) -> dict:
    """单次 120 s 仿真，返回误差/NIS/航向统计。"""
    global _ORIG_TWO_SAMPLE
    # 每个进程独立设置口径，避免 monkeypatch 串扰
    ekf_core.two_sample_delta_v = (
        legacy_two_sample if variant in ("legacy_rot", "legacy_both")
        else _ORIG_TWO_SAMPLE
    )
    cfg_p0["hdg"] = math.pi if variant in ("legacy_hdg", "legacy_both") else 0.2

    rng = np.random.default_rng(1000 + seed)
    ekf = Ekf15(cfg)
    ekf.initialize(np.array([0.0, 0.0, -G_TRUTH]), np.zeros(3), HOME_LLA)
    # 注入 30° 初始航向误差：用于区分 P0=0.2（固件）与 P0=π（旧）的收敛行为；
    # 真值仍从航向 0 开始，GNSS 速度观测会纠正该误差。
    yaw0 = math.radians(HEADING_ERR_DEG)
    ekf.quat = np.array([math.cos(0.5 * yaw0), 0.0, 0.0, math.sin(0.5 * yaw0)])
    ekf.Cnb = quat_to_dcm(ekf.quat)
    # A-03 修复后 home_lla 已自动设置；显式断言一次
    assert hasattr(ekf, "home_lla")

    dt = cfg["dt"]
    n = int(duration / dt)
    q = np.array([1.0, 0.0, 0.0, 0.0])  # body -> NED
    v = np.zeros(3)
    p = np.zeros(3)
    ba_true = np.array([0.03, -0.02, 0.05])
    bg_true = np.array([0.0005, -0.0004, 0.0006])
    pos_err = []
    vel_err = []
    nis = []
    rejects = 0
    yaw_err = []
    gnss_hz = 10
    last_gnss = -1
    for k in range(n):
        t = k * dt
        a_ned, w = truth_profile(t)
        Cnb = quat_to_dcm(q)
        f_b = Cnb.T @ (a_ned - np.array([0.0, 0.0, G_TRUTH]))
        # 真值积分（半隐式）
        v = v + a_ned * dt
        p = p + v * dt
        q = quat_mult(q, rotvec_to_quat(w * dt))
        q = q / np.linalg.norm(q)
        # 传感器
        dth = w * dt + rng.normal(0.0, cfg["gyro_std"], 3) * math.sqrt(dt)
        dv = f_b * dt + rng.normal(0.0, cfg["accel_std"], 3) * math.sqrt(dt)
        dth += bg_true * dt
        dv += ba_true * dt
        d1, d2 = 0.5 * dth, 0.5 * dth
        v1, v2 = 0.5 * dv, 0.5 * dv
        ekf.time_update(d1, d2, v1, v2, dt)
        # GNSS（10 Hz，全程可用；A-01/A-02 对照不引入失锁这一额外变量）
        gt = int(round(t * gnss_hz))
        if gt != last_gnss:
            last_gnss = gt
            z_pos = p + rng.normal(0.0, [2.0, 2.0, 3.0])
            z_vel = v + rng.normal(0.0, [0.15, 0.15, 0.25])
            res = ekf.measurement_update_gnss(
                z_pos, z_vel, [2.0, 3.0, 0.15, 0.25], age_s=0.0,
                hist=ekf.hist)
            if res.get("fused"):
                nis.append(float(res["nis"]))
            else:
                rejects += 1
        est = ekf.ned_from_lla(ekf.lla, ekf.home_lla)
        e_p = est - p
        e_v = ekf.vel - v
        pos_err.append(e_p)
        vel_err.append(e_v)
        # 航向误差（ZYX yaw）
        yaw_true = quat_to_euler(q)[0]
        yaw_est = quat_to_euler(ekf.quat)[0]
        yaw_err.append((yaw_est - yaw_true + math.pi) % (2 * math.pi) - math.pi)
    pe = np.array(pos_err)
    ve = np.array(vel_err)
    ye = np.array(yaw_err)
    m = n // 4  # 后 3/4 统计（跳过初始化）
    return {
        "variant": variant,
        "seed": seed,
        "pos_rmse_ne": float(np.sqrt(np.mean(pe[m:, :2] ** 2))),
        "pos_rmse_d": float(np.sqrt(np.mean(pe[m:, 2] ** 2))),
        "vel_rmse_ne": float(np.sqrt(np.mean(ve[m:, :2] ** 2))),
        "vel_rmse_d": float(np.sqrt(np.mean(ve[m:, 2] ** 2))),
        "final_pos_err": float(np.linalg.norm(pe[-1])),
        "yaw_rms_deg": float(np.degrees(np.sqrt(np.mean(ye[m:] ** 2)))),
        "nis_mean": float(np.mean(nis)) if nis else float("nan"),
        "nis_count": len(nis),
        "reject_frames": rejects,
    }


_ORIG_TWO_SAMPLE = ekf_core.two_sample_delta_v


def open_loop_rotation(duration: float = 10.0) -> dict:
    """无 GNSS 开环旋转机动：直接量化旋转项双计的累积速度误差（A-01）。"""
    dt = cfg["dt"]
    n = int(duration / dt)
    w = np.array([2.0, 1.0, 0.5])
    a_ned = np.array([5.0, 0.0, 0.0])
    q = np.array([1.0, 0.0, 0.0, 0.0])
    v_true = np.zeros(3)
    v_new = np.zeros(3)
    v_legacy = np.zeros(3)
    g = np.array([0.0, 0.0, G_TRUTH])
    for _ in range(n):
        Cnb = quat_to_dcm(q)
        f_b = Cnb.T @ (a_ned - g)
        dth = w * dt
        dv = f_b * dt
        d1, d2, v1, v2 = 0.5 * dth, 0.5 * dth, 0.5 * dv, 0.5 * dv
        q_mid = quat_mult(q, rotvec_to_quat(0.5 * dth))
        q_mid = q_mid / np.linalg.norm(q_mid)
        Cnb_mid = quat_to_dcm(q_mid)
        v_new += Cnb_mid @ _ORIG_TWO_SAMPLE(d1, d2, v1, v2) + g * dt
        v_legacy += Cnb_mid @ legacy_two_sample(d1, d2, v1, v2) + g * dt
        v_true += a_ned * dt
        q = quat_mult(q, rotvec_to_quat(dth))
        q = q / np.linalg.norm(q)
    return {
        "duration_s": duration,
        "omega_radps": w.tolist(),
        "accel_mps2": a_ned.tolist(),
        "vel_err_new_mps": float(np.linalg.norm(v_new - v_true)),
        "vel_err_legacy_mps": float(np.linalg.norm(v_legacy - v_true)),
        "bias_legacy_minus_new_mps": float(
            np.linalg.norm(v_legacy - v_new)),
    }


def _worker(args):
    """多进程入口（顶层函数，Windows spawn 可 pickle）。"""
    variant, seed = args
    t0 = time.time()
    r = run_one(variant, seed)
    r["seconds"] = round(time.time() - t0, 2)
    return r


def summarize(rows: list[dict]) -> dict:
    """按口径汇总均值/最大值。"""
    out = {}
    for v in sorted({r["variant"] for r in rows}):
        sub = [r for r in rows if r["variant"] == v]
        out[v] = {
            "runs": len(sub),
            "pos_rmse_ne": float(np.mean([r["pos_rmse_ne"] for r in sub])),
            "pos_rmse_d": float(np.mean([r["pos_rmse_d"] for r in sub])),
            "vel_rmse_ne": float(np.mean([r["vel_rmse_ne"] for r in sub])),
            "vel_rmse_d": float(np.mean([r["vel_rmse_d"] for r in sub])),
            "final_pos_err": float(np.mean([r["final_pos_err"] for r in sub])),
            "yaw_rms_deg": float(np.mean([r["yaw_rms_deg"] for r in sub])),
            "nis_mean": float(np.nanmean([r["nis_mean"] for r in sub])),
            "reject_frames": int(np.mean([r["reject_frames"] for r in sub])),
        }
    return out


def main() -> int:
    """命令行入口：多进程跑矩阵并写 JSON。"""
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--workers", type=int, default=min(8, os.cpu_count() or 4))
    ap.add_argument("--out", default=os.path.join(
        OUT, "p0-validation-2026-10-23.json"))
    args = ap.parse_args()
    variants = ("new", "legacy_rot", "legacy_hdg", "legacy_both")
    jobs = [(v, s) for v in variants for s in range(args.seeds)]
    print(f"matrix: {len(jobs)} runs, workers={args.workers}, "
          f"duration=60 s, dt={cfg['dt']}")
    t0 = time.time()
    from concurrent.futures import ProcessPoolExecutor
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        rows = list(ex.map(_worker, jobs))
    summary = summarize(rows)
    open_loop = open_loop_rotation(10.0)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump({"summary": summary, "open_loop_rotation": open_loop,
                   "runs": rows,
                   "elapsed_s": round(time.time() - t0, 1)}, f,
                  ensure_ascii=False, indent=2)
    print(f"elapsed {time.time()-t0:.1f} s -> {args.out}")
    print(f"open-loop rotation 10s: err_new={open_loop['vel_err_new_mps']:.4f} "
          f"err_legacy={open_loop['vel_err_legacy_mps']:.4f} m/s")
    print(f"{'variant':>12} {'pos_ne':>8} {'pos_d':>7} {'vel_ne':>7} "
          f"{'yaw_rms':>8} {'nis':>6} {'rej':>4}")
    for v, d in summary.items():
        print(f"{v:>12} {d['pos_rmse_ne']:8.3f} {d['pos_rmse_d']:7.3f} "
              f"{d['vel_rmse_ne']:7.3f} {d['yaw_rms_deg']:8.2f} "
              f"{d['nis_mean']:6.2f} {d['reject_frames']:4d}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
