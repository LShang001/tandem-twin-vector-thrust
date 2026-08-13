# -*- coding: utf-8 -*-
"""
修复前全面核查仿真（v2）：对每个已确认问题的端到端影响量化
同时实现"固件当前逻辑(bug)"与"修复后逻辑"，输出 before/after 对比

E1  H1   降级 GNSS 300s，backup_ahrs_yaw 缠绕对回退航向输出的污染
E2  H2   45°/85° 俯仰过渡飞行，标量航向融合：H=e8(bug) vs H=C_b^n 第三行(修复)
         量化: 有效 yaw 修正系数 / 额外 roll 变化
E3  MED-1,2 静止辅助档位 + 辅助动作分布（理想静止 与 中低置信度 两场景）
E4  MED-3 epoch 时间映射负值/陈旧判定
E5  MED-6 静止 30s + 多径，双矢量航向对 EKF yaw 的累计拉偏
E6  MED-0b 500ms 卡顿帧，单子样路径速度积分误差（dt 低估）
"""
import json
import math
import os

import numpy as np

OUT = os.path.join(os.path.dirname(__file__), "..", "sim-data")
os.makedirs(OUT, exist_ok=True)


def wrap_pi(a):
    a = math.fmod(a + math.pi, 2 * math.pi)
    if a < 0:
        a += 2 * math.pi
    return a - math.pi


# =====================================================================
# E1: H1 降级路径 AHRS 航向校正开环
# =====================================================================
def e1_ahrs_yaw_correction(T=300.0):
    """模拟降级 GNSS 持续期间 backup_ahrs_yaw 输出漂移"""
    def simulate(fixed, T, dt=0.005):
        raw = 0.0                       # Madgwick 原始 yaw（缓慢漂移 1°/s）
        ekf_yaw = 30.0 * math.pi / 180  # EKF 恒定 30°（真实航向）
        corr, backup = 0.0, 0.0
        max_step = 0.0025
        for k in range(int(T / dt)):
            raw = wrap_pi(raw + 1.0 * math.pi / 180 * dt)
            if fixed:
                yerr = wrap_pi(ekf_yaw - backup)           # 残差闭环
            else:
                yerr = wrap_pi(ekf_yaw - (backup - corr))  # 双重抵消 → 开环
            yerr = max(-max_step, min(max_step, yerr))
            corr = wrap_pi(corr + yerr)
            backup = wrap_pi(raw + corr)
        return backup, corr

    bug_bk, bug_corr = simulate(False, T)
    fix_bk, fix_corr = simulate(True, T)
    print("[E1/H1] 降级 GNSS 300s 后 backup_ahrs_yaw 输出:")
    print("  当前(bug)  : backup=%.1f° corr=%.1f° (应≈EKF 30°)" %
          (bug_bk * 180 / math.pi, bug_corr * 180 / math.pi))
    print("  修复后      : backup=%.1f° corr=%.1f°" %
          (fix_bk * 180 / math.pi, fix_corr * 180 / math.pi))
    return dict(bug_backup_deg=bug_bk * 180 / math.pi,
                fix_backup_deg=fix_bk * 180 / math.pi,
                bug_corr_deg=bug_corr * 180 / math.pi,
                fix_corr_deg=fix_corr * 180 / math.pi)


# =====================================================================
# E2: H2 标量航向量测倾斜补偿（大俯仰过渡飞行）
# =====================================================================
def e2_scalar_yaw_tilt():
    """量化 H=e8 vs H=投影 在俯仰 θ 下的 yaw 修正效率与额外 roll 变化"""
    def compute(theta_deg):
        th = math.radians(theta_deg)
        # C_b^n 第三行（真航向轴在机体系）—— 俯仰 θ 时 C_b^n=Ry(θ)
        c_axis = np.array([-math.sin(th), 0.0, math.cos(th)])  # NED z 在机体系
        # H=e8 (bug): 只取 z 分量
        h_bug = np.array([0.0, 0.0, 1.0])
        # H=投影 (fix): 真航向轴
        h_fix = c_axis
        # 有效 yaw 修正系数 = C_b^n(2,:)·H^T（注入方向与真航向轴的投影）
        eff_bug = abs(np.dot(c_axis, h_bug))
        eff_fix = abs(np.dot(c_axis, h_fix))
        # 额外 roll 变化：注入方向中垂直于真航向轴的分量
        roll_bug = math.sqrt(max(0.0, 1.0 - eff_bug ** 2))
        roll_fix = math.sqrt(max(0.0, 1.0 - eff_fix ** 2))
        return eff_bug, eff_fix, roll_bug, roll_fix

    print("[E2/H2] 标量航向量测倾斜补偿（单次修正单位注入）:")
    for th in [0, 30, 45, 60, 85]:
        eb, ef, rb, rf = compute(th)
        print("  俯仰=%.0f°: 有效yaw修正 bug=%.0f%% fix=%.0f%% | 额外roll bug=%.0f%% fix=%.0f%%"
              % (th, 100 * eb, 100 * ef, 100 * rb, 100 * rf))
    return dict([("theta%d" % th, dict(eff_bug=compute(th)[0], eff_fix=compute(th)[1],
                                       roll_bug=compute(th)[2], roll_fix=compute(th)[3]))
                 for th in [0, 30, 45, 60, 85]])


# =====================================================================
# E3: MED-1/2 静止辅助档位与辅助动作分布
# =====================================================================
def e3_static_tier():
    """复现固件静止辅助调度：档位分配 + SelectStaticAidAction"""
    ZUPT_DIV, GRAV_DIV, SG_DIV = 4, 8, 8
    ENTER_MIN = 20

    def tier(frames, conf, fixed_tier1):
        if fixed_tier1:
            if conf < 0.35 or frames < 36:
                return 1
        else:
            if conf < 0.35 or frames < 16:
                return 1
        if conf < 0.80 or frames < 100:
            return 2
        return 3

    def action(zupt_due, grav_due, sg_due, tier_1, fixed_zupt):
        if tier_1:
            if fixed_zupt:
                if grav_due:
                    return "Gravity"
                if zupt_due:
                    return "Zupt"
                if sg_due:
                    return "StaticGyro"
                return "None"
            else:
                # bug: prefer_gravity_first 只查 gravity
                return "Gravity" if grav_due else "None"
        if zupt_due:
            return "Zupt"
        if grav_due:
            return "Gravity"
        if sg_due:
            return "StaticGyro"
        return "None"

    def simulate(N_frames, raw_conf, fixed_tier1, fixed_zupt):
        cnt = {"Zupt": 0, "Gravity": 0, "StaticGyro": 0, "None": 0}
        tier_counts = {1: 0, 2: 0, 3: 0}
        for counter in range(1, N_frames + 1):
            frames = ENTER_MIN + counter - 1  # confirmed_static_frames
            dwell = min(1.0, frames / 60.0)
            conf = min(raw_conf, dwell)
            t = tier(frames, conf, fixed_tier1)
            tier_counts[t] += 1
            zupt_due = (counter % ZUPT_DIV) == 0
            grav_due = (counter % GRAV_DIV) == 1
            sg_due = (counter % SG_DIV) == 3
            cnt[action(zupt_due, grav_due, sg_due, t == 1, fixed_zupt)] += 1
        return cnt, tier_counts

    # 场景 A: 理想静止 raw=1.0（确认后 40 帧 = 200ms）
    bugA, tierA = simulate(40, 1.0, False, False)
    fixA, fixTierA = simulate(40, 1.0, True, True)
    # 场景 B: 中低置信度 raw=0.3（档 1 持续 → 饿死 ZUPT）
    bugB, _ = simulate(40, 0.3, False, False)
    fixB, _ = simulate(40, 0.3, True, True)

    print("[E3/MED-1,2] 理想静止(raw=1.0) 确认后 200ms 档位分布:",
          "bug T1=%d帧 T2=%d帧" % (tierA[1], tierA[2]),
          "| fix T1=%d帧 T2=%d帧" % (fixTierA[1], fixTierA[2]))
    print("  动作分布 bug:", bugA)
    print("  动作分布 fix:", fixA)
    print("  中低置信度(raw=0.3): bug ZUPT=%d fix ZUPT=%d (饿死对比)" % (bugB["Zupt"], fixB["Zupt"]))
    print("  → MED-1: bug 档1 仅 %d 帧 (确认帧), 修复后 %d 帧(80ms 弱约束)" % (tierA[1], fixTierA[1]))
    print("  → MED-2: 低置信度下 bug ZUPT 饿死 %d 次, 修复后正常" % (40 - bugB["Zupt"] - bugB["Gravity"] - bugB["StaticGyro"] - (40 - fixB["Zupt"])))
    return dict(bugA=bugA, fixA=fixA, bugB=bugB, fixB=fixB,
                tierA_bug=dict(tierA), tierA_fix=dict(fixTierA))


# =====================================================================
# E4: MED-3 epoch 时间映射
# =====================================================================
def e4_epoch_timing():
    """映射时间负值/陈旧时的年龄判定（模拟 InsGnssMapEpochToLocalTime + AgeSeconds）"""
    def map_age(mapped_us, now_us, fixed):
        if mapped_us < 0:
            return "invalid" if fixed else None  # fix: 判无效; bug: 继续
        age_us = now_us - mapped_us
        if age_us < 0:
            return "invalid" if fixed else 0.0
        if age_us > 0x7FFFFFFF:      # int32 上限（35.8min）
            return "invalid" if fixed else 0.0
        return float(age_us) * 1e-6

    now = 1_000_000
    r1_bug = map_age(-5000, now, False)
    r1_fix = map_age(-5000, now, True)
    r2_bug = map_age(now - 30 * 60 * 1_000_000, now, False)
    r2_fix = map_age(now - 30 * 60 * 1_000_000, now, True)
    print("[E4/MED-3] epoch 时间映射异常判定:")
    print("  未来映射(-5ms) : 当前=%(r)s 修复=invalid" % {"r": r1_bug})
    print("  陈旧30min      : 当前=%(r)s 修复=invalid" % {"r": r2_bug})
    return dict(future_bug=r1_bug, future_fix=r1_fix,
                stale_bug=r2_bug, stale_fix=r2_fix)


# =====================================================================
# E5: MED-6 静止时双矢量航向拉偏
# =====================================================================
def e5_dual_vector_static(T=30.0):
    """静止 30s + GNSS 多径噪声，双矢量航向对 EKF yaw 的累计拉偏"""
    def simulate(gate_static, T, dt=0.005):
        rng = np.random.default_rng(7)
        yaw_est = 0.0
        corr_total = 0.0
        n_fused = 0
        for k in range(int(T / dt)):
            if k % 200 == 0 and k > 200:
                v_n = rng.normal(0, 0.8)
                v_e = rng.normal(0, 0.8)
                v_fx = rng.normal(0, 0.6)
                v_fy = rng.normal(0, 0.6)
                sg, sf = math.hypot(v_n, v_e), math.hypot(v_fx, v_fy)
                if gate_static:
                    passed = False  # 修复：静止直接跳过
                else:
                    passed = sg >= 1.0 and sf >= 1.0 and abs(sf / sg - 1.0) <= 0.35
                if passed:
                    est_yaw = math.atan2(v_e, v_n) - math.atan2(v_fy, v_fx)
                    dy = wrap_pi(est_yaw - yaw_est)
                    corr_total += dy
                    yaw_est = wrap_pi(yaw_est + 0.05 * dy)  # 简化卡尔曼增益
                    n_fused += 1
        return yaw_est * 180 / math.pi, corr_total * 180 / math.pi, n_fused

    bug_yaw, bug_corr, bug_n = simulate(False, T)
    fix_yaw, fix_corr, fix_n = simulate(True, T)
    print("[E5/MED-6] 静止 30s 双矢量航向累计影响:")
    print("  当前(bug): 融合 %d 帧, 累计注入 %.1f°, 末态 yaw=%.1f°" % (bug_n, bug_corr, bug_yaw))
    print("  修复后   : 融合 %d 帧 (静止跳过), yaw 保持 0°" % fix_n)
    return dict(bug_yaw_deg=bug_yaw, bug_corr_deg=bug_corr, bug_n=bug_n, fix_n=fix_n)


# =====================================================================
# E6: MED-0b 溢出帧 dt 低估
# =====================================================================
def e6_overflow_dt():
    """500ms 卡顿帧：单子样路径速度积分误差（bug dt=64样本 vs 修复 dt=真实）"""
    a = 2.0
    n_samples = 100
    buffered = min(n_samples, 64)
    real_dt = n_samples * 0.005
    bug_dt = buffered * 0.005
    fix_dt = real_dt
    bug_dv, fix_dv, true_dv = a * bug_dt, a * fix_dt, a * real_dt
    print("[E6/MED-0b] 500ms 卡顿帧(100样本,缓冲64溢出):")
    print("  真实速度增量=%.3f m/s" % true_dv)
    print("  当前(bug) dt=%.3fs → %.3f m/s (低估 %.0f%%)" % (bug_dt, bug_dv, 100 * (1 - bug_dv / true_dv)))
    print("  修复后   dt=%.3fs → %.3f m/s (正确)" % (fix_dt, fix_dv))
    return dict(true_dv=true_dv, bug_dv=bug_dv, fix_dv=fix_dv,
                bug_dt=bug_dt, fix_dt=fix_dt)


def main():
    results = {
        "E1_H1": e1_ahrs_yaw_correction(),
        "E2_H2": e2_scalar_yaw_tilt(),
        "E3_tier": e3_static_tier(),
        "E4_epoch": e4_epoch_timing(),
        "E5_dualvec": e5_dual_vector_static(),
        "E6_overflow": e6_overflow_dt(),
    }
    with open(os.path.join(OUT, "audit_before.json"), "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, indent=2, default=float)
    print("\n[完成] 修复前核查写入 sim-data/audit_before.json")


if __name__ == "__main__":
    main()
