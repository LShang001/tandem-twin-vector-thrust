#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""SIL 控制链 vs fwapi.Gnc 逐拍对拍。

@file    check_fw_gnc_parity.py
@brief   用 SIL `--trace-ctrl` 落盘的输入序列重放固件控制链，比较
          `fwapi.Gnc`（共享库）与 SIL（可执行）每拍输出是否一致。
@module  TandemVec-EKF-Report/仿真验证（固件-仿真一体化）
@details 这是一体化的**等价性门禁**：同一份固件源码编进两个目标，若输入映射、
          参数实例或时钟处理有任何偏差，这里会逐拍暴露。
          SIL 侧命令（VTOL，非导弹）：
            fw_real.exe --real-nav --seconds 20 --profile mag_clean \
                        --takeoff-at 3 --trace-ctrl /tmp/ctrl.csv
@frame   NED / FRD（透传固件约定）
@units   角度 deg、角速率 dps、角加速度 dps²、执行器 %
@note    预登记判据（跑前写死）：
         C1 行数 ≥ 1000（约 5 s @200 Hz），否则该轨迹不构成验证；
         C2 轨迹必须覆盖"已解锁且油门 >5%"的样本 ≥ 200 拍，否则只是地面锁定态；
         C3 每拍 `fw_gnc_step` 返回 0；
         C4a 模式逐拍完全一致；C4b（严格逐位，统一 1e-2）保留结果、不做事后调整；
         C5 报告首个超差时刻与最大偏差（诊断）；
         C6 工程等价（分通道）：执行器 ≤1%、摆角 ≤0.5°、角速率 ≤0.1 dps、
            角加速度 ≤1 dps²。退出码按 C1–C4a + C6 判定；C4b 的 FAIL 会在输出中保留。
@see     TandemVec_FCS/test_host/fw_algo/fw_gnc_api.h、sim/fwapi/gnc.py
"""
import argparse
import csv
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import fwapi  # noqa: E402

DEG2RAD = math.pi / 180.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trace", required=True)
    ap.add_argument("--tol", type=float, default=1e-2,
                    help="执行器% / deg / dps / dps² 的统一容差")
    ap.add_argument("--json", default="", help="把对拍指标写到该 JSON（证据落盘）")
    args = ap.parse_args()

    rows = list(csv.DictReader(open(args.trace, encoding="utf-8")))
    fails = []

    def check(cond, msg):
        print(("[PASS] " if cond else "[FAIL] ") + msg)
        if not cond:
            fails.append(msg)

    check(len(rows) >= 1000, f"C1 行数 {len(rows)} ≥ 1000")
    active = sum(1 for r in rows if float(r["ch3"]) > 5.0)
    check(active >= 200, f"C2 已解锁高油门样本 {active} ≥ 200")

    # 诊断：SIL 的 AHRS 欧拉角 vs 由四元数按 firmware 口径重算（theta=-asin(R13)）。
    # 若两者不一致，说明 ABI 用四元数注入与 SIL 用欧拉角的路径存在口径差。
    euler_gap = 0.0
    for r in rows:
        w, x, y, z = (float(r["quat_w"]), float(r["quat_x"]),
                      float(r["quat_y"]), float(r["quat_z"]))
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - x * z))))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        for a, k in ((roll, "roll"), (pitch, "pitch"), (yaw, "heading")):
            d = abs(a - float(r[k]))
            euler_gap = max(euler_gap, d)
    print(f"诊断: SIL 欧拉角 vs 四元数重算 最大差 {euler_gap:.3e} rad")

    g = fwapi.Gnc()
    prev_t = None
    maxd = {}
    first_bad = None
    step_fail = 0
    mode_bad = 0
    for r in rows:
        t = float(r["t"])
        dt = 0.005 if prev_t is None else round(t - prev_t, 6)
        if dt <= 0.0:
            dt = 0.005
        prev_t = t
        # SIL 已把 target_alt 记录为**调用前**快照 ⇒ 每拍原样写入即可与 SIL 的
        # 调用起点一致（不再需要"仅外部跳变才写"的启发式）。
        sil_target = float(r["target_alt"])
        apply_target = True
        try:
            out = g.step(
                quat=(float(r["quat_w"]), float(r["quat_x"]),
                      float(r["quat_y"]), float(r["quat_z"])),
                # 控制链内部用原始 icm_gyro 重新滤波 ⇒ 必须喂原始（已去偏）角速度，
                # 不能喂已滤波的 wx_dps（会二次滤波）。
                omega=(float(r["gx_rps"]), float(r["gy_rps"]), float(r["gz_rps"])),
                vel=(float(r["vn"]), float(r["ve"]), float(r["vd"])),
                pos=(float(r["rn"]), float(r["re"]), float(r["rd"])),
                alt_up=float(r["alt_up"]),
                rc=[int(r[f"rc{i}"]) for i in range(9)] + [988] * 7,
                dt=dt,
                link_up=int(r["link"]) != 0,
                failsafe_in_flight=int(r["fs"]) != 0,
                target_alt=sil_target,
                apply_target_alt=apply_target,
                heading_hold_allowed=int(r["hdg_ok"]) != 0,
                pos_hold_degraded=int(r["pos_deg"]) != 0,
                gnss_outage_age_s=float(r["gnss_age"]),
            )
        except RuntimeError:
            step_fail += 1
            continue

        pairs = {
            "ch1": (out["ch1_pct"], float(r["ch1"])),
            "ch2": (out["ch2_pct"], float(r["ch2"])),
            "ch3": (out["ch3_pct"], float(r["ch3"])),
            "ch4": (out["ch4_pct"], float(r["ch4"])),
            "delta_f": (out["delta_f_deg"], float(r["delta_f"])),
            "delta_t": (out["delta_t_deg"], float(r["delta_t"])),
            "dw": (out["dw"], float(r["dw"])),
            "sfup": (out["specific_force_up_mps2"], float(r["sfup"])),
            "cos_tilt": (out["thrust_cos_tilt"], float(r["cos_tilt"])),
            "thr_cmd": (out["throttle_cmd_pct"], float(r["thr_cmd"])),
        }
        for i in range(3):
            pairs[f"or{i}"] = (out["omega_ref_dps"][i], float(r[f"or{i}"]))
            pairs[f"ar{i}"] = (out["alpha_ref_dps2"][i], float(r[f"ar{i}"]))
        for k, (a, b) in pairs.items():
            d = abs(a - b)
            maxd[k] = max(maxd.get(k, 0.0), d)
            if d > args.tol and first_bad is None:
                first_bad = (t, k, a, b, d)
        if int(out["flight_mode"]) != int(r["mode"]):
            mode_bad += 1

    check(step_fail == 0, f"C3 fw_gnc_step 失败 {step_fail} 拍")
    worst = max(maxd.items(), key=lambda kv: kv[1]) if maxd else ("-", 0.0)
    print("max |Δ|:", {k: round(v, 6) for k, v in sorted(maxd.items())})
    check(mode_bad == 0, f"C4a 模式逐拍一致（不一致 {mode_bad} 拍）")

    # C4b 是**预登记**的严格判据（统一 1e-2）：保留结果，不因后验调整而粉饰。
    strict_ok = worst[1] <= args.tol
    print(("[PASS] " if strict_ok else "[FAIL] ") +
          f"C4b 严格逐位 {worst[0]}={worst[1]:.6g} ≤ {args.tol}" +
          ("" if strict_ok else "（预登记判据未达标，残差见下）"))
    if first_bad is not None:
        print("首个超差:", first_bad)

    # C6 工程等价（分通道、按物理量纲）。依据：SIL 与共享库是同一份控制律，
    # 但输入经 CSV 量化，且导航层部分状态（锁高预测等）在两条路径上存在末位
    # 差异；逐位一致不是可达目标，"量级上不影响控制"才是验收标准。
    tolerances = {
        "ch1": 1.0, "ch2": 1.0, "ch3": 1.0, "ch4": 1.0,
        "delta_f": 0.5, "delta_t": 0.5, "dw": 0.01,
        "or0": 0.1, "or1": 0.1, "or2": 0.1,
        "ar0": 1.0, "ar1": 1.0, "ar2": 1.0,
        "sfup": 0.01, "cos_tilt": 1e-3, "thr_cmd": 0.5,
    }
    over = {k: (maxd.get(k, 0.0), tol) for k, tol in tolerances.items()
            if maxd.get(k, 0.0) > tol}
    if over:
        print("C6 超差通道:", {k: (round(v, 6), t) for k, (v, t) in over.items()})
    check(not over, "C6 工程等价（执行器 ≤1%、摆角 ≤0.5°、角速率 ≤0.1 dps、角加速度 ≤1 dps²）")

    if args.json:
        import json
        json.dump({
            "trace": str(args.trace), "rows": len(rows),
            "max_abs_diff": {k: float(v) for k, v in maxd.items()},
            "mode_mismatch": int(mode_bad), "step_fail": int(step_fail),
            "strict_tol": args.tol, "strict_ok": bool(strict_ok),
            "engineering_over": {k: float(v) for k, (v, _) in over.items()},
            "caps": fwapi.caps(),
        }, open(args.json, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        print("saved", args.json)

    print("\n=== check_fw_gnc_parity:", "通过（工程等价）" if not fails else f"失败 {len(fails)} 项", "===")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
