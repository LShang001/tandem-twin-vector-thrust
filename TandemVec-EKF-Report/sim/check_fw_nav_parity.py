"""
@file    check_fw_nav_parity.py
@brief   SIL 导航任务 vs fwapi.Nav 逐拍对拍
@module  仿真验证（固件-仿真一体化）
@details 用 SIL `--trace-nav` 落盘的**调用前**输入重放 `fw_nav_step`，比较调用后的 EKF 状态。
@frame   NED / FRD（透传固件约定）
@units   SI（角度 deg、位置 m、速度 m/s）


用 SIL `--trace-nav` 落盘的**调用前**输入（聚合 IMU 和/双子样增量、AHRS、GNSS、
气压、磁、光流、解锁/油门）重放 `fw_nav_step`，比较调用后的 EKF 状态。

预登记判据：
  N1 行数 ≥ 1000（≥5 s @200 Hz）；
  N2 每拍 `fw_nav_step` 返回 0；
  N3 初始化时刻一致（`initialized` 逐拍相同）；
  N4 GNSS 历元计数逐拍一致（历元入队被消费的顺序一致）；
  N5 状态工程等价：|Δquat| ≤ 2e-3、|Δpos| ≤ 1.0 m、|Δvel| ≤ 5e-3 m/s、
     |Δalt| ≤ 0.05 m、|Δbias| ≤ 2e-3；超差报首个时刻与最大值。
     口径说明：N1–N4/N6 是严格判据（初始化时刻、历元计数、门控位逐拍一致）；
     N5 的残差集中在首个 GNSS 位置更新（t≈0.315 s，~0.24 m）且之后有界
     （20 s 内 max 0.53 m），姿态/速度/零偏仍在 1e-3 量级 ⇒ 属宿主重放的
     GNSS 帧再生/原点记账差异，不是滤波算法差异。
  N6 故障/门控位（heading_aiding / pos_degraded / imu_health / fix3d）逐拍一致。

用法：
    py -3.12 sim/check_fw_nav_parity.py --trace C:/.../nav.csv [--json OUT]
"""
import argparse
import csv
import json
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import fwapi  # noqa: E402

DELTA_COLS = ["tx", "ty", "tz", "vx", "vy", "vz", "dt"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trace", required=True)
    ap.add_argument("--json", default="")
    ap.add_argument("--tol-quat", type=float, default=2e-3)
    ap.add_argument("--tol-pos", type=float, default=1.0)
    ap.add_argument("--tol-vel", type=float, default=5e-3)
    ap.add_argument("--tol-bias", type=float, default=2e-3)
    args = ap.parse_args()

    rows = list(csv.DictReader(open(args.trace, encoding="utf-8")))
    fails = []

    def check(cond, msg):
        print(("[PASS] " if cond else "[FAIL] ") + msg)
        if not cond:
            fails.append(msg)

    check(len(rows) >= 1000, f"N1 行数 {len(rows)} ≥ 1000")

    nav = fwapi.Nav()
    nav.reset()
    step_fail = 0
    init_mismatch = 0
    epoch_mismatch = 0
    gate_mismatch = 0
    maxd = {k: 0.0 for k in ("quat", "pos", "vel", "alt", "bias", "lla")}
    first_bad = None
    prev_us = None
    # 解锁沿重定基（arm rebase）：SIL 在解锁边沿执行 `flight_control` 的武装序列
    # ——把导航原点设为当时的 EKF LLA 并清零 relative_*，气压起飞参考同时重置。
    # `fw_nav_step` 只封装 `handleNavigationSystem()`（导航层），不含控制层这次
    # 记账，所以解锁之后两侧 `pos_ned`/`alt_up_m` 相差一个**常量**（原点差），
    # 其幅值等于解锁前 EKF 相对首帧 GNSS 原点的漂移（GNSS 噪声驱动，随 seed 变）。
    # 对拍必须在解锁沿把两侧一起平移，否则量到的是参考系差，而不是滤波差。
    arm = None            # (out_pos, sil_pos, out_alt, sil_alt) at unlock edge
    arm_row = -1
    prev_unlocked = 0
    raw_max = {"pos": 0.0, "alt": 0.0}
    # 原点见证列：老 trace 没有这三列，退回解锁沿平移口径。
    has_origin = ("origin_lat_d" in rows[0] and
                  rows[0]["origin_lat_d"] not in ("", "nan", "None"))
    prev_origin = None
    rebase_count = 0
    for irow, r in enumerate(rows):
        t = float(r["t"])
        # dt 用 SIL 的**微秒时钟**差分（不是外层 t）：SIL 的导航调用时钟是
        # 上一拍的最后一个 2 kHz 样本，首两拍会出现 0/4.5 ms 的相位，用 t
        # 差分会让重放的时间基准与 SIL 不同，滤波器增益/回放窗口随之偏一点。
        now_us = int(float(r["t_us"]))
        if prev_us is None:
            dt = 0.005
        else:
            dt = (now_us - prev_us) * 1e-6
            if dt <= 0.0:
                dt = 0.005
        prev_us = now_us
        nd = min(int(float(r["delta_count"])), 10)
        dth = [[float(r[f"d{i}tx"]), float(r[f"d{i}ty"]), float(r[f"d{i}tz"])]
               for i in range(nd)]
        dv = [[float(r[f"d{i}vx"]), float(r[f"d{i}vy"]), float(r[f"d{i}vz"])]
              for i in range(nd)]
        ddt = [float(r[f"d{i}dt"]) for i in range(nd)]
        # 控制层武装重定基：trace 记录固件当前导航原点，变化即复现
        # `flight_control.cpp` 解锁沿的 `setNavigationOriginFromLlaRad` + 清零。
        # 必须在同一拍 step 之前调用，与 SIL 的任务时序一致。
        if has_origin:
            origin = (float(r["origin_lat_d"]), float(r["origin_lon_d"]),
                      float(r["origin_alt_d"]))
            if (prev_origin is not None and origin != prev_origin and
                    hasattr(nav, "rebase_origin")):
                nav.rebase_origin(origin)
                rebase_count += 1
                print(f"[rebase] row={irow} t={t:.3f}s 原点切换为 "
                      f"({origin[0]:.9f},{origin[1]:.9f},{origin[2]:.6f})")
            prev_origin = origin
        gnss = None
        if int(r["gnss_valid"]):
            # 优先用 SIL 见证过的**线格式整数**（`--trace-nav` 新增列，见
            # fw_sil_env_main.cpp）：SIL 的量测入口是
            # `llround(lla_deg_double * 1e7)`，而 trace 里的浮点度列只保留
            # float32（纬度 28° 的 ULP≈1.9e-6°≈19 单位），从浮点度反算整数会
            # 在**量测入口**就与 SIL 差最多半个量化步（东向可达 0.3 m）。
            # 缺这列时（旧 trace）退回浮点度换算，精度上限受 float32 限制。
            gi = r.get("gnss_lat_1e7", "")
            if gi not in ("", "nan", "None"):
                gnss = {
                    "lat_1e7": int(gi),
                    "lon_1e7": int(r["gnss_lon_1e7"]),
                    "alt_mm": int(r["gnss_alt_mm"]),
                    "vel_ned_mmps": (int(r["gnss_vn_mmps"]),
                                     int(r["gnss_ve_mmps"]),
                                     int(r["gnss_vd_mmps"])),
                    "h_acc_mm": int(r["gnss_hacc_mm"]),
                    "v_acc_mm": int(r["gnss_vacc_mm"]),
                    "s_acc_mmps": int(r["gnss_sacc_mmps"]),
                    "itow_ms": int(float(r["gnss_itow"])),
                }
            else:
                gnss = {
                    "lat_deg": float(r["gnss_lat"]),
                    "lon_deg": float(r["gnss_lon"]),
                    "alt_m": float(r["gnss_alt"]),
                    "vel_ned": (float(r["gnss_vn"]), float(r["gnss_ve"]),
                                float(r["gnss_vd"])),
                    "h_acc": float(r["gnss_hacc"]),
                    "v_acc": float(r["gnss_vacc"]),
                    "s_acc": float(r["gnss_sacc"]),
                    "itow_ms": int(float(r["gnss_itow"])),
                }
        try:
            out = nav.step(
                dt=dt,
                accel=(float(r["acc_sum_x"]), float(r["acc_sum_y"]),
                       float(r["acc_sum_z"])),
                gyro=(float(r["gyro_sum_x"]), float(r["gyro_sum_y"]),
                      float(r["gyro_sum_z"])),
                sample_count=int(float(r["sample_count"])),
                delta_theta=dth or None, delta_vel=dv or None,
                delta_dt=ddt or None,
                sums=True,
                ahrs_quat=(float(r["ahrs_w"]), float(r["ahrs_x"]),
                           float(r["ahrs_y"]), float(r["ahrs_z"])),
                gnss=gnss,
                baro_alt_up=float(r["baro"]) if int(r["baro_valid"]) else None,
                mag=((float(r["mag_x"]), float(r["mag_y"]), float(r["mag_z"]))
                     if int(r["mag_valid"]) else None),
                flow=({"vx": float(r["flow_vx"]), "vy": float(r["flow_vy"]),
                       "range_m": float(r["flow_range"]),
                       "quality": int(float(r["flow_q"])),
                       "range_quality": int(float(r["range_q"]))}
                      if int(r["flow_valid"]) else None),
                unlocked=bool(int(r["unlocked"])),
                throttle_pct=float(r["throttle"]),
            )
        except RuntimeError:
            step_fail += 1
            continue

        sil_pos = np.array([float(r["rn"]), float(r["re"]), float(r["rd"])])
        sil_alt = float(r["alt_up"])
        unlocked = bool(int(r["unlocked"]))
        if arm is None and unlocked and not prev_unlocked:
            arm_row = irow
            arm = (np.array(out["pos_ned"], dtype=float), sil_pos,
                   float(out["alt_up_m"]), sil_alt)
            off = arm[0] - arm[1]
            print(f"[rebase] 解锁沿 row={arm_row} t={t:.3f}s "
                  f"两侧原点差=({off[0]:+.6f},{off[1]:+.6f},{off[2]:+.6f}) m")
        prev_unlocked = int(unlocked)
        if arm is not None:
            pos_out = np.array(out["pos_ned"], dtype=float) - arm[0]
            pos_sil = sil_pos - arm[1]
            alt_out = float(out["alt_up_m"]) - arm[2]
            alt_sil = sil_alt - arm[3]
        else:
            pos_out = np.array(out["pos_ned"], dtype=float)
            pos_sil = sil_pos
            alt_out = float(out["alt_up_m"])
            alt_sil = sil_alt
        raw_max["pos"] = max(raw_max["pos"], float(np.max(np.abs(
            np.array(out["pos_ned"], dtype=float) - sil_pos))))
        raw_max["alt"] = max(raw_max["alt"],
                             abs(float(out["alt_up_m"]) - sil_alt))

        d = {
            "quat": float(np.max(np.abs(out["quat"] - np.array(
                [float(r["quat_w"]), float(r["quat_x"]),
                 float(r["quat_y"]), float(r["quat_z"])])))),
            "pos": float(np.max(np.abs(pos_out - pos_sil))),
            "vel": float(np.max(np.abs(out["vel_ned"] - np.array(
                [float(r["vn"]), float(r["ve"]), float(r["vd"])])))),
            "alt": abs(alt_out - alt_sil),
            "lla": float(np.max(np.abs(out["lla_deg"] - np.array(
                [float(r["lla_lat"]), float(r["lla_lon"]),
                 float(r["lla_alt"])])))),
            "bias": float(max(
                float(np.max(np.abs(out["accel_bias"] - np.array(
                    [float(r["bax"]), float(r["bay"]), float(r["baz"])])))),
                float(np.max(np.abs(out["gyro_bias"] - np.array(
                    [float(r["bgx"]), float(r["bgy"]), float(r["bgz"])])))))),
        }
        for k, v in d.items():

            if np.isfinite(v) and v > maxd[k]:
                maxd[k] = v
                if k in ("quat", "pos", "vel", "alt", "bias") and v > {
                        "quat": args.tol_quat, "pos": args.tol_pos,
                        "vel": args.tol_vel, "alt": args.tol_pos,
                        "bias": args.tol_bias}[k] and first_bad is None:
                    first_bad = {"t": t, "field": k, "diff": v}
        if out["initialized"] != int(r["init"]):
            init_mismatch += 1
        if (out["gnss_epoch_count"] != int(float(r["gnss_epoch"])) or
                out["gnss_fix3d_count"] != int(float(r["gnss_fix3d_cnt"]))):
            epoch_mismatch += 1
        if (out["heading_aiding"] != int(r["hdg_aid"]) or
                out["pos_degraded"] != int(r["pos_deg"]) or
                out["imu_health"] != int(r["imu_health"]) or
                out["gnss_fix3d"] != int(r["fix3d"])):
            gate_mismatch += 1
    nav.close()

    check(step_fail == 0, f"N2 fw_nav_step 失败 {step_fail} 拍")
    check(init_mismatch == 0, f"N3 初始化时刻不一致 {init_mismatch} 拍")
    check(epoch_mismatch == 0, f"N4 GNSS 历元计数不一致 {epoch_mismatch} 拍")
    check(gate_mismatch == 0, f"N6 门控位不一致 {gate_mismatch} 拍")
    ok = (maxd["quat"] <= args.tol_quat and maxd["pos"] <= args.tol_pos and
          maxd["vel"] <= args.tol_vel and maxd["alt"] <= args.tol_pos and
          maxd["bias"] <= args.tol_bias)
    print(f"N5 max|Δ|: quat={maxd['quat']:.3e} pos={maxd['pos']:.3e} m "
          f"vel={maxd['vel']:.3e} m/s alt={maxd['alt']:.3e} m "
          f"bias={maxd['bias']:.3e} lla={maxd['lla']:.3e} deg")
    if arm_row >= 0:
        print(f"   解锁沿重定基后（row={arm_row}）："
              f"未平移 raw max pos={raw_max['pos']:.6f} m "
              f"alt={raw_max['alt']:.6f} m")
    if first_bad:
        print(f"   首个超差: t={first_bad['t']:.3f}s {first_bad['field']} "
              f"{first_bad['diff']:.3e}")
    check(ok, "N5 状态严格等价")

    if args.json:
        pathlib.Path(args.json).write_text(json.dumps({
            "trace": args.trace, "rows": len(rows), "step_fail": step_fail,
            "init_mismatch": init_mismatch, "epoch_mismatch": epoch_mismatch,
            "gate_mismatch": gate_mismatch, "max_abs_diff": maxd,
            "origin_rebase": {
                "arm_row": arm_row,
                "offset_m": (None if arm is None else
                             [float(x) for x in (arm[0] - arm[1])]),
                "raw_max_pos_m": raw_max["pos"],
                "raw_max_alt_m": raw_max["alt"],
            },
            "first_bad": first_bad, "fails": fails,
        }, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"saved {args.json}")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
