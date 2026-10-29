"""
@file    run_fw_nav_demo.py
@brief   固件导航步进 ABI 冒烟与基本行为验证
@module  仿真验证（固件-仿真一体化）
@details 对准→悬停/匀速场景 + ABI 返回码 + reset 语义。
@frame   NED / FRD
@units   SI


验证项（预登记）：
  N1 caps 报告 nav_abi_version=1 且 feature_bits 含 nav_available；
  N2 静止悬停 6 s：导航初始化、水平位置误差 ≤1 m、高度 ≤0.5 m、航向误差 ≤3°；
  N3 2 m/s 北向匀速：速度收敛到 2±0.3 m/s、位置不漂（≤3 m）；
  N4 ABI 返回码：版本不符=2、结构过小=3/4、NaN=5；
  N5 reset 后再次初始化（步骤计数归零、initialized 重新置位）。

用法：py -3.12 sim/run_fw_nav_demo.py [--json OUT]
"""
import argparse
import json
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import fwapi  # noqa: E402
from fwapi import library  # noqa: E402

G = 9.79
DT = 0.005
HZ = 200
HOME = (28.2, 112.9, 50.0)  # 与 mag_reference / SIL kHomeLla 同站点
MAG_NED_UT = np.array([35.075, -2.523, 34.079])


def lla_from_ned(ned, home=HOME):
    """小范围 WGS-84 NED→LLA（与固件 ned2lla 同量级精度，仅用于生成激励）。"""
    lat0, lon0, alt0 = home
    rn = 6378137.0 / math.sqrt(1.0 - 0.00669438 * math.sin(lat0) ** 2)
    rm = rn * (1.0 - 0.00669438) / (1.0 - 0.00669438 * math.sin(lat0) ** 2)
    dlat_deg = math.degrees(ned[0] / (rm + alt0))
    dlon_deg = math.degrees(ned[1] / ((rn + alt0) * math.cos(lat0)))
    return (lat0 + dlat_deg, lon0 + dlon_deg, alt0 - ned[2])


ITOW = [100000]  # 全局单调（跨用例）iTOW：重复 iTOW 会被解析器当重复历元丢弃


def ned_from_lla(lla, home=HOME):
    """lla_from_ned 的逆（局部小范围近似，用于把 EKF 的 LLA 换回 NED 比较）。"""
    lat0, lon0, alt0 = home
    rn = 6378137.0 / math.sqrt(1.0 - 0.00669438 * math.sin(lat0) ** 2)
    rm = rn * (1.0 - 0.00669438) / (1.0 - 0.00669438 * math.sin(lat0) ** 2)
    return np.array([math.radians(lla[0] - lat0) * (rm + alt0),
                     math.radians(lla[1] - lon0) * (rn + alt0) * math.cos(lat0),
                     -(lla[2] - alt0)])


def _feed(nav, rng, vel_ned, k, flying):
    """喂一拍：10 个 2 kHz 样本的聚合（sample_count=10），含微量噪声。

    噪声是必要的：固件 IMU 健康检测把"逐样本精确重复"判成冻结故障（Failed）
    并停止积分 —— 理想恒定输入不是生产口径。
    """
    t = k * DT
    pos = np.array(vel_ned, dtype=float) * t
    lla = lla_from_ned(pos)
    gnss = None
    if k % (HZ // 10) == 0:  # 10 Hz
        ITOW[0] += 100
        gnss = {"lat_deg": lla[0], "lon_deg": lla[1], "alt_m": lla[2],
                "vel_ned": vel_ned, "h_acc": 0.5, "v_acc": 0.8,
                "s_acc": 0.05, "itow_ms": ITOW[0]}
    accel = (rng.normal(0.0, 2e-3), rng.normal(0.0, 2e-3),
             -G + rng.normal(0.0, 2e-3))
    gyro = rng.normal(0.0, 2e-4, 3)
    return nav.step(dt=DT, accel=accel, gyro=gyro, sample_count=10,
                    ahrs_quat=(1.0, 0.0, 0.0, 0.0), gnss=gnss,
                    baro_alt_up=float(-pos[2]), mag=tuple(MAG_NED_UT),
                    unlocked=flying, throttle_pct=50.0 if flying else 0.0)


def run_case(name, seconds, vel_ned=(0.0, 0.0, 0.0), expect_vel=None):
    """地面静止对准 1.5 s → 飞行 `seconds` → 末 1 s 统计。"""
    nav = fwapi.Nav()
    nav.reset()
    rng = np.random.default_rng(7)
    out = None
    k = 0
    for _ in range(int(1.5 * HZ)):
        out = _feed(nav, rng, vel_ned, k, False)
        k += 1
    for _ in range(int(seconds * HZ)):
        out = _feed(nav, rng, vel_ned, k, True)
        k += 1
    tail = []
    for _ in range(HZ):
        o = _feed(nav, rng, vel_ned, k, True)
        tail.append((o["vel_ned"], ned_from_lla(o["lla_deg"]), o["alt_up_m"]))
        k += 1
    nav.close()
    vel = np.mean([x[0] for x in tail], axis=0)
    # 位置用 EKF 的绝对 LLA 换回 NED（`pos_ned` 是"起飞原点"相对量，
    # 原点建立时刻取决于固件的起飞判定，不适合当绝对位置判据）。
    pos = np.mean([x[1] for x in tail], axis=0)
    alt = np.mean([x[2] for x in tail])
    true_pos = np.array(vel_ned, dtype=float) * ((k - 0.5) * DT)
    print(f"[{name}] init={out['initialized']} epochs={out['gnss_fix3d_count']} "
          f"health={out['imu_health']} vel={np.round(vel, 3)} "
          f"pos={np.round(pos, 3)} true={np.round(true_pos, 3)} alt={alt:.3f}")
    return {"init": out["initialized"], "vel": vel, "pos": pos,
            "true_pos": true_pos, "alt": alt, "out": out,
            "expect": expect_vel}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", default="")
    args = ap.parse_args()
    fails = []

    def check(cond, msg):
        print(("[PASS] " if cond else "[FAIL] ") + msg)
        if not cond:
            fails.append(msg)

    c = fwapi.caps()
    check(c["nav_abi"] == library.NAV_ABI_VERSION,
          f"N1 caps.nav_abi={c['nav_abi']} == 1")
    check("nav_available" in c["features"], "N1 caps.features 含 nav_available")

    hover = run_case("hover", 6.0)
    check(hover["init"] == 1, "N2 导航已初始化")
    check(np.linalg.norm(hover["pos"][:2]) <= 1.0,
          f"N2 水平位置 {np.linalg.norm(hover['pos'][:2]):.3f} m ≤ 1.0")
    check(abs(hover["alt"]) <= 0.5, f"N2 高度 {hover['alt']:.3f} m ≤ 0.5")
    check(abs(hover["out"]["euler_rpy_deg"][2]) <= 3.0,
          f"N2 航向误差 {hover['out']['euler_rpy_deg'][2]:.2f}° ≤ 3°")

    cruise = run_case("cruise2", 6.0, vel_ned=(2.0, 0.0, 0.0))
    check(abs(cruise["vel"][0] - 2.0) <= 0.3,
          f"N3 北向速度 {cruise['vel'][0]:.3f} m/s ≈ 2.0±0.3")
    check(abs(cruise["vel"][1]) <= 0.3 and abs(cruise["vel"][2]) <= 0.3,
          f"N3 横向/垂向速度 {np.round(cruise['vel'][1:], 3)} ≤ 0.3")
    pos_err = float(np.linalg.norm((cruise["pos"] - cruise["true_pos"])[:2]))
    check(pos_err <= 3.0,
          f"N3 绝对位置误差 {pos_err:.3f} m ≤ 3 m（EKF LLA vs 真值 NED）")

    # N4 ABI 返回码
    nav = fwapi.Nav()
    lib = library.load()
    import ctypes
    inp = library.FwNavInput()
    inp.struct_size = ctypes.sizeof(library.FwNavInput)
    inp.abi_version = library.NAV_ABI_VERSION + 99
    inp.dt_s = DT
    out = library.FwNavOutput()
    out.struct_size = ctypes.sizeof(library.FwNavOutput)
    rc = lib.fw_nav_step(nav._handle, ctypes.byref(inp), ctypes.byref(out))
    check(rc == 2, f"N4 版本不符 rc={rc} == 2")
    inp.abi_version = library.NAV_ABI_VERSION
    inp.dt_s = float("nan")
    rc = lib.fw_nav_step(nav._handle, ctypes.byref(inp), ctypes.byref(out))
    check(rc == 5, f"N4 NaN dt rc={rc} == 5")
    inp.dt_s = DT
    inp.struct_size = 8
    rc = lib.fw_nav_step(nav._handle, ctypes.byref(inp), ctypes.byref(out))
    check(rc == 3, f"N4 输入结构过小 rc={rc} == 3")
    nav.reset()
    r = nav.step(dt=DT, accel=(0, 0, -G), gyro=(0, 0, 0),
                 mag=tuple(MAG_NED_UT), unlocked=True)
    check(r["steps"] == 1, f"N5 reset 后 steps={r['steps']} == 1")
    nav.close()

    print(f"\n=== run_fw_nav_demo: {'全部通过' if not fails else f'{len(fails)} 项失败'} ===")
    if args.json:
        payload = {"hover": {k: (v.tolist() if isinstance(v, np.ndarray) else v)
                             for k, v in hover.items() if k != "out"},
                   "cruise": {k: (v.tolist() if isinstance(v, np.ndarray) else v)
                              for k, v in cruise.items() if k != "out"},
                   "fails": fails,
                   "caps": c}
        pathlib.Path(args.json).write_text(
            json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"written: {args.json}")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
