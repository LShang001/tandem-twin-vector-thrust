# -*- coding: utf-8 -*-
"""check_consistency.py — EKF 导航一致性回归门（NEES/NIS）。

@file    check_consistency.py
@module  analysis/EKF 研究仿真（一致性门禁）
@brief   跑标称/高噪声 GNSS 场景，检查 NEES（目标 1）、NIS（目标 6）与 RMSE 是否越界。
@details 用 `run_scenarios.simulate_gnss`（15 态 ESKF 与固件同构）逐种子仿真；
         输出 CSV 行并在任一指标越界时返回码 1。默认 5 seeds、60 s。
@theory  NEES = eᵀP⁻¹e / n，一致时均值≈1；NIS = rᵀS⁻¹r，6 维 GNSS 一致时≈6。
         @ref REF-121 §7.3.1 p.255–259（NIS/χ² 门控）
@frame   NED + FRD；@units SI
@note    本门禁是**研究模型**（非固件 SIL），用于 EKF 改动前的基础一致性回归；
         阈值按 2026-10-25 基线（NEES_vel 0.4、NIS 4.8）留出改进空间，不承诺实机。
@test    正常基线应 PASS；把 Q 调大/量测 R 调小应触发 FAIL。
@see     ekf_core.py、run_scenarios.py、TandemVec-EKF-Report/无磁EKF收敛仿真-2026-10-25.md
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import run_scenarios as R  # noqa: E402
from ekf_core import Ekf15, cfg as base_cfg, HOME_LLA, G_MPS2  # noqa: E402


def make_filter(scale):
    c = dict(base_cfg)
    c["q_accel_scale"] = scale
    e = Ekf15(c)
    e.home_lla = HOME_LLA.copy()
    e.initialize(np.array([0.0, 0.0, -G_MPS2]), np.zeros(3), HOME_LLA)
    return e


def run_case(name, noise, scale, seeds, duration):
    R.make_ekf = lambda: make_filter(scale)
    rows = []
    for seed in range(seeds):
        R.RNG = np.random.default_rng(seed + 100)
        r = R.simulate_gnss(duration, (5.0, 0.0, 0.0), gnss_hz=10, age_s=0.0,
                            gnss_noise=noise, outage=(None, None),
                            rng_seed=seed, replay=True)
        pe, ve = r["pos_err"], r["vel_err"]
        pp, pv, nis = r["p_pos"], r["p_vel"], r["nis"]
        rows.append((
            float(np.sqrt(np.mean(np.sum(pe * pe, axis=1)))),
            float(np.sqrt(np.mean(np.sum(ve * ve, axis=1)))),
            float(np.mean(np.sum((pe / pp) ** 2, axis=1)) / 3.0),
            float(np.mean(np.sum((ve / pv) ** 2, axis=1)) / 3.0),
            float(np.mean(nis[:, 1])) if len(nis) else float("nan"),
        ))
    a = np.array(rows)
    return dict(name=name, scale=scale,
                pos_rmse=float(np.median(a[:, 0])),
                vel_rmse=float(np.median(a[:, 1])),
                nees_pos=float(np.median(a[:, 2])),
                nees_vel=float(np.median(a[:, 3])),
                nis=float(np.median(a[:, 4])))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--q-scale", type=float, default=1.0)
    ap.add_argument("--strict", action="store_true",
                    help="NEES 必须落在 [0.5,2.0] 且 NIS 在 [4,8]")
    args = ap.parse_args()

    cases = [
        ("nominal", None),
        ("highnoise", (5.0, 8.0, 0.5, 0.8)),
    ]
    ok = True
    for name, noise in cases:
        m = run_case(name, noise, args.q_scale, args.seeds, args.seconds)
        print("CONSISTENCY {name} q={scale:.2f} pos={pos_rmse:.3f} "
              "vel={vel_rmse:.3f} nees_pos={nees_pos:.3f} "
              "nees_vel={nees_vel:.3f} nis={nis:.3f}".format(**m))
        if m["pos_rmse"] > 2.0 or m["vel_rmse"] > 0.4:
            ok = False
            print("FAIL RMSE", name)
        if args.strict:
            if not (4.0 <= m["nis"] <= 8.0):
                ok = False
                print("FAIL NIS", name)
            if not (0.5 <= m["nees_vel"] <= 2.0) or not (
                    0.5 <= m["nees_pos"] <= 2.0):
                ok = False
                print("FAIL NEES", name)
    print("RESULT", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
