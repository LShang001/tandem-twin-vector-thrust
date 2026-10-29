# -*- coding: utf-8 -*-
"""run_fw_ctrl_demo.py — 固件控制分配 ABI 的 Python 直调演示与不变量检查。

@file    run_fw_ctrl_demo.py
@module  analysis/固件算法直调（控制分配演示）
@brief   用 fwapi.TandemVecAllocator / thrust_alloc 扫 α 与油门，验证：
           ① 零油门（<5%）摆角/差速归中；
           ② 摆角/差速随 α 单调、符号正确；
           ③ 限幅触发 sat 标记；
           ④ 推力分配在 cos 下限与最大推力处饱和；
         并把响应曲线落盘（JSON + PNG）。不含任何控制公式复刻。
@details 输出：sim-data/fw_ctrl_demo_<date>.json、figs/fw_ctrl_demo_<date>.png
@frame   α/ω 存档系（x=滚转、y=俯仰、z=推力轴）；推力分配 NED 天向。
@units   rad/s²、%、rad、N。
"""
from __future__ import annotations

import datetime as _dt
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import fwapi

plt.rcParams["font.family"] = "sans-serif"
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

DATA = os.path.normpath(os.path.join(HERE, "..", "sim-data"))
FIGS = os.path.normpath(os.path.join(HERE, "..", "figs"))


def main():
    """扫 α/油门/推力输入并检查执行器不变量。"""
    p = fwapi.default_params()
    pd = fwapi.params_to_dict(p)
    checks = {}

    # ① 零油门归中
    a = fwapi.TandemVecAllocator(params=p)
    z = a.step(3.0, -3.0, 2.0, throttle_pct=0.0, dt=0.005)
    checks["zero_throttle_centered"] = (
        abs(z["delta_f_rad"]) < 1e-6 and abs(z["delta_t_rad"]) < 1e-6
        and abs(z["dw"]) < 1e-6 and z["valid"])

    # ② α 扫描（50% 油门，单轴；量程按轴选取：滚转/俯仰 15° 限幅约 13 rad/s²，
    #    差速 0.7 需要更大的 α_yaw）
    ranges = {"roll": (-20.0, 20.0), "pitch": (-20.0, 20.0), "yaw": (-100.0, 100.0)}
    sweeps = {}
    sweep_x = {}
    for name, idx in (("roll", 0), ("pitch", 1), ("yaw", 2)):
        alphas = np.linspace(*ranges[name], 81)
        sweep_x[name] = alphas
        rows = []
        inst = fwapi.TandemVecAllocator(params=p)  # 每个扫描独立状态，避免 τm 递推耦合
        for v in alphas:
            alpha = [0.0, 0.0, 0.0]
            alpha[idx] = float(v)
            out = inst.step(*alpha, throttle_pct=50.0, dt=0.005)
            rows.append({k: out[k] for k in ("valid", "delta_f_rad", "delta_t_rad",
                                             "dw", "motor1_pct", "motor2_pct",
                                             "sat_delta_f", "sat_delta_t", "sat_dw")})
        sweeps[name] = rows

    # ③ 限幅与符号
    roll = sweeps["roll"]
    checks["roll_sign"] = roll[-1]["delta_f_rad"] * roll[0]["delta_f_rad"] < 0
    checks["roll_saturation"] = bool(roll[-1]["sat_delta_f"])
    checks["pitch_saturation"] = bool(sweeps["pitch"][-1]["sat_delta_t"])
    checks["yaw_saturation"] = bool(sweeps["yaw"][-1]["sat_dw"])
    checks["all_finite"] = all(
        np.isfinite([r[k] for r in rows for k in
                     ("delta_f_rad", "delta_t_rad", "dw", "motor1_pct", "motor2_pct")]).all()
        for rows in sweeps.values())

    # ④ 推力分配
    thrust = []
    for a_up in np.linspace(0.0, 30.0, 61):
        for cos_t in (1.0, 0.5, 0.2):
            r = fwapi.thrust_alloc(a_up, cos_t, p.m, 13.73)
            thrust.append({"a_up": float(a_up), "cos": cos_t, **r})
    checks["thrust_cos_floor"] = all(
        r["throttle_pct"] <= 100.0 + 1e-6 for r in thrust)
    checks["thrust_saturates"] = any(r["saturated_high"] for r in thrust)
    checks["thrust_monotone"] = all(
        thrust[i]["total_thrust_n"] <= thrust[i + 1]["total_thrust_n"] + 1e-6
        for i in range(len(thrust) - 1)
        if thrust[i]["cos"] == thrust[i + 1]["cos"])

    ok = all(bool(v) for v in checks.values())

    date = _dt.date.today().isoformat()
    os.makedirs(DATA, exist_ok=True)
    os.makedirs(FIGS, exist_ok=True)
    out_json = os.path.join(DATA, f"fw_ctrl_demo_{date}.json")
    out_png = os.path.join(FIGS, f"fw_ctrl_demo_{date}.png")
    payload = {"date": date, "abi": fwapi.ABI_VERSION,
               "lib": str(fwapi.library_path()), "params": pd,
               "checks": checks, "pass": ok,
               "sweeps": {k: v for k, v in sweeps.items()},
               "sweep_x": {k: v.tolist() for k, v in sweep_x.items()},
               "thrust": thrust,
               "note": "Python 经 C ABI 直调固件分配器；无控制公式复刻。"}
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=1)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))

    ax = axes[0][0]
    ax.plot(sweep_x["roll"], [np.degrees(r["delta_f_rad"]) for r in sweeps["roll"]])
    ax.axhline(15, color="grey", ls="--", lw=1)
    ax.axhline(-15, color="grey", ls="--", lw=1)
    ax.set_xlabel("α_roll [rad/s²]")
    ax.set_ylabel("δf [deg]")
    ax.set_title("滚转 → 上摆")
    ax.grid(alpha=0.3)

    ax = axes[0][1]
    ax.plot(sweep_x["pitch"], [np.degrees(r["delta_t_rad"]) for r in sweeps["pitch"]])
    ax.axhline(15, color="grey", ls="--", lw=1)
    ax.axhline(-15, color="grey", ls="--", lw=1)
    ax.set_xlabel("α_pitch [rad/s²]")
    ax.set_ylabel("δt [deg]")
    ax.set_title("俯仰 → 下摆")
    ax.grid(alpha=0.3)

    ax = axes[1][0]
    ax.plot(sweep_x["yaw"], [r["dw"] for r in sweeps["yaw"]])
    ax.axhline(0.7, color="grey", ls="--", lw=1)
    ax.axhline(-0.7, color="grey", ls="--", lw=1)
    ax.set_xlabel("α_yaw [rad/s²]")
    ax.set_ylabel("Δω")
    ax.set_title("偏航 → 差速")
    ax.grid(alpha=0.3)

    ax = axes[1][1]
    for cos_t, lab in ((1.0, "cos=1.0"), (0.5, "cos=0.5"), (0.2, "cos=0.2(下限)")):
        vals = [r["total_thrust_n"] for r in thrust if r["cos"] == cos_t]
        ax.plot([r["a_up"] for r in thrust if r["cos"] == cos_t], vals, label=lab)
    ax.axhline(13.73, color="grey", ls="--", lw=1, label="最大推力")
    ax.set_xlabel("期望天向比力 [m/s²]")
    ax.set_ylabel("总推力 [N]")
    ax.set_title("推力分配与饱和")
    ax.grid(alpha=0.3)
    ax.legend()

    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    plt.close(fig)

    print(json.dumps(checks, ensure_ascii=False, indent=1))
    print("PASS" if ok else "FAIL")
    print("out:", out_json)
    print("fig:", out_png)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
