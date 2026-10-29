#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§9.4 #3：直接注入验证"速度误差越过物理门 ⇒ 锁死"（预登记判据在文件头）。

@file    run_gate_lockout_inject.py
@brief   在 10–20 s 窗口内给**导航输入**加一个短时加速度偏置台阶（不改真值/控制），
         让 EKF 速度误差按 magnitudes 阶梯增长，检验越过 NE 10 m/s / D 8 m/s 物理门后
         是否出现"整帧拒绝 → 自由惯导 → 继续被拒"的自持锁死。
@module  TandemVec-EKF-Report/仿真驱动（L2 研究设施）
@details 背景：§4.24ap 的"双稳锁死"是由时序 + 门限行为推断的；§9.4 #3 要求"窗内注入
         一个速度/姿态台阶确认"。SIL 设施 `--nav-accel-bias-step "t0:t1:ax:ay:az"`
         （`test_host/fw_sil/fw_sil_env_main.cpp`）只把偏置加在喂给导航的 IMU 样本上，
         弹体真值、控制输入都不变；自证据计数 `nav_bias_frames` 进 JSON。
@frame   机体系 FRD；加速度偏置 m/s²，沿机体 x（助推段≈天向）
@units   加速度 m/s²、速度 m/s、时间 s、位置 m
@note    预登记判据（跑前写死，不事后改口径）：
         I1 注入自证据：每个非零臂 `nav_bias_frames > 0`，否则该臂作废；
         I2 剂量：注入结束时刻的 |vz 误差| ≈ bias × 窗口长度（±35%），
            否则说明偏置没有真正进入估计，整批结论作废；
         I3 锁死：过门臂（剂量 > D 门 8 m/s）在 [t1, t1+5] 内
            gate_vel=1 帧占比 > 0.5，且窗口末端 |vz 误差| > 4 m/s（门限一半）
            或出现 reacquire_resets>0；
         I4 对照：0 臂同 seed 不产生持续拒绝（gate_vel 占比 < 0.1，rejects ≤ 3）；
         I5 证伪条件：若过门臂在 5 s 内自行恢复到 |vz 误差| < 4 m/s、gate_vel 占比 < 0.1
            且无重锚 ⇒ "越过门限 ⇒ 必然锁死"被否证，只能写成"需要持续越门/特定相位"。
@test    本脚本输出 JSON 到 sim-data/gate_lockout_inject_<tag>.json
@see     TandemVec-EKF-Report/导弹6DOF-导航SIL-2026-10-27.md §4.24ap/§4.24aq/§9.4
"""
import argparse
import json
import os
import subprocess

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
BASE = ["--real-nav", "--missile", "--seconds", "45", "--imu-fs-g", "32",
        "--imu-health-accel-range", "313.8", "--profile", "mag_clean",
        "--imu-grade", "icm", "--gnss-rate-hz", "10"]
THRUST_N = 22000          # ≈28.1 g：§4.24bd 在该档 0/30，基线干净
T_INJ0, T_INJ1 = 15.0, 15.3
BIASES = [0.0, 20.0, 40.0, 80.0]   # 机体系 x [m/s²]；×0.3 s = 0/6/12/24 m/s
D_GATE = 8.0


def run_one(exe, seed, bias, tag_dir):
    out = os.path.join(tag_dir, f"trace_s{seed}_b{int(bias)}.csv")
    cmd = [exe] + BASE + ["--ms-thrust", str(THRUST_N), "--seed", str(seed),
                          "--trace-csv", out,
                          "--nav-accel-bias-step",
                          f"{T_INJ0}:{T_INJ1}:{bias}:0:0"]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
    j = None
    for line in (r.stdout + r.stderr).splitlines():
        if line.startswith("SIL_REAL_JSON "):
            j = json.loads(line[len("SIL_REAL_JSON "):].replace("nan", "NaN")
                           .replace("inf", "Infinity"))
    if j is None or r.returncode != 0:
        return None
    t = pd.read_csv(out).set_index("t")
    t["vz_err"] = t.ekf_vz - t.truth_vz
    t_end = T_INJ1
    i_end = (t.index >= t_end - 1e-9).argmax()
    vz_end = float(t.vz_err.iloc[i_end])
    win = t[(t.index >= t_end) & (t.index < t_end + 5.0)]
    gate_frac = float(win.gate_vel.mean()) if len(win) else float("nan")
    i5 = (t.index >= t_end + 5.0 - 1e-9).argmax()
    vz_5 = float(t.vz_err.iloc[i5])
    return {
        "seed": seed, "bias": bias, "dose_mps": bias * (T_INJ1 - T_INJ0),
        "nav_bias_frames": j.get("nav_bias_frames", 0),
        "pos_rms": j["pos_rms"], "pos_max": j["pos_max"],
        "nav_vel_rms": j["nav_vel_rms"], "att_rmse": j["nav_att_rmse"],
        "rejects": j["outlier_rejects"], "reanchors": j["reacquire_resets"],
        "vz_err_at_inj_end": vz_end, "vz_err_at_t1p5": vz_5,
        "gate_frac_after": gate_frac,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", required=True)
    ap.add_argument("--seeds", default="1,2,3")
    ap.add_argument("--tag", default="2026-10-29")
    ap.add_argument("--bind-core", type=int, default=-1)
    args = ap.parse_args()
    if args.bind_core >= 0:
        import psutil
        psutil.Process().cpu_affinity([args.bind_core])
    seeds = [int(s) for s in args.seeds.split(",") if s]
    # 目录名以 `_traces` 结尾 ⇒ 命中 .gitignore 的 `sim-data/*_traces/`（轨迹可重生成，不入库）
    tag_dir = os.path.join(SD, f"gate_lockout_{args.tag}_traces")
    os.makedirs(tag_dir, exist_ok=True)

    rows = []
    for bias in BIASES:
        for seed in seeds:
            r = run_one(args.exe, seed, bias, tag_dir)
            if r is not None:
                rows.append(r)
            print("done", bias, seed, "ok" if r else "NO-JSON", flush=True)

    verdict = {"exe": args.exe, "thrust_n": THRUST_N, "seeds": seeds,
               "bias_mps2": BIASES, "window_s": [T_INJ0, T_INJ1],
               "d_gate_mps": D_GATE, "rows": rows, "checks": {}}
    for bias in BIASES:
        sub = [r for r in rows if r["bias"] == bias]
        if len(sub) != len(seeds):
            verdict["checks"][f"b{int(bias)}"] = {"I1_ok": False, "n": len(sub)}
            continue
        dose = bias * (T_INJ1 - T_INJ0)
        vig = np.array([abs(r["vz_err_at_inj_end"]) for r in sub])
        gate = np.array([r["gate_frac_after"] for r in sub])
        vz5 = np.array([abs(r["vz_err_at_t1p5"]) for r in sub])
        verdict["checks"][f"b{int(bias)}"] = {
            "I1_ok": all(r["nav_bias_frames"] > 0 for r in sub) if bias > 0 else True,
            "I2_dose_ok": bool(np.all(np.abs(vig - dose) <= 0.35 * max(dose, 1e-9)))
                          if bias > 0 else True,
            "vz_err_at_inj_end": vig.round(3).tolist(),
            "gate_frac_after_5s": gate.round(3).tolist(),
            "vz_err_at_t1p5": vz5.round(3).tolist(),
            "rejects": [r["rejects"] for r in sub],
            "reanchors": [r["reanchors"] for r in sub],
            "pos_rms": [round(r["pos_rms"], 3) for r in sub],
        }
        if bias == 0:
            verdict["checks"]["I4_ok"] = bool(np.all(gate < 0.1) and
                                              all(r["rejects"] <= 3 for r in sub))
        else:
            locked = (gate > 0.5) & ((vz5 > 4.0) | (np.array([r["reanchors"] for r in sub]) > 0))
            recovered = (vz5 < 4.0) & (gate < 0.1) & (np.array([r["reanchors"] for r in sub]) == 0)
            verdict["checks"][f"b{int(bias)}"]["I3_locked"] = locked.astype(int).tolist()
            verdict["checks"][f"b{int(bias)}"]["I5_falsified"] = recovered.astype(int).tolist()

    out_path = os.path.join(SD, f"gate_lockout_inject_{args.tag}.json")
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(verdict, f, ensure_ascii=False, indent=1)
    print(json.dumps(verdict["checks"], ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
