# -*- coding: utf-8 -*-
"""共轴 SIL 场景矩阵：传感器输入/控制指令/plant 响应。

@file    run_coax_sil_campaign.py
@module  FCS/验证工具（共轴 SIL 场景矩阵）
@brief   批量运行 fw_coax 的传感器/指令/plant 场景并收集 SIL_REAL_JSON
@details 输入：fw_coax.exe 与场景表；处理：subprocess 运行、解析 JSON 指标；
         输出：tmp/coax_sil_campaign.json 与控制台表。
@theory  N/A（验证工具）
@ref     test_host/fw_sil/README.md、docs/07-验证与确认/VER-007-共轴SIL场景矩阵-2026-11-05.md
@frame   N/A
@units   指标单位随 SIL_REAL_JSON 字段（m/deg/m/s）
"""
import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BIN = os.environ.get(
    "COAX_SIL_BIN",
    r"C:\Users\12631\AppData\Local\Temp\tv_fcs_fw_sil_coax\fw_coax.exe",
)

BASE = ["--real-nav", "--coax-plant-model", "rigorous", "--seconds", "30"]
SENSORS = [
    ("mag_clean_poshold", ["--profile", "mag_clean", "--pos-hold"]),
    ("mag_nominal_poshold", ["--profile", "mag_nominal", "--pos-hold"]),
    ("mag_disturbed", ["--profile", "mag_disturbed", "--pos-hold"]),
    ("nomag_degraded", ["--profile", "nomag_degraded", "--pos-hold"]),
    ("gnss_outage", ["--profile", "mag_clean", "--pos-hold",
                     "--gnss-outage", "15:22"]),
    ("vib_2g_120hz", ["--profile", "mag_clean", "--pos-hold",
                      "--vib", "2.0", "--vib-hz", "120"]),
    ("baro_noise_drift", ["--profile", "mag_clean", "--pos-hold",
                          "--baro-noise", "1.0", "--baro-drift", "0.05"]),
    ("imu_bias_step", ["--profile", "mag_clean", "--pos-hold",
                       "--nav-accel-bias-step", "8:12:0.2:0:0",
                       "--nav-gyro-bias-step", "8:12:0:0:0.01"]),
    ("imu_nan", ["--profile", "mag_clean", "--pos-hold",
                 "--imu-fault", "nan", "--imu-fault-window", "8:12"]),
    ("imu_stuck", ["--profile", "mag_clean", "--pos-hold",
                   "--imu-fault", "stuck", "--imu-fault-window", "8:12"]),
    ("imu_dropout", ["--profile", "mag_clean", "--pos-hold",
                     "--imu-dropout", "8:12"]),
    ("gnss_spoof", ["--profile", "mag_clean", "--pos-hold",
                    "--gnss-spoof", "5", "--gnss-spoof-start", "10"]),
    ("rc_loss", ["--profile", "mag_clean", "--pos-hold",
                 "--rc-loss", "8:12"]),
]
COMMANDS = [
    ("manual_attitude_step", ["--profile", "mag_clean", "--manual"]),
    ("fwd_stick_poshold", ["--profile", "mag_clean", "--pos-hold",
                            "--fwd-stick", "10:12"]),
]
PLANT = [
    ("thrust_scale_0p8", ["--profile", "mag_clean", "--pos-hold",
                          "--coax-thrust-scale", "0.8"]),
    ("inertia_scale_2", ["--profile", "mag_clean", "--pos-hold",
                         "--coax-inertia-scale", "2.0"]),
    ("yaw_scale_2", ["--profile", "mag_clean", "--pos-hold",
                     "--coax-yaw-scale", "2.0"]),
    ("mass_scale_1p2", ["--profile", "mag_clean", "--pos-hold",
                        "--coax-mass", "1.2"]),
]


def run(name, args, seed):
    cmd = [BIN] + BASE + args + ["--seed", str(seed)]
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    line = None
    for ln in p.stdout.splitlines():
        if ln.startswith("SIL_REAL_JSON "):
            line = ln[len("SIL_REAL_JSON "):]
    if line is None:
        return {"scenario": name, "seed": seed, "rc": p.returncode,
                "error": (p.stderr or p.stdout)[-300:]}
    d = json.loads(line)
    d.update({"scenario": name, "seed": seed, "rc": p.returncode})
    return d


def main():
    rows = []
    for group, cases in (("sensor", SENSORS), ("command", COMMANDS),
                         ("plant", PLANT)):
        for name, args in cases:
            for seed in (1, 2, 3):
                r = run(name, args, seed)
                r["group"] = group
                rows.append(r)
                print(f"{name:24s} seed{seed} rc={r.get('rc')} "
                      f"alt={r.get('final_alt')} roll={r.get('max_roll')} "
                      f"pitch={r.get('max_pitch')} posmax={r.get('pos_max')} "
                      f"navinit={r.get('nav_init')} err={r.get('error','')}")
    out = os.path.join(ROOT, "tmp", "coax_sil_campaign.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(rows, f, ensure_ascii=False, indent=2)
    print("saved", out)


if __name__ == "__main__":
    main()
