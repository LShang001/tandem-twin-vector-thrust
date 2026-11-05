#!/usr/bin/env python3
"""@file    run_att_aid_bias_mc.py
@module  测试/宿主机
@brief   失联瞬间陀螺零偏阶跃的姿态方案对比（AHRS / 纯陀螺 / 有界兜底）
@frame   NED + 机体系（姿态角与角速率）
@units   deg、deg/s（零偏 dps）、s
@details 回放 `--nav-gyro-bias-step` 故障注入，比较 AHRS（mode 0）/纯陀螺（mode 1）/
         有界兜底（mode 5）在 0.15/0.3/0.6 dps 未标定零偏下的漂移。

用法：
  py -3.12 test_host/fw_sil/run_att_aid_bias_mc.py --bin <fw_real.exe> [--seeds 10]

证据等级：L2（真实固件源码 + SIL real-nav；未上板）。
"""
from __future__ import annotations

import argparse
import json
import statistics
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LEVELS = [0.15, 0.3, 0.6]   # dps，施加在 x/y（倾斜轴）
MODES = [0, 1, 5]
BASE = ["--real-nav", "--profile", "mag_clean", "--pos-hold", "--seconds", "55",
        "--gnss-outage", "20:50"]


def run(bin_path: Path, cwd: str, mode: int, seed: int, level: float):
    cmd = [str(bin_path), *BASE, "--seed", str(seed), "--att-aid-mode", str(mode),
           "--nav-gyro-bias-step", f"20:55:{level}:{level}:0"]
    out = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, timeout=600)
    for line in out.stdout.splitlines():
        if line.startswith("SIL_REAL_JSON "):
            return json.loads(line[len("SIL_REAL_JSON "):])
    return {"_error": out.stdout[-300:] + out.stderr[-300:]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", required=True)
    ap.add_argument("--seeds", type=int, default=10)
    args = ap.parse_args()
    bin_path = Path(args.bin).resolve()
    cwd = str(bin_path.parents[2] / "TandemVec_FCS")

    print(f"{'level':6s} {'mode':4s} {'n':>3s} {'att_med':>8s} {'att_p95':>8s} "
          f"{'att_max':>8s} {'pitch':>7s} {'pos':>8s} {'fuse':>6s}")
    for level in LEVELS:
        for mode in MODES:
            recs = [r for r in (run(bin_path, cwd, mode, s, level)
                                for s in range(1, args.seeds + 1))
                    if "_error" not in r]
            if not recs:
                continue
            att = sorted(r["nav_att_rmse"] for r in recs)
            p95 = att[int(round(0.95 * (len(att) - 1)))]
            print(f"{level:<6.2f} {mode:<4d} {len(recs):3d} "
                  f"{statistics.median(att):8.3f} {p95:8.3f} {max(att):8.3f} "
                  f"{statistics.median(r['nav_pitch_rmse'] for r in recs):7.3f} "
                  f"{statistics.median(r['pos_rms'] for r in recs):8.2f} "
                  f"{statistics.median(r['att_fused'] for r in recs):6.0f}")


if __name__ == "__main__":
    main()
