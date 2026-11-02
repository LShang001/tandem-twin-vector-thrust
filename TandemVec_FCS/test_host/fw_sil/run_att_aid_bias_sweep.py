#!/usr/bin/env python3
"""run_att_aid_bias_sweep.py — 陀螺零偏阶跃的时机/方向/持续性扫描。

在前一版（固定 20 s、双轴、永久阶跃）基础上扩展，回答三个工程问题：
  1. 阶跃发生在 GNSS 有效期内（可被 EKF 在线估计）vs 失联瞬间（不可观测）
     对姿态漂移的影响差多少；
  2. 单轴（滚转/俯仰/航向）与双轴阶跃的差别（航向有磁观测）；
  3. 永久 vs 瞬时（热漂恢复）阶跃；叠加风/振动后的表现。

用法：
  py -3.12 test_host/fw_sil/run_att_aid_bias_sweep.py --bin <fw_real.exe> [--seeds 10]

证据等级：L2（真实固件源码 + SIL real-nav；未上板）。
"""
from __future__ import annotations

import argparse
import json
import statistics
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

# name -> (bias step args 或 None, 附加环境参数)
CONDITIONS = {
    "step05_xy":  ("5:55:0.3:0.3:0", []),        # 阶跃在 GNSS 有效期内（可估计）
    "step20_x":   ("20:55:0.3:0:0", []),          # 失联瞬间，单轴滚转
    "step20_y":   ("20:55:0:0.3:0", []),          # 失联瞬间，单轴俯仰
    "step20_xy":  ("20:55:0.3:0.3:0", []),        # 失联瞬间，双轴
    "step20_z":   ("20:55:0:0:0.3", []),          # 失联瞬间，航向轴（磁可观测）
    "step20_xy_t": ("20:35:0.3:0.3:0", []),       # 30 s 失联内只漂 15 s（热漂恢复）
    "step20_xy_wind": ("20:55:0.3:0.3:0",
                       ["--wind-force", "1.5", "--wind-window", "20:50"]),
    "step20_xy_vib": ("20:55:0.3:0.3:0", ["--vib", "1.0"]),
}
MODES = [1, 5]
BASE = ["--real-nav", "--profile", "mag_clean", "--pos-hold", "--seconds", "55",
        "--gnss-outage", "20:50"]


def run(bin_path: Path, cwd: str, mode: int, seed: int, cond: str):
    bias, extra = CONDITIONS[cond]
    cmd = [str(bin_path), *BASE, "--seed", str(seed), "--att-aid-mode", str(mode),
           "--nav-gyro-bias-step", bias, *extra]
    out = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, timeout=600)
    for line in out.stdout.splitlines():
        if line.startswith("SIL_REAL_JSON "):
            return json.loads(line[len("SIL_REAL_JSON "):])
    return {"_error": out.stdout[-300:] + out.stderr[-300:]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", required=True)
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--out", default="", help="可选：逐 seed JSON 输出路径")
    args = ap.parse_args()
    bin_path = Path(args.bin).resolve()
    cwd = str(bin_path.parents[2] / "TandemVec_FCS")

    print(f"{'cond':16s} {'mode':4s} {'n':>3s} {'att_med':>8s} {'att_max':>8s} "
          f"{'pitch':>7s} {'yaw':>7s} {'pos':>8s} {'fuse':>6s}")
    all_recs = []
    for cond in CONDITIONS:
        for mode in MODES:
            recs = [r for r in (run(bin_path, cwd, mode, s, cond)
                                for s in range(1, args.seeds + 1))
                    if "_error" not in r]
            for r in recs:
                r["_cond"] = cond
                r["_mode"] = mode
            all_recs.extend(recs)
            if not recs:
                continue
            att = sorted(r["nav_att_rmse"] for r in recs)
            print(f"{cond:16s} {mode:<4d} {len(recs):3d} "
                  f"{statistics.median(att):8.3f} {max(att):8.3f} "
                  f"{statistics.median(r['nav_pitch_rmse'] for r in recs):7.3f} "
                  f"{statistics.median(r['nav_yaw_rmse'] for r in recs):7.3f} "
                  f"{statistics.median(r['pos_rms'] for r in recs):8.2f} "
                  f"{statistics.median(r['att_fused'] for r in recs):6.0f}")
    if args.out:
        out = Path(args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(all_recs, ensure_ascii=False), encoding="utf-8")
        print(f"[out] {out}")


if __name__ == "__main__":
    main()
