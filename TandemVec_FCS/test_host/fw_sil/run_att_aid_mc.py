#!/usr/bin/env python3
"""run_att_aid_mc.py — 无 GNSS 运动段姿态辅助量测源的大规模蒙特卡洛。

对比三种 `g_att_aid_mode`（0=AHRS 姿态量测、1=重力方向直连、2=重力优先+AHRS 回退），
在 GNSS 失联/风/振动/失联时长等条线下跑同一 SIL 二进制（real-nav，VTOL mag_clean），
输出逐 seed JSON 与汇总统计。

用法：
  py -3.12 test_host/fw_sil/run_att_aid_mc.py --bin <fw_real.exe> \
      --out <json> [--jobs 16] [--seeds 30]

证据等级：L2（真实固件源码 + SIL real-nav；未上板）。
"""
from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import statistics
import subprocess
import sys
from pathlib import Path

# 条线：name -> (extra args, seconds, seeds)
# 注：mag_clean profile 的 wind_force 默认 0，`--no-wind` 对它无效果；
# 需要风时必须显式 --wind-force（N），并用 --wind-window 覆盖失联窗。
CONDITIONS = {
    "clean":        (["--pos-hold"], 40.0, 20),
    "out5":         (["--pos-hold", "--gnss-outage", "20:25"], 40.0, 50),
    "out10":        (["--pos-hold", "--gnss-outage", "20:30"], 40.0, 100),
    "out10_wind":   (["--pos-hold", "--gnss-outage", "20:30",
                      "--wind-force", "1.5", "--wind-window", "20:30"], 40.0, 50),
    "out10_vib1g":  (["--pos-hold", "--gnss-outage", "20:30",
                      "--vib", "1.0"], 40.0, 50),
    "out10_noph":   (["--gnss-outage", "20:30"], 40.0, 50),
    "out20":        (["--pos-hold", "--gnss-outage", "20:40"], 55.0, 50),
    # 长失联 + 理想光流：验证“外部速度参考是唯一让倾斜可观的物理途径”
    "out20_flow":   (["--pos-hold", "--gnss-outage", "20:40", "--flow"], 55.0, 20),
}
MODES = (0, 1, 2)
FIELDS = [
    "nav_att_rmse", "nav_roll_rmse", "nav_pitch_rmse", "nav_yaw_rmse",
    "nav_att_max", "nav_roll_max", "nav_pitch_max", "nav_yaw_max",
    "pos_rms", "pos_max", "max_roll", "max_pitch",
    "att_att", "att_fused", "nav_init", "final_alt", "vel_rms", "nav_vel_rms",
]


def _run(job):
    bin_path, cwd, mode, cond_name, seed = job
    extra, seconds, _ = CONDITIONS[cond_name]
    cmd = [str(bin_path), "--real-nav", "--profile", "mag_clean",
           "--seed", str(seed), "--seconds", str(seconds),
           "--att-aid-mode", str(mode), *extra]
    out = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, timeout=600)
    for line in out.stdout.splitlines():
        if line.startswith("SIL_REAL_JSON "):
            rec = json.loads(line[len("SIL_REAL_JSON "):])
            rec["_mode"] = mode
            rec["_cond"] = cond_name
            rec["_seed"] = seed
            return rec
    return {"_mode": mode, "_cond": cond_name, "_seed": seed, "_error":
            (out.stdout[-800:] + out.stderr[-800:])}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--jobs", type=int, default=16)
    ap.add_argument("--only", default="", help="逗号分隔的条件名（默认全部）")
    ap.add_argument("--modes", default="0,1,2", help="逗号分隔的模式（默认 0,1,2）")
    args = ap.parse_args()

    bin_path = Path(args.bin).resolve()
    cwd = str(bin_path.parents[2] / "TandemVec_FCS")
    modes = tuple(int(x) for x in args.modes.split(",") if x != "")
    jobs = []
    conds = [c for c in CONDITIONS if (not args.only or c in args.only.split(","))]
    for cond in conds:
        _, _, n = CONDITIONS[cond]
        for mode in modes:
            for seed in range(1, n + 1):
                jobs.append((bin_path, cwd, mode, cond, seed))

    print(f"[mc] {len(jobs)} runs, {args.jobs} workers", file=sys.stderr)
    recs = []
    with mp.Pool(args.jobs) as pool:
        for i, rec in enumerate(pool.imap_unordered(_run, jobs), 1):
            recs.append(rec)
            if i % 30 == 0:
                print(f"[mc] {i}/{len(jobs)}", file=sys.stderr)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(recs, ensure_ascii=False), encoding="utf-8")

    # 汇总：中位数 + p95 + max
    print(f"{'cond':13s} {'mode':4s} {'n':>3s} " +
          " ".join(f"{f:>14s}" for f in
                   ("att_med", "att_p95", "att_max", "pitch_med", "yaw_med",
                    "pos_med", "pos_p95", "fuse")))
    for cond in conds:
        for mode in modes:
            rs = [r for r in recs
                  if r["_cond"] == cond and r["_mode"] == mode and "_error" not in r]
            if not rs:
                continue
            def stat(key, fn=statistics.median):
                xs = [r[key] for r in rs if isinstance(r.get(key), (int, float))]
                return fn(xs) if xs else float("nan")
            atts = sorted(r["nav_att_rmse"] for r in rs
                          if isinstance(r.get("nav_att_rmse"), (int, float)))
            p95 = atts[int(round(0.95 * (len(atts) - 1)))] if atts else float("nan")
            pos = sorted(r["pos_rms"] for r in rs
                         if isinstance(r.get("pos_rms"), (int, float)))
            pos95 = pos[int(round(0.95 * (len(pos) - 1)))] if pos else float("nan")
            print(f"{cond:13s} {mode:<4d} {len(rs):3d} "
                  f"{stat('nav_att_rmse'):14.3f} {p95:14.3f} "
                  f"{max(atts) if atts else float('nan'):14.3f} "
                  f"{stat('nav_pitch_rmse'):14.3f} {stat('nav_yaw_rmse'):14.3f} "
                  f"{stat('pos_rms'):14.3f} {pos95:14.3f} "
                  f"{stat('att_fused', sum):14.1f}")


if __name__ == "__main__":
    main()
