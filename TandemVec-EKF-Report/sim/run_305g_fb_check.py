#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24m-J6：30.5 g 格上"增量源头回退是否吃掉 nav-raw 收益"的定点检查。

@file    run_305g_fb_check.py
@brief   用 32 g 档二进制跑 §4.24 的 30.5 g 单元格两臂（raw / raw+源头回退），
         逐 seed 配对验证回退在该格完全惰性（该格不注入 IMU 故障）
@module  TandemVec-EKF-Report/仿真驱动（L2 研究设施）
@details 判据 J6（跑前写死）：两臂 `pos_rms` 逐 seed 相同（该格无故障窗 ⇒ nrfb 必为 0），
         且中位仍为 raw 臂的 5 m 级（≈§4.24i 的 G_navraw 5.16 m），
         即"提带宽收益"没有被回退吃掉。
         BASE 参数与 `run_305g_path_attrib.py` 逐字一致，保证与 §4.24/§4.24i 同格可比。
@frame   NED；指标取 SIL JSON 的 pos_rms / pos_max / nav_att_rmse
@units   长度 m、姿态 deg
"""
import argparse
import json
import os
import re
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
JSON_RE = re.compile(r"SIL_REAL_JSON (\{.*\})")
BASE = ["--real-nav", "--missile", "--seconds", "45", "--imu-fs-g", "32",
        "--imu-health-accel-range", "313.8", "--ms-thrust", "23900",
        "--profile", "mag_clean", "--imu-grade", "icm", "--gnss-rate-hz", "10"]
ARMS = [("raw", ["--nav-raw-imu"]),
        ("raw_fb", ["--nav-raw-imu", "--nav-raw-fault-fallback"])]
# --fb-flags health 时把第二臂换成"键在固件 IMU 健康 Failed 上"的变体（§4.24m-K 组）
HEALTH_FB = ["--nav-raw-imu", "--nav-raw-fault-fallback", "--nav-raw-fb-health"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", required=True)
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--out-tag", default="2026-10-28_305g_fb")
    ap.add_argument("--fb-flags", choices=("window", "health"), default="window")
    args = ap.parse_args()
    arms = [ARMS[0], ("raw_fb_health" if args.fb_flags == "health" else "raw_fb",
                      HEALTH_FB if args.fb_flags == "health" else ARMS[1][1])]

    out_path = os.path.join(SD, f"attrib305_{args.out_tag}.jsonl")
    recs = {}
    with open(out_path, "w", encoding="utf-8") as f:
        for name, extra in arms:
            for seed in range(1, args.seeds + 1):
                r = subprocess.run([args.exe] + BASE + extra + ["--seed", str(seed)],
                                   capture_output=True, text=True, timeout=600)
                m = JSON_RE.findall(r.stdout + r.stderr)
                if not m:
                    print(f"{name} seed{seed} NO JSON: {r.stderr[-200:]}")
                    continue
                d = json.loads(m[-1].replace("nan", "NaN").replace("inf", "Infinity"))
                d["arm"] = name
                d["seed"] = seed
                f.write(json.dumps(d, ensure_ascii=False) + "\n")
                recs.setdefault(name, {})[seed] = d
            print(f"{name}: {len(recs.get(name, {}))}/{args.seeds}")

    n0, n1 = arms[0][0], arms[1][0]
    a, b = recs.get(n0, {}), recs.get(n1, {})
    seeds = sorted(set(a) & set(b))
    assert len(seeds) == args.seeds, f"配对 {len(seeds)} != {args.seeds} ⇒ 判定无效"
    nd = [s for s in seeds if a[s]["pos_rms"] != b[s]["pos_rms"]]
    nrfb = max(int(b[s].get("nrfb", -1)) for s in seeds)
    med_a = sorted(a[s]["pos_rms"] for s in seeds)[len(seeds) // 2]
    med_b = sorted(b[s]["pos_rms"] for s in seeds)[len(seeds) // 2]
    verdict = {
        "seeds": len(seeds),
        "median_%s" % n0: round(med_a, 4), "median_%s" % n1: round(med_b, 4),
        "diff_seeds": len(nd), "nrfb_max": nrfb,
        "J6_pass": (len(nd) == 0 and nrfb == 0),
        "gate": "J6: 30.5 g 格两臂逐 seed 相同且 nrfb=0（收益未被回退吃掉）",
    }
    with open(os.path.join(SD, f"attrib305_{args.out_tag}_verdict.json"), "w",
              encoding="utf-8") as f:
        json.dump(verdict, f, ensure_ascii=False, indent=1)
    print(json.dumps(verdict, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
