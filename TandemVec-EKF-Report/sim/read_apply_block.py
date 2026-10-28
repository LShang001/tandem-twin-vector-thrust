# -*- coding: utf-8 -*-
"""应用侧可观测读数（通用多臂）：自动延时估计有没有被应用、应用了多少、值多少。

@file    read_apply_block.py
@brief   对若干 SIL 二进制 × 若干注入延迟档跑 N seeds，汇总 pos_rms / ad_app / ad_blk
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 关键指标是 ad_app（估计值真正替换固定先验的历元数）——上一轮四臂 A/B 里
         base 臂 ad_app=0 却报 ad_valid=1，说明"锁存标志"不能当"在用"证据。

用法：
  py -3.12 sim/read_apply_block.py --arm full=C:/.../fw_real.exe \
      --arm novib=C:/.../fw_real.exe --lat 200 --seeds 20 --tag 2026-10-27_vibveto
"""
import argparse
import collections
import concurrent.futures as cf
import hashlib
import json
import os
import re
import subprocess

import psutil

ARGS = ["--real-nav", "--missile", "--seconds", "45", "--gnss-rate-hz", "10",
        "--profile", "mag_clean", "--imu-grade", "icm"]
JRE = re.compile(r"SIL_REAL_JSON (\{.*\})")
CORE = 2
HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(os.path.dirname(HERE), "sim-data")


def run(exe, seed, lat):
    cmd = [exe] + ARGS + ["--seed", str(seed), "--gnss-latency-ms", str(lat)]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                         text=True, env=os.environ.copy())
    try:
        if hasattr(psutil.Process(p.pid), "cpu_affinity"):
            p.cpu_affinity([CORE])
    except Exception:
        pass
    out, err = p.communicate(timeout=900)
    m = JRE.findall((out or "") + (err or ""))
    if not m:
        return None
    d = json.loads(m[-1].replace("nan", "NaN").replace("inf", "Infinity"))
    d["seed"] = seed
    return d


def med(vals):
    v = sorted(vals)
    return v[len(v) // 2] if v else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", action="append", required=True)
    ap.add_argument("--lat", type=int, action="append", default=[200])
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--workers", type=int, default=10)
    ap.add_argument("--tag", default="2026-10-27")
    args = ap.parse_args()

    arms = []
    for item in args.arm:
        name, exe = item.split("=", 1)
        arms.append((name, exe, hashlib.md5(open(exe, "rb").read()).hexdigest()))
    seeds = list(range(1, args.seeds + 1))
    all_rows = {}
    print(f"{'延迟':>6} {'臂':<10}{'n':>4}{'pos_rms中位':>13}{'τ̂中位':>9}"
          f"{'app中位':>9}{'app=0数':>9}  blk分布")
    print("-" * 88)
    for lat in args.lat:
        for name, exe, md5 in arms:
            with cf.ThreadPoolExecutor(max_workers=args.workers) as pool:
                rows = [r for r in pool.map(lambda s: run(exe, s, lat), seeds) if r]
            if not rows:
                print(f"{lat:>6} {name:<10}   0  无输出")
                continue
            all_rows[f"{lat}:{name}"] = rows
            app = [r.get("ad_app", -1) for r in rows]
            print(f"{lat:>6} {name:<10}{len(rows):>4}{med([r['pos_rms'] for r in rows]):>13.2f}"
                  f"{med([r.get('ad_tau_ms', -1) for r in rows]):>9.1f}"
                  f"{med(app):>9.0f}{sum(1 for a in app if a == 0):>9}  "
                  f"{dict(collections.Counter(r.get('ad_blk', -1) for r in rows))}")
    out = os.path.join(DATA, f"apply_block_{args.tag}.json")
    with open(out, "w", encoding="utf-8") as fh:
        json.dump({"arms": {n: m for n, _e, m in arms}, "lats": args.lat,
                   "rows": {k: v for k, v in all_rows.items()}},
                  fh, ensure_ascii=False)
    print("json:", out)


if __name__ == "__main__":
    main()
