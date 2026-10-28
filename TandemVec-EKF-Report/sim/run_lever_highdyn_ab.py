# -*- coding: utf-8 -*-
"""高动态宿主台架上的 GNSS 杆臂 × 速度子集 / 持久重锚 交互 A/B。
@file    run_lever_highdyn_ab.py
@brief   同一台架、不同 lever/补偿/子集配置下比较位置/速度/姿态 RMSE。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施，宿主台架非固件闭环）
@details 用 --lever-m 2 --lever-comp 2 打开 EKF 侧杆臂补偿；--vel-subset 1 打开
         速度子集软恢复。长失联段触发持久重锚，用于量化"重锚未扣杆臂"的后果。
@frame   NED
@units   m / m/s / deg

用法：py -3.12 sim/run_lever_highdyn_ab.py --exe /tmp/host_bin_lev/highdyn.exe
"""
import argparse
import concurrent.futures as cf
import hashlib
import json
import os
import re
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(os.path.dirname(HERE), "sim-data")

ARMS = [
    ("名义无杆臂", ["--lever-m", "0", "--lever-comp", "0"]),
    ("杆臂2m_不补偿", ["--lever-m", "2", "--lever-comp", "0"]),
    ("杆臂2m_补偿", ["--lever-m", "2", "--lever-comp", "2"]),
    ("杆臂2m_补偿_子集", ["--lever-m", "2", "--lever-comp", "2",
                          "--vel-subset", "1"]),
    ("杆臂2m_不补偿_子集", ["--lever-m", "2", "--lever-comp", "0",
                            "--vel-subset", "1"]),
]
BASE = ["--profile", "turn", "--load", "8", "--speed", "80", "--duration", "40",
        "--latency-ms", "20", "--accel-fs-g", "16", "--imu-rate", "400"]
OUTAGE = ["--outage", "10:30"]


def run(exe, extra, seed):
    cmd = [exe] + BASE + OUTAGE + ["--seed", str(seed), "--json"] + extra
    out = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    txt = out.stdout + out.stderr
    m = re.findall(r"\{.*\}", txt)
    if not m:
        return None
    try:
        d = json.loads(m[-1])
    except json.JSONDecodeError:
        return None
    d["seed"] = seed
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", required=True)
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--workers", type=int, default=10)
    ap.add_argument("--out-tag", default="2026-10-27")
    ap.add_argument("--extra", default="",
                    help="追加到基座参数（如 --profile roll --rate-dps 600）")
    ap.add_argument("--no-outage", action="store_true")
    args = ap.parse_args()
    global BASE, OUTAGE
    if args.extra:
        BASE = BASE + args.extra.split()
    if args.no_outage:
        OUTAGE = []
    md5 = hashlib.md5(open(args.exe, "rb").read()).hexdigest()
    jobs = [(name, extra, s) for name, extra in ARMS
            for s in range(1, args.seeds + 1)]
    res = {}
    with cf.ThreadPoolExecutor(max_workers=args.workers) as pool:
        futs = {pool.submit(run, args.exe, e, s): (n, s) for n, e, s in jobs}
        for f in cf.as_completed(futs):
            n, s = futs[f]
            d = f.result()
            if d:
                res.setdefault(n, []).append(d)

    print(f"exe={args.exe}\nmd5={md5}")
    print(f"{'臂':<22}{'n':>4}{'pos_rmse':>11}{'pos_max':>10}{'vel':>9}"
          f"{'att_deg':>10}{'fused':>8}{'rej':>7}{'fail':>6}")
    print("-" * 88)
    rows = []
    for name, _ in ARMS:
        sel = res.get(name, [])
        if not sel:
            continue
        g = lambda k: sum(r.get(k, 0.0) for r in sel) / len(sel)
        mx = max(r.get("pos_max", 0.0) for r in sel)
        fail = sum(1 for r in sel if r.get("fail", 0))
        print(f"{name:<22}{len(sel):>4}{g('pos_rmse'):>11.3f}{mx:>10.2f}"
              f"{g('vel_rmse'):>9.3f}{g('att_rmse_deg'):>10.3f}"
              f"{g('fused'):>8.0f}{g('rejected'):>7.0f}{fail:>6}")
        rows.append({"arm": name, "n": len(sel),
                     "pos_rmse": round(g("pos_rmse"), 4),
                     "pos_max": round(mx, 3),
                     "vel_rmse": round(g("vel_rmse"), 4),
                     "att_rmse_deg": round(g("att_rmse_deg"), 4),
                     "att_max_deg": round(g("att_max_deg"), 4),
                     "nees_pos_med": round(g("nees_pos_med"), 4),
                     "fused": round(g("fused"), 1),
                     "rejected": round(g("rejected"), 1), "fail": fail})
    out_json = os.path.join(DATA, f"lever_highdyn_{args.out_tag}.json")
    with open(out_json, "w", encoding="utf-8") as fh:
        json.dump({"exe": args.exe, "md5": md5, "base": BASE + OUTAGE,
                   "seeds": args.seeds, "arms": rows}, fh,
                  ensure_ascii=False, indent=1)
    print("json:", out_json)


if __name__ == "__main__":
    main()
