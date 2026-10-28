# -*- coding: utf-8 -*-
"""GNSS 接收机延迟 × 自动延时估计搜索范围 A/B（导弹 6DOF SIL）。

@file    run_latency_auto_ab.py
@brief   比较默认（0~150 ms 网格 + 100 ms 偏差门限）与放宽（0~300 ms + 300 ms）
         两套编译期参数在 0~350 ms 注入延迟下的位置误差与 τ 估计收敛。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 同一份源码、只差两个编译期宏；绑核 core2；记录各自 md5。
         判据：(1) 延迟 >150 ms 时默认臂是否因网格上界而欠补偿；
         (2) 放宽臂是否恢复到"编译期匹配延迟"的 ~9 m 水平；
         (3) 0 延迟臂是否因放宽而误拟合出假 τ（回归风险）。
@frame   NED
@units   m / ms

用法：
  py -3.12 sim/run_latency_auto_ab.py --exe-def /tmp/tv_dly_def/fw_real.exe \
      --exe-wide /tmp/tv_dly_wide/fw_real.exe --seeds 20
"""
import argparse
import concurrent.futures as cf
import hashlib
import json
import math
import os
import re
import subprocess

import psutil

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(os.path.dirname(HERE), "sim-data")
JSON_RE = re.compile(r"SIL_REAL_JSON (\{.*\})")
CORE = 2

BASE = ["--real-nav", "--missile", "--seconds", "45", "--gnss-rate-hz", "10",
        "--profile", "mag_clean", "--imu-grade", "icm"]
LATENCIES = [0, 50, 100, 150, 200, 250, 300, 350]
KEYS = ["pos_rms", "pos_max", "nav_att_rmse", "nav_vel_rms", "ad_tau_ms",
        "ad_sharp", "ad_valid", "outlier_rejects", "reacquire_resets",
        "gnss_age"]


def run_one(exe, lat, seed, log):
    cmd = [exe] + BASE + ["--seed", str(seed), "--gnss-latency-ms", str(lat)]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                         text=True, env=os.environ.copy())
    try:
        if hasattr(psutil.Process(p.pid), "cpu_affinity"):
            p.cpu_affinity([CORE])
    except Exception:
        pass
    out, err = p.communicate(timeout=900)
    m = JSON_RE.findall(out + err)
    if not m:
        log.write(f"[lat={lat} seed={seed}] NO JSON rc={p.returncode}\n")
        return None
    d = json.loads(m[-1].replace("nan", "NaN").replace("inf", "Infinity"))
    d["lat"] = lat
    d["seed"] = seed
    return d


def stat(rows, key):
    vals = [r.get(key) for r in rows
            if isinstance(r.get(key), (int, float))
            and math.isfinite(r.get(key, float("nan")))]
    if not vals:
        return float("nan"), float("nan"), 0
    vals.sort()
    return vals[len(vals) // 2], vals[min(len(vals) - 1, int(0.95 * len(vals)))], \
        len(vals)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", action="append", default=[],
                    help="标签=可执行文件路径，可重复；替代 --exe-def/--exe-wide")
    ap.add_argument("--exe-def", default=None)
    ap.add_argument("--exe-wide", default=None)
    ap.add_argument("--latencies", default=None,
                    help="逗号分隔的注入延迟档 [ms]，默认脚本内置 8 档")
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--out-tag", default="2026-10-27")
    args = ap.parse_args()

    spec = args.arm
    if args.exe_def:
        spec = [f"默认150ms={args.exe_def}"] + spec
    if args.exe_wide:
        spec = spec + [f"放宽300ms={args.exe_wide}"]
    if not spec:
        ap.error("至少给一个 --arm 标签=二进制")
    lats = ([int(x) for x in args.latencies.split(",")]
            if args.latencies else LATENCIES)

    arms = []
    for item in spec:
        name, exe = item.split("=", 1)
        arms.append((name, exe,
                     hashlib.md5(open(exe, "rb").read()).hexdigest()))
    seeds = list(range(1, args.seeds + 1))
    jobs = [(name, exe, lat, s) for name, exe, _ in arms
            for lat in lats for s in seeds]
    out_jsonl = os.path.join(DATA, f"latency_auto_{args.out_tag}.jsonl")
    open(out_jsonl, "w", encoding="utf-8").close()
    rows = []
    with open(out_jsonl + ".log", "w", encoding="utf-8") as log:
        for name, exe, md5 in arms:
            log.write(f"arm={name} exe={exe} md5={md5}\n")
        log.flush()
        with cf.ThreadPoolExecutor(max_workers=args.workers) as pool:
            futs = {pool.submit(run_one, exe, lat, s, log): (n, lat, s)
                    for (n, exe, lat, s) in jobs}
            for f in cf.as_completed(futs):
                d = f.result()
                if d:
                    d["arm"] = futs[f][0]
                    rows.append(d)
                    with open(out_jsonl, "a", encoding="utf-8") as fh:
                        fh.write(json.dumps(d, ensure_ascii=False) + "\n")

    print(f"{'注入延迟':>9} {'臂':<14}{'n':>4}{'pos_rms':>10}{'p95':>8}"
          f"{'pos_max':>9}{'att':>7}{'τ中位ms':>9}{'τp95ms':>9}{'valid':>7}{'拒帧':>7}"
          f"{'剔除':>7}")
    print("-" * 100)
    summary = []
    for lat in lats:
        for name, _exe, _md5 in arms:
            sel = [r for r in rows if r["lat"] == lat and r["arm"] == name]
            if not sel:
                continue
            pm, pp, n = stat(sel, "pos_rms")
            px, _, _ = stat(sel, "pos_max")
            at, _, _ = stat(sel, "nav_att_rmse")
            tm, tp, _ = stat(sel, "ad_tau_ms")
            nv = sum(1 for r in sel if r.get("ad_valid", 0))
            rj = sum(r.get("outlier_rejects", 0) for r in sel)
            er = sum(r.get("ad_er", 0) for r in sel)
            print(f"{lat:>9} {name:<14}{n:>4}{pm:>10.2f}{pp:>8.2f}{px:>9.2f}"
                  f"{at:>7.2f}{tm:>9.1f}{tp:>9.1f}{nv:>7}{rj:>7}{er:>7}")
            summary.append({"lat": lat, "arm": name, "n": n,
                            "pos_rms_med": round(pm, 3),
                            "pos_rms_p95": round(pp, 3),
                            "pos_max_med": round(px, 3),
                            "att_med": round(at, 3),
                            "tau_med_ms": round(tm, 1),
                            "tau_p95_ms": round(tp, 1),
                            "ad_valid_n": nv, "rejects": rj, "erased": er})
    out_json = os.path.join(DATA, f"latency_auto_{args.out_tag}.json")
    with open(out_json, "w", encoding="utf-8") as fh:
        json.dump({"arms": {n: m for n, _e, m in arms}, "seeds": args.seeds,
                   "latencies": lats, "rows": summary},
                  fh, ensure_ascii=False, indent=1)
    print("json:", out_json)


if __name__ == "__main__":
    main()
