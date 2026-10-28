# -*- coding: utf-8 -*-
"""弹载飞控物理减震座 SIL 扫描：激励频率 × 固有频率 × 阻尼比。

@file    run_vib_isolator_sweep.py
@brief  用 SIL 减震座传递率模型评估机械隔振对 IMU/导航的影响。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 只注入 IMU/GNSS；弹体 6.4 g 标称弹道；单频正弦振动。
         输出 sim-data/vib-iso-<tag>.{json,jsonl}。
@frame   NED；姿态 ZYX 欧拉角。
@units   g、Hz、°、m、m/s。
"""
import argparse
import concurrent.futures as cf
import hashlib
import json
import os
import re
import subprocess

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
JSON_RE = re.compile(r"SIL_REAL_JSON (\{.*\})")
BASE = ["--real-nav", "--missile", "--seconds", "45", "--profile",
        "mag_clean", "--imu-grade", "icm"]

FREQS = [20.0, 30.0, 40.0, 50.0, 60.0, 80.0, 100.0, 120.0, 150.0]
MOUNTS = [0.0, 15.0, 20.0, 25.0, 30.0, 40.0, 50.0, 60.0, 80.0]  # 0 = 未装
ZETAS = [0.05, 0.10, 0.15, 0.25, 0.40]


def resolve_bin(path):
    if os.path.exists(path):
        return path
    if path.startswith("/tmp/"):
        alt = os.path.join(os.environ.get("TEMP", ""), path[5:])
        if os.path.exists(alt):
            return alt
    return path


def md5(path):
    h = hashlib.md5()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def one(job):
    name, exe, seed, vib_g, vib_hz, fn, zeta = job
    cmd = [exe] + BASE + ["--seed", str(seed), "--vib", f"{vib_g:.3f}",
                          "--vib-hz", f"{vib_hz:.1f}", "--vib-start", "1.5"]
    if fn > 0:
        cmd += ["--vib-iso-fn", f"{fn:.1f}", "--vib-iso-zeta", f"{zeta:.3f}"]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
    except subprocess.TimeoutExpired:
        return dict(config=name, seed=seed, no_json=True, timeout=True)
    mm = JSON_RE.findall(r.stdout + r.stderr)
    if not mm:
        return dict(config=name, seed=seed, no_json=True,
                    stderr=r.stderr[-300:])
    d = json.loads(mm[-1].replace("nan", "NaN").replace("inf", "Infinity"))
    d.update(config=name, seed=seed, vib_g=vib_g, vib_hz=vib_hz, iso_fn=fn,
             iso_zeta=zeta)
    return d


def stat(v):
    v = np.array([x for x in v if isinstance(x, (int, float)) and np.isfinite(x)],
                 float)
    if not v.size:
        return dict(n=0)
    return dict(n=int(len(v)), med=float(np.median(v)),
                p95=float(np.percentile(v, 95)), mx=float(np.max(v)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", default="/tmp/tv_fcs_missile_prod_fresh_iso/fw_real.exe")
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--seeds-2g", type=int, default=20)
    ap.add_argument("--seeds-zeta", type=int, default=20)
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--tag", default="2026-10-27")
    args = ap.parse_args()

    exe = resolve_bin(args.bin)
    jobs = []
    # 1 g 单频扫频 × 不同固有频率
    for f in FREQS:
        for fn in MOUNTS:
            for s in range(1, args.seeds + 1):
                jobs.append((f"f{f:03.0f}_fn{fn:03.0f}_z015", exe, s, 1.0, f,
                             fn, 0.15))
    # 2 g / 120 Hz × 不同固有频率
    for fn in [0.0, 20.0, 30.0, 40.0, 50.0, 60.0]:
        for s in range(1, args.seeds_2g + 1):
            jobs.append((f"f120_g2_fn{fn:03.0f}_z015", exe, s, 2.0, 120.0,
                         fn, 0.15))
    # 1 g / 80 Hz、fn=50 × 阻尼比
    for z in ZETAS:
        for s in range(1, args.seeds_zeta + 1):
            jobs.append((f"f080_fn025_z{z:0.2f}".replace(".", "p"), exe, s,
                         1.0, 80.0, 25.0, z))

    print("jobs", len(jobs), flush=True)
    rows = []
    with cf.ProcessPoolExecutor(max_workers=args.workers) as pool:
        for i, r in enumerate(pool.map(one, jobs, chunksize=2)):
            rows.append(r)
            if (i + 1) % 50 == 0:
                print("done", i + 1, flush=True)

    ok = [r for r in rows if not r.get("no_json")]
    groups = {}
    for r in ok:
        groups.setdefault(r["config"], []).append(r)
    out = dict(tag=args.tag, bin=exe, bin_md5=md5(exe), seeds=args.seeds,
               seeds_2g=args.seeds_2g, seeds_zeta=args.seeds_zeta,
               n_jobs=len(rows), n_ok=len(ok), groups={})
    for name, rs in groups.items():
        out["groups"][name] = dict(
            n=len(rs), nav_init=int(sum(1 for r in rs if r.get("nav_init"))),
            vib_g=rs[0]["vib_g"], vib_hz=rs[0]["vib_hz"],
            iso_fn=rs[0]["iso_fn"], iso_zeta=rs[0]["iso_zeta"],
            att=stat([r.get("nav_att_rmse") for r in rs]),
            att_max=stat([r.get("nav_att_max") for r in rs]),
            pos=stat([r.get("pos_rms") for r in rs]),
            pos_max=stat([r.get("pos_max") for r in rs]),
            vel=stat([r.get("nav_vel_rms") for r in rs]))
    d = os.path.join(HERE, "..", "sim-data")
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, f"vib-iso-{args.tag}.jsonl"), "w",
              encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
    with open(os.path.join(d, f"vib-iso-{args.tag}.json"), "w",
              encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    print("out", os.path.join(d, f"vib-iso-{args.tag}.json"))


if __name__ == "__main__":
    main()
