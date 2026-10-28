# -*- coding: utf-8 -*-
"""6DOF 导弹弹体 + 真实固件导航任务 SIL 矩阵。
@file    run_missile_sil_mc.py
@brief   6DOF 导弹弹体 + 真实固件导航 SIL 矩阵驱动。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 按 seeds×IMU 档×GNSS 条件调用 fw_real.exe --missile，解析 SIL_REAL_JSON。
@frame   N/A
@units   N/A

只注入传感器（IMU/UBX/气压/磁），固件跑真实 handleNavigationSystem()；
plant 为 test_host/fw_sil/missile_plant.h（6DOF 刚体 + 气动 + 理想自驾）。

用法：
  py -3.12 sim/run_missile_sil_mc.py \
      --bin-full /tmp/tv_fcs_missile/fw_real.exe \
      --bin-mcu  /tmp/tv_fcs_missile_mcu/fw_real.exe \
      --bin-align /tmp/tv_fcs_missile_align/fw_real.exe \
      --seeds 20
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

BASE = ["--real-nav", "--missile", "--seconds", "45", "--seed"]

# name, binary key, profile, imu grade, extra args
CONFIGS = [
    ("clean_ideal_full", "full", "mag_clean", "ideal", []),
    ("clean_icm_full", "full", "mag_clean", "icm", []),
    ("nomag_icm_full", "full", "clean", "icm", []),
    ("clean_icm_mcu", "mcu", "mag_clean", "icm", []),
    ("clean_ideal_mcu", "mcu", "mag_clean", "ideal", []),
    ("outage5_icm_full", "full", "mag_clean", "icm",
     ["--gnss-outage", "4:9"]),
    ("outage5_icm_mcu", "mcu", "mag_clean", "icm",
     ["--gnss-outage", "4:9"]),
    ("outage10_icm_full", "full", "mag_clean", "icm",
     ["--gnss-outage", "4:14"]),
    ("outage20_icm_full", "full", "mag_clean", "icm",
     ["--gnss-outage", "4:24"]),
    ("outage30_icm_full", "full", "mag_clean", "icm",
     ["--gnss-outage", "4:34"]),
    ("hz5_icm_full", "full", "mag_clean", "icm", ["--gnss-rate-hz", "5"]),
    ("hz20_icm_full", "full", "mag_clean", "icm", ["--gnss-rate-hz", "20"]),
    ("imu_nan_icm_full", "full", "mag_clean", "icm",
     ["--imu-fault", "nan", "--imu-fault-window", "12:13"]),
    ("imu_nan2s_icm_full", "full", "mag_clean", "icm",
     ["--imu-fault", "nan", "--imu-fault-window", "12:14"]),
    ("imu_stuck_icm_full", "full", "mag_clean", "icm",
     ["--imu-fault", "stuck", "--imu-fault-window", "12:13"]),
    ("imu_stuck2s_icm_full", "full", "mag_clean", "icm",
     ["--imu-fault", "stuck", "--imu-fault-window", "12:14"]),
    ("imu_nan_long_icm_full", "full", "mag_clean", "icm",
     ["--imu-fault", "nan", "--imu-fault-window", "12:45"]),
    ("gnss_spoof20_icm_full", "full", "mag_clean", "icm",
     ["--gnss-spoof", "20", "--gnss-spoof-start", "25"]),
    ("gnss_spoof30_icm_full", "full", "mag_clean", "icm",
     ["--gnss-spoof", "30", "--gnss-spoof-start", "25"]),
    ("mag_dropout_icm_full", "full", "mag_clean", "icm",
     ["--mag-dropout", "20:25"]),
    ("baro_drift_icm_full", "full", "mag_clean", "icm",
     ["--baro-drift", "0.5"]),
    ("thrust8g_icm_full", "full", "mag_clean", "icm",
     ["--ms-thrust", "8000"]),
    ("thrust8g_outage5_icm_full", "full", "mag_clean", "icm",
     ["--ms-thrust", "8000", "--gnss-outage", "4:9"]),
    ("thrust8g_nan_icm_full", "full", "mag_clean", "icm",
     ["--ms-thrust", "8000", "--imu-fault", "nan",
      "--imu-fault-window", "12:13"]),
    ("thrust8g_spoof30_icm_full", "full", "mag_clean", "icm",
     ["--ms-thrust", "8000", "--gnss-spoof", "30",
      "--gnss-spoof-start", "25"]),
    ("thrust8g_hz5_icm_full", "full", "mag_clean", "icm",
     ["--ms-thrust", "8000", "--gnss-rate-hz", "5"]),
    ("thrust10g_icm_full", "full", "mag_clean", "icm",
     ["--ms-thrust", "10000"]),
    ("thrust10g_outage5_icm_full", "full", "mag_clean", "icm",
     ["--ms-thrust", "10000", "--gnss-outage", "4:9"]),
    ("thrust10g_icm_mcu", "mcu", "mag_clean", "icm",
     ["--ms-thrust", "10000"]),
    ("thrust10g_imu_nan_icm_full", "full", "mag_clean", "icm",
     ["--ms-thrust", "10000", "--imu-fault", "nan",
      "--imu-fault-window", "12:13"]),
    ("thrust10g_spoof30_icm_full", "full", "mag_clean", "icm",
     ["--ms-thrust", "10000", "--gnss-spoof", "30",
      "--gnss-spoof-start", "25"]),
    ("thrust10g_magdrop_icm_full", "full", "mag_clean", "icm",
     ["--ms-thrust", "10000", "--mag-dropout", "20:25"]),
    ("thrust10g_barodrift_icm_full", "full", "mag_clean", "icm",
     ["--ms-thrust", "10000", "--baro-drift", "0.5"]),
    ("vib1g_6p4g_icm_full", "full", "mag_clean", "icm",
     ["--vib", "1.0", "--vib-hz", "80", "--vib-start", "1.5"]),
    ("vib1g_8p4g_icm_full", "full", "mag_clean", "icm",
     ["--ms-thrust", "8000", "--vib", "1.0", "--vib-hz", "80", "--vib-start", "1.5"]),
    ("vib2g_8p4g_icm_full", "full", "mag_clean", "icm",
     ["--ms-thrust", "8000", "--vib", "2.0", "--vib-hz", "120", "--vib-start", "1.5"]),
    ("viblaunch1g_6p4g_icm_full", "full", "mag_clean", "icm",
     ["--vib", "1.0", "--vib-hz", "80"]),
    ("viblaunch2g_8p4g_icm_full", "full", "mag_clean", "icm",
     ["--ms-thrust", "8000", "--vib", "2.0", "--vib-hz", "120"]),
    ("latency100_icm_full", "full", "mag_clean", "icm",
     ["--gnss-latency-ms", "100"]),
    ("latency150_icm_full", "full", "mag_clean", "icm",
     ["--gnss-latency-ms", "150"]),
    ("latency200_icm_full", "full", "mag_clean", "icm",
     ["--gnss-latency-ms", "200"]),
    ("noise_icm_full", "full", "mag_clean", "icm",
     ["--gnss-pos-std", "2.5", "--gnss-vel-std", "0.30"]),
    ("low_full", "full", "mag_clean", "low", []),
    ("adverse_full", "full", "mag_clean", "adverse", []),
    ("low_align", "align", "mag_clean", "low", []),
    ("adverse_align", "align", "mag_clean", "adverse", []),
    ("clean_icm_align", "align", "mag_clean", "icm", []),
]


def resolve_bin(path):
    # Git Bash 的 /tmp 在 Windows Python 下不是同一路径；回退到 TEMP。
    if path and os.path.exists(path):
        return path
    if path and path.startswith("/tmp/"):
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
    name, exe, profile, grade, extra, seed, suffix = job
    cmd = [exe] + BASE + [str(seed), "--profile", profile,
                          "--imu-grade", grade] + extra
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
    except subprocess.TimeoutExpired:
        return dict(config=name + suffix, seed=seed, no_json=True, timeout=True)
    mm = JSON_RE.findall(r.stdout + r.stderr)
    if not mm:
        return dict(config=name + suffix, seed=seed, no_json=True,
                    stderr=r.stderr[-400:])
    d = json.loads(mm[-1].replace("nan", "NaN").replace("inf", "Infinity"))
    d.update(config=name + suffix, seed=seed, profile=profile,
             imu_grade=grade, extra=" ".join(extra))
    return d


def stat(v):
    v = np.array([x for x in v if isinstance(x, (int, float)) and np.isfinite(x)],
                 float)
    if not len(v):
        return dict(n=0)
    return dict(n=int(len(v)), med=float(np.median(v)),
                p90=float(np.percentile(v, 90)),
                p95=float(np.percentile(v, 95)), mx=float(v.max()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin-full", default="/tmp/tv_fcs_missile/fw_real.exe")
    ap.add_argument("--bin-mcu", default="/tmp/tv_fcs_missile_mcu/fw_real.exe")
    ap.add_argument("--bin-align",
                    default="/tmp/tv_fcs_missile_align/fw_real.exe")
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--seed-start", type=int, default=1)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--tag", default="2026-10-27")
    ap.add_argument("--only", default="")
    # 单二进制 A/B（如 AHRS 门控变体）：覆盖三个路径并给 config 加后缀。
    ap.add_argument("--override-bin", default="")
    ap.add_argument("--name-suffix", default="")
    # 同一二进制运行时开关（如 --no-imu-lpf）追加到所有命令，用于严格 A/B。
    ap.add_argument("--append-args", default="")
    ap.add_argument("--bind-core", type=int, default=-1,
                    help="把本进程（及其派生的全部 worker/SIL 子进程）绑到指定逻辑核。"
                         "边际 seed 的双吸引子来自宿主核相关浮点调度，跨轮 A/B 必须同核。")
    args = ap.parse_args()
    if args.bind_core >= 0:
        # Windows 没有 preexec_fn；子进程继承父进程亲和性掩码，绑一次父进程即可。
        import psutil
        psutil.Process().cpu_affinity([args.bind_core])

    bins = dict(full=resolve_bin(args.bin_full),
                mcu=resolve_bin(args.bin_mcu),
                align=resolve_bin(args.bin_align))
    if args.override_bin:
        bins = dict(full=resolve_bin(args.override_bin),
                    mcu=resolve_bin(args.override_bin),
                    align=resolve_bin(args.override_bin))
    # 在跑长矩阵前记录二进制哈希（跨轮 A/B 必须可追溯）。
    hashes = {}
    for k, p in bins.items():
        hashes[k] = md5(p) if os.path.exists(p) else "MISSING"
    # 仅有 md5 不够：opt-in 宏（如 BFS_NAV_IMU_OUTAGE_FALLBACK）是编译期写进
    # EXTRA_CXXFLAGS 的，不记下来就无法事后区分"同源码不同配置"，会造出跨二进制误归因
    # （本轮 imu_stuck 3.11→34.37 即为此）。
    build_ctx = {"EXTRA_CXXFLAGS": os.environ.get("EXTRA_CXXFLAGS", ""),
                 "OPT": os.environ.get("OPT", ""),
                 "bind_core": args.bind_core,
                 "append_args": args.append_args,
                 "bin_paths": {k: str(p) for k, p in bins.items()}}

    append = args.append_args.split() if args.append_args else []
    jobs = []
    only = set(x for x in args.only.split(",") if x)
    for name, bkey, profile, grade, extra in CONFIGS:
        if only and name not in only:
            continue
        for seed in range(args.seed_start, args.seed_start + args.seeds):
            jobs.append((name, bins[bkey], profile, grade, extra + append,
                         seed, args.name_suffix))
    print("jobs", len(jobs), flush=True)

    rows = []
    with cf.ProcessPoolExecutor(max_workers=args.workers) as ex:
        for i, r in enumerate(ex.map(one, jobs, chunksize=2)):
            rows.append(r)
            if (i + 1) % 50 == 0:
                print("done", i + 1, flush=True)

    out_dir = os.path.join(HERE, "..", "sim-data")
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, f"missile-sil-{args.tag}.jsonl")
    with open(out, "w", encoding="utf-8") as f:
        for d in rows:
            f.write(json.dumps(d, ensure_ascii=False) + "\n")

    ok = [r for r in rows if not r.get("no_json")]
    groups = {}
    for r in ok:
        groups.setdefault(r["config"], []).append(r)
    summary = dict(tag=args.tag, seeds=args.seeds, seed_start=args.seed_start,
                   binaries=hashes, build_ctx=build_ctx,
                   n_jobs=len(rows), n_ok=len(ok),
                   no_json=len(rows) - len(ok),
                   traj=dict(
                       vmax=stat([r.get("ms_vmax") for r in ok]),
                       gmax=stat([r.get("ms_gmax") for r in ok]),
                       rate_max=stat([r.get("ms_rate_max") for r in ok]),
                       qbar_max=stat([r.get("ms_qbar_max") for r in ok])),
                   groups={})
    for name, rs in groups.items():
        summary["groups"][name] = dict(
            n=len(rs),
            nav_init=int(sum(1 for r in rs if r.get("nav_init"))),
            pos_rmse=stat([r.get("pos_rms") for r in rs]),
            pos_max=stat([r.get("pos_max") for r in rs]),
            att_rmse=stat([r.get("nav_att_rmse") for r in rs]),
            att_max=stat([r.get("nav_att_max") for r in rs]),
            roll_rmse=stat([r.get("nav_roll_rmse") for r in rs]),
            pitch_rmse=stat([r.get("nav_pitch_rmse") for r in rs]),
            yaw_rmse=stat([r.get("nav_yaw_rmse") for r in rs]),
            vel_rms=stat([r.get("nav_vel_rms") for r in rs]),
            fix3d=stat([r.get("fix3d") for r in rs]),
            outliers=stat([r.get("outlier_rejects") for r in rs]),
            reanchors=stat([r.get("reacquire_resets") for r in rs]),
            mag_gate=stat([r.get("mag_gate") for r in rs]),
            mag_fused=stat([r.get("mag_fused") for r in rs]),
        )
    sm = os.path.join(out_dir, f"missile-sil-{args.tag}.json")
    with open(sm, "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=1)
    print(json.dumps(summary["groups"], ensure_ascii=False, indent=1))
    print("out", out)


if __name__ == "__main__":
    main()
