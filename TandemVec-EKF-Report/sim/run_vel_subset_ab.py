# -*- coding: utf-8 -*-
"""GNSS 速度子集软恢复 A/B（32 g 导弹 SIL，30.5 g / 10 Hz 尾部）。
@file    run_vel_subset_ab.py
@brief   比较 --gnss-vel-subset 0|1 在 30.5 g 尾部 seed、spoof、标称下的表现。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 同一二进制 runtime 开关 A/B；绑核 core2；记录二进制 md5。
@frame   NED
@units   m / m/s / deg

用法：
  py -3.12 sim/run_vel_subset_ab.py --exe <fw_real.exe> --out-tag 2026-10-27
"""
import argparse
import concurrent.futures as cf
import hashlib
import json
import math
import os
import re
import subprocess
import tempfile

import numpy as np

import psutil

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(os.path.dirname(HERE), "sim-data")
JSON_RE = re.compile(r"SIL_REAL_JSON (\{.*\})")

CORE = 2
TAIL_SEEDS = [21, 26, 27, 28, 38, 44, 50]

BASE = ["--real-nav", "--missile", "--seconds", "45", "--imu-fs-g", "32",
        "--imu-health-accel-range", "313.8", "--gnss-rate-hz", "10",
        "--ms-thrust", "23900", "--profile", "mag_clean", "--imu-grade", "icm"]

ARMS = [
    ("tail_off", "mag_clean", [("--gnss-vel-subset", "0")], TAIL_SEEDS),
    ("tail_on", "mag_clean", [("--gnss-vel-subset", "1")], TAIL_SEEDS),
    ("spoof20_off", "mag_clean",
     [("--gnss-vel-subset", "0"), ("--gnss-spoof", "20"),
      ("--gnss-spoof-start", "25")], [1, 2, 3, 4, 5]),
    ("spoof20_on", "mag_clean",
     [("--gnss-vel-subset", "1"), ("--gnss-spoof", "20"),
      ("--gnss-spoof-start", "25")], [1, 2, 3, 4, 5]),
    ("spoof30_off", "mag_clean",
     [("--gnss-vel-subset", "0"), ("--gnss-spoof", "30"),
      ("--gnss-spoof-start", "25")], [1, 2, 3, 4, 5]),
    ("spoof30_on", "mag_clean",
     [("--gnss-vel-subset", "1"), ("--gnss-spoof", "30"),
      ("--gnss-spoof-start", "25")], [1, 2, 3, 4, 5]),
    ("nominal_off", "mag_clean", [("--gnss-vel-subset", "0")],
     [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]),
    ("nominal_on", "mag_clean", [("--gnss-vel-subset", "1")],
     [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]),
    # 20 Hz 对照：确认速率修复后本特性是否为中性（不与之叠加收益）
    ("hz20_off", "mag_clean",
     [("--gnss-vel-subset", "0"), ("--gnss-rate-hz", "20")], TAIL_SEEDS),
    ("hz20_on", "mag_clean",
     [("--gnss-vel-subset", "1"), ("--gnss-rate-hz", "20")], TAIL_SEEDS),
    # 速度欺骗专项：位置不跳、速度恒偏 20 m/s（>10 m/s 速度门），
    # 检验本特性会不会把假速度当"可信 GNSS"融合进去。
    ("vspoof20_off", "mag_clean",
     [("--gnss-vel-subset", "0"), ("--gnss-spoof", "0"),
      ("--gnss-spoof-ramp", "20"), ("--gnss-spoof-start", "25")],
     [1, 2, 3, 4, 5]),
    ("vspoof20_on", "mag_clean",
     [("--gnss-vel-subset", "1"), ("--gnss-spoof", "0"),
      ("--gnss-spoof-ramp", "20"), ("--gnss-spoof-start", "25")],
     [1, 2, 3, 4, 5]),
    ("vsipoof5_off", "mag_clean",
     [("--gnss-vel-subset", "0"), ("--gnss-spoof", "0"),
      ("--gnss-spoof-ramp", "5"), ("--gnss-spoof-start", "25")],
     [1, 2, 3, 4, 5]),
    ("vsipoof5_on", "mag_clean",
     [("--gnss-vel-subset", "1"), ("--gnss-spoof", "0"),
      ("--gnss-spoof-ramp", "5"), ("--gnss-spoof-start", "25")],
     [1, 2, 3, 4, 5]),
    # 30.5 g / 10 Hz 全 50 seeds：给尾部发生率一个population 级证据
    ("pop50_off", "mag_clean", [("--gnss-vel-subset", "0")],
     list(range(1, 51))),
    ("pop50_on", "mag_clean", [("--gnss-vel-subset", "1")],
     list(range(1, 51))),
    # 可靠性交互：速度-only 融合会把 time_since_gnss_update_s_ 清零，
    # 可能推迟需要 age>=2 s 的持久硬重锚 —— 必须在失联/故障组合上验证。
    ("out5_off", "mag_clean",
     [("--gnss-vel-subset", "0"), ("--gnss-outage", "4:9")],
     list(range(1, 21))),
    ("out5_on", "mag_clean",
     [("--gnss-vel-subset", "1"), ("--gnss-outage", "4:9")],
     list(range(1, 21))),
    ("out20_off", "mag_clean",
     [("--gnss-vel-subset", "0"), ("--gnss-outage", "4:24")],
     list(range(1, 21))),
    ("out20_on", "mag_clean",
     [("--gnss-vel-subset", "1"), ("--gnss-outage", "4:24")],
     list(range(1, 21))),
    ("vib_off", "mag_clean",
     [("--gnss-vel-subset", "0"), ("--vib", "1.0"), ("--vib-hz", "80"),
      ("--vib-start", "1.5")], list(range(1, 11))),
    ("vib_on", "mag_clean",
     [("--gnss-vel-subset", "1"), ("--vib", "1.0"), ("--vib-hz", "80"),
      ("--vib-start", "1.5")], list(range(1, 11))),
    ("nan_off", "mag_clean",
     [("--gnss-vel-subset", "0"), ("--imu-fault", "nan"),
      ("--imu-fault-window", "12:13")], list(range(1, 11))),
    ("nan_on", "mag_clean",
     [("--gnss-vel-subset", "1"), ("--imu-fault", "nan"),
      ("--imu-fault-window", "12:13")], list(range(1, 11))),
]

KEYS = ["pos_rms", "pos_max", "max_roll", "outlier_rejects", "reacquire_resets",
        "vel_sub_rec", "nav_alt_mean", "final_alt", "rms_pitch", "mag_rej"]


def run_one(exe, name, profile, extra, seed, log):
    cmd = [exe] + BASE + ["--seed", str(seed)] + [
        tok for pair in extra for tok in pair]
    env = os.environ.copy()
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                         text=True, env=env)
    try:
        for _ in range(200):
            if hasattr(psutil.Process(p.pid), "cpu_affinity"):
                p.cpu_affinity([CORE])
                break
    except Exception:
        pass
    out, err = p.communicate(timeout=600)
    m = JSON_RE.findall(out + err)
    if not m:
        log.write(f"[{name} seed={seed}] NO JSON rc={p.returncode}\n")
        log.write(out[-1500:] + "\n" + err[-800:] + "\n")
        return None
    d = json.loads(m[-1].replace("nan", "NaN").replace("inf", "Infinity"))
    d["arm"] = name
    d["seed"] = seed
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", required=True)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--out-tag", default="2026-10-27")
    ap.add_argument("--arms", default="",
                    help="只跑这些 arm（逗号分隔），用于补充专项矩阵")
    ap.add_argument("--extra", default="",
                    help="追加到每个 job 的参数（如 --gnss-lever-inject 2,0,0）")
    args = ap.parse_args()
    if args.extra:
        toks = args.extra.split()
        for _n, _p, _e, _s in ARMS:
            _e.extend([(toks[i], toks[i + 1]) for i in range(0, len(toks) - 1, 2)])
    if args.arms:
        keep = set(args.arms.split(","))
        arms = [a for a in ARMS if a[0] in keep]
    else:
        arms = ARMS

    exe = args.exe
    if not os.path.exists(exe):
        alt = os.path.join(tempfile.gettempdir(),
                           exe.split("/tmp/")[-1].lstrip("/"))
        if os.path.exists(alt):
            exe = alt
    md5 = hashlib.md5(open(exe, "rb").read()).hexdigest()
    os.makedirs(DATA, exist_ok=True)
    out_jsonl = os.path.join(DATA, f"sil32g_velrec_{args.out_tag}.jsonl")
    open(out_jsonl, "w", encoding="utf-8").close()
    log_path = out_jsonl + ".log"

    jobs = [(name, prof, extra, s)
            for name, prof, extra, seeds in arms for s in seeds]
    rows = []
    with open(log_path, "w", encoding="utf-8") as log:
        log.write(f"exe={exe} md5={md5} core={CORE} jobs={len(jobs)}\n")
        with cf.ThreadPoolExecutor(max_workers=args.workers) as pool:
            futs = {pool.submit(run_one, exe, n, p, e, s, log): (n, s)
                    for (n, p, e, s) in jobs}
            for f in cf.as_completed(futs):
                d = f.result()
                if d:
                    rows.append(d)
                    with open(out_jsonl, "a", encoding="utf-8") as fh:
                        fh.write(json.dumps(d, ensure_ascii=False) + "\n")

    summary = {"exe": exe, "md5": md5, "core": CORE, "tag": args.out_tag,
               "arms": {}}
    for name, prof, extra, seeds in arms:
        sel = [r for r in rows if r["arm"] == name]
        if not sel:
            continue
        s = {"n": len(sel)}
        for k in KEYS:
            vals = [r.get(k, 0.0) for r in sel
                    if isinstance(r.get(k), (int, float))
                    and math.isfinite(r.get(k, float("nan")))]
            if not vals:
                s[k + "_mean"] = None
                continue
            vals.sort()
            mid = vals[len(vals) // 2] if len(vals) % 2 else \
                (vals[len(vals) // 2 - 1] + vals[len(vals) // 2]) / 2
            s[k + "_mean"] = round(sum(vals) / len(vals), 3)
            s[k + "_med"] = round(mid, 3)
            s[k + "_max"] = round(max(vals), 3)
        s["seeds"] = sorted(r["seed"] for r in sel)
        s["bad_nav"] = sum(1 for r in sel if not r.get("nav_init", 0))
        pr = sorted(r["pos_rms"] for r in sel
                    if isinstance(r.get("pos_rms"), (int, float))
                    and math.isfinite(r["pos_rms"]))
        if pr:
            s["n_tail_gt25m"] = sum(1 for x in pr if x > 25.0)
            s["n_tail_gt15m"] = sum(1 for x in pr if x > 15.0)
            s["pos_rms_p95"] = round(np.percentile(pr, 95), 3)
        summary["arms"][name] = s
    out_json = os.path.join(DATA, f"sil32g_velrec_{args.out_tag}.json")
    with open(out_json, "w", encoding="utf-8") as fh:
        json.dump(summary, fh, ensure_ascii=False, indent=1)
    print(json.dumps(summary["arms"], ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
