"""Step-maneuver navigation matrix for the real-nav SIL.

Each run: altitude hold until t=20 s, then AUTO_POSITION with a one-shot
horizontal target offset (--pos-leg).  Optional 5 s GNSS outage / crosswind
overlaps the maneuver at t=25..30 s.  The JSON emitted by the SIL only covers
EKF-vs-truth error and attitude; target tracking and speed are recovered from
the 1 Hz truth samples printed to stdout.
"""
import argparse
import concurrent.futures as cf
import json
import math
import os
import re
import subprocess
import sys

EXE = r"C:\Users\12631\AppData\Local\Temp\inc_rate\fw_real.exe"
OUTDIR = r"C:\Users\12631\AppData\Local\Temp\man2"
DIRS = {"N": (1, 0), "E": (0, 1), "S": (-1, 0), "W": (0, -1), "NE": (1, 1)}
DISTS = [3.0, 10.0, 25.0, 50.0]
CONDS = ["nominal", "outage", "wind", "both"]
SEEDS = range(1, 41)
SECONDS = 45
STEP_AT = 20.0
FAULT = "25:30"

TRUTH_RE = re.compile(r"^t=\s*([0-9.]+).*?truth=\(\s*(-?[0-9.]+),\s*(-?[0-9.]+)\)")
JSON_RE = re.compile(r"SIL_REAL_JSON (\{.*\})")


def one(task):
    cond, direction, dist, seed = task
    dn, de = DIRS[direction]
    norm = math.hypot(dn, de)
    leg_n = dist * dn / norm
    leg_e = dist * de / norm
    tag = f"{cond}_{direction}_{int(dist)}_s{seed}"
    log = os.path.join(OUTDIR, tag + ".log")
    cmd = [EXE, "--real-nav", "--profile", "mag_clean", "--pos-hold",
           "--pos-hold-at", str(STEP_AT), "--pos-leg", f"{leg_n:.3f}:{leg_e:.3f}",
           "--flow", "--seed", str(seed), "--seconds", str(SECONDS)]
    if cond in ("outage", "both"):
        cmd += ["--gnss-outage", FAULT]
    if cond in ("wind", "both"):
        cmd += ["--wind-window", FAULT, "--wind-force", "1.0"]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
        text = r.stdout + r.stderr
    except subprocess.TimeoutExpired:
        return dict(tag=tag, cond=cond, dir=direction, dist=dist, seed=seed,
                    timeout=True)
    with open(log, "w", encoding="utf-8") as f:
        f.write(text)
    mm = JSON_RE.findall(text)
    if not mm:
        return dict(tag=tag, cond=cond, dir=direction, dist=dist, seed=seed,
                    no_json=True)
    d = json.loads(mm[-1].replace("nan", "NaN"))
    truth = []
    for line in text.splitlines():
        m = TRUTH_RE.match(line)
        if m:
            truth.append((float(m.group(1)), float(m.group(2)), float(m.group(3))))
    peak_speed = 0.0
    for a, b in zip(truth, truth[1:]):
        dt = b[0] - a[0]
        if dt > 0 and a[0] >= 10.0:
            peak_speed = max(peak_speed, math.hypot(b[1] - a[1], b[2] - a[2]) / dt)
    end_err = None
    if truth:
        end_err = math.hypot(truth[-1][1] - leg_n, truth[-1][2] - leg_e)
    out = dict(tag=tag, cond=cond, dir=direction, dist=dist, seed=seed,
               leg_n=leg_n, leg_e=leg_e, peak_speed=peak_speed,
               end_target_err=end_err)
    for k in ("pos_rms", "pos_max", "max_roll", "max_pitch", "final_alt",
              "heading_aiding", "pos_degraded", "outlier_rejects",
              "reacquire_resets", "gnss_susp", "gnss_crej", "mag_gate"):
        out[k] = d.get(k)
    out["nav_fail"] = bool(d["pos_rms"] > 3.0 or d["pos_max"] > 10.0
                           or abs(d["final_alt"] - 10.0) > 3.0
                           or d["max_roll"] > 45.0)
    out["stab_fail"] = bool(abs(d["final_alt"] - 10.0) > 3.0
                            or d["max_roll"] > 45.0)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--limit", type=int, default=0)
    args = ap.parse_args()
    os.makedirs(OUTDIR, exist_ok=True)
    tasks = []
    for cond in CONDS:
        for direction in DIRS:
            for dist in ([0.0] if direction == "N" else []) + DISTS:
                for seed in SEEDS:
                    tasks.append((cond, direction, dist, seed))
    if args.limit:
        tasks = tasks[:args.limit]
    print(f"tasks={len(tasks)} workers={args.workers}", flush=True)
    out_path = os.path.join(OUTDIR, "results.jsonl")
    done = 0
    with open(out_path, "w", encoding="utf-8") as f, \
            cf.ProcessPoolExecutor(max_workers=args.workers) as ex:
        for res in ex.map(one, tasks, chunksize=4):
            f.write(json.dumps(res) + "\n")
            f.flush()
            done += 1
            if done % 100 == 0:
                print(f"{done}/{len(tasks)}", flush=True)
    print("DONE", out_path, flush=True)


if __name__ == "__main__":
    sys.exit(main())
