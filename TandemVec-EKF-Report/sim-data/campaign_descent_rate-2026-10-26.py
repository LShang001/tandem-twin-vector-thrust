"""Descent-rate trade study: 1/2/3 m/s at high altitude + wind."""
import concurrent.futures as cf
import json
import math
import os
import re
import subprocess

TMP = r"C:\Users\12631\AppData\Local\Temp"
ARMS = {
    "r1": TMP + r"\camp_combo6b\fw_real.exe",
    "r2": TMP + r"\camp_rate2\fw_real.exe",
    "r3": TMP + r"\camp_rate3\fw_real.exe",
}
OUT = TMP + r"\campaign_descent_rate"
DIRS = {"N": (1, 0), "E": (0, 1), "S": (-1, 0), "W": (0, -1), "NE": (1, 1)}
# (name, dur, alt, wind, seeds)
CELLS = [
    ("out20_alt30_w1", 20, 30.0, 1.0, 20),
    ("out60_alt30_w1", 60, 30.0, 1.0, 20),
    ("out20_alt30_w0", 20, 30.0, 0.0, 10),
    ("out20_alt20_w1", 20, 20.0, 1.0, 10),
]
JSON_RE = re.compile(r"SIL_REAL_JSON (\{.*\})")


def run(task):
    arm, exe, cell, dur, alt, wind, dv, seed = task
    dn, de = DIRS.get(dv, (0.0, 0.0))
    norm = math.hypot(dn, de) or 1.0
    ln, le = 10 * dn / norm, 10 * de / norm
    end = 20.0 + dur
    seconds = 100.0 if dur >= 60 else alt + 45.0
    tag = f"{arm}_{cell}_{dv}_s{seed}"
    cmd = [exe, "--real-nav", "--profile", "mag_clean", "--pos-hold",
           "--pos-hold-at", "20", "--pos-leg", f"{ln:.3f}:{le:.3f}",
           "--gnss-outage", f"20:{end:g}", "--alt", f"{alt:g}",
           "--seed", str(seed), "--seconds", f"{seconds:g}"]
    if wind > 0.0:
        cmd += ["--wind-window", f"20:{end:g}", "--wind-force", f"{wind:g}"]
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=320)
    m = JSON_RE.findall(p.stdout + p.stderr)
    if not m:
        return dict(tag=tag, arm=arm, cell=cell, dir=dv, seed=seed, no_json=True)
    d = json.loads(m[-1].replace("nan", "NaN"))
    out = dict(tag=tag, arm=arm, cell=cell, dir=dv, seed=seed)
    for k in ("land_t", "land_hold", "land_pos_err", "land_roll",
              "max_roll_preland", "land_vz_down", "max_desc_vz_down",
              "final_alt", "pos_rms", "pos_max"):
        out[k] = d.get(k)
    out["landed"] = d.get("final_alt") is not None and d["final_alt"] < 1.0
    return out


def main():
    os.makedirs(OUT, exist_ok=True)
    tasks = []
    for arm, exe in ARMS.items():
        if not os.path.exists(exe):
            print("missing", arm, exe, flush=True)
            continue
        for cell, dur, alt, wind, nseed in CELLS:
            for dv in ["H"] + list(DIRS):
                for s in range(1, nseed + 1):
                    tasks.append((arm, exe, cell, dur, alt, wind, dv, s))
    print("tasks", len(tasks), flush=True)
    path = os.path.join(OUT, "results.jsonl")
    done = 0
    with open(path, "w", encoding="utf-8") as f, \
            cf.ProcessPoolExecutor(max_workers=6) as ex:
        for r in ex.map(run, tasks, chunksize=2):
            f.write(json.dumps(r) + "\n")
            f.flush()
            done += 1
            if done % 50 == 0:
                print("done", done, flush=True)
    print("wrote", path, flush=True)


if __name__ == "__main__":
    main()
