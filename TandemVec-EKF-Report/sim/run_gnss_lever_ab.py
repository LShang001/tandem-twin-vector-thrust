# -*- coding: utf-8 -*-
"""GNSS 天线杆臂端到端注入与固件补偿 A/B（导弹 6DOF SIL，高角速率弹道）。

@file    run_gnss_lever_ab.py
@brief   用 --gnss-lever-inject 把 UBX 量测搬到天线相位中心，用 --gnss-lever
         配置固件补偿，比较 无补偿 / 正确补偿 / 反号补偿 的位置/速度/姿态误差。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 同一二进制 runtime 开关 A/B；绑核 core2；记录二进制 md5。
@frame   NED（量测在体轴偏移 C_b^n·r 处）
@units   m / m/s / deg
@theory p_ant = p_imu + C_b^n r；v_ant = v_imu + C_b^n(ω×r)。
        @ref REF-120 §9.6 杆臂/旋转项

用法：
  py -3.12 sim/run_gnss_lever_ab.py --exe /tmp/tv_lever/fw_real.exe \
      --out-tag 2026-10-27 --seeds 20
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

# 生产 8 g 口径的导弹 SIL 基座（armed、mag_clean、ICM、10 Hz GNSS、默认 6.4 g 推力）
BASE = ["--real-nav", "--missile", "--seconds", "45",
        "--gnss-rate-hz", "10", "--profile", "mag_clean", "--imu-grade", "icm"]

# (工况名, 额外参数)：注入与补偿分开写，便于核对补偿口径
LEVER_CASES = {
    "clean": [],
    "8p4g": [("--ms-thrust", "8000")],
    "out5": [("--gnss-outage", "4:9")],
    "out20": [("--gnss-outage", "4:24")],
    "hz5": [("--gnss-rate-hz", "5")],
    "lat200": [("--gnss-latency-ms", "200")],
    "vib1g": [("--vib", "1.0"), ("--vib-hz", "80"), ("--vib-start", "1.5")],
}


def arms_for(seeds):
    """base/inj/cmp/neg 四臂 × 每个工况。"""
    out = []
    for case, extra in LEVER_CASES.items():
        out.append((f"{case}_none", extra + [("--gnss-lever-inject", "0,0,0")],
                    seeds))
        out.append((f"{case}_inj_x2", extra + [("--gnss-lever-inject", "2,0,0")],
                    seeds))
        out.append((f"{case}_cmp_x2",
                    extra + [("--gnss-lever-inject", "2,0,0"),
                             ("--gnss-lever", "2,0,0")], seeds))
        out.append((f"{case}_neg_x2",
                    extra + [("--gnss-lever-inject", "2,0,0"),
                             ("--gnss-lever", "-2,0,0")], seeds))
    return out


KEYS = ["pos_rms", "pos_max", "nav_vel_rms", "nav_att_rmse", "nav_yaw_rmse",
        "nav_pitch_rmse", "max_roll", "outlier_rejects", "reacquire_resets",
        "vel_sub_rec", "final_alt"]


def run_one(exe, name, extra, seed, log):
    cmd = [exe] + BASE + ["--seed", str(seed)] + [
        tok for pair in extra for tok in pair]
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
        log.write(f"[{name} seed={seed}] NO JSON rc={p.returncode}\n")
        log.write(out[-1200:] + "\n" + err[-600:] + "\n")
        return None
    d = json.loads(m[-1].replace("nan", "NaN").replace("inf", "Infinity"))
    d["arm"] = name
    d["seed"] = seed
    return d


def stats(sel, key):
    vals = [r.get(key) for r in sel
            if isinstance(r.get(key), (int, float))
            and math.isfinite(r.get(key, float("nan")))]
    if not vals:
        return None
    vals.sort()
    mid = vals[len(vals) // 2] if len(vals) % 2 else \
        (vals[len(vals) // 2 - 1] + vals[len(vals) // 2]) / 2
    return {"n": len(vals), "med": round(mid, 3),
            "p95": round(vals[min(len(vals) - 1, int(0.95 * len(vals)))], 3),
            "max": round(max(vals), 3)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", required=True)
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--out-tag", default="2026-10-27")
    ap.add_argument("--cases", default="",
                    help="只跑这些工况（逗号分隔，默认全部）")
    ap.add_argument("--vtol", action="store_true",
                    help="用 VTOL 生产口径（悬停/定点，姿态近似平稳，"
                         "位置通道可用整窗平均判读）而不是导弹弹道")
    args = ap.parse_args()
    global BASE
    if args.vtol:
        BASE = ["--real-nav", "--seconds", "40", "--profile", "mag_clean",
                "--imu-grade", "icm", "--pos-hold"]
        LEVER_CASES.clear()
        LEVER_CASES.update({"hover": [],
                            "out5": [("--gnss-outage", "20:25")],
                            "wind": [("--wind-force", "1.0")]})

    seeds = list(range(1, args.seeds + 1))
    all_arms = arms_for(seeds)
    if args.cases:
        keep = set(args.cases.split(","))
        all_arms = [a for a in all_arms
                    if a[0].split("_", 1)[0] in keep]

    exe = args.exe
    md5 = hashlib.md5(open(exe, "rb").read()).hexdigest()
    os.makedirs(DATA, exist_ok=True)
    out_jsonl = os.path.join(DATA, f"gnss_lever_{args.out_tag}.jsonl")
    open(out_jsonl, "w", encoding="utf-8").close()
    log_path = out_jsonl + ".log"

    jobs = [(name, extra, s) for name, extra, sds in all_arms for s in sds]
    rows = []
    with open(log_path, "w", encoding="utf-8") as log:
        log.write(f"exe={exe} md5={md5} core={CORE} jobs={len(jobs)}\n")
        log.flush()
        with cf.ThreadPoolExecutor(max_workers=args.workers) as pool:
            futs = {pool.submit(run_one, exe, n, e, s, log): (n, s)
                    for (n, e, s) in jobs}
            for f in cf.as_completed(futs):
                d = f.result()
                if d:
                    rows.append(d)
                    with open(out_jsonl, "a", encoding="utf-8") as fh:
                        fh.write(json.dumps(d, ensure_ascii=False) + "\n")

    summary = {"exe": exe, "md5": md5, "core": CORE, "tag": args.out_tag,
               "n_rows": len(rows), "arms": {}}
    for name, _, _ in all_arms:
        sel = [r for r in rows if r["arm"] == name]
        if not sel:
            continue
        s = {"n": len(sel), "bad_nav": sum(1 for r in sel
                                           if not r.get("nav_init", 0))}
        for k in KEYS:
            st = stats(sel, k)
            if st:
                s[k] = st
        summary["arms"][name] = s

    out_json = os.path.join(DATA, f"gnss_lever_{args.out_tag}.json")
    with open(out_json, "w", encoding="utf-8") as fh:
        json.dump(summary, fh, ensure_ascii=False, indent=1)

    hdr = f"{'arm':<16}{'n':>4}{'bad':>4}{'pos_rms':>10}{'p95':>9}{'max':>9}" \
          f"{'pos_max':>10}{'vel_rms':>9}{'att':>8}{'yaw':>8}{'roll':>8}" \
          f"{'rej':>7}{'rec':>6}{'vsub':>6}"
    print(hdr)
    print("-" * len(hdr))
    for name, _, _ in all_arms:
        a = summary["arms"].get(name)
        if not a:
            continue
        g = lambda k, s="med": (a.get(k) or {}).get(s, float("nan"))
        print(f"{name:<16}{a['n']:>4}{a['bad_nav']:>4}"
              f"{g('pos_rms'):>10.3f}{g('pos_rms','p95'):>9.2f}"
              f"{g('pos_rms','max'):>9.2f}{g('pos_max'):>10.2f}"
              f"{g('nav_vel_rms'):>9.3f}{g('nav_att_rmse'):>8.2f}"
              f"{g('nav_yaw_rmse'):>8.2f}{g('max_roll'):>8.1f}"
              f"{g('outlier_rejects','max'):>7.0f}{g('reacquire_resets','max'):>6.0f}"
              f"{g('vel_sub_rec','max'):>6.0f}")
    print("json:", out_json)


if __name__ == "__main__":
    main()
