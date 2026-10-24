#!/usr/bin/env python3
"""run_liquid_mc.py — 液体火箭 L1 回收 SIL 随机参数蒙特卡洛。

@file    run_liquid_mc.py
@module  固件/宿主 SIL（液体推进随机参数 MC）
@brief   在推进/回收参数空间随机采样，统计 CCAEGT/自研制导下的着陆包线
@details 采样高度、推进剂、Isp、Tmax/Tmin、阀门/推力速率、初始姿态；
         调用 test_liquid_rocket_sil 二进制，记录着陆速度/姿态/通过率。
         参数为 L1 模型假设，不是本机标定值。
"""
import argparse
import csv
import json
import math
import random
import re
import subprocess
from pathlib import Path

LINE_RE = re.compile(
    r"L-SIL landed=(\d+) h=([-\d.]+) v=([-\d.]+) m=([-\d.]+) "
    r"prop=([-\d.]+) roll=([-\d.]+) pitch=([-\d.]+) "
    r"max_roll=([-\d.]+) max_pitch=([-\d.]+)")


def finite(v):
    return isinstance(v, (int, float)) and math.isfinite(float(v))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", required=True)
    ap.add_argument("--cases", type=int, default=200)
    ap.add_argument("--seed", type=int, default=20261024)
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    bin_path = Path(args.bin)
    if not bin_path.exists():
        print(f"[FAIL] binary not found: {bin_path}")
        return 2

    rng = random.Random(args.seed)
    rows = []
    for i in range(args.cases):
        h0 = rng.uniform(20.0, 40.0)
        prop = rng.uniform(2.5, 4.5)
        isp = rng.uniform(150.0, 250.0)
        tmax = rng.uniform(180.0, 220.0)
        tmin = rng.uniform(60.0, 100.0)
        valve = rng.uniform(5.0, 20.0)
        thrust_rate = rng.uniform(900.0, 2700.0)
        roll0 = rng.uniform(-5.0, 5.0)
        pitch0 = rng.uniform(-5.0, 5.0)
        cmd = [str(bin_path), "--h0", f"{h0:.2f}", "--prop", f"{prop:.3f}",
               "--isp", f"{isp:.1f}", "--tmax", f"{tmax:.1f}",
               "--tmin", f"{tmin:.1f}", "--valve-rate", f"{valve:.1f}",
               "--thrust-rate", f"{thrust_rate:.0f}",
               "--roll0", f"{roll0:.2f}", "--pitch0", f"{pitch0:.2f}"]
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
        m = LINE_RE.search(proc.stdout)
        if proc.returncode != 0 or not m:
            rows.append(dict(case=i, h0=h0, prop=prop, isp=isp, tmax=tmax,
                             tmin=tmin, valve=valve, thrust_rate=thrust_rate,
                             roll0=roll0, pitch0=pitch0, ok=0,
                             landing_v=None, max_roll=None, max_pitch=None))
            continue
        landed = int(m.group(1))
        v = float(m.group(3))
        max_roll = float(m.group(8))
        max_pitch = float(m.group(9))
        ok = (landed == 1 and 0.0 <= v <= 4.0
              and max_roll <= 20.0 and max_pitch <= 20.0)
        rows.append(dict(case=i, h0=h0, prop=prop, isp=isp, tmax=tmax,
                         tmin=tmin, valve=valve, thrust_rate=thrust_rate,
                         roll0=roll0, pitch0=pitch0, ok=1 if ok else 0,
                         landing_v=v, max_roll=max_roll,
                         max_pitch=max_pitch))

    n = len(rows)
    passed = sum(r["ok"] for r in rows)
    vs = sorted(r["landing_v"] for r in rows if finite(r["landing_v"]))
    rolls = sorted(r["max_roll"] for r in rows if finite(r["max_roll"]))
    pitches = sorted(r["max_pitch"] for r in rows if finite(r["max_pitch"]))

    def pct(xs, q):
        if not xs:
            return None
        k = (len(xs) - 1) * q
        lo, hi = int(math.floor(k)), int(math.ceil(k))
        return xs[lo] if lo == hi else xs[lo] * (hi - k) + xs[hi] * (k - lo)

    out = dict(seed=args.seed, cases=n, passed=passed,
               pass_rate=passed / n if n else 0.0,
               v_p50=pct(vs, 0.5), v_p95=pct(vs, 0.95),
               roll_p95=pct(rolls, 0.95), pitch_p95=pct(pitches, 0.95),
               rows=rows)
    out_path = Path(args.out) if args.out else (
        Path("sim-data") / "liquid-mc-2026-10-24" / "liquid_mc.json")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(out, ensure_ascii=False, indent=2),
                        encoding="utf-8")
    csv_path = out_path.with_suffix(".csv")
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    worst = sorted(rows, key=lambda r: (r["ok"], -(r["landing_v"] or 1e9)))[:8]
    print(f"liquid MC: {passed}/{n} pass, landing v P50/P95="
          f"{out['v_p50']:.3f}/{out['v_p95']:.3f} m/s, "
          f"roll P95={out['roll_p95']:.2f}°, pitch P95={out['pitch_p95']:.2f}°")
    print(f"saved: {out_path} / {csv_path}")
    for r in worst:
        print(f"  case{r['case']:03d} ok={r['ok']} h0={r['h0']:.1f} "
              f"prop={r['prop']:.2f} isp={r['isp']:.0f} "
              f"T=[{r['tmin']:.0f},{r['tmax']:.0f}] valve={r['valve']:.1f} "
              f"rate={r['thrust_rate']:.0f} v={r['landing_v']} "
              f"roll={r['max_roll']} pitch={r['max_pitch']}")
    return 0 if passed == n else 1


if __name__ == "__main__":
    raise SystemExit(main())
