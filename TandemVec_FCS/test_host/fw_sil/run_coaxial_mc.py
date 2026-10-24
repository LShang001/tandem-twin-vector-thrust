#!/usr/bin/env python3
"""run_coaxial_mc.py — 共轴 SIL plant 随机参数蒙特卡洛。

@file    run_coaxial_mc.py
@module  固件/宿主 SIL（共轴 plant 随机参数 MC）
@brief   在 L1 plant 参数空间随机采样，统计共轴控制律的定高/位置保持包线
@details 采样质量、最大推力、力臂、横向/轴向惯量、差速力矩；固定 mag_clean、
         --pos-hold、40 s。输出 JSON/CSV 与 P50/P95、通过率、最坏案例。
         参数为 L1 假设，不是实机辨识；结论不能外推实机。
"""
import argparse
import csv
import json
import math
import random
import re
import subprocess
from pathlib import Path


def finite(v):
    return isinstance(v, (int, float)) and math.isfinite(float(v))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", required=True)
    ap.add_argument("--cases", type=int, default=100)
    ap.add_argument("--seconds", type=float, default=40.0)
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
        mass = rng.uniform(0.70, 1.05)
        thrust = rng.uniform(12.0, 16.0)
        lever = rng.uniform(0.08, 0.22)
        iy = rng.uniform(0.03, 0.10)
        iz = rng.uniform(0.04, 0.12)
        yawk = rng.uniform(0.0002, 0.0012)
        seed = rng.randint(1, 10)
        cmd = [str(bin_path), "--real-nav", "--profile", "mag_clean",
               "--pos-hold", "--seconds", str(args.seconds), "--seed",
               str(seed), "--no-wind", "--mag",
               "--coax-mass", f"{mass:.4f}", "--coax-thrust", f"{thrust:.2f}",
               "--coax-lever", f"{lever:.4f}", "--coax-iy", f"{iy:.4f}",
               "--coax-iz", f"{iz:.4f}", "--coax-yawk", f"{yawk:.6f}"]
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
        data = None
        for line in reversed(proc.stdout.splitlines()):
            if line.startswith("SIL_REAL_JSON "):
                raw = re.sub(r"(?<=[:\[,])-?(?:nan|inf)(?=[,}\]])", "null",
                             line[len("SIL_REAL_JSON "):], flags=re.IGNORECASE)
                data = json.loads(raw)
                break
        if data is None:
            rows.append(dict(case=i, mass=mass, thrust=thrust, lever=lever,
                             iy=iy, iz=iz, yawk=yawk, seed=seed,
                             ok=0, final_alt=None, hold_end=None,
                             max_roll=None, max_pitch=None))
            continue
        alt = data.get("final_alt")
        hold = data.get("hold_end_rms")
        roll = data.get("max_roll")
        pitch = data.get("max_pitch")
        ok = (data.get("nav_init") == 1 and finite(alt) and 8.0 <= alt <= 11.0
              and finite(hold) and hold <= 6.0
              and finite(roll) and roll <= 30.0
              and finite(pitch) and pitch <= 30.0)
        rows.append(dict(case=i, mass=mass, thrust=thrust, lever=lever,
                         iy=iy, iz=iz, yawk=yawk, seed=seed, ok=1 if ok else 0,
                         final_alt=alt, hold_end=hold, max_roll=roll,
                         max_pitch=pitch))

    n = len(rows)
    passed = sum(r["ok"] for r in rows)
    holds = sorted(r["hold_end"] for r in rows if finite(r["hold_end"]))
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
               hold_p50=pct(holds, 0.5), hold_p95=pct(holds, 0.95),
               roll_p50=pct(rolls, 0.5), roll_p95=pct(rolls, 0.95),
               pitch_p50=pct(pitches, 0.5), pitch_p95=pct(pitches, 0.95),
               rows=rows)
    out_path = Path(args.out) if args.out else (
        Path("sim-data") / "coax-mc-2026-10-24" / "coax_mc.json")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(out, ensure_ascii=False, indent=2),
                        encoding="utf-8")
    csv_path = out_path.with_suffix(".csv")
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    worst = sorted(rows, key=lambda r: (r["ok"], -(r["hold_end"] or 1e9)))[:8]
    print(f"coax MC: {passed}/{n} pass, hold P50/P95="
          f"{out['hold_p50']:.3f}/{out['hold_p95']:.3f} m, "
          f"roll P95={out['roll_p95']:.3f}°, pitch P95={out['pitch_p95']:.3f}°")
    print(f"saved: {out_path} / {csv_path}")
    for r in worst:
        print(f"  case{r['case']:03d} ok={r['ok']} mass={r['mass']:.2f} "
              f"T={r['thrust']:.1f} L={r['lever']:.2f} Iy={r['iy']:.2f} "
              f"Iz={r['iz']:.2f} yawk={r['yawk']:.5f} "
              f"alt={r['final_alt']} hold={r['hold_end']} "
              f"roll={r['max_roll']} pitch={r['max_pitch']}")
    return 0 if passed == n else 1


if __name__ == "__main__":
    raise SystemExit(main())
