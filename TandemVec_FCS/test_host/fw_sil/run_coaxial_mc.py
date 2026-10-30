#!/usr/bin/env python3
"""run_coaxial_mc.py — 共轴 SIL plant 随机参数蒙特卡洛。

@file    run_coaxial_mc.py
@module  固件/宿主 SIL（共轴 plant 随机参数 MC）
@brief   在 L1 plant 参数空间随机采样，统计共轴控制律的定高/位置保持包线
@details 采样质量、最大推力、力臂、横向/轴向惯量、差速力矩；固定 mag_clean、
         --pos-hold、40 s。输出 JSON/CSV 与 P50/P95、通过率、最坏案例。
         参数为 L1 假设，不是实机辨识；结论不能外推实机。
         --realistic 把 --coax-realistic 透传给二进制（task-1 的 plant 新开关；
         二进制内若不含该开关字面量则拒绝运行，避免伪造 realistic 证据）。
         --out 默认 test_host/fw_sil/sim-data/coax-mc-YYYY-MM-DD/coax_mc.json。

用法（在 TandemVec_FCS 目录下）：
    py -3.12 test_host/fw_sil/run_coaxial_mc.py --bin /tmp/tv_coax_bin/fw_coax.exe
    py -3.12 .../run_coaxial_mc.py --bin ... --realistic \
        --out test_host/fw_sil/sim-data/coax-2026-10-30/mc_realistic.json

退出码：0=100% 通过；1=存在失败案例；2=二进制缺失或不支持 --realistic。
"""
import argparse
import csv
import datetime
import json
import math
import random
import re
import subprocess
from pathlib import Path

# CSV/JSON 每行字段（顺序稳定，便于跨次对比；新增字段只追加不重排）。
ROW_FIELDS = [
    "case", "mass", "thrust", "lever", "iy", "iz", "yawk", "seed",
    "ok", "nav_init", "final_alt", "hold_end", "max_roll", "max_pitch",
    "severity", "reasons",
]

HOLD_GATE = 6.0       # 位置保持末端 RMS 上限 [m]
ALT_RANGE = (8.0, 11.0)
ATT_GATE = 30.0       # |roll|/|pitch| 上限 [deg]


def finite(v):
    return isinstance(v, (int, float)) and math.isfinite(float(v))


def num_or_none(v):
    """非有限值转 None，保证 JSON 合法（SIL 故障场景可能出现 NaN/Inf）。"""
    return float(v) if finite(v) else None


def pct(xs, q):
    if not xs:
        return None
    k = (len(xs) - 1) * q
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return xs[lo] if lo == hi else xs[lo] * (hi - k) + xs[hi] * (k - lo)


def supports_realistic(bin_path):
    """检查二进制中是否编入 --coax-realistic 字面量。

    未知参数在 SIL 参数解析里会被静默忽略，因此不能只看“命令能跑通”；
    必须确认开关真的编进二进制，否则 realistic 结果与 baseline 相同会误导。
    """
    try:
        return b"--coax-realistic" in bin_path.read_bytes()
    except OSError:
        return False


def evaluate(row):
    """按 MC 判据填写 ok / severity / reasons（判据与历史 MC 保持一致）。"""
    reasons = []
    nav_init = row.get("nav_init")
    alt = row.get("final_alt")
    hold = row.get("hold_end")
    roll = row.get("max_roll")
    pitch = row.get("max_pitch")
    if nav_init != 1:
        reasons.append("nav_init=0")
    if not finite(alt):
        reasons.append("final_alt 缺失/非有限")
    elif not (ALT_RANGE[0] <= alt <= ALT_RANGE[1]):
        reasons.append(f"final_alt={alt:.3f} 越界[{ALT_RANGE[0]},{ALT_RANGE[1]}]")
    if not finite(hold):
        reasons.append("hold_end 缺失/非有限")
    elif hold > HOLD_GATE:
        reasons.append(f"hold_end={hold:.3f}>{HOLD_GATE}")
    if not finite(roll):
        reasons.append("max_roll 缺失/非有限")
    elif roll > ATT_GATE:
        reasons.append(f"max_roll={roll:.3f}>{ATT_GATE}")
    if not finite(pitch):
        reasons.append("max_pitch 缺失/非有限")
    elif pitch > ATT_GATE:
        reasons.append(f"max_pitch={pitch:.3f}>{ATT_GATE}")
    row["ok"] = 0 if reasons else 1
    # 严重度：各门限相对超限量的和，用于“最差 8 例”排序（仅诊断用）。
    sev = 0.0
    if finite(hold):
        sev += max(0.0, hold - HOLD_GATE) / HOLD_GATE
    if finite(roll):
        sev += max(0.0, roll - ATT_GATE) / ATT_GATE
    if finite(pitch):
        sev += max(0.0, pitch - ATT_GATE) / ATT_GATE
    if finite(alt) and not (ALT_RANGE[0] <= alt <= ALT_RANGE[1]):
        sev += min(abs(alt - ALT_RANGE[0]), abs(alt - ALT_RANGE[1])) / 10.0
    row["severity"] = round(sev, 4)
    row["reasons"] = "; ".join(reasons)
    return row


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", required=True)
    ap.add_argument("--cases", type=int, default=100)
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--seed", type=int, default=20261024)
    ap.add_argument("--out", default="",
                    help="输出 JSON 路径；默认 sim-data/coax-mc-YYYY-MM-DD/coax_mc[_realistic].json")
    ap.add_argument("--realistic", action="store_true",
                    help="透传 --coax-realistic（task-1 plant 新开关；未落地时拒绝运行）")
    args = ap.parse_args()
    if args.cases < 1:
        print("[FAIL] --cases 必须 >= 1")
        return 2
    bin_path = Path(args.bin)
    if not bin_path.exists():
        print(f"[FAIL] binary not found: {bin_path}")
        return 2
    realistic_args = []
    if args.realistic:
        if not supports_realistic(bin_path):
            print("[FAIL] 二进制内未找到 --coax-realistic 开关：task-1 的 plant "
                  "新开关尚未落地，拒绝生成 realistic 证据（避免伪造）。")
            return 2
        realistic_args = ["--coax-realistic"]

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
               "--coax-iz", f"{iz:.4f}", "--coax-yawk", f"{yawk:.6f}",
               *realistic_args]
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
        data = None
        for line in reversed(proc.stdout.splitlines()):
            if line.startswith("SIL_REAL_JSON "):
                raw = re.sub(r"(?<=[:\[,])-?(?:nan|inf)(?=[,}\]])", "null",
                             line[len("SIL_REAL_JSON "):], flags=re.IGNORECASE)
                try:
                    data = json.loads(raw)
                except json.JSONDecodeError:
                    data = None
                break
        row = dict(case=i, mass=mass, thrust=thrust, lever=lever,
                   iy=iy, iz=iz, yawk=yawk, seed=seed,
                   nav_init=None, final_alt=None, hold_end=None,
                   max_roll=None, max_pitch=None)
        if data is None:
            row["reasons"] = "no SIL_REAL_JSON"
            row.update(ok=0, severity=99.0,
                       nav_init=None, final_alt=None, hold_end=None,
                       max_roll=None, max_pitch=None)
        else:
            row.update(
                nav_init=data.get("nav_init"),
                final_alt=num_or_none(data.get("final_alt")),
                hold_end=num_or_none(data.get("hold_end_rms")),
                max_roll=num_or_none(data.get("max_roll")),
                max_pitch=num_or_none(data.get("max_pitch")),
            )
            evaluate(row)
        rows.append({k: row.get(k) for k in ROW_FIELDS})

    n = len(rows)
    passed = sum(r["ok"] for r in rows)
    holds = sorted(float(r["hold_end"]) for r in rows if finite(r["hold_end"]))
    rolls = sorted(float(r["max_roll"]) for r in rows if finite(r["max_roll"]))
    pitches = sorted(float(r["max_pitch"]) for r in rows if finite(r["max_pitch"]))
    worst = sorted(rows, key=lambda r: (1 - r["ok"], r["severity"]),
                   reverse=True)[:8]

    out = dict(
        seed=args.seed, cases=n, passed=passed,
        pass_rate=passed / n if n else 0.0,
        realistic=bool(args.realistic), seconds=args.seconds,
        argv_realistic=realistic_args,
        hold_p50=pct(holds, 0.5), hold_p95=pct(holds, 0.95),
        roll_p50=pct(rolls, 0.5), roll_p95=pct(rolls, 0.95),
        pitch_p50=pct(pitches, 0.5), pitch_p95=pct(pitches, 0.95),
        worst8=worst, rows=rows)
    if not args.out:
        tag = "_realistic" if args.realistic else ""
        date = datetime.date.today().isoformat()
        out_path = (Path(__file__).resolve().parent / "sim-data"
                    / f"coax-mc-{date}" / f"coax_mc{tag}.json")
    else:
        out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(out, ensure_ascii=False, indent=2),
                        encoding="utf-8")
    csv_path = out_path.with_suffix(".csv")
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=ROW_FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    def f2(v):
        return "nan" if v is None else f"{v:.3f}"

    print(f"coax MC{' (realistic)' if args.realistic else ''}: {passed}/{n} pass, "
          f"hold P50/P95={f2(out['hold_p50'])}/{f2(out['hold_p95'])} m, "
          f"roll P95={f2(out['roll_p95'])}°, pitch P95={f2(out['pitch_p95'])}°")
    print(f"saved: {out_path} / {csv_path}")
    for r in worst:
        print(f"  case{r['case']:03d} ok={r['ok']} mass={r['mass']:.2f} "
              f"T={r['thrust']:.1f} L={r['lever']:.2f} Iy={r['iy']:.2f} "
              f"Iz={r['iz']:.2f} yawk={r['yawk']:.5f} "
              f"alt={r['final_alt']} hold={r['hold_end']} "
              f"roll={r['max_roll']} pitch={r['max_pitch']} "
              f"sev={r['severity']} [{r['reasons']}]")
    return 0 if passed == n else 1


if __name__ == "__main__":
    raise SystemExit(main())
