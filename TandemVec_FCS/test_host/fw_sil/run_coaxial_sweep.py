#!/usr/bin/env python3
"""run_coaxial_sweep.py — 共轴 SIL plant 参数/故障扰动扫描。

@file    run_coaxial_sweep.py
@module  固件/宿主 SIL（共轴 plant 参数/故障扫描）
@brief   共轴 SIL 的 plant 参数与执行器故障扰动扫描
@details 只改变 plant 的质量/推力/惯量/力臂/差速力矩与执行器故障，固件控制律不变；
         输出逐案例 PASS/FAIL、指标与 JSON 汇总，失败退出码非 0。
         --append-args 追加到每次 SIL 命令（如 "--coax-realistic"），
         用于 task-1 落地后的同分布 A/B；默认不追加。

只改变 plant 的质量/推力/惯量/力臂/差速力矩与执行器故障，固件控制律不变；
用于找“实飞结构控制律在 L1 plant 参数偏差下是否仍有界”。不是实机结论。

用法（在 TandemVec_FCS 目录下）：
    py -3.12 test_host/fw_sil/run_coaxial_sweep.py --bin /tmp/tv_coax_bin/fw_coax.exe
    py -3.12 .../run_coaxial_sweep.py --bin ... --append-args=--coax-realistic

退出码：0=18 组全过；1=存在失败；2=二进制缺失。
"""
import argparse
import datetime
import json
import math
import re
import subprocess
from pathlib import Path

CASES = [
    # name, mass, thrust, lever, iy, iz, yawk, extra
    ("nominal", 0.84, 14.0, 0.15, 0.05, 0.08, 0.0005, []),
    ("mass_low", 0.70, 14.0, 0.15, 0.05, 0.08, 0.0005, []),
    ("mass_high", 1.00, 14.0, 0.15, 0.05, 0.08, 0.0005, []),
    ("thrust_low", 0.84, 12.0, 0.15, 0.05, 0.08, 0.0005, []),
    ("thrust_high", 0.84, 16.0, 0.15, 0.05, 0.08, 0.0005, []),
    ("lever_low", 0.84, 14.0, 0.08, 0.05, 0.08, 0.0005, []),
    ("lever_high", 0.84, 14.0, 0.22, 0.05, 0.08, 0.0005, []),
    ("iy_low", 0.84, 14.0, 0.15, 0.03, 0.08, 0.0005, []),
    ("iy_high", 0.84, 14.0, 0.15, 0.08, 0.08, 0.0005, []),
    ("iz_low", 0.84, 14.0, 0.15, 0.05, 0.04, 0.0005, []),
    ("iz_high", 0.84, 14.0, 0.15, 0.05, 0.12, 0.0005, []),
    ("yawk_low", 0.84, 14.0, 0.15, 0.05, 0.08, 0.0002, []),
    ("yawk_high", 0.84, 14.0, 0.15, 0.05, 0.08, 0.0010, []),
    ("combined_low", 0.70, 12.0, 0.08, 0.03, 0.04, 0.0002, []),
    ("combined_high", 1.00, 16.0, 0.22, 0.08, 0.12, 0.0010, []),
    ("motor_loss_30", 0.84, 14.0, 0.15, 0.05, 0.08, 0.0005,
     ["--fault", "3", "--fault-at", "15"]),
    ("both_loss_60", 0.84, 14.0, 0.15, 0.05, 0.08, 0.0005,
     ["--fault", "5", "--fault-at", "15", "--loss", "0.4"]),
    ("servo_jam", 0.84, 14.0, 0.15, 0.05, 0.08, 0.0005,
     ["--fault", "4", "--fault-at", "15"]),
]

HOLD_GATE = 6.0
ALT_RANGE = (8.0, 11.0)
ATT_GATE = 30.0


def finite(v):
    return isinstance(v, (int, float)) and math.isfinite(float(v))


def num_or_none(v):
    return float(v) if finite(v) else None


def classify(name, extra, data):
    """按历史口径给出 (ok, status, reasons)。

    舵机卡死（fault=4）只验收 F4 检测；双发同时 60% 损失（fault=5）只验收
    F3 检测——不把故障后重构/降落品质当验收门（当前固件没有该能力）。
    """
    reasons = []
    fault_id = 0
    if "--fault" in extra:
        fault_id = int(extra[extra.index("--fault") + 1])
    if fault_id == 4:
        ok = data.get("f4") == 1
        status = "DETECTED" if ok else "MISS"
        if not ok:
            reasons.append("f4=0（舵机卡死未检测）")
        return ok, status, reasons
    if fault_id == 5:
        ok = data.get("f3") == 1
        status = "DETECTED" if ok else "MISS"
        if not ok:
            reasons.append("f3=0（双发损失未检测）")
        return ok, status, reasons

    alt = data.get("final_alt")
    hold = data.get("hold_end_rms")
    roll = data.get("max_roll")
    pitch = data.get("max_pitch")
    if data.get("nav_init") != 1:
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
    ok = not reasons
    return ok, ("PASS" if ok else "FAIL"), reasons


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", required=True)
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--append-args", default="",
                    help="追加到每次 SIL 命令的运行时开关（如 '--coax-realistic'）")
    ap.add_argument("--out", default="",
                    help="输出 JSON 路径；默认 sim-data/coax-sweep-YYYY-MM-DD/coax_sweep.json")
    args = ap.parse_args()
    bin_path = Path(args.bin)
    if not bin_path.exists():
        print(f"[FAIL] binary not found: {bin_path}")
        return 2
    append = args.append_args.split()
    rows = []
    failures = 0
    for name, mass, thrust, lever, iy, iz, yawk, extra in CASES:
        cmd = [str(bin_path), "--real-nav", "--profile", "mag_clean",
               "--pos-hold", "--seconds", str(args.seconds),
               "--seed", str(args.seed), "--no-wind", "--mag",
               "--coax-mass", str(mass), "--coax-thrust", str(thrust),
               "--coax-lever", str(lever), "--coax-iy", str(iy),
               "--coax-iz", str(iz), "--coax-yawk", str(yawk),
               *extra, *append]
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
        row = dict(name=name, mass=mass, thrust=thrust, lever=lever,
                   iy=iy, iz=iz, yawk=yawk, extra=extra, append=append)
        if data is None:
            print(f"[FAIL] {name}: no JSON / rc={proc.returncode}")
            row.update(ok=0, status="FAIL",
                       reasons=f"no SIL_REAL_JSON / rc={proc.returncode}",
                       final_alt=None, hold_end=None, max_roll=None,
                       max_pitch=None, f3=None, f4=None)
            failures += 1
            rows.append(row)
            continue
        ok, status, reasons = classify(name, extra, data)
        if not ok:
            failures += 1
        row.update(
            ok=1 if ok else 0, status=status,
            reasons="; ".join(reasons),
            final_alt=num_or_none(data.get("final_alt")),
            hold_end=num_or_none(data.get("hold_end_rms")),
            max_roll=num_or_none(data.get("max_roll")),
            max_pitch=num_or_none(data.get("max_pitch")),
            f3=data.get("f3"), f4=data.get("f4"))
        rows.append(row)
        alt, hold = data.get("final_alt"), data.get("hold_end_rms")
        roll, pitch = data.get("max_roll"), data.get("max_pitch")
        print(f"[{status}] {name:14s} mass={mass:.2f} T={thrust:.0f} "
              f"L={lever:.2f} Iy={iy:.2f} Iz={iz:.2f} "
              f"alt={alt if alt is not None else 'nan'} "
              f"hold={hold if hold is not None else 'nan'} "
              f"roll={roll if roll is not None else 'nan'} "
              f"pitch={pitch if pitch is not None else 'nan'} "
              f"f3={data.get('f3')} f4={data.get('f4')}")
        for r in reasons:
            print(f"       - {r}")

    passed = sum(r["ok"] for r in rows)
    out = dict(total=len(rows), passed=passed, failed=failures,
               seconds=args.seconds, seed=args.seed, append_args=append,
               rows=rows)
    if not args.out:
        date = datetime.date.today().isoformat()
        out_path = (Path(__file__).resolve().parent / "sim-data"
                    / f"coax-sweep-{date}" / "coax_sweep.json")
    else:
        out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(out, ensure_ascii=False, indent=2),
                        encoding="utf-8")
    print(f"\n=== coaxial sweep: {len(CASES)} cases, {failures} failed ===")
    print(f"saved: {out_path}")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
