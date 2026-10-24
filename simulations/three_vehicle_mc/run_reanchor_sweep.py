"""run_reanchor_sweep.py — GNSS 失联时长 × seeds 的重锚稳健性扫描。

@file    run_reanchor_sweep.py
@module  仿真/固件 SIL（GNSS 失联重锚验证）
@brief   调用 fw_sil 真实导航二进制，扫描不同 GNSS 失联窗口，输出
          hold_end/max_roll/final_alt/pos_degraded 的 JSON。
@details 输入：--bin SIL 二进制；--seeds；--windows A:B,...；处理：逐 case
          subprocess 运行并解析 SIL_REAL_JSON；输出：JSON + 控制台表。
@theory  GNSS 失联期间位置保持会累积误差；重捕获时必须重锚目标，否则
          大姿态修正会与航向/位置环耦合发散。@ref VER-003 §3.21
@frame   NED/FRD（由 SIL 内部处理）
@units   s、m、deg
@note    L2 SIL；不是实机结论。
@test    py -3.12 run_reanchor_sweep.py --bin /tmp/reanchor/fw_real.exe
@see     VER-003、TandemVec_FCS/test_host/fw_sil/run_real_acceptance.py
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
from pathlib import Path


def run_case(binary: str, seed: int, start: float, end: float,
             seconds: float = 40.0) -> dict:
    cmd = [
        binary, "--real-nav", "--profile", "mag_nominal", "--pos-hold",
        "--seconds", str(seconds), "--seed", str(seed),
        "--gnss-outage", f"{start}:{end}",
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    text = proc.stdout
    m = re.search(r"SIL_REAL_JSON (.*)", text)
    if not m:
        return {"seed": seed, "outage_s": end - start, "error": "no_json",
                "returncode": proc.returncode}
    try:
        d = json.loads(re.sub(r"(?<=[:\[,])-?(?:nan|inf)(?=[,}\]])", "null",
                              m.group(1), flags=re.I))
    except Exception as exc:  # noqa: BLE001
        return {"seed": seed, "outage_s": end - start, "error": str(exc)}
    return {
        "seed": seed,
        "outage_s": round(end - start, 2),
        "final_alt": d.get("final_alt"),
        "hold_end_rms": d.get("hold_end_rms"),
        "pos_rms": d.get("pos_rms"),
        "max_roll": d.get("max_roll"),
        "max_pitch": d.get("max_pitch"),
        "pos_degraded": d.get("pos_degraded"),
        "mag_fused": d.get("mag_fused"),
        "mag_gate": d.get("mag_gate"),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", required=True)
    ap.add_argument("--seeds", default="1,2,3,7")
    ap.add_argument("--windows", default="20:20.5,20:21,20:22,20:25,20:30")
    ap.add_argument("--out", default="sim-data/reanchor-sweep-2026-10-25")
    args = ap.parse_args()
    seeds = [int(s) for s in args.seeds.split(",") if s]
    windows = []
    for w in args.windows.split(","):
        a, b = w.split(":")
        windows.append((float(a), float(b)))
    rows = []
    for start, end in windows:
        for seed in seeds:
            row = run_case(args.bin, seed, start, end)
            rows.append(row)
            print(row)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "sweep.json").write_text(
        json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8")
    print("saved:", out / "sweep.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
