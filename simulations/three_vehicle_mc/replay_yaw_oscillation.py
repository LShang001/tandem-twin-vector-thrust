"""replay_yaw_oscillation.py — 用 SIL 逐秒轨迹回放偏航振荡检测器。

@file    replay_yaw_oscillation.py
@module  仿真/三机型快速 MC（航向失效回放）
@brief   解析固件 SIL 的 `t= / roll= / pitch= / yaw=` 逐秒行，对 yaw 解卷绕
          得到 1 Hz 角速度，回放 `YawOscillationDetector`，输出首次触发时刻。
@details 输入：一个或多个 SIL trace 日志；处理：正则提取、角度解卷绕、
          1 Hz 差分、检测器状态机；输出：JSON（每个文件的触发时刻/最大率）。
@theory  位置保持 runaway 的根因是航向环极限环；检测器应在位置大幅漂移前
          由持续高偏航角速度触发。@ref VER-003 §3.14
@frame   NED yaw（deg）→ 机体系 z 角速度近似
@units   deg、deg/s、s
@note    L2 SIL 轨迹回放，不是实机验证；1 Hz 采样会低估瞬时峰值。
@test    py -3.12 replay_yaw_oscillation.py --trace <log>
@see     TandemVec_FCS/include/YawOscillationDetector.h、VER-003
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path


def parse_trace(path: Path):
    rows = []
    pat = re.compile(
        r"t=\s*([\d.]+).*?roll=\s*(-?[\d.]+) pitch=\s*(-?[\d.]+) "
        r"yaw=\s*(-?[\d.]+)"
    )
    for line in path.read_text(errors="replace").splitlines():
        m = pat.search(line)
        if m:
            rows.append(
                (float(m.group(1)), float(m.group(2)), float(m.group(3)),
                 float(m.group(4)))
            )
    return rows


def unwrap_delta(prev: float, cur: float) -> float:
    return ((cur - prev + 180.0) % 360.0) - 180.0


def replay(rows, rate_threshold=35.0, trigger_s=1.0, recover_s=3.0):
    if len(rows) < 2:
        return None
    rates = []
    for i in range(1, len(rows)):
        dt = rows[i][0] - rows[i - 1][0]
        if dt <= 0:
            continue
        rates.append((rows[i][0], unwrap_delta(rows[i - 1][3], rows[i][3]) / dt))

    high = 0.0
    quiet = 0.0
    oscillating = False
    first_trigger = None
    for t, rate in rates:
        rate_abs = abs(rate)
        if rate_abs > rate_threshold:
            high += 1.0
            quiet = 0.0
            if high >= trigger_s and not oscillating:
                oscillating = True
                first_trigger = t
        else:
            high = 0.0
            if rate_abs < 10.0:
                quiet += 1.0
                if quiet >= recover_s:
                    oscillating = False
                    quiet = 0.0
            else:
                quiet = 0.0
    return {
        "first_trigger_s": first_trigger,
        "max_abs_rate_dps": round(max(abs(r) for _, r in rates), 2),
        "final_yaw_deg": rows[-1][3],
        "final_pos": None,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--trace", nargs="+", required=True)
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    result = {}
    for path in args.trace:
        p = Path(path)
        rows = parse_trace(p)
        result[p.name] = replay(rows)
        print(p.name, result[p.name])
    if args.out:
        Path(args.out).write_text(
            json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print("saved:", args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
