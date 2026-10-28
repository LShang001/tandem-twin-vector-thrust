# -*- coding: utf-8 -*-
"""归档主表 vs 最终默认二进制的逐格比对（§4.22.3 再基线）。

@file    compare_missile_rebaseline.py
@brief   读两份 `run_missile_sil_mc.py` 产物，逐条件输出 pos/att/vel 的 med/p95/mx 差值
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 为什么必须做：报告 §4 主表产出于 `vib_high` 否决仍开、且无陀螺 Q 夹持与
         `auto_delay_valid` 修复的二进制（`prod_main40` md5 `d320af…`）。默认值翻转
         会影响所有曾被该否决封住应用的格子，因此归档数字必须逐格重述或明确"未变"。
         比对口径：同 runner、同 seeds、同条件表，只有二进制不同；差值按 med 的相对
         百分比给出，并标出 med 变化 >5% 或 mx 变化 >15% 的格子。
"""
import json
import sys

BASE = "D:/纵列双发矢量推力飞行器/TandemVec-EKF-Report/sim-data/"
OLD = BASE + "missile-sil-2026-10-27_prod_main40.json"
NEW = (sys.argv[1] if len(sys.argv) > 1 else
       BASE + "missile-sil-2026-10-27_main40_defreb.json")
METRICS = ["pos_rmse", "pos_max", "att_rmse", "vel_rmse"]


def load(path):
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


SUFFIX = sys.argv[2] if len(sys.argv) > 2 else "defreb"


def strip_suffix(key):
    suf = "_" + SUFFIX
    return key[: -len(suf)] if key.endswith(suf) else key


def main():
    old = load(OLD)["groups"]
    new = load(NEW)["groups"]
    newmap = {strip_suffix(k): v for k, v in new.items()
              if isinstance(v, dict) and "pos_rmse" in v}
    common = [k for k in old if isinstance(old[k], dict)
              and "pos_rmse" in old[k] and k in newmap]
    print(f"old cells={len(old)} new cells={len(newmap)} 可比={len(common)}")
    if not common:
        print("OLD keys:", sorted(old)[:12])
        print("NEW keys:", sorted(newmap)[:12])
        return 1
    hdr = f"{'条件':<26}" + "".join(f"{m + ' med':>16}{'' if i else ''}"
                                   for i, m in enumerate(METRICS[:2]))
    print(f"{'条件':<26}" + "".join(f"{m:>22}" for m in METRICS))
    print(f"{'':<26}" + "".join(f"{'old→new (Δ%)':>22}" for _ in METRICS))
    print("-" * 114)
    flagged = []
    for k in sorted(common):
        line = f"{k:<26}"
        for m in METRICS:
            try:
                a = old[k][m]["med"]
                b = newmap[k][m]["med"]
            except KeyError:
                line += f"{'—':>22}"
                continue
            pct = (b - a) / a * 100 if a else 0.0
            line += f"{a:>8.2f}→{b:<7.2f}{pct:+5.1f}%"
            if abs(pct) > 5.0:
                flagged.append((k, m, a, b, pct))
            amx = old[k][m].get("mx", 0.0)
            bmx = newmap[k][m].get("mx", 0.0)
            if amx and abs((bmx - amx) / amx * 100) > 15.0:
                flagged.append((k, m + ".mx", amx, bmx,
                                (bmx - amx) / amx * 100))
        print(line)
    print("\n超阈值格子（|Δmed|>5% 或 |Δmx|>15%）：")
    if not flagged:
        print("  无 —— 最终默认二进制逐格复现归档主表（可比 %d 格）" % len(common))
    for k, m, a, b, p in sorted(flagged, key=lambda r: -abs(r[4]))[:30]:
        print(f"  {k:<26}{m:<14}{a:>9.3f} → {b:<9.3f}{p:+7.1f}%")
    return 0


if __name__ == "__main__":
    sys.exit(main())
