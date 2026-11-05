# -*- coding: utf-8 -*-
"""@file    check_socp_joint.py
@module  测试/宿主机
@brief   SOCP 轨迹规划 + GUIDED 跟踪联合仿真结果的自动判定（逐场景阈值）
@details 读 guidance_bridge.py --out 生成的 CSV，按场景比对终端精度：最终高度
         |up|≤0.05 m、横向 |east|/|north|≤0.10 m、过程中最低高度 ≥-0.60 m
         （SIL 无地面，负值表示超调）、最大垂直速度 ≤6.0 m/s；阈值可命令行覆盖，
         退出码 0=PASS / 1=FAIL。
@frame   NED（位置/速度/姿态误差取自 SIL 真值对比）
@units   m、m/s、deg、s

用法：
  py -3.12 TandemVec_FCS/test_host/fw_sil/check_socp_joint.py tmp/socp_joint2.csv

判定（默认，可按场景覆盖）：
  - 最终高度 |up| <= 0.05 m
  - 最终横向 |east|,|north| <= 0.10 m
  - 过程中最低高度 >= -0.60 m（SIL 无地面；负值代表超调，不是撞地）
  - 最大垂直速度 <= 6.0 m/s

退出码 0 = PASS，1 = FAIL。CSV 由 guidance_bridge.py --out 生成。
"""
import argparse
import csv
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--max-final-up", type=float, default=0.05)
    ap.add_argument("--max-final-lateral", type=float, default=0.10)
    ap.add_argument("--min-up", type=float, default=-0.60)
    ap.add_argument("--max-vz", type=float, default=6.0)
    a = ap.parse_args()

    rows = list(csv.DictReader(open(a.csv, encoding="utf-8")))
    if not rows:
        print("[FAIL] CSV 为空")
        return 1

    def col(name):
        return [float(r[name]) for r in rows]

    up = col("r_up")
    e = col("r_e")
    n = col("r_n")
    vz = col("v_up")

    final_up = abs(up[-1])
    final_lat = max(abs(e[-1]), abs(n[-1]))
    min_up = min(up)
    max_vz = max(abs(v) for v in vz)

    checks = [
        ("final |up|", final_up, a.max_final_up, "<="),
        ("final lateral", final_lat, a.max_final_lateral, "<="),
        ("min up", min_up, a.min_up, ">="),
        ("max |vz|", max_vz, a.max_vz, "<="),
    ]
    ok = True
    for name, val, lim, op in checks:
        good = val <= lim if op == "<=" else val >= lim
        print(f"[{'PASS' if good else 'FAIL'}] {name}={val:.4f} {op} {lim:.4f}")
        ok &= good

    print(f"rows={len(rows)} final_up={up[-1]:.4f} min_up={min_up:.4f} "
          f"max|vz|={max_vz:.3f}")
    print("=== PASS ===" if ok else "=== FAIL ===")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
