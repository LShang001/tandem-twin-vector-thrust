# -*- coding: utf-8 -*-
"""从 200 Hz 追踪 CSV 直接核对 GNSS 杆臂注入符号（不做任何姿态反推）。
@file    check_lever_sign.py
@brief   比较 SIL 实际注入的 lever_ned、量测−真值偏置、以及滤波器位置误差响应。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@frame   NED
@units   m

用法：py -3.12 sim/check_lever_sign.py <csv> [<csv> ...]
"""
import csv
import math
import os
import sys


def mean(vals):
    vals = [v for v in vals if v == v]
    return sum(vals) / len(vals) if vals else float("nan")


def load(path, t_min=10.0):
    rows = []
    with open(path, encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            try:
                if float(r["t"]) < t_min:
                    continue
                rows.append(r)
            except (KeyError, ValueError):
                continue
    return rows


def f(r, k):
    try:
        return float(r[k])
    except (KeyError, ValueError):
        return float("nan")


def report(path):
    rows = load(path)
    if not rows:
        print(os.path.basename(path), "no rows")
        return
    lv_n = mean([f(r, "lv_n") for r in rows])
    lv_e = mean([f(r, "lv_e") for r in rows])
    lv_d = mean([f(r, "lv_d") for r in rows])
    bias_n = mean([f(r, "msr_n") - f(r, "true_n") for r in rows])
    bias_e = mean([f(r, "msr_e") - f(r, "true_e") for r in rows])
    err_n = mean([f(r, "ekf_n") - f(r, "true_n") for r in rows])
    err_e = mean([f(r, "ekf_e") - f(r, "true_e") for r in rows])
    # 沿航迹单位向量（用真值位置差分，避免依赖姿态约定）
    tr = [(f(r, "true_n"), f(r, "true_e")) for r in rows]
    vx = vy = 0.0
    n = 0
    for i in range(1, len(tr)):
        dn, de = tr[i][0] - tr[i - 1][0], tr[i][1] - tr[i - 1][1]
        m = math.hypot(dn, de)
        if m > 1e-6:
            vx += dn / m
            vy += de / m
            n += 1
    print(f"{os.path.basename(path)}")
    print(f"  SIL 注入 lever_ned    N {lv_n:8.2f}  E {lv_e:8.2f}  D {lv_d:8.2f}")
    print(f"  量测−IMU 真值偏置      N {bias_n:8.2f}  E {bias_e:8.2f}")
    print(f"  滤波器位置误差(ekf−true) N {err_n:8.2f}  E {err_e:8.2f}")
    if n:
        print(f"  平均沿航迹单位向量      N {vx / n:8.3f}  E {vy / n:8.3f}"
              f"   (20 m 杆臂应沿此方向)")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        report(p)
