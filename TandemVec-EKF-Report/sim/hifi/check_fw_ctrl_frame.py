"""
@file    check_fw_ctrl_frame.py
@brief   hifi 模型系 ↔ 固件体系的映射物理符号判定
@module  仿真验证（固件-仿真一体化）
@details 枚举 24 个真旋转，用物理力矩方向判定 hifi plant 与固件控制链是否存在刚性置换。
@frame   hifi 模型系 / 固件体系（FRD）
@units   SI（力矩 N·m、角度 rad）


背景
----
hifi（`hifi_core.Dynamics`）与固件分配器（`TandemVec_Propulsion.h::computeWrench`）
共用同一"模型系"物理量：x'=推力轴、y'=下摆、z'=上摆；M=[Mx,My,Mz] 同轴、
公式逐行相同。但固件控制链的姿态/角速率通道用的是**存档体系**
（`flight_control.cpp`：滚转=绕 x_b、俯仰=绕 y_b、偏航=绕 z_b），SIL plant 再按
`M_body=(-Mz',My',-Mx')` 把它嵌进 FRD（`fw_sil_main.cpp:57`）。

本脚本不靠命名考古，用**物理符号测试**判定两系关系：给定模型系 +1° 姿态误差，
把该状态按候选刚性置换喂给固件控制链（`fwapi.GncTrack`），用固件返回的执行器
指令按 `computeWrench` 公式算**模型系物理力矩** M；正确映射应满足 M·e > 0
（力矩指向目标，hifi plant 会朝减小误差的方向转）。

候选集 = 全部 24 个真旋转（带符号置换、det=+1），不是两个手挑的猜测。

用法
----
    py -3.12 sim/hifi/check_fw_ctrl_frame.py [--json OUT]
"""
import argparse
import itertools
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from fwapi import GncTrack  # noqa: E402
from hifi_core import P, quat_conj, quat_mult  # noqa: E402

R2D = 180.0 / np.pi
D2R = np.pi / 180.0


def q_axis(axis, deg):
    """绕模型系单轴的小角度四元数（w,x,y,z，与 hifi/fwapi 一致）。"""
    half = 0.5 * deg * D2R
    v = np.zeros(3)
    v[axis] = np.sin(half)
    return np.array([np.cos(half), *v])


def rot_matrix_to_quat(m):
    """3×3 旋转矩阵 → 四元数 (w,x,y,z)。仅用于候选置换（正交、det=+1）。"""
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def proper_signed_permutations():
    """全部 24 个真旋转（置换 + 逐轴符号，det=+1）。"""
    out = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            m = np.zeros((3, 3))
            for row, col in enumerate(perm):
                m[row, col] = signs[row]
            if np.linalg.det(m) > 0.0:
                out.append((perm, signs, m))
    return out


def wrench_model(df, dt, dw, w0):
    """computeWrench 的模型系物理力矩（与 TandemVec_Propulsion.h 逐行一致）。

    wf = w0·√(1+dw)、wt = w0·√(1−dw)；Jp·ω̇ 项在稳态指令阶跃下不参与符号判定。
    """
    wf = w0 * np.sqrt(max(0.0, 1.0 + dw))
    wt = w0 * np.sqrt(max(0.0, 1.0 - dw))
    Tf, Tt = P["kT"] * wf * wf, P["kT"] * wt * wt
    Qf, Qt = P["kQ"] * wf * wf, P["kQ"] * wt * wt
    cf, sf = np.cos(df), np.sin(df)
    ct, st = np.cos(dt), np.sin(dt)
    return np.array([
        -Qf * cf + Qt * ct,
        -P["b"] * Tt * st - Qf * sf,
        P["a"] * Tf * sf - Qt * st,
    ])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", default=None)
    ap.add_argument("--err-deg", type=float, default=1.0)
    args = ap.parse_args()

    ctrl = GncTrack()
    throttle = 50.0
    w0 = (throttle / 100.0) * P["wMax"]

    cands = proper_signed_permutations()
    rows = []
    for idx, (perm, signs, m) in enumerate(cands):
        q_map = rot_matrix_to_quat(m)
        name = "".join("xyz"[perm[r]] + ("+" if signs[r] > 0 else "-")
                       for r in range(3))
        for axis in range(3):
            q_cur = np.array([1.0, 0.0, 0.0, 0.0])
            q_tgt = q_axis(axis, args.err_deg)
            out = ctrl.step(
                quat=tuple(quat_mult(q_cur, quat_conj(q_map))),
                omega=(0.0, 0.0, 0.0),
                q_target=tuple(quat_mult(q_tgt, quat_conj(q_map))),
                throttle_pct=throttle,
                dt=0.005,
                heading_hold_allowed=True,
            )
            df = float(out["delta_f_deg"]) * D2R
            dt = float(out["delta_t_deg"]) * D2R
            dw = float(np.clip(out["dw"], -P["dwMax"], P["dwMax"]))
            wr = wrench_model(df, dt, dw, w0)
            e = np.zeros(3)
            e[axis] = 1.0
            rows.append({
                "candidate": f"{idx:02d}:{name}",
                "err_axis": "xyz"[axis],
                "M_model": [float(x) for x in wr],
                "proj_along_err": float(wr[axis]),
                "correct_dir": bool(wr[axis] > 0.0),
                "alpha_fw_dps2": [float(x) for x in out["alpha_ref_dps2"]],
                "delta_f_deg": float(out["delta_f_deg"]),
                "delta_t_deg": float(out["delta_t_deg"]),
                "dw": dw,
            })

    print(f"=== hifi 模型系 → 固件体系：{len(cands)} 个真旋转候选的物理符号判定 ===")
    print(f"(模型系 +{args.err_deg:g}° 误差应产生 M·e > 0；M 用 computeWrench 物理力矩)")
    by_cand = {}
    for r in rows:
        by_cand.setdefault(r["candidate"], []).append(r)

    passed = []
    for cand, rs in by_cand.items():
        ok = all(r["correct_dir"] for r in rs)
        proj = " ".join(f"{r['proj_along_err']:+.5f}" for r in rs)
        if ok:
            passed.append(cand)
            print(f"{cand:16s} PASS  M·e = [{proj}]")
        elif all(not r["correct_dir"] for r in rs):
            print(f"{cand:16s} FAIL  M·e = [{proj}]  (三轴全反)")

    print(f"\n--- 通过三轴方向测试的映射：{len(passed)} / {len(by_cand)} ---")
    for c in passed:
        print(f"  {c}")

    if not passed:
        print("\n结论：**不存在**真旋转能把 hifi 模型系与固件控制链对齐到同一物理响应。")
        print("⇒ hifi plant 与固件 plant 不是同一刚体坐标约定（差反射/惯性轴分配），")
        print("  机械替换 FirmwareControl 需要改 hifi 的 plant，不是换控制器。")
    elif len(passed) == 1:
        print(f"\n唯一可行映射：{passed[0]}")
    else:
        print("\n多个映射同时通过 ⇒ 该静态度量不足以判别，需闭环响应判别。")

    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump({"rows": rows, "passed": passed,
                       "n_candidates": len(by_cand)},
                      f, ensure_ascii=False, indent=2)
        print(f"\nwritten: {args.json}")
    return 0 if len(passed) == 1 else 1


if __name__ == "__main__":
    sys.exit(main())
