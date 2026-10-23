# -*- coding: utf-8 -*-
"""run_sweep.py — EKF 参数敏感性扫描（15 维，按组运行）。

@file    run_sweep.py
@module  analysis/EKF 优化研究（灵敏度）
@brief   对 GNSS/Q/静止辅助/失联重捕获四组参数做单因子扫描。
@details 组别：gnss（R 地板、NIS 门限、双矢量噪声、延迟回放）、qnoise（Q 标定）、
         static（ZUPT/重力/静止陀螺噪声与检测器阈值）、outage（失联膨胀/重捕获）。
         输出：sim-data/opt/sweep_<group>.json。
@theory  R/Q 标定与一致性 @ref REF-121 §2.2.1 p.33、§3.10 p.129–139；
         NIS 门限的 χ² 分位含义 @ref REF-121 §7.3.1 p.255–259。
@frame  NED + FRD。
@units   SI；噪声 std 按 m/s²、m/s、rad、rad/s 区分。
@note    单因子扫描不覆盖参数耦合（如 R 与 Q 同调）；"最优值"只在所扫区间内成立。
         GNSS R 地板经实测不可收紧，扫描结论仅供定位敏感方向。
@see     opt_ekf.py、run_baseline.py、run_variants.py
"""
import copy
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from opt_ekf import Scenario, build_cfg, OptEkf
from run_baseline import base_scenarios

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                   "sim-data", "opt")
os.makedirs(OUT, exist_ok=True)

SEEDS = (1, 2, 3)


def _run(spec, cfgd, opts, seeds=SEEDS, t0=None, t1=None):
    """跑一个配置，返回多种子平均指标。
    扫描统一启用静态门控 + 现实 P0 航向（排除已定位机制干扰，专测参数效应）"""
    spec = copy.deepcopy(spec)
    if spec.get("gnss"):
        spec["gnss"] = dict(spec["gnss"], static_gate=True)
    spec.setdefault("p0_hdg", 0.2)
    rows = []
    for seed in seeds:
        sc = Scenario(spec, lambda: OptEkf(cfgd, opts))
        sc.run(seed=seed)
        rows.append(sc.metrics(t0=t0, t1=t1))
    agg = {}
    for k in rows[0]:
        vals = [r[k] for r in rows]
        agg[k] = float(np.mean(vals))
    return agg


def _spec(name, **mods):
    sp = copy.deepcopy(base_scenarios()[name])
    for k, v in mods.items():
        if isinstance(v, dict) and isinstance(sp.get(k), dict):
            sp[k] = {**sp[k], **v}
        else:
            sp[k] = v
    return sp


# ---------------------------------------------------------------------------
# 扫描定义：每项 = (组, 名称, 说明, 场景, [(标签, spec修改, cfg覆盖, opts覆盖, 窗口)])
# ---------------------------------------------------------------------------
def sweeps():
    """返回 15 个扫描维度定义列表（group/name/desc/scenarios/points），供 run_group 消费。"""
    S = []
    add = lambda g, n, d, scs, points: S.append(dict(group=g, name=n, desc=d,
                                                     scenarios=scs, points=points))
    # ---- GNSS 组 ----
    add("gnss", "gnss_r_floor", "GNSS R 地板缩放（实际噪声固定 2/3/0.15/0.25）",
        ["SC2_cruise_gnss", "SC5_outage_reacq", "SC6_outliers"],
        [(f"x{v}", dict(gnss={"r_floor": (2*v, 3*v, 0.15*v, 0.25*v)}), {}, {}, None)
         for v in (0.25, 0.5, 0.75, 1.0, 1.5, 2.0)])
    add("gnss", "gnss_nis", "GNSS NIS 门限",
        ["SC2_cruise_gnss", "SC6_outliers"],
        [(f"{v}", {}, {"nis_gnss": v}, {}, None)
         for v in (10.0, 16.0, 30.0, 45.0, 80.0)])
    add("gnss", "dv_noise", "双矢量航向基础噪声缩放",
        ["SC2_cruise_gnss"],
        [(f"x{v}", {"fw_overrides": {"dv_base_noise": 0.08 * v}}, {}, {}, None)
         for v in (0.5, 1.0, 2.0, 4.0)])
    add("gnss", "delay_age", "GNSS 量测年龄（延迟回放）",
        ["SC3_cruise_turn"],
        [(f"{v}s", {"gnss": {"age": v}}, {}, {}, None) for v in
         (0.0, 0.015, 0.05, 0.12)] +
        [("0.05s_noreplay", {"gnss": {"age": 0.05}, "no_replay": True}, {}, {}, None)])
    # ---- Q 噪声组 ----
    add("qnoise", "q_accel", "加计白噪声 Q 假设（真实传感器 0.05）",
        ["SC2_cruise_gnss", "SC3_cruise_turn"],
        [(f"{v}", {}, {"accel_std": v}, {}, None)
         for v in (0.05, 0.1, 0.25, 0.5, 1.0)])
    add("qnoise", "q_gyro", "陀螺白噪声 Q 假设（真实传感器 0.0015）",
        ["SC2_cruise_gnss", "SC4_transition"],
        [(f"{v}", {}, {"gyro_std": v}, {}, None)
         for v in (0.0005, 0.0015, 0.003, 0.006)])
    add("qnoise", "q_gyro_bias", "陀螺零偏游走 Q 缩放",
        ["SC8_bias_jump", "SC5_outage_reacq"],
        [(f"x{v}", {}, {"gyro_bias_std": 0.00524 * v}, {}, None)
         for v in (0.25, 1.0, 4.0)])
    add("qnoise", "q_accel_bias", "加计零偏游走 Q 缩放",
        ["SC8_bias_jump"],
        [(f"x{v}", {}, {"accel_bias_std": 0.01 * v}, {}, None)
         for v in (0.25, 1.0, 4.0)])
    # ---- 静止辅助组 ----
    add("static", "zupt_noise", "ZUPT 速度噪声（档3 vel_ne）缩放",
        ["SC1_static_align"],
        [(f"x{v}", {"fw_overrides": {"tier3_vel_ne": 0.02 * v,
                                     "tier3_vel_d": 0.04 * v}}, {}, {}, None)
         for v in (0.5, 1.0, 2.0, 4.0)])
    add("static", "sg_noise", "静止陀螺量测噪声",
        ["SC1_static_align"],
        [(f"{v:g}", {"fw_overrides": {"sg_noise": v}}, {}, {}, None)
         for v in (1e-5, 1e-4, 1e-3, 1e-2)])
    add("static", "grav_noise", "重力方向量测噪声（档3）缩放",
        ["SC1_static_align"],
        [(f"x{v}", {"fw_overrides": {"tier3_grav": 0.03 * v}}, {}, {}, None)
         for v in (0.5, 1.0, 2.0, 4.0)])
    add("static", "zupt_nis", "ZUPT NIS 门限（自适应关，暴露门控作用）",
        ["SC2_cruise_gnss"],
        [(f"{v}", {"static_aid": {"enable": True, "tier": True,
                                  "zupt_adapt": False}},
          {"nis_zupt": v}, {}, None)
         for v in (16.3, 40.0, 100.0, 1000.0)] +
        [("adapt_on", {"static_aid": {"enable": True, "tier": True,
                                      "zupt_adapt": True}},
          {"nis_zupt": 1000.0}, {}, None)])
    add("static", "sd_enter", "静止检测进入帧数",
        ["SC1_static_align", "SC2_cruise_gnss", "SC7_hover_vib"],
        [(f"{v}", {"fw_overrides": {"sd_enter": v}}, {}, {}, None)
         for v in (10, 20, 40)])
    add("static", "sd_exit", "静止检测退出帧数",
        ["SC1_static_align", "SC2_cruise_gnss", "SC7_hover_vib"],
        [(f"{v}", {"fw_overrides": {"sd_exit": v}}, {}, {}, None)
         for v in (3, 5, 10, 20)])
    add("static", "sd_da", "静止检测 Δaccel 门限",
        ["SC1_static_align", "SC2_cruise_gnss", "SC7_hover_vib"],
        [(f"{v}", {"fw_overrides": {"sd_da": v}}, {}, {}, None)
         for v in (0.25, 0.5, 1.0)])
    # ---- 失联/重捕获组 ----
    add("outage", "outage_infl", "失联协方差膨胀速率缩放",
        ["SC5_outage_reacq"],
        [(f"x{v}", {}, {"outage_rate": [1.44*v, 1.44*v, 3.24*v,
                                        0.1225*v, 0.1225*v, 0.25*v]}, {},
          (30.0, 80.0))
         for v in (0.5, 1.0, 2.0, 4.0)])
    add("outage", "reacq_infl", "重捕获 R 膨胀缩放",
        ["SC5_outage_reacq"],
        [(f"x{v}", {}, {"reacq_r_infl": [4*v, 4*v, 16*v, 0.25*v, 0.25*v,
                                         0.49*v]}, {}, (80.0, 95.0))
         for v in (0.5, 1.0, 2.0)])
    add("outage", "outage_cap", "失联膨胀 scale 封顶",
        ["SC5_outage_reacq"],
        [(f"{v}", {}, {"outage_cap": v}, {}, (30.0, 80.0))
         for v in (1.0, 2.0, 4.0, 8.0)])
    return S


def run_group(group):
    """跑一个参数组（gnss/qnoise/static/outage）的全部扫描点，返回可 JSON 化的结果 dict。"""
    out = {}
    for sw in [s for s in sweeps() if s["group"] == group]:
        print(f"[{sw['name']}] {sw['desc']}")
        rows = []
        for label, spec_mod, cfg_mod, opt_mod, win in sw["points"]:
            row = {"label": label}
            for scn in sw["scenarios"]:
                spec = _spec(scn, **spec_mod)
                t0, t1 = win if win else (None, None)
                m = _run(spec, build_cfg(cfg_mod), opt_mod, t0=t0, t1=t1)
                row[scn] = {k: round(v, 4) for k, v in m.items()}
            rows.append(row)
        out[sw["name"]] = dict(desc=sw["desc"], scenarios=sw["scenarios"],
                               rows=rows)
        # 打印关键指标
        for r in rows:
            s0 = sw["scenarios"][0]
            print(f"  {r['label']:>14s}  pos_ne={r[s0].get('pos_ne_rmse', float('nan')):.3f} "
                  f"vel_ne={r[s0].get('vel_ne_rmse', float('nan')):.3f} "
                  f"yaw={r[s0].get('yaw_rmse_deg', float('nan')):.1f} "
                  f"gnssNIS={r[s0].get('gnss_nis_mean', float('nan')):.1f} "
                  f"accept={r[s0].get('gnss_accept', float('nan')):.3f}")
    with open(os.path.join(OUT, f"sweep_{group}.json"), "w",
              encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(f"\n[完成] 写入 sweep_{group}.json")


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("group", choices=["gnss", "qnoise", "static", "outage",
                                      "all"])
    args = ap.parse_args()
    if args.group == "all":
        for g in ("gnss", "qnoise", "static", "outage"):
            run_group(g)
    else:
        run_group(args.group)
