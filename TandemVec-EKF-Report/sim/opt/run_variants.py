# -*- coding: utf-8 -*-
"""
EKF 算法变体 A/B 对比
=====================
以固件现值（含 25Hz 气压辅助）为基准，量化各算法改进的收益/代价：
  baro_off     —— 关闭气压辅助（旧仿真口径）：量化失联期垂直漂移收益
  q_calib      —— Q 标定（accel_std 0.25→0.05，对齐真实传感器噪声）
  r_tight      —— GNSS R 地板 ×0.5（更紧的 GNSS 融合）
  adr          —— 新息方差自适应 R
  combo        —— q_calib + r_tight + adr 组合
  zupt_nis16   —— ZUPT NIS 门限 1000→16.3（χ²(3) 99.9%，恢复统计意义）
  h2_off       —— 标量航向退回旧 H=e8（量化 H2 投影修复价值）
  reacq_decay  —— 重捕获窗口内 R 膨胀逐帧衰减
输出 sim-data/opt/variants.json
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

SEEDS = (1, 2, 3, 4, 5)

VARIANTS = {
    "base": dict(cfg={}, fw={}, spec={}, opts={},
                 desc="固件现值（含 25Hz 气压辅助、静止强制、P0=π）"),
    "gate": dict(cfg={}, fw={}, spec={"gnss": {"static_gate": True}}, opts={},
                 desc="静态门控：GNSS 速度>1.5 m/s 强制退出静止（修复F1）"),
    "yawfix": dict(cfg={}, fw={"dv_min_speed": 0.5},
                   spec={"p0_hdg": 0.2}, opts={"yaw_p_floor": 0.05,
                                               "yaw_reacq": True},
                   desc="航向修复包：P0=0.2 + 速度门0.5 + P地板0.05 + 再锚定（修复F2）"),
    "gate_yawfix": dict(cfg={}, fw={"dv_min_speed": 0.5},
                        spec={"gnss": {"static_gate": True}, "p0_hdg": 0.2},
                        opts={"yaw_p_floor": 0.05, "yaw_reacq": True},
                        desc="组合：静态门控 + 航向修复包"),
    "baro_off": dict(cfg={}, fw={}, spec={"baro": {"enable": False}}, opts={},
                     desc="关闭气压辅助（量化垂直通道价值）"),
    "q_calib": dict(cfg={"accel_std": 0.05}, fw={}, spec={}, opts={},
                    desc="Q 标定：加计白噪声 0.25→0.05（=真实传感器）"),
    "r_tight": dict(cfg={}, fw={},
                    spec={"gnss": {"r_floor": (1.0, 1.5, 0.075, 0.125)}},
                    opts={}, desc="GNSS R 地板 ×0.5"),
    "adr": dict(cfg={}, fw={}, spec={},
                opts={"adaptive_r": True, "adr_window": 40,
                      "adr_max_scale": 8.0},
                desc="新息方差自适应 R"),
    "combo": dict(cfg={"accel_std": 0.05}, fw={},
                  spec={"gnss": {"static_gate": True,
                                 "r_floor": (1.0, 1.5, 0.075, 0.125)}},
                  opts={"adaptive_r": True, "adr_window": 40,
                        "adr_max_scale": 8.0},
                  desc="组合：静态门控+Q标定+R收紧+自适应R"),
    "zupt_nis16": dict(cfg={"nis_zupt": 16.3}, fw={}, spec={}, opts={},
                       desc="ZUPT NIS 门限 1000→16.3（χ²(3) 99.9%）"),
    "h2_off": dict(cfg={}, fw={}, spec={"yaw_aid": {"enable": True, "h2": False}},
                   opts={}, desc="标量航向退回旧 H=e8（无倾斜补偿）"),
    "reacq_decay": dict(cfg={}, fw={}, spec={},
                        opts={"reacq_decay": True, "reacq_window": 6.0},
                        desc="重捕获 R 膨胀窗口内逐帧衰减"),
    "split": dict(cfg={}, fw={}, spec={"gnss": {"split": True}}, opts={},
                  desc="GNSS 位置/速度分离序贯更新（独立门控）"),
}

# 每个变体的评估场景 + 统计窗口（None=全窗口）
EVAL = {
    "gate": [("SC2_cruise_gnss", (60.0, 120.0)),
             ("SC3_cruise_turn", (30.0, 90.0)),
             ("SC8_bias_jump", (35.0, 60.0)),
             ("SC5_outage_reacq", None)],
    "yawfix": [("SC4_transition", None),
               ("SC3_cruise_turn", (30.0, 90.0))],
    "gate_yawfix": [("SC2_cruise_gnss", (60.0, 120.0)),
                    ("SC3_cruise_turn", (30.0, 90.0)),
                    ("SC4_transition", None),
                    ("SC5_outage_reacq", None),
                    ("SC8_bias_jump", (35.0, 60.0))],
    "baro_off": [("SC5_outage_reacq", (30.0, 80.0)),
                 ("SC4_transition", None)],
    "q_calib": [("SC2_cruise_gnss", (60.0, 120.0)),
                ("SC3_cruise_turn", (30.0, 90.0)),
                ("SC8_bias_jump", (35.0, 60.0))],
    "r_tight": [("SC2_cruise_gnss", (60.0, 120.0)),
                ("SC6_outliers", None)],
    "adr": [("SC5_outage_reacq", (80.0, 95.0)),
            ("SC6_outliers", None)],
    "combo": [("SC2_cruise_gnss", (60.0, 120.0)),
              ("SC5_outage_reacq", (30.0, 80.0)),
              ("SC6_outliers", None),
              ("SC8_bias_jump", (35.0, 60.0))],
    "zupt_nis16": [("SC2_cruise_gnss", (60.0, 120.0)),
                   ("SC1_static_align", None)],
    "h2_off": [("SC4_transition", (15.0, 60.0))],
    "reacq_decay": [("SC5_outage_reacq", (80.0, 95.0))],
    "split": [("SC2_cruise_gnss", (60.0, 120.0)),
              ("SC6_outliers", None),
              ("SC5_outage_reacq", None)],
}

KEYS = ["pos_ne_rmse", "pos_d_rmse", "vel_ne_rmse", "vel_d_rmse",
        "yaw_rmse_deg", "att_rmse_deg", "gnss_nis_mean", "gnss_accept",
        "gnss_conservative", "ba_err", "bg_err_radps"]


def run_variant(vname, scn, win, seeds=SEEDS):
    v = VARIANTS[vname]
    spec = copy.deepcopy(base_scenarios()[scn])
    for k, val in v["spec"].items():
        if isinstance(val, dict) and isinstance(spec.get(k), dict):
            spec[k] = {**spec[k], **val}
        else:
            spec[k] = val
    cfgd = build_cfg(v["cfg"])
    rows = []
    for seed in seeds:
        sc = Scenario(spec, lambda: OptEkf(cfgd, v["opts"]))
        sc.run(seed=seed)
        t0, t1 = win if win else (None, None)
        rows.append(sc.metrics(t0=t0, t1=t1))
    agg = {}
    for k in rows[0]:
        agg[k] = float(np.mean([r[k] for r in rows]))
    return agg


def main():
    # 基准：所有涉及场景
    base_scns = sorted({s for lst in EVAL.values() for s, _ in lst})
    baseline = {}
    for scn in base_scns:
        baseline[scn] = run_variant("base", scn, None)
        print(f"[base/{scn}] pos_ne={baseline[scn]['pos_ne_rmse']:.3f} "
              f"pos_d={baseline[scn]['pos_d_rmse']:.3f} "
              f"yaw={baseline[scn]['yaw_rmse_deg']:.1f}")
    out = {"baseline": {k: {kk: round(vv, 4) for kk, vv in v.items()}
                        for k, v in baseline.items()},
           "variants": {}}
    for vname in VARIANTS:
        if vname == "base":
            continue
        res = {}
        for scn, win in EVAL[vname]:
            res[scn] = run_variant(vname, scn, win)
            # 相对基准（同窗口下基准重算）
            base_same = run_variant("base", scn, win)
            d = {k: round(res[scn][k] - base_same[k], 4) for k in KEYS
                 if k in res[scn]}
            print(f"[{vname}/{scn}] "
                  f"pos_ne={res[scn]['pos_ne_rmse']:.3f} "
                  f"(Δ{res[scn]['pos_ne_rmse']-base_same['pos_ne_rmse']:+.3f}) "
                  f"pos_d={res[scn]['pos_d_rmse']:.3f} "
                  f"(Δ{res[scn]['pos_d_rmse']-base_same['pos_d_rmse']:+.3f}) "
                  f"yaw={res[scn]['yaw_rmse_deg']:.1f} "
                  f"(Δ{res[scn]['yaw_rmse_deg']-base_same['yaw_rmse_deg']:+.1f})")
            res[scn + "_delta_vs_base"] = d
            res[scn + "_base_same_window"] = {
                k: round(v, 4) for k, v in base_same.items()}
        out["variants"][vname] = dict(desc=VARIANTS[vname]["desc"], results=res)
    with open(os.path.join(OUT, "variants.json"), "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(f"\n[完成] 写入 {os.path.join(OUT, 'variants.json')}")


if __name__ == "__main__":
    main()
