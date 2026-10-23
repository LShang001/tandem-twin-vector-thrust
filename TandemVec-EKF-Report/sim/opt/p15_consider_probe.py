# -*- coding: utf-8 -*-
"""p15_consider_probe.py — P1-5 航向 consider 探索性探针（5 种子，非严格 Schmidt）。

@file    p15_consider_probe.py
@module  analysis/EKF 优化研究（探针）
@brief   对比 consider_heading 开关在 SC3/SC4 上的航向踢动与位置影响。
@details 实现方式：UpgradeEkf(..., {"consider_heading": True}) 把 GNSS 更新注入的 x(8)
         （航向失准角）清零，协方差仍按标准 KF 更新——这是**趋势探针**，不是严格 Schmidt。
         结果：SC3 无收益略差；SC4 过渡 kick 3.0°→0.09°、yaw 略好、位置不变。
         运行：cd TandemVec-EKF-Report/sim/opt && py -3.12 p15_consider_probe.py
@theory  Schmidt/consider 分析与航向弱可观 @ref docs/08-参考资料/notes/02-卡尔曼滤波理论.md §3 P0-2；
         航向踢动机制 @ref docs/08-参考资料/notes/04-航向与磁辅助.md。
@frame  NED + FRD；x(8) 为航向失准角。
@units   deg（结论）、m（位置）、无量纲 NIS。
@note    严格 Schmidt 应同时把 P 的航向行/列冻结；本探针没做，故结论只能定性。
@see     upgrade_candidates.py、@ref TandemVec-EKF-Report/导航算法升级评估-专业文献调研-2026-10.md
"""
import numpy as np

from upgrade_candidates import (
    UpgradeScenario, UpgradeEkf, scenario_spec,
    build_cfg, CUR_SPEC, CUR_CFG, deep_merge, extra_metrics,
)


def run_case(scn, win, consider, seeds=5):
    """对比 consider_heading 开/关：逐种子跑场景，返回指标列表（航向踢动/位置）。"""
    vals = []
    for seed in range(1, seeds + 1):
        spec = deep_merge(scenario_spec(scn), CUR_SPEC)
        sc = UpgradeScenario(
            spec,
            lambda: UpgradeEkf(build_cfg(CUR_CFG),
                               {"consider_heading": consider}),
            {"mag": None},
        )
        sc.run(seed=seed)
        m = sc.metrics(t0=win[0] if win else None,
                       t1=win[1] if win else None)
        e = extra_metrics(sc, t0=win[0] if win else None,
                          t1=win[1] if win else None)
        vals.append([m["pos_ne_rmse"], m["yaw_rmse_deg"],
                     e.get("yaw_kick_max_deg", 0.0), m["gnss_accept"]])
    return np.array(vals).mean(axis=0)


if __name__ == "__main__":
    for scn, win in [("SC3_cruise_turn", (30.0, 90.0)),
                     ("SC4_transition", None)]:
        print(f"\n{scn}")
        for consider in (False, True):
            a = run_case(scn, win, consider)
            print(f"consider={consider!s:5s}  "
                  f"pos={a[0]:6.2f} m  yaw={a[1]:5.3f}°  "
                  f"kick={a[2]:5.3f}°  acc={a[3]:.3f}")
