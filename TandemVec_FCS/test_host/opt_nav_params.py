#!/usr/bin/env python3
"""连续参数优化：LPF 带宽与 hAcc 门限（CMA-ES）。

项目规范（AGENTS.md，2026-11-01）：连续参数最终调参默认用现代优化器，
网格/手扫只用于初值与可视化。

目标（越小越好，单位混合但权重固定）：
    cost = att_static + att_lowend_maneuver + 2*(pos_static + pos_lowend_maneuver)
其中姿态项取 roll/pitch RMSE 均值（deg）、位置项取 RMSE（m）；LowEnd_5Hz
用来体现 hAcc 门控收益，static 用 M8N_10Hz 用来约束正常 GNSS 下的噪声。

用法：
    python test_host/opt_nav_params.py
环境变量：
    IMUPERF_EXE  test_imu_nav_perf 编译产物路径（必填）
"""
import os
import subprocess
import sys

import cma


def make_cost(exe: str):
    def cost(x):
        lpf_g, lpf_a, hacc = x
        env = dict(os.environ)
        env.update(
            IMUPERF_OPT="1",
            IMUPERF_LPF_GYRO=f"{lpf_g:.3f}",
            IMUPERF_LPF_ACCEL=f"{lpf_a:.3f}",
            IMUPERF_HACC=f"{hacc:.3f}",
        )
        proc = subprocess.run(
            [exe], env=env, capture_output=True, text=True, timeout=1800
        )
        for line in proc.stdout.splitlines():
            if line.startswith("OPT_RESULT,"):
                return float(line.split(",")[1])
        print("评估失败，stdout 尾部：", proc.stdout[-400:], file=sys.stderr)
        print("stderr 尾部：", proc.stderr[-400:], file=sys.stderr)
        return 1.0e9

    return cost


def main():
    exe = os.environ.get("IMUPERF_EXE")
    if not exe or not os.path.isfile(exe):
        print("需要 IMUPERF_EXE 指向 test_imu_nav_perf 的编译产物", file=sys.stderr)
        return 2

    # 初值 = 当前默认（陀螺 200Hz / 加计 100Hz / 门限 5m），边界覆盖手册档位。
    x0 = [200.0, 100.0, 5.0]
    sigma0 = [80.0, 60.0, 2.5]
    bounds = [[50.0, 25.0, 2.0], [400.0, 400.0, 12.0]]
    es = cma.CMAEvolutionStrategy(
        x0,
        1.0,
        {
            # cma 4.x 要求 sigma0 为标量；各维尺度用 CMA_stds 表达。
            "CMA_stds": sigma0,
            "bounds": bounds,
            "popsize": 8,
            "maxfevals": 200,
            "verbose": -9,
            "seed": 1,
            "CMA_diagonal": False,
        },
    )
    es.optimize(make_cost(exe))
    best = es.result
    print("BEST", [round(v, 2) for v in best.xbest], "cost", round(best.fbest, 4),
          "evals", es.countevals)
    return 0


if __name__ == "__main__":
    sys.exit(main())
