# -*- coding: utf-8 -*-
"""verify_p0_fixes.py — 研究仿真 P0 修复的轻量回归（A-01/A-02/A-03）。

@file    verify_p0_fixes.py
@module  analysis/EKF 研究仿真（修复回归）
@brief   锁定 2026-10-23 三项仿真修正：双子样旋转项不再双计、P0 航向=0.2、
          initialize() 自动设置 home_lla；不跑长时间 MC。
@details 输入：合成双子样增量与静基座初值；
          处理：调用 `two_sample_delta_v()` 与 `Ekf15` 的初始化/GNSS 更新；
          输出：PASS/FAIL 计数与退出码（非零即失败）。
@theory  划摇 ΔV=(2/3)(Δθ1×ΔV2+ΔV1×Δθ2) @ref REF-120 §9.6 (9.6.25) p.301；
          旋转项只配合起始姿态，@ref REF-120 §9.6.1 (9.6.17) p.298；
          P0 航向语义 @ref TandemVec_FCS/src/navigation_task.cpp、
          @ref AGENTS.md「EKF 导航三个机制性风险」（2026-08-13）。
@frame  NED + FRD；四元数 (w,x,y,z)。
@units   SI（m、m/s、rad、s）。
@note    这些是“仿真口径与固件对齐”的回归，不是固件算法测试；固件侧由
          TandemVec_FCS/test_host 覆盖。
@see     ekf_core.py、verify.py、ANALYSIS-代码理论核查.md
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ekf_core import (  # noqa: E402
    Ekf15,
    G_MPS2,
    HOME_LLA,
    cfg,
    cfg_p0,
    two_sample_delta_v,
)

FAILS = 0


def check(name: str, cond: bool) -> None:
    """记录一条断言并打印结果。"""
    global FAILS
    print(("[PASS] " if cond else "[FAIL] ") + name)
    if not cond:
        FAILS += 1


def test_two_sample_no_double_counting() -> None:
    """A-01：双子样速度增量不得再叠加 ½Δθ×ΔV（中点姿态已含旋转）。"""
    dt = 0.005
    omega = np.array([0.0, 0.0, 2.0])  # 绕 z 的 2 rad/s
    f_body = np.array([10.0, 0.0, 0.0])  # 机体系比力 10 m/s²
    dtheta = omega * dt
    dv = f_body * dt
    # 两个等分子样
    dtheta1, dtheta2 = 0.5 * dtheta, 0.5 * dtheta
    dv1, dv2 = 0.5 * dv, 0.5 * dv
    new = two_sample_delta_v(dtheta1, dtheta2, dv1, dv2)
    old = new + 0.5 * np.cross(dtheta, dv)
    # 等分子样下划摇项自身相消，只剩 ΔV，旧实现会多出 ½Δθ×ΔV
    check("A-01 新实现 = ΔV（无旋转项）", np.allclose(new, dv, atol=1e-12))
    extra = np.linalg.norm(old - new)
    check("A-01 旧实现多算 ½|Δθ×ΔV|", extra > 1e-6)
    check("A-01 多算量级符合解析值 ½|ω×f|dt²",
          abs(extra - 0.5 * np.linalg.norm(np.cross(omega, f_body)) * dt * dt) < 1e-12)


def test_p0_heading_default() -> None:
    """A-02：仿真 P0 航向默认值与固件 0.2 rad 一致（π 只作显式覆盖）。"""
    check("A-02 cfg_p0['hdg'] == 0.2", abs(cfg_p0["hdg"] - 0.2) < 1e-12)
    try:
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "opt"))
        import opt_ekf  # noqa: F401

        check("A-02 opt_ekf.FW['p0_hdg'] == 0.2",
              abs(opt_ekf.FW["p0_hdg"] - 0.2) < 1e-12)
    except Exception as exc:  # pragma: no cover - 依赖路径异常时明确报告
        check(f"A-02 导入 opt_ekf 失败：{exc}", False)


def test_home_lla_initialized() -> None:
    """A-03：initialize() 后 home_lla 可直接使用，GNSS 更新不因缺属性失败。"""
    ekf = Ekf15(cfg)
    ekf.initialize(np.array([0.0, 0.0, -G_MPS2]), np.zeros(3), HOME_LLA)
    check("A-03 initialize 设置 home_lla", hasattr(ekf, "home_lla"))
    check("A-03 home_lla == 初始 LLA", np.allclose(ekf.home_lla, HOME_LLA))
    res = ekf.measurement_update_gnss(
        np.zeros(3), np.zeros(3), [2.0, 3.0, 0.15, 0.25]
    )
    check("A-03 零残差 GNSS 更新可执行", bool(res.get("fused", False)))
    check("A-03 零残差 NIS 接近 0", float(res.get("nis", 1e9)) < 1e-6)


def main() -> int:
    """运行三组回归，返回失败数（0 = 通过）。"""
    print("== 研究仿真 P0 修复回归（A-01/A-02/A-03）==")
    test_two_sample_no_double_counting()
    test_p0_heading_default()
    test_home_lla_initialized()
    print(f"\n=== 结果: {'全部通过' if FAILS == 0 else '存在失败'}（失败 {FAILS}）===")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
