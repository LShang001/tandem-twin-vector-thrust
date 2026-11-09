# -*- coding: utf-8 -*-
"""
@file    test_stepresponse.py
@module  GCS/单测 阶跃响应分析
@brief   用解析可算的合成数据验证指标算法，不靠"看起来对"
@details 每一项都对照理论值：
         理想一阶/二阶系统的上升时间、超调、调节时间都有公式，
         算法算出来必须对得上，否则调参界面给出的数字就是假的。
@theory  二阶系统 ζ/ωn 与超调/上升时间的关系
@frame   N/A
@units   s / %
@ref     server/stepresponse.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from server.stepresponse import analyze_pair, detect_steps, step_metrics  # noqa: E402

DT = 0.005   # 200 Hz，与固件导航频率一致


def _gen(seconds, fn):
    """按 DT 生成 [(t, v)]"""
    n = int(seconds / DT)
    return [(i * DT, fn(i * DT)) for i in range(n)]


def _step_cmd(seconds, t_step, before, after):
    return _gen(seconds, lambda t: before if t < t_step else after)


def _second_order(seconds, t_step, zeta, wn, amp):
    """标准二阶阶跃响应（t>=t_step 起跳）"""
    def f(t):
        if t < t_step:
            return 0.0
        tau = t - t_step
        wd = wn * math.sqrt(max(0.0, 1 - zeta * zeta))
        e = math.exp(-zeta * wn * tau)
        if wd < 1e-9:
            return amp * (1 - e * (1 + zeta * wn * tau))
        return amp * (1 - e * (math.cos(wd * tau)
                               + (zeta * wn / wd) * math.sin(wd * tau)))
    return _gen(seconds, f)


def test_detect_single_step():
    """@brief 检测到一次阶跃，且时刻正确"""
    cmd = _step_cmd(4.0, 1.0, 0.0, 10.0)
    steps = detect_steps(cmd, min_delta=5.0)
    assert len(steps) == 1
    assert abs(steps[0] - 1.0) < 0.02


def test_detect_ignores_noise():
    """@brief 小幅噪声不误判为阶跃"""
    import random
    random.seed(0)
    cmd = _gen(3.0, lambda t: random.gauss(0, 0.3))
    assert detect_steps(cmd, min_delta=5.0) == []


def test_detect_ignores_ramp():
    """@brief 连续斜坡不是阶跃：没有前后平台"""
    cmd = _gen(4.0, lambda t: min(10.0, max(0.0, (t - 1.0) * 5.0)))
    steps = detect_steps(cmd, min_delta=3.0)
    assert steps == []


def test_rise_time_first_order_like():
    """@brief 一阶响应（τ=0.2s）：10%→90% 上升时间理论 = τ·ln(9) ≈ 0.439 s"""
    tau = 0.2
    cmd = _step_cmd(3.0, 1.0, 0.0, 10.0)
    fb = _gen(3.0, lambda t: 0.0 if t < 1.0 else 10.0 * (1 - math.exp(-(t - 1.0) / tau)))
    r = step_metrics(cmd, fb, 1.0)
    assert r['ok']
    assert abs(r['rise_time_s'] - tau * math.log(9)) < 0.03
    # 一阶无超调
    assert r['overshoot_pct'] < 0.5


def test_overshoot_critical_damped():
    """@brief ζ=1 临界阻尼：超调应接近 0"""
    cmd = _step_cmd(6.0, 1.0, 0.0, 10.0)
    fb = _second_order(6.0, 1.0, zeta=1.0, wn=8.0, amp=10.0)
    r = step_metrics(cmd, fb, 1.0, window_s=4.0)
    assert r['ok']
    assert r['overshoot_pct'] < 1.0


def test_overshoot_underdamped():
    """@brief ζ=0.5：理论超调 exp(-πζ/√(1-ζ²)) ≈ 16.3%"""
    zeta = 0.5
    cmd = _step_cmd(8.0, 1.0, 0.0, 10.0)
    fb = _second_order(8.0, 1.0, zeta=zeta, wn=8.0, amp=10.0)
    r = step_metrics(cmd, fb, 1.0, window_s=6.0)
    assert r['ok']
    expected = math.exp(-math.pi * zeta / math.sqrt(1 - zeta ** 2)) * 100
    assert abs(r['overshoot_pct'] - expected) < 2.0, (r['overshoot_pct'], expected)


def test_settling_time_underdamped():
    """@brief ζ=0.5, wn=8：2% 调节时间理论 ≈ 4/(ζωn) = 1.0 s"""
    zeta, wn = 0.5, 8.0
    cmd = _step_cmd(8.0, 1.0, 0.0, 10.0)
    fb = _second_order(8.0, 1.0, zeta=zeta, wn=wn, amp=10.0)
    r = step_metrics(cmd, fb, 1.0, window_s=6.0)
    assert r['ok']
    assert abs(r['settling_time_s'] - 1.0) < 0.35


def test_steady_state_error_zero_when_perfect():
    """@brief 完美跟踪时稳态误差应接近 0"""
    cmd = _step_cmd(3.0, 1.0, 0.0, 10.0)
    fb = _gen(3.0, lambda t: 0.0 if t < 1.0 else 10.0 * (1 - math.exp(-(t - 1.0) / 0.08)))
    r = step_metrics(cmd, fb, 1.0)
    assert r['ok']
    assert r['steady_error_pct'] < 1.0


def test_steady_state_error_detected():
    """@brief 只跟到 80% 时稳态误差应报 ~20%"""
    cmd = _step_cmd(3.0, 1.0, 0.0, 10.0)
    fb = _gen(3.0, lambda t: 0.0 if t < 1.0 else 8.0 * (1 - math.exp(-(t - 1.0) / 0.08)))
    r = step_metrics(cmd, fb, 1.0)
    assert r['ok']
    assert abs(r['steady_error_pct'] - 20.0) < 3.0


def test_analyze_pair_summary():
    """@brief 多次阶跃的汇总：n_steps/n_ok 与平均值都要给出"""
    def cmd_fn(t):
        return 5.0 if (t % 2.0) < 1.0 else -5.0
    cmd = _gen(9.0, cmd_fn)
    fb = _gen(9.0, lambda t: cmd_fn(t - 0.1))
    out = analyze_pair(cmd, fb, min_delta=5.0, window_s=0.8)
    s = out['summary']
    assert s['n_steps'] >= 2
    assert s['n_ok'] >= 1
    assert s['avg_rise_time_s'] is not None


def test_insufficient_data_reports_reason():
    """@brief 数据不足要明确返回 ok=False 和原因，不能给个 0 糊弄"""
    r = step_metrics([(0.0, 0.0)], [], 0.0)
    assert r['ok'] is False
    assert 'reason' in r
