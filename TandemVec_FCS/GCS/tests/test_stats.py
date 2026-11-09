# -*- coding: utf-8 -*-
"""
@file    test_stats.py
@module  GCS/单测 区间统计
@brief   用解析可算的信号验证 RMS / 峰峰值 / 均值 / 标准差 / crest
@details 一个算错的 RMS 会让人把好飞机当坏飞机，所以每项都对解析值。
@theory  RMS(正弦,幅值A) = A/√2；RMS(方波,±A) = A；标准差(正弦) = A/√2
@frame   N/A
@units   无量纲（信号自定）
@ref     server/stats.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from server.stats import series_stats  # noqa: E402

FS = 50.0


def _gen(seconds, fn, t_start=0.0):
    n = int(seconds * FS)
    return [(t_start + i / FS, fn(i / FS)) for i in range(n)]


def test_constant_signal():
    """@brief 常量：RMS=|v|、峰峰=0、标准差=0"""
    s = _gen(2.0, lambda t: 3.0)
    r = series_stats(s)
    assert r['ok']
    assert abs(r['rms'] - 3.0) < 1e-9
    assert abs(r['peak_to_peak']) < 1e-9
    assert abs(r['std']) < 1e-9
    assert abs(r['mean'] - 3.0) < 1e-9


def test_sine_rms_and_crest():
    """@brief 正弦 幅值 2：RMS = 2/√2、crest ≈ 1.414、峰峰 = 4"""
    s = _gen(4.0, lambda t: 2.0 * math.sin(2 * math.pi * 5.0 * t))
    r = series_stats(s)
    assert r['ok']
    assert abs(r['rms'] - 2.0 / math.sqrt(2)) < 0.02, r['rms']
    # 峰峰值容差放宽到 0.25：5 Hz 正弦 @ 50 Hz 采样，每周期只有 10 个点，
    # 峰值很可能落在两个采样点之间（实测 3.80 而非 4.00）。这是**采样效应**，
    # 不是算法错——用更高采样率或多周期会让它变小，但离散采样永远差一点。
    assert abs(r['peak_to_peak'] - 4.0) < 0.25, r['peak_to_peak']
    # crest 同样受采样效应影响：峰值没采到时 vmax 偏小，crest 也跟着偏小
    # （实测 1.345）。理论值 1.414 是连续信号的结果。
    assert abs(r['crest_factor'] - math.sqrt(2)) < 0.12, r['crest_factor']
    assert abs(r['mean']) < 0.02


def test_square_wave_crest():
    """@brief 方波 ±2：RMS=2、crest≈1（比方波陡的正弦 crest 明显更大）"""
    s = _gen(4.0, lambda t: 2.0 if (t % 0.2) < 0.1 else -2.0)
    r = series_stats(s)
    assert r['ok']
    assert abs(r['rms'] - 2.0) < 0.05, r['rms']
    assert r['crest_factor'] < 1.1, r['crest_factor']
    assert abs(r['peak_to_peak'] - 4.0) < 1e-6


def test_sine_std_equals_rms_when_zero_mean():
    """@brief 零均值正弦：标准差应等于 RMS"""
    s = _gen(4.0, lambda t: 1.5 * math.sin(2 * math.pi * 3.0 * t))
    r = series_stats(s)
    assert abs(r['std'] - r['rms']) < 0.02


def test_time_window_selection():
    """@brief 只统计选定区间：前半段 1、后半段 5，选后半段均值应为 5"""
    def f(t):
        return 1.0 if t < 2.0 else 5.0
    s = _gen(4.0, f)
    t_mid = 2.0
    r = series_stats(s, t0=t_mid, t1=s[-1][0])
    assert r['ok']
    assert abs(r['mean'] - 5.0) < 1e-6
    assert abs(r['peak_to_peak']) < 1e-6
    # 前半段
    r2 = series_stats(s, t0=0.0, t1=1.9)
    assert abs(r2['mean'] - 1.0) < 1e-6


def test_reversed_window_is_swapped():
    """@brief 传反了 t0>t1 要自动交换，不能返回空"""
    s = _gen(4.0, lambda t: 2.0 + math.sin(t))
    r = series_stats(s, t0=3.0, t1=1.0)
    assert r['ok']
    assert r['n'] > 10


def test_insufficient_samples():
    """@brief 区间内样本太少要明确拒绝"""
    s = _gen(4.0, lambda t: 1.0)
    r = series_stats(s, t0=2.0, t1=2.001)
    assert r['ok'] is False and 'reason' in r


def test_empty_series():
    """@brief 空数据明确拒绝"""
    r = series_stats([])
    assert r['ok'] is False


def test_t_vmax_points_at_max():
    """@brief t_vmax 必须真的指向最大值那一刻"""
    def f(t):
        return 1.0 if abs(t - 1.234) > 0.05 else 9.0
    s = _gen(3.0, f)
    r = series_stats(s)
    assert r['ok']
    assert abs(r['vmax'] - 9.0) < 1e-9
    assert abs(r['t_vmax'] - 1.234) < 0.1, r['t_vmax']
