#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    stats.py
@module  GCS/后端 时序区间统计
@brief   对选中时间区间算 RMS / 峰峰值 / 均值 / 标准差 / 极值时刻
@details 为什么需要：现场判断振动等级最常用的是"这段的 RMS 和峰峰值是多少"，
         而不是盯着波形看。RMS 反映能量（与结构疲劳直接相关），峰峰值反映
         瞬时最大偏差（与限幅/间隙相关），两者的比值还能看出波形是不是纯正弦。

         算在后端而不是前端：前端没有测试框架，算法放在这里才能用单测钉住。
         数值对不对是这类工具的生命线——一个算错的 RMS 会让人把好飞机当坏飞机。

@theory  RMS = sqrt(mean(x²))；样本标准差用 N-1（无偏）
@frame   N/A
@units   与被统计量同单位；时间 s
@test    tests/test_stats.py：已知信号（正弦/方波/常量）的解析值对照
@see     TandemVec_FCS/GCS/web/src/scope.mjs
"""

from __future__ import annotations

import math
from typing import Sequence

Series = Sequence[tuple[float, float]]


def series_stats(series: Series, t0: float | None = None,
                 t1: float | None = None) -> dict:
    """@brief 统计给定时间区间内的样本

    @param series [(t, v), ...]（t 升序）
    @param t0 区间起点（含）；None 表示从头
    @param t1 区间终点（含）；None 表示到尾
    @return {ok, n, t0, t1, span, rms, peak_to_peak, mean, std,
             vmin, vmax, t_vmax, crest_factor}

    @note crest_factor = |vmax| / rms，纯正弦约 1.414，方波接近 1。
          它是"波形陡不陡"的快速判据：振动波形 crest 远大于 1.414 说明有冲击成分。
    """
    if not series:
        return {'ok': False, 'reason': '没有数据'}
    lo = series[0][0] if t0 is None else t0
    hi = series[-1][0] if t1 is None else t1
    if hi < lo:
        lo, hi = hi, lo
    sel = [(t, v) for t, v in series if lo <= t <= hi]
    if len(sel) < 2:
        return {'ok': False, 'reason': f'选定区间样本太少（{len(sel)} 点）'}

    vals = [v for _t, v in sel]
    n = len(vals)
    mean = sum(vals) / n
    var = sum((v - mean) ** 2 for v in vals) / (n - 1)
    std = math.sqrt(var)
    rms = math.sqrt(sum(v * v for v in vals) / n)
    vmin = min(vals)
    vmax = max(vals)
    t_vmax = next(t for t, v in sel if v == vmax)
    span = sel[-1][0] - sel[0][0]
    crest = (abs(vmax) / rms) if rms > 1e-12 else None

    return {
        'ok': True,
        'n': n,
        't0': round(sel[0][0], 3),
        't1': round(sel[-1][0], 3),
        'span_s': round(span, 3),
        'duration_s': round(hi - lo, 3),
        'rms': rms,
        'peak_to_peak': vmax - vmin,
        'mean': mean,
        'std': std,
        'vmin': vmin,
        'vmax': vmax,
        't_vmax': round(t_vmax, 3),
        'crest_factor': (round(crest, 3) if crest is not None else None),
    }
