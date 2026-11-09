#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    stepresponse.py
@module  GCS/后端 阶跃响应分析
@brief   从遥测时间序列里自动找阶跃，并算出上升时间/超调/调节时间/稳态误差
@details 为什么做这个：调参时最想知道的是"改了 kp 之后响应变快还是变振荡"，
         而这件事靠盯着实时波形用眼睛判断既不准也不可复现。阶跃响应指标是
         经典控制里的标准判据，算出来就是可对比的数字。

         算法分两步：
         ① **检测**：指令通道（如 target_roll_deg）出现突变，且突变前后各有
            一段相对稳定的平台——只有"平台 → 突变 → 新平台"才算一次阶跃，
            噪声抖动和连续机动都不算。
         ② **评估**：以突变时刻为零点，取反馈通道的响应，算
            rise_time（10%→90%）、overshoot（超调百分比）、
            settling_time（进入 ±2% 并保持）、steady_state_error。

         为什么用指令通道找阶跃而不是反馈通道：反馈是"结果"，它可能因为
         控制饱和、限幅、机械间隙而根本没跟上指令；用指令找零点才能度量
         "控制跟没跟上"。两者都要能选，所以接口接受两个通道名。

@theory  经典二阶系统阶跃响应指标（上升时间/超调/调节时间/稳态误差）
@frame   N/A
@units   角度 deg、角速度 deg/s、时间 s、超调 %
@note    样本不均匀时按实际时间戳插值，不假设等间隔——遥测确实会掉帧。
@test    tests/test_stepresponse.py：理想二阶/一阶响应、超调、纯延迟、噪声不误检
@see     TandemVec_FCS/GCS/web/src/tuning.mjs
"""

from __future__ import annotations

import math
from typing import Iterable, Sequence

Series = Sequence[tuple[float, float]]   # [(t_sec, value), ...]


def _interp(series: Series, t: float) -> float | None:
    """在时间序列上线性插值取 t 时刻的值（t 超出范围返回 None）"""
    if not series:
        return None
    if t < series[0][0] or t > series[-1][0]:
        return None
    lo, hi = 0, len(series) - 1
    while lo <= hi:
        mid = (lo + hi) // 2
        if series[mid][0] < t:
            lo = mid + 1
        elif series[mid][0] > t:
            hi = mid - 1
        else:
            return series[mid][1]
    # lo 为第一个大于 t 的下标
    t0, v0 = series[max(0, lo - 1)]
    t1, v1 = series[min(len(series) - 1, lo)]
    if t1 == t0:
        return v1
    return v0 + (v1 - v0) * (t - t0) / (t1 - t0)


def detect_steps(series: Series, min_delta: float,
                 plateau_s: float = 0.15,
                 plateau_tol: float = 0.15,
                 max_steps: int = 50) -> list[float]:
    """@brief 在指令序列里找阶跃时刻
    @param series 指令通道 [(t, v), ...]
    @param min_delta 阶跃最小幅值（小于此值视为噪声/缓慢机动）
    @param plateau_s 突变前后要求的最短平台时长
    @param plateau_tol 平台判定容差（相对 min_delta 的比例）
    @param max_steps 最多返回多少个
    @return 阶跃时刻列表（秒，按时间升序）

    @note 判据是「平台 → 突变 → 平台」，而不是单纯的相邻样本差值超阈值：
          连续斜坡机动也会满足差值判据，但它不是阶跃，用它算上升时间没有意义。
    """
    n = len(series)
    if n < 8:
        return []
    tol = max(1e-9, plateau_tol * min_delta)
    steps: list[float] = []

    def plateau_ok(i0: int, i1: int) -> bool:
        """区间 [i0, i1) 内是否足够平稳且足够长"""
        if i1 <= i0:
            return False
        if series[i1 - 1][0] - series[i0][0] < plateau_s:
            return False
        vals = [v for _, v in series[i0:i1]]
        return (max(vals) - min(vals)) <= tol

    for i in range(2, n - 2):
        before = series[i - 1][1]
        after = series[i][1]
        delta = after - before
        if abs(delta) < min_delta:
            continue
        # 突变点前应有一段平台。这里按**时间**回溯而不是固定样本数——
        # 遥测帧率会变（200 Hz 导航 / 掉帧时更低），绑定样本数会让判据随帧率失真。
        # 平台区间是 [j0, i)（不含突变点本身），所以时长按 series[i-1] 算，
        # 用 series[i] 会少算一个采样间隔，导致刚好够长的平台被判不够。
        j0 = i - 1
        while j0 > 0 and (series[i - 1][0] - series[j0][0]) < plateau_s:
            j0 -= 1
        if not plateau_ok(j0, i):
            continue
        # 突变后：取一小段看是否保持在新值附近（排除单点毛刺）
        j = i + 1
        while j < n and (series[j][0] - series[i][0]) < plateau_s:
            j += 1
        if j >= n:
            continue
        post = [v for _, v in series[i:j]]
        if (max(post) - min(post)) > max(tol * 4, abs(delta) * 0.25):
            continue
        if steps and (series[i][0] - steps[-1]) < max(0.5, plateau_s * 2):
            continue   # 太近的视为同一次
        steps.append(series[i][0])
        if len(steps) >= max_steps:
            break
    return steps


def step_metrics(cmd: Series, fb: Series, t_step: float,
                 window_s: float = 3.0,
                 settle_band: float = 0.02) -> dict:
    """@brief 计算一次阶跃响应的指标
    @param cmd 指令通道序列（用于确定阶跃前后的目标值）
    @param fb 反馈通道序列
    @param t_step 阶跃发生时刻
    @param window_s 评估窗口长度
    @param settle_band 调节时间判据带宽（相对阶跃幅值）
    @return 指标字典；数据不足时 ok=False 并给出原因
    """
    # 阶跃前值必须取"最后一个严格小于 t_step 的样本"，不能用插值：
    # 阶跃边缘不是线性的，插值会在 [前一采样, 阶跃采样] 之间造出一个
    # 根本不存在的中间值（实测：0→10 的阶跃在 0.999 s 处被插成 8.0），
    # 导致幅值算错、后续所有指标跟着错。
    y0 = None
    for ts, v in cmd:
        if ts < t_step:
            y0 = v
        else:
            break
    y_target = None
    # 目标值取阶跃后窗口内的中位数（更抗振铃）
    post = [v for t, v in cmd if t_step < t <= t_step + window_s]
    if post:
        post_sorted = sorted(post)
        y_target = post_sorted[len(post_sorted) // 2]
    if y0 is None or y_target is None:
        return {'ok': False, 'reason': '指令数据不足（阶跃前后取不到值）'}

    amp = y_target - y0
    if abs(amp) < 1e-9:
        return {'ok': False, 'reason': '阶跃幅值为 0'}

    samples = [(t, v) for t, v in fb if t_step - 0.05 <= t <= t_step + window_s]
    if len(samples) < 5:
        return {'ok': False, 'reason': f'反馈样本太少（{len(samples)} 点）'}

    # 归一化进度：0 = 阶跃前值，1 = 目标值
    def prog(v: float) -> float:
        return (v - y0) / amp

    t10 = t90 = None
    peak = -math.inf
    t_peak = None
    for t, v in samples:
        p = prog(v)
        if t10 is None and p >= 0.10:
            t10 = t
        if t90 is None and p >= 0.90:
            t90 = t
        if p > peak:
            peak = p
            t_peak = t

    rise_time = (t90 - t10) if (t10 is not None and t90 is not None) else None
    overshoot = max(0.0, (peak - 1.0) * 100.0) if peak > -math.inf else None

    # 调节时间：从某时刻起之后所有样本都落在 ±settle_band 内
    settle_time = None
    band = max(abs(settle_band), 1e-6)
    for idx in range(len(samples)):
        tail = samples[idx:]
        if all(abs(prog(v) - 1.0) <= band for _, v in tail):
            if idx == 0:
                settle_time = 0.0
            else:
                settle_time = samples[idx][0] - t_step
            break

    # 稳态误差：窗口后 20% 的平均进度与 1 的偏差
    tail_n = max(3, len(samples) // 5)
    tail_vals = [prog(v) for _, v in samples[-tail_n:]]
    ss_prog = sum(tail_vals) / len(tail_vals)
    ss_err_pct = abs(1.0 - ss_prog) * 100.0

    # 振铃强度：阶跃后段过零次数（正负交替）——调参时判断"有没有振荡"的直接证据
    after_settle = [prog(v) - 1.0 for t, v in samples if t >= t_step + 0.2]
    crossings = 0
    for a, b in zip(after_settle, after_settle[1:]):
        if a == 0 or b == 0:
            continue
        if (a > 0) != (b > 0):
            crossings += 1

    return {
        'ok': True,
        't_step': t_step,
        'y_before': y0,
        'y_target': y_target,
        'amplitude': amp,
        'rise_time_s': rise_time,
        'overshoot_pct': overshoot,
        'peak_time_s': (t_peak - t_step) if t_peak is not None else None,
        'settling_time_s': settle_time,
        'steady_error_pct': ss_err_pct,
        'ring_crossings': crossings,
        'n_samples': len(samples),
    }


def analyze_pair(cmd: Series, fb: Series, min_delta: float,
                 window_s: float = 3.0, **kw) -> dict:
    """@brief 一次性分析整段数据里所有阶跃，给出汇总与逐次指标"""
    steps = detect_steps(cmd, min_delta, **kw)
    results = [step_metrics(cmd, fb, t, window_s=window_s) for t in steps]
    good = [r for r in results if r.get('ok')]
    summary = {'n_steps': len(steps), 'n_ok': len(good)}
    if good:
        def avg(key):
            vals = [r[key] for r in good if r.get(key) is not None]
            return (sum(vals) / len(vals)) if vals else None
        summary.update({
            'avg_rise_time_s': avg('rise_time_s'),
            'avg_overshoot_pct': avg('overshoot_pct'),
            'avg_settling_time_s': avg('settling_time_s'),
            'avg_steady_error_pct': avg('steady_error_pct'),
            'max_overshoot_pct': max(
                (r['overshoot_pct'] for r in good if r.get('overshoot_pct') is not None),
                default=None),
        })
    return {'summary': summary, 'steps': results}
