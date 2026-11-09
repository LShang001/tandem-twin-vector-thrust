#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    spectrum.py
@module  GCS/后端 频谱分析
@brief   Welch 功率谱估计 + 主峰识别 + 频带能量，供波形页做频域视图
@details 为什么波形页需要频谱：时域曲线看得出"在抖"，但看不出**抖在哪个频率**。
         姿态抖动 12 Hz 和 45 Hz 在时域上都是"毛刺"，但原因完全不同——
         前者多半是结构/桨叶一阶，后者更像传感器噪声或控制环振荡。
         频谱把这个问题变成可读的数字。

         实现选择：
         · Welch 法（分段 + 汉宁窗 + 平均）而不是直接 FFT 整段——
           单次 FFT 的谱估计方差极大，毛刺太多没法判读；分段平均后谱线才稳定。
         · 只用 numpy，不引入 scipy。项目其他部分都是标准库，为一个 Welch
           拉进 scipy 不划算；numpy 的 rfft 足够，几十行就写完。

         采样率从时间戳实际间隔估计，不假设等间隔——遥测会掉帧，
         用名义频率会让谱线整体偏移。

@theory  功率谱估计（Welch 法）；汉宁窗抑制频谱泄漏；频率分辨率 = fs/N
@frame   N/A
@units   频率 Hz、功率谱密度（相对值，未归一化物理量纲）
@note    输入长度不足两段时返回 None 而不是强行出图——半段窗的谱没有统计意义。
@test    tests/test_spectrum.py：单频/双频/白噪声/分辨率/输入不足
@see     TandemVec_FCS/GCS/web/src/scope.mjs
"""

from __future__ import annotations

from typing import Sequence

Series = Sequence[tuple[float, float]]   # [(t_sec, value), ...]


def estimate_fs(series: Series) -> float | None:
    """@brief 从时间戳估计实际采样率（取间隔中位数，抗掉帧）
    @return 采样率 Hz；样本不足返回 None

    @note 用中位数而不是均值：偶发的大间隔（丢帧）会把均值拉低，
          而中位数仍然是主导采样周期。
    """
    if len(series) < 3:
        return None
    diffs = [series[i][0] - series[i - 1][0] for i in range(1, len(series))]
    diffs = [d for d in diffs if d > 0]
    if not diffs:
        return None
    diffs.sort()
    med = diffs[len(diffs) // 2]
    if med <= 0:
        return None
    return 1.0 / med


def welch_psd(series: Series, nperseg: int = 256, overlap: float = 0.5):
    """@brief Welch 功率谱估计
    @param series [(t, v), ...]（时间用不到，但保留统一接口便于换数据源）
    @param nperseg 每段样本数（决定频率分辨率）
    @param overlap 段间重叠比例
    @return (freqs, psd, fs, n_seg) 或 None（样本不足）

    @note 返回的 psd 未做物理量纲归一化，只用于**相对比较与峰值定位**。
          要绝对幅值需要乘窗函数相干增益，本项目用不到。
    """
    import numpy as np

    xs = np.asarray([v for _t, v in series], dtype=float)
    fs = estimate_fs(series)
    if fs is None or len(xs) < nperseg * 2:
        return None
    xs = xs - xs.mean()

    step = max(1, int(nperseg * (1.0 - overlap)))
    win = np.hanning(nperseg)
    win_pow = float((win ** 2).sum())
    if win_pow <= 0:
        return None

    segs = []
    for i in range(0, len(xs) - nperseg + 1, step):
        seg = xs[i:i + nperseg] * win
        spec = np.fft.rfft(seg)
        # 单边谱：除以窗能量，让不同段长/窗型的幅值可比
        segs.append((np.abs(spec) ** 2) / win_pow)
    if not segs:
        return None
    psd = np.mean(np.asarray(segs), axis=0)
    freqs = np.fft.rfftfreq(nperseg, d=1.0 / fs)
    return freqs, psd, fs, len(segs)


def spectrum_summary(series: Series, nperseg: int = 256,
                     top_n: int = 5) -> dict:
    """@brief 频谱摘要：主峰、前 N 个峰、频带能量占比
    @return {ok, fs, n_seg, resolution_hz, peaks:[{freq_hz,power,share}], ...}
    """
    out = welch_psd(series, nperseg=nperseg)
    if out is None:
        return {'ok': False, 'reason': '样本不足（至少需要 2 段窗长）'}
    freqs, psd, fs, n_seg = out

    total = float(psd.sum())
    if total <= 0:
        return {'ok': False, 'reason': '信号为常量（无波动，无需频谱）'}

    # 找局部峰：比左右邻居都大。跳过直流 bin（freq=0）
    peaks = []
    for i in range(1, len(psd) - 1):
        if psd[i] > psd[i - 1] and psd[i] >= psd[i + 1]:
            peaks.append((float(freqs[i]), float(psd[i])))
    peaks.sort(key=lambda p: p[1], reverse=True)
    top = peaks[:top_n]

    # 频带能量：低频（<1Hz，慢漂） / 中频（1–10Hz，机体运动） /
    # 高频（>10Hz，振动与噪声）。这三个带对应不同的物理来源，分开看才有诊断意义。
    def band(lo, hi):
        m = (freqs >= lo) & (freqs < hi)
        return float(psd[m].sum()) / total * 100.0

    return {
        'ok': True,
        'fs': round(fs, 2),
        'n_seg': n_seg,
        'resolution_hz': round(float(freqs[1] - freqs[0]), 4) if len(freqs) > 1 else None,
        'peaks': [
            # 占比用 4 位小数：主峰常占 99.9%，次峰可能只有 0.00x%。
            # 2 位会把所有次峰都显示成 0，界面看起来像"没有其它分量"。
            {'freq_hz': round(f, 3), 'power': round(p, 6),
             'share_pct': round(p / total * 100.0, 4)}
            for f, p in top
        ],
        'bands': {
            'low_lt1hz_pct': round(band(0, 1), 2),
            'mid_1_10hz_pct': round(band(1, 10), 2),
            'high_gt10hz_pct': round(band(10, 1e9), 2),
        },
        'freqs': [round(float(f), 3) for f in freqs],
        'psd_norm': [round(float(p) / total, 6) for p in psd],
    }


def compare_channels(chans: dict, nperseg: int = 256,
                     use_common_nperseg: bool = True) -> dict:
    """@brief 多通道频谱对比：各自谱 + 峰值对照 + 共模峰识别

    @param chans {name: [(t,v), ...]}，最多取前 4 个（再多图上分不清）
    @param nperseg 每段样本数
    @param use_common_nperseg 是否用各通道可用样本数的最小值统一窗长
    @return {ok, channels:[{name,fs,n_seg,peaks,bands,psd_norm,freqs}], common:[...]}

    @details 为什么要做多通道对比：单通道谱只能说明"这里有峰"。要让频谱变成
             诊断工具，必须回答"这个峰是几个轴共有的还是一根轴独有的"——
               · 三轴同频 → 多半是结构/桨叶/共模振动，改单个轴增益没用
               · 单轴独有 → 传感器、机械间隙、该轴执行器或配线问题
             共模判定用 ±1 个频率分辨率的容差，因为不同通道的 fs 可能略有差异。

    @note 统一窗长（取各通道样本数允许的最大值）是为了让不同通道的谱线落在
          同一频率网格上，否则画在一起横轴对不齐，视觉上会误判。
    """
    names = list(chans.keys())[:4]
    if not names:
        return {'ok': False, 'reason': '未选择通道'}

    results = []
    ok_names = []
    for n in names:
        r = spectrum_summary(chans[n], nperseg=nperseg)
        if r.get('ok'):
            r['name'] = n
            results.append(r)
            ok_names.append(n)
    if not results:
        return {'ok': False, 'reason': '所选通道样本都不足（至少需要 2 段窗长）'}

    # 共模峰：在 >=2 个通道的峰表里都能找到（±1 分辨率）
    common = []
    ref = results[0]
    ref_res = ref.get('resolution_hz') or 0.2
    tol = max(ref_res * 1.5, 0.15)
    for p in ref.get('peaks', []):
        hits = [ref['name']]
        for other in results[1:]:
            for q in other.get('peaks', []):
                if abs(q['freq_hz'] - p['freq_hz']) <= tol:
                    hits.append(other['name'])
                    break
        if len(hits) >= 2:
            common.append({
                'freq_hz': p['freq_hz'],
                'share_pct': p['share_pct'],
                'channels': hits,
                'n_channels': len(hits),
            })
    common.sort(key=lambda c: c['n_channels'], reverse=True)

    return {
        'ok': True,
        'channels': results,
        'common': common[:6],
        'n_compared': len(results),
        'tol_hz': round(tol, 3),
    }

