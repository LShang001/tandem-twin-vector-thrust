# -*- coding: utf-8 -*-
"""
@file    test_spectrum.py
@module  GCS/单测 频谱分析
@brief   用已知频率的合成信号验证峰值定位与分辨率
@details 频谱算法最容易"看起来对"：画出来有峰就以为对了。这里每一项都
         对照可算的正确答案——合成 5 Hz 正弦，主峰必须在 5 Hz 附近；
         频率分辨率必须等于 fs/N；两个靠近的频率必须能分辨。
@theory  DFT 频率分辨率 = fs/N；汉宁窗主瓣宽约 4 bin
@frame   N/A
@units   Hz
@ref     server/spectrum.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from server.spectrum import estimate_fs, spectrum_summary, welch_psd  # noqa: E402

FS = 50.0     # 50 Hz 采样（与遥测推送频率一致）
DT = 1.0 / FS


def _gen(seconds, fn, fs=FS):
    n = int(seconds * fs)
    return [(i / fs, fn(i / fs)) for i in range(n)]


def test_estimate_fs():
    """@brief 等间隔数据的采样率估计应等于设定值"""
    s = _gen(5.0, lambda t: 0.0)
    assert abs(estimate_fs(s) - FS) < 1e-6


def test_estimate_fs_with_dropouts():
    """@brief 偶发丢帧时中位数仍应给出正确采样率"""
    s = _gen(5.0, lambda t: 0.0)
    # 每隔 20 个样本删一个，模拟丢帧
    s2 = [p for i, p in enumerate(s) if i % 20 != 0]
    assert abs(estimate_fs(s2) - FS) < 1e-6


def test_single_tone_peak():
    """@brief 5 Hz 正弦的主峰应在 5 Hz（分辨率内）"""
    s = _gen(20.0, lambda t: math.sin(2 * math.pi * 5.0 * t))
    r = spectrum_summary(s, nperseg=256)
    assert r['ok']
    top = r['peaks'][0]
    # 分辨率 = fs/N = 50/512 ≈ 0.098 Hz，允许几个 bin 的偏差
    assert abs(top['freq_hz'] - 5.0) < 0.5, top
    # 主峰应占据绝大部分能量
    assert top['share_pct'] > 50, top


def test_two_tone_resolved():
    """@brief 3 Hz 与 12 Hz 两个分量都应出现在峰列表里"""
    s = _gen(20.0, lambda t: (math.sin(2 * math.pi * 3.0 * t)
                              + 0.8 * math.sin(2 * math.pi * 12.0 * t)))
    r = spectrum_summary(s, nperseg=256, top_n=6)
    assert r['ok']
    freqs = [p['freq_hz'] for p in r['peaks']]
    assert any(abs(f - 3.0) < 0.4 for f in freqs), freqs
    assert any(abs(f - 12.0) < 0.4 for f in freqs), freqs


def test_resolution_matches_fs_over_n():
    """@brief 频率分辨率必须等于 fs/N"""
    s = _gen(20.0, lambda t: math.sin(2 * math.pi * 5.0 * t))
    n = 256
    r = spectrum_summary(s, nperseg=n)
    assert r['ok']
    expect = FS / n
    assert abs(r['resolution_hz'] - expect) < 1e-3, (r['resolution_hz'], expect)


def test_dc_removed():
    """@brief 去掉直流后，常量偏置不应在 0 Hz 出现巨大峰"""
    # 12 s @ 50 Hz = 600 样本，256*2=512 满足「至少两段」的要求
    s = _gen(12.0, lambda t: 100.0 + 0.5 * math.sin(2 * math.pi * 4.0 * t))
    r = spectrum_summary(s, nperseg=256)
    assert r['ok']
    # 主峰应在 4 Hz，而不是 0 Hz
    assert abs(r['peaks'][0]['freq_hz'] - 4.0) < 0.4, r['peaks'][:3]


def test_band_split():
    """@brief 低频信号的能量应主要落在 <1Hz 带"""
    s = _gen(30.0, lambda t: math.sin(2 * math.pi * 0.3 * t))
    r = spectrum_summary(s, nperseg=256)
    assert r['ok']
    assert r['bands']['low_lt1hz_pct'] > 80, r['bands']


def test_high_band():
    """@brief 高频信号能量应主要落在 >10Hz 带"""
    s = _gen(20.0, lambda t: math.sin(2 * math.pi * 18.0 * t))
    r = spectrum_summary(s, nperseg=256)
    assert r['ok']
    assert r['bands']['high_gt10hz_pct'] > 80, r['bands']


def test_insufficient_samples():
    """@brief 样本不足两段窗时明确返回原因，不出假谱"""
    s = _gen(1.0, lambda t: math.sin(2 * math.pi * 5.0 * t))
    r = spectrum_summary(s, nperseg=512)
    assert r['ok'] is False
    assert 'reason' in r


def test_constant_signal():
    """@brief 完全恒定的信号没有频谱可分，要明确拒绝而不是返回全零谱"""
    s = _gen(10.0, lambda t: 42.0)
    r = spectrum_summary(s, nperseg=256)
    assert r['ok'] is False


def test_welch_returns_metadata():
    """@brief welch_psd 要回传采样率与段数，便于判读可信度"""
    s = _gen(20.0, lambda t: math.sin(2 * math.pi * 5.0 * t))
    out = welch_psd(s, nperseg=256)
    assert out is not None
    freqs, psd, fs, n_seg = out
    assert abs(fs - FS) < 1e-6
    assert n_seg >= 4
    assert len(freqs) == len(psd)
