#!/usr/bin/env python3
"""bb_tool.analyze_overshoot（姿态阶跃超调分析）的单元测试。

@file    TandemVec_FCS/GCS/tests/test_bb_tool.py
@module  上位机/黑匣子分析
@brief   验证阶跃超调分析：无目标列提示、无阶跃、正/负向超调、小阶跃忽略。
@details 被测：`server/bb_tool.py` 的 `analyze_overshoot`（纯函数，脱离硬件可测）。
         ★ 数据结构的正确理解（第一版测试因此全挂）：
         - 固件黑匣子通道表第一项是 t_ms、**没有 seq**（communication.cpp:135）
         - 后端 preview = [seq, t_ms, *vals, seg]（main.py:838）
         - 所以 r[2 + columns.index(name)] 是对的，columns[0]=t_ms → preview[2]
@theory  N/A（分析工具回归）
@frame   N/A
@units   N/A
"""

from __future__ import annotations

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from server.bb_tool import analyze_overshoot

# 与固件通道表一致：第一列 t_ms，无 seq
COLS = ['t_ms', 'roll_deg', 'pitch_deg', 'target_roll_deg', 'target_pitch_deg']


def _mk_rows(samples):
    """samples: [(t_ms, roll, pitch, target_roll, target_pitch)]

    按后端 preview 的实际布局构造：[seq, t_ms, t_ms, roll, pitch, tgt_roll, tgt_pitch]
    （columns[0] 是 t_ms，对应 preview[2]，所以 vals[0] 也是 t_ms）
    """
    return [[0, s[0], s[0], s[1], s[2], s[3], s[4]] for s in samples]


def test_no_target_columns(capsys):
    """列里没有目标姿态通道时，给出明确提示而不是崩溃。"""
    analyze_overshoot(_mk_rows([(0, 0.0, 0.0, 0.0, 0.0)]),
                      ['t_ms', 'roll_deg', 'pitch_deg'])
    out = capsys.readouterr().out
    assert '无目标姿态通道' in out


def test_no_step_reports_none(capsys):
    """目标保持恒定（无 ≥3° 阶跃）时，报告未检测到阶跃。"""
    rows = _mk_rows([(i * 10, 0.0, 0.0, 0.0, 0.0) for i in range(50)])
    analyze_overshoot(rows, COLS)
    out = capsys.readouterr().out
    assert '未检测到目标阶跃' in out


def test_detects_overshoot(capsys):
    """构造 10° 阶跃 + 实际冲到 12°，应报出约 20% 超调。"""
    samples = []
    for i in range(100):
        t_ms = i * 10
        if t_ms < 500:
            target, actual = 0.0, 0.0
        elif t_ms < 600:
            target, actual = 10.0, 12.0     # 过冲 2° / 阶跃 10° = 20%
        else:
            target, actual = 10.0, 10.0
        samples.append((t_ms, actual, 0.0, target, 0.0))
    analyze_overshoot(_mk_rows(samples), COLS)
    out = capsys.readouterr().out
    assert 'roll' in out
    m = re.search(r'超调\s+([+-][\d.]+)%', out)
    assert m, f'未找到超调数值，输出：{out}'
    assert 15.0 <= abs(float(m.group(1))) <= 25.0, m.group(1)


def test_small_step_ignored(capsys):
    """阶跃幅度 < 3° 时按设计忽略（阈值 3.0）。"""
    samples = []
    for i in range(100):
        t_ms = i * 10
        target = 0.0 if t_ms < 500 else 2.0   # 只有 2°，低于 3° 阈值
        samples.append((t_ms, target, 0.0, target, 0.0))
    analyze_overshoot(_mk_rows(samples), COLS)
    out = capsys.readouterr().out
    assert '未检测到目标阶跃' in out


def test_negative_step(capsys):
    """负向阶跃也要能检出，超调按同向计算。"""
    samples = []
    for i in range(100):
        t_ms = i * 10
        if t_ms < 500:
            target, actual = 0.0, 0.0
        elif t_ms < 600:
            target, actual = -10.0, -12.0    # 负向过冲 2° / 阶跃 10° = 20%
        else:
            target, actual = -10.0, -10.0
        samples.append((t_ms, actual, 0.0, target, 0.0))
    analyze_overshoot(_mk_rows(samples), COLS)
    out = capsys.readouterr().out
    m = re.search(r'超调\s+([+-][\d.]+)%', out)
    assert m, f'负向阶跃未检出，输出：{out}'
    assert 15.0 <= abs(float(m.group(1))) <= 25.0, m.group(1)
