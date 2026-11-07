# -*- coding: utf-8 -*-
"""bb_tool.analyze_overshoot（姿态阶跃超调分析）的单元测试。

2026-11-08 补：bb_tool.py 此前零测试覆盖。BbTool 类需要 WebSocket 难测，
但 analyze_overshoot 是纯函数，可脱离硬件验证。

★ 数据结构的正确理解（第一次写测试时搞错，记下来）：
  - 固件黑匣子通道表第一项是 ``t_ms``，**没有 seq**（communication.cpp:135）
  - 后端 preview = ``[r[0](seq), r[1](t_ms), *vals, r[3](seg)]``（main.py:838）
    即前两格是 seq/t_ms，然后才是 22 个通道值
  - 所以 ``analyze_overshoot`` 用 ``r[2 + columns.index(name)]`` 是对的：
    columns[0]='t_ms' 对应 vals[0] 即 preview[2]

所以构造测试数据时：rows 每行 = [seq, t_ms, t_ms_again, roll, pitch, tgt_roll, tgt_pitch]，
columns = ['t_ms', 'roll_deg', 'pitch_deg', 'target_roll_deg', 'target_pitch_deg']。
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
