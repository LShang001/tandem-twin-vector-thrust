# -*- coding: utf-8 -*-
"""
@file    test_calibration.py
@module  GCS/单测 校准向导
@brief   磁标定输出解析与质量判据
@details 用固件真实打印格式（communication.cpp magCalPrintStats）做样本，
         覆盖：正常解析、单位识别、缺字段、各档质量判定。
@theory  N/A
@frame   N/A
@units   uT / 无量纲
@ref     server/calibration.py
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from server.calibration import (  # noqa: E402
    FIELD_MAG_REF_UT,
    MAG_WIZARD,
    assess_magcal,
    parse_magcal_line,
)


def _feed(lines):
    st = {}
    for ln in lines:
        parse_magcal_line(ln, st)
    return st


# 固件 magCalPrintStats 的真实输出形状
GOOD_LINES = [
    '[DBG]   samples = 42',
    '[DBG]   residualRms = 0.0123',
    '[DBG]   eigRatio (Q) = 1.054',
    '[DBG]   coverageRatio = 0.87654',
    '[DBG]   fieldMag (raw) = 48.5 uT',
]


def test_parse_all_fields():
    """@brief 五个字段全部能解析出来"""
    st = _feed(GOOD_LINES)
    assert st['samples'] == 42
    assert abs(st['residual_rms'] - 0.0123) < 1e-9
    assert abs(st['eig_ratio'] - 1.054) < 1e-9
    assert abs(st['coverage'] - 0.87654) < 1e-9
    assert abs(st['field_mag_ut'] - 48.5) < 1e-9


def test_parse_ignores_non_matching():
    """@brief 无关的 DBG 行不改变状态"""
    st = {}
    assert parse_magcal_line('[DBG] some other log line', st) is False
    assert st == {}
    assert parse_magcal_line('', st) is False


def test_parse_eig_ratio_without_unit_suffix():
    """@brief 固件有两种写法（带 (Q) 和不带），都要认"""
    st = {}
    parse_magcal_line('[DBG]   eigRatio = 1.10', st)
    assert abs(st['eig_ratio'] - 1.10) < 1e-9


def test_assess_good_result():
    """@brief 全部指标达标 → ok=True、score=100、无 issue"""
    r = assess_magcal(_feed(GOOD_LINES))
    assert r['ok'] is True
    assert r['score'] == 100
    assert r['issues'] == []


def test_assess_too_few_samples():
    """@brief 样本不足要明确指出"""
    lines = ['[DBG]   samples = 8'] + GOOD_LINES[1:]
    r = assess_magcal(_feed(lines))
    assert r['ok'] is False
    assert any('样本' in x for x in r['issues'])


def test_assess_bad_residual():
    """@brief 残差过大 → 不通过"""
    lines = list(GOOD_LINES)
    lines[1] = '[DBG]   residualRms = 0.2100'
    r = assess_magcal(_feed(lines))
    assert r['ok'] is False
    assert any('残差' in x for x in r['issues'])


def test_assess_bad_eig_ratio():
    """@brief 特征值比偏离 1 太多 → 软铁畸变警告"""
    lines = list(GOOD_LINES)
    lines[2] = '[DBG]   eigRatio (Q) = 2.400'
    r = assess_magcal(_feed(lines))
    assert r['ok'] is False
    assert any('特征值比' in x for x in r['issues'])


def test_assess_low_coverage():
    """@brief 覆盖率低 → 提示换姿态重采"""
    lines = list(GOOD_LINES)
    lines[3] = '[DBG]   coverageRatio = 0.21000'
    r = assess_magcal(_feed(lines))
    assert r['ok'] is False
    assert any('覆盖' in x for x in r['issues'])


def test_assess_field_magnitude_out_of_range():
    """@brief 总场强度偏离本地参考太多 → 疑似磁源干扰"""
    lines = list(GOOD_LINES)
    lines[4] = f'[DBG]   fieldMag (raw) = {FIELD_MAG_REF_UT + 30:.1f} uT'
    r = assess_magcal(_feed(lines))
    assert r['ok'] is False
    assert any('总场' in x for x in r['issues'])


def test_assess_empty_state():
    """@brief 什么都没读到 → 不能判合格"""
    r = assess_magcal({})
    assert r['ok'] is False
    assert r['issues']


def test_wizard_has_save_step():
    """@brief 向导必须包含 save 步骤——不保存等于白标（重启回旧参数）"""
    ids = [s['id'] for s in MAG_WIZARD]
    assert 'save' in ids
    assert ids.index('save') > ids.index('assess'), '保存应在判读之后'


def test_wizard_steps_have_required_fields():
    """@brief 每步都要有 id/标题/命令/提示，界面才画得出来"""
    for s in MAG_WIZARD:
        for k in ('id', 'title', 'cmd', 'hint'):
            assert k in s and s[k], f'步骤 {s.get("id")} 缺字段 {k}'
