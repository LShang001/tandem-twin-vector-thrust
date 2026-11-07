#!/usr/bin/env python3
"""cli.py 里两个纯函数的单元测试。

@file    TandemVec_FCS/GCS/tests/test_cli_helpers.py
@module  上位机/命令行
@brief   锁定写后读回比对（_val_close）与参数名解析（_param_id_of）的判据。
@details cli.py 共 29 个函数，多数是 cmd_* 需要真实串口，无法脱离硬件测试。
         但其中两个是纯逻辑，且都在安全敏感路径上：
         - `_val_close`：参数写入后的读回比对，判错会让「写失败」误报成成功
           （或反之），直接关系到上板调参是否可信。
         - `_param_id_of`：参数名 → ID 的解析，解析错会写进别的参数槽位。
         本测试只覆盖这两个，不假装覆盖整个 CLI。
@theory  N/A（命令行辅助函数回归）
@frame   N/A
@units   _val_close 的 tol 为相对容差（无量纲）；integer 模式按四舍五入取整
"""
from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from server.cli import _param_id_of, _val_close
from server import params as pm


class TestValClose:
    """写后读回比对：float 用相对容差，整型按四舍五入。"""

    def test_none_never_matches(self):
        assert not _val_close(None, 1.0)
        assert not _val_close(1.0, None)
        assert not _val_close(None, None)

    def test_float_exact(self):
        assert _val_close(1.5, 1.5)

    def test_float_within_tolerance(self):
        # 默认 tol=1e-4，参考值 100 → 允许 0.01 的绝对差
        assert _val_close(100.009, 100.0)
        assert not _val_close(100.02, 100.0)

    def test_float_tolerance_scales_with_magnitude(self):
        # 相对容差：小值域用绝对下限 1.0，大值域按比例放宽
        assert _val_close(0.00005, 0.0)      # |diff| <= 1e-4 * max(1, 0)
        assert not _val_close(0.001, 0.0)
        assert _val_close(1000.05, 1000.0)   # 1e-4*1000 = 0.1

    def test_float_custom_tol(self):
        assert not _val_close(1.04, 1.0, tol=1e-4)
        assert _val_close(1.04, 1.0, tol=0.05)

    def test_integer_mode_rounds(self):
        # 整型模式按四舍五入比，不做相对容差
        assert _val_close(5.4, 5, is_float=False)
        assert not _val_close(5.6, 5, is_float=False)
        assert _val_close(5.5, 6, is_float=False)   # Python round-half-even: round(5.5)=6? 实际 6.0→6


class TestParamIdOf:
    """参数名 → ID 解析。"""

    def test_numeric_id_passthrough(self):
        assert _param_id_of('0') == 0
        assert _param_id_of('168') == 168

    def test_known_name(self):
        names = pm.expected_names()
        for probe in names[:3] + names[-3:]:
            assert _param_id_of(probe) == names.index(probe)

    def test_name_with_spaces_tolerated(self):
        names = pm.expected_names()
        # 找一个带空格的参数名（若有），否则用首项构造
        probe = next((n for n in names if ' ' in n), None)
        if probe is None:
            pytest.skip('名字表里没有带空格的参数名')
        assert _param_id_of(probe.replace(' ', '')) == names.index(probe)

    def test_unknown_name_exits(self):
        with pytest.raises(SystemExit) as ei:
            _param_id_of('definitely_not_a_param_name_xyz')
        assert ei.value.code == 1
