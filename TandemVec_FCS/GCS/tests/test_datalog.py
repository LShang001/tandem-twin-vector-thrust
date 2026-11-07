# -*- coding: utf-8 -*-
"""datalog（遥测 CSV 记录器）的单元测试。

2026-11-08 补：该模块此前零测试覆盖，但它是飞行数据落盘的唯一通道——
列顺序、缺列填空、类型格式化（bool→0/1、float→4 位小数）一旦出错，
记录下来的数据就不可信，而记录本身是事后分析的全部依据。
"""
from __future__ import annotations

import csv
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from server.datalog import CsvRecorder, RECORD_FIELDS, _fmt, list_recordings


def test_fmt_bool_and_float():
    """bool 转 0/1、float 保留 4 位、其它原样。"""
    assert _fmt(True) == '1'
    assert _fmt(False) == '0'
    assert _fmt(1.23456789) == '1.2346'
    assert _fmt(0.5) == '0.5000'
    assert _fmt(7) == 7
    assert _fmt('abc') == 'abc'


def test_recorder_lifecycle(tmp_path):
    """start 后 active、stop 后不 active；重复 start 不重开。"""
    rec = CsvRecorder(str(tmp_path / 'r.csv'))
    assert not rec.active
    rec.start()
    assert rec.active
    rec.start()  # 幂等，不应抛错
    assert rec.active
    rec.stop()
    assert not rec.active
    rec.stop()  # 幂等


def test_recorder_writes_header_and_rows(tmp_path):
    """表头 = RECORD_FIELDS；写入的一行按列名字段取值。"""
    path = tmp_path / 'r.csv'
    rec = CsvRecorder(str(path))
    rec.start()
    rec.write({'t_ms': 1000, 'roll_deg': 1.5, 'unlocked': True})
    rec.stop()

    with open(path, encoding='utf-8', newline='') as f:
        rows = list(csv.DictReader(f))

    assert len(rows) == 1
    assert rows[0]['t_ms'] == '1000'
    assert rows[0]['roll_deg'] == '1.5000'
    assert rows[0]['unlocked'] == '1'


def test_recorder_missing_fields_blank(tmp_path):
    """快照缺列时该列留空，不抛错（restval=''）。"""
    path = tmp_path / 'r.csv'
    rec = CsvRecorder(str(path))
    rec.start()
    rec.write({'t_ms': 1})  # 只给一列
    rec.stop()

    with open(path, encoding='utf-8', newline='') as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 1
    assert rows[0]['t_ms'] == '1'
    # 其它列应为空串，不是 None 或缺键
    assert rows[0]['roll_deg'] == ''


def test_recorder_ignores_unknown_fields(tmp_path):
    """快照里多出的列被忽略（extrasaction='ignore'），不抛错。"""
    path = tmp_path / 'r.csv'
    rec = CsvRecorder(str(path))
    rec.start()
    rec.write({'t_ms': 2, 'not_a_field': 99})
    rec.stop()
    with open(path, encoding='utf-8', newline='') as f:
        rows = list(csv.DictReader(f))
    assert 'not_a_field' not in rows[0]
    assert rows[0]['t_ms'] == '2'


def test_write_before_start_is_noop(tmp_path):
    """未 start 时 write 静默返回，不应崩溃（调用方可能先写后开）。"""
    rec = CsvRecorder(str(tmp_path / 'r.csv'))
    rec.write({'t_ms': 5})  # 不抛
    assert not (tmp_path / 'r.csv').exists()


def test_list_recordings_sorted_desc(tmp_path):
    """list_recordings 只列 .csv，返回 name/size/mtime，按 mtime 倒序。"""
    import time
    for name in ('a.csv', 'b.csv'):
        p = tmp_path / name
        p.write_text('x\n', encoding='utf-8')
        time.sleep(0.01)
    (tmp_path / 'ignore.txt').write_text('x', encoding='utf-8')

    items = list_recordings(str(tmp_path))
    assert [it['name'] for it in items] == ['b.csv', 'a.csv']
    assert all(set(it) == {'name', 'size', 'mtime'} for it in items)


def test_list_recordings_missing_dir():
    """目录不存在时返回空列表，不抛错。"""
    assert list_recordings('/nonexistent/path/xyz') == []


def test_record_fields_unique_and_ordered():
    """RECORD_FIELDS 无重复——重复列名会让 DictWriter 静默覆盖。"""
    assert len(RECORD_FIELDS) == len(set(RECORD_FIELDS)), "RECORD_FIELDS 存在重复列名"
    assert 't_ms' in RECORD_FIELDS
