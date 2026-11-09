# -*- coding: utf-8 -*-
"""
@file    test_alarms.py
@module  GCS/单测 报警引擎
@brief   报警状态机判据：上升沿触发、同态不重复、恢复记录、阈值边界
@details 这几条是报警系统最容易写错的地方：
         ① 50 Hz 喂入时如果同态也推送，界面会被刷屏；
         ② 恢复不记事件，事后就查不到"什么时候好的"；
         ③ 阈值用 < 还是 <= 会让边界读数反复跳。
@theory  N/A
@frame   N/A
@units   见各断言
@note    时间用注入的固定值，不依赖真实时钟，测试可复现。
@ref     server/alarms.py
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from server.alarms import (  # noqa: E402
    BAT_CRIT_V,
    BAT_WARN_V,
    AlarmEngine,
)


def _engine() -> AlarmEngine:
    return AlarmEngine()


def test_battery_warn_triggers_once():
    """@brief 低电压首次触发一次；持续低电压不重复产生事件"""
    eng = _engine()
    snap = {'bat_voltage_v': BAT_WARN_V - 0.2}
    ev1 = eng.feed(snap, now=100.0)
    assert [e['key'] for e in ev1] == ['battery']
    assert ev1[0]['kind'] == 'raise'
    # 同一状态再喂 10 次，不应再产生事件
    for i in range(10):
        assert eng.feed(snap, now=101.0 + i) == []
    assert len(eng.active) == 1


def test_battery_recovery_recorded():
    """@brief 电压恢复要记一条 clear 事件并把告警从活动表移除"""
    eng = _engine()
    eng.feed({'bat_voltage_v': BAT_WARN_V - 0.2}, now=100.0)
    evs = eng.feed({'bat_voltage_v': 12.4}, now=110.0)
    assert len(evs) == 1
    assert evs[0]['kind'] == 'clear'
    assert evs[0]['key'] == 'battery'
    assert eng.active == {}


def test_battery_critical_text_distinguished():
    """@brief 严重欠压与普通偏低要给出不同描述（同一规则两档阈值）"""
    eng = _engine()
    evs = eng.feed({'bat_voltage_v': BAT_CRIT_V - 0.1}, now=100.0)
    assert evs and '严重' in evs[0]['text']


def test_battery_zero_is_ignored():
    """@brief 电压为 0 通常是没数据（AnoCom 缺该帧），不能当欠压报警"""
    eng = _engine()
    assert eng.feed({'bat_voltage_v': 0.0}, now=100.0) == []
    assert eng.feed({}, now=100.0) == []


def test_sats_below_threshold():
    """@brief 卫星数不足触发；等于阈值不触发（边界取 <）"""
    eng = _engine()
    assert eng.feed({'gps_sats': 5}, now=100.0) != []
    eng2 = _engine()
    assert eng2.feed({'gps_sats': 6}, now=100.0) == []


def test_attitude_limit():
    """@brief 姿态超限：45° 不触发，超过才触发"""
    eng = _engine()
    assert eng.feed({'roll_deg': 45.0, 'pitch_deg': 0.0}, now=100.0) == []
    eng2 = _engine()
    evs = eng2.feed({'roll_deg': 46.0, 'pitch_deg': 0.0}, now=100.0)
    assert evs and evs[0]['key'] == 'attitude'


def test_link_timeout():
    """@brief 链路中断按 link_age_s 判，未超时不报"""
    eng = _engine()
    assert eng.feed({}, link_age_s=1.0, now=100.0) == []
    evs = eng.feed({}, link_age_s=3.0, now=103.0)
    assert evs and evs[0]['key'] == 'link'


def test_gps_fix_requires_3d():
    """@brief fix<3 报警，fix=3 正常"""
    eng = _engine()
    assert eng.feed({'gps_fix': 2}, now=100.0) != []
    eng2 = _engine()
    assert eng2.feed({'gps_fix': 3}, now=100.0) == []


def test_snapshot_shape():
    """@brief snapshot 要给出活动告警、事件、规则说明三部分"""
    eng = _engine()
    eng.feed({'gps_sats': 2}, now=100.0)
    snap = eng.snapshot()
    assert 'active' in snap and 'events' in snap and 'rules' in snap
    assert len(snap['active']) == 1
    assert any(r['key'] == 'battery' for r in snap['rules'])


def test_clear_events_keeps_active():
    """@brief 清空历史不应把当前活动的告警也抹掉——它仍然真实存在"""
    eng = _engine()
    eng.feed({'gps_sats': 2}, now=100.0)
    eng.clear_events()
    assert eng.events == []
    assert len(eng.active) == 1


def test_events_capped():
    """@brief 事件表有上限，长期运行不能无限增长"""
    from server.alarms import MAX_EVENTS
    eng = _engine()
    # 反复触发/恢复，制造大量事件
    for i in range(MAX_EVENTS):
        eng.feed({'gps_sats': 2}, now=1000.0 + i * 2)
        eng.feed({'gps_sats': 20}, now=1001.0 + i * 2)
    assert len(eng.events) <= MAX_EVENTS
