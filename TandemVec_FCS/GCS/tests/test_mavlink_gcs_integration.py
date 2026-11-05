# -*- coding: utf-8 -*-
"""GCS MAVLink TX泵/RX快照/幂等连接

@file    test_mavlink_gcs_integration.py
@module  GCS/测试（MAVLink 服务与集成）
@brief   GCS MAVLink TX泵/RX快照/幂等连接
@details 无硬件单元测试；用 pymavlink 构造/解析 MAVLink 帧，验证 GCS 双向链路。
@theory  N/A
@ref     GCS-MAVLink双向-2026-11-04.md
@frame   MAVLink v1/v2 测试帧
@units   测试断言单位随字段后缀
"""
import pytest

pymavlink = pytest.importorskip('pymavlink')
from pymavlink import mavutil

from server import main as m
from server.mavlink_service import MavlinkService
from server.serial_link import FakeSerialLink


def _vehicle_frame(send_fn):
    buf = bytearray()

    class Sink:
        def write(self, data):
            buf.extend(data)
            return len(data)

    v = mavutil.mavlink.MAVLink(Sink())
    v.srcSystem = 1
    v.srcComponent = 1
    send_fn(v)
    return bytes(buf)


def test_mav_pump_writes_to_serial():
    g = m.GCS()
    g.protocol = 'mavlink'
    g.link = FakeSerialLink()
    g.link.connect()
    g.mav = MavlinkService()
    g.mav.request_params()
    g.mav_pump()
    assert len(g.link.written) > 0
    assert g.link.written[0] == 0xFD  # v2


def test_gcs_mavlink_rx_updates_snapshot(monkeypatch):
    g = m.GCS()
    g.protocol = 'mavlink'
    g.link = FakeSerialLink()
    g.link.connect()
    g.mav = MavlinkService()
    monkeypatch.setattr(m, 'g', g)

    frame = _vehicle_frame(lambda v: v.attitude_send(
        0, 0.2, -0.1, 0.3, 0.01, 0.02, 0.03))
    m._on_telemetry_rx(frame)
    assert g.mav.frames == 1
    assert g.snap['roll_deg'] == pytest.approx(11.4592, abs=1e-3)
    assert g.snap['pitch_deg'] == pytest.approx(-5.7296, abs=1e-3)
    assert g.stat_tele == 1


def test_gcs_mavlink_param_value_updates_params(monkeypatch):
    g = m.GCS()
    g.protocol = 'mavlink'
    g.link = FakeSerialLink()
    g.link.connect()
    g.mav = MavlinkService()
    monkeypatch.setattr(m, 'g', g)

    frame = _vehicle_frame(lambda v: v.param_value_send(
        b'att_roll.kp\x00\x00\x00\x00\x00', 2.4, 9, 169, 2))
    m._on_telemetry_rx(frame)
    assert g.mav.params['att_roll.kp'] == pytest.approx(2.4)
    assert g.snap['param_count'] == 169


def test_connect_idempotent_same_port_protocol():
    g = m.GCS()
    from server.serial_link import FakeSerialLink
    g.link = FakeSerialLink(port='COMX', baud=921600)
    g.link.connect()
    g.protocol = 'mavlink'
    # 应直接返回，不得 disconnect 掉已有链路
    g.connect('COMX', 921600, protocol='mavlink')
    assert g.link is not None
    assert g.link.is_open
