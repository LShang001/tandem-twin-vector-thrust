# -*- coding: utf-8 -*-
"""MAVLink 服务单元测试（无硬件）。"""
import struct

import pytest

pymavlink = pytest.importorskip('pymavlink')
from pymavlink import mavutil

from server.mavlink_service import MavlinkService, TARGET_SYSID, TARGET_COMPID


class Vehicle:
    """测试用飞控端 MAVLink 打包器。"""

    def __init__(self):
        self.sink = bytearray()

        class Sink:
            def __init__(self, buf):
                self.buf = buf

            def write(self, data):
                self.buf.extend(data)
                return len(data)

        self.mav = mavutil.mavlink.MAVLink(Sink(self.sink))
        self.mav.srcSystem = TARGET_SYSID
        self.mav.srcComponent = TARGET_COMPID

    def bytes(self):
        out = bytes(self.sink)
        self.sink.clear()
        return out


def feed_vehicle(svc, fn):
    v = Vehicle()
    fn(v)
    svc.feed(v.bytes())


def test_attitude_and_global_position():
    svc = MavlinkService()
    feed_vehicle(svc, lambda v: v.mav.attitude_send(0, 0.1, -0.2, 1.0, 0.01, 0.02, 0.03))
    feed_vehicle(svc, lambda v: v.mav.global_position_int_send(
        0, 300000000, 1200000000, 123456, 45678, 100, 200, -50, 9000))
    assert svc.snap['roll_deg'] == pytest.approx(5.7296, abs=1e-3)
    assert svc.snap['pitch_deg'] == pytest.approx(-11.4592, abs=1e-3)
    assert svc.snap['yaw_deg'] == pytest.approx((57.2958 + 360) % 360, abs=1e-3)
    assert svc.snap['lat_deg'] == pytest.approx(30.0)
    assert svc.snap['lon_deg'] == pytest.approx(120.0)
    assert svc.snap['alt_msl_m'] == pytest.approx(123.456)
    assert svc.snap['rel_alt_m'] == pytest.approx(45.678)
    assert svc.snap['v_north_mps'] == pytest.approx(1.0)
    assert svc.snap['v_down_mps'] == pytest.approx(-0.5)
    assert svc.snap['heading_deg'] == pytest.approx(90.0)


def test_status_rc_and_raw_imu():
    svc = MavlinkService()
    feed_vehicle(svc, lambda v: v.mav.sys_status_send(
        0, 0, 0, 0, 12600, 1234, -1, 0, 0, 0, 0, 0, 0))
    feed_vehicle(svc, lambda v: v.mav.rc_channels_raw_send(
        0, 1, 1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 8))
    feed_vehicle(svc, lambda v: v.mav.raw_imu_send(
        0, 1000, 2000, -3000, 100, 200, 300, 0, 0, 0))
    assert svc.snap['bat_voltage_v'] == pytest.approx(12.6)
    assert svc.snap['bat_current_a'] == pytest.approx(12.34)
    assert svc.snap['rc_channels'][:3] == [1000, 1100, 1200]
    assert svc.snap['acc_x_ms2'] == pytest.approx(9.80665)
    assert svc.snap['gyro_z_dps'] == pytest.approx(17.1887, abs=1e-3)


def test_heartbeat_armed_mode_and_gps():
    svc = MavlinkService()
    feed_vehicle(svc, lambda v: v.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_QUADROTOR,
        mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        128, 7, mavutil.mavlink.MAV_STATE_ACTIVE))
    feed_vehicle(svc, lambda v: v.mav.gps_raw_int_send(
        0, 3, 300000000, 1200000000, 100000, 120, 200, 500, 9000, 12, 100000))
    assert svc.snap['armed'] is True
    assert svc.snap['flight_mode'] == 7
    assert svc.snap['fix_type'] == 3
    assert svc.snap['sats'] == 12


def test_param_value_and_count():
    svc = MavlinkService()
    feed_vehicle(svc, lambda v: v.mav.param_value_send(
        b'att_roll.kp\x00\x00\x00\x00\x00', 2.5, 9, 148, 3))
    assert svc.params['att_roll.kp'] == pytest.approx(2.5)
    assert svc.params_received == 1
    assert svc.snap['param_count'] == 148
    assert svc.snap['param_index'] == 3


def test_statustext_bytes_and_command_ack():
    svc = MavlinkService()
    feed_vehicle(svc, lambda v: v.mav.statustext_send(4, b'hello\x00'))
    feed_vehicle(svc, lambda v: v.mav.command_ack_send(400, 2))
    assert svc.status_text[-1] == 'hello'
    assert svc.snap['statustext'] == 'hello'
    assert svc.last_ack == (400, 2)


def test_home_and_autopilot_version():
    svc = MavlinkService()
    feed_vehicle(svc, lambda v: v.mav.home_position_send(
        300000000, 1200000000, 123000, 0, 0, 0,
        [1.0, 0, 0, 0], 0, 0, 0, 0))
    feed_vehicle(svc, lambda v: v.mav.autopilot_version_send(
        0, 1, 2, 3, 4, b'T' * 8, b'M' * 8, b'O' * 8, 0, 0, 0, b'U' * 18))
    assert svc.snap['home_lat_deg'] == pytest.approx(30.0)
    assert svc.snap['home_alt_m'] == pytest.approx(123.0)
    assert svc.snap['capabilities'] == 0
    assert svc.snap['flight_sw_version'] == 1


def test_outgoing_param_requests_and_set():
    svc = MavlinkService()
    svc.request_params()
    svc.request_param('att_roll.kp')
    svc.set_param('att_roll.kp', 2.6)
    tx = svc.drain_tx()
    assert tx.count(b'\xfd') == 3
    assert len(tx) > 20
    assert svc.drain_tx() == b''


def test_outgoing_commands():
    svc = MavlinkService()
    svc.arm(True)
    svc.set_mode(7)
    svc.reboot()
    svc.request_autopilot_version()
    svc.mission_request_list()
    tx = svc.drain_tx()
    assert tx.count(b'\xfd') == 5


def test_heartbeat_periodic_starts_tx():
    svc = MavlinkService()
    svc.feed(b'')
    assert len(svc.drain_tx()) > 0


def test_missing_pymavlink(monkeypatch):
    import server.mavlink_service as m
    monkeypatch.setattr(m, 'mavlink2', None)
    with pytest.raises(RuntimeError):
        MavlinkService()


def test_vfr_hud_and_mission_ack():
    svc = MavlinkService()
    feed_vehicle(svc, lambda v: v.mav.vfr_hud_send(12.5, 15.0, 90, 45, 120.0, 1.2))
    feed_vehicle(svc, lambda v: v.mav.mission_ack_send(
        TARGET_SYSID, TARGET_COMPID, 3))
    assert svc.snap['airspeed_mps'] == pytest.approx(12.5)
    assert svc.snap['groundspeed_mps'] == pytest.approx(15.0)
    assert svc.snap['heading_deg'] == 90
    assert svc.snap['mission_ack_type'] == 3


def test_mission_count_zero():
    svc = MavlinkService()
    feed_vehicle(svc, lambda v: v.mav.mission_count_send(
        TARGET_SYSID, TARGET_COMPID, 0))
    assert svc.snap['mission_count'] == 0


def test_ping_and_timesync_tx():
    svc = MavlinkService()
    svc.ping(7)
    svc.timesync()
    tx = svc.drain_tx()
    assert tx.count(b'\xfd') == 2
    assert len(tx) >= 20


def test_param_request_list_and_set_tx_are_v2():
    svc = MavlinkService()
    svc.request_params()
    svc.set_param('att_roll.kp', 2.6)
    tx = svc.drain_tx()
    assert tx[0] == 0xFD
    assert tx.count(b'\xfd') == 2
