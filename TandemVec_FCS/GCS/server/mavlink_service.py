# -*- coding: utf-8 -*-
"""
mavlink_service.py — GCS 的 MAVLink v2 双向服务

@file    mavlink_service.py
@module  GCS/MAVLink 协议服务
@brief   QGC/MP 兼容的 MAVLink 解析、遥测快照、参数与命令发送
@details 输入：串口字节流（MavlinkService.feed）；处理：pymavlink 解帧、
        维护遥测字段/PARAM_VALUE/STATUSTEXT/COMMAND_ACK；输出：GCS snapshot
        字典与待发送字节（drain_tx）。
@theory  N/A（协议适配层）
@ref     mavlink.io services/parameter、services/command、messages/common
@frame   MAVLink v2（0xFD）；GCS system=255/component=190，飞控 target=1/1
@units   遥测保持 GCS 既有后缀（_deg/_dps/_ms2/_m/_mps/_v）
@note    pymavlink 为可选依赖：缺失时构造抛 RuntimeError，AnoCom 路径不受影响。
"""
from __future__ import annotations

import time
from typing import Any, Optional

try:
    from pymavlink import mavutil
    from pymavlink.dialects.v20 import common as mavlink2
except ImportError:  # pragma: no cover - 无 pymavlink 的环境仍可 import
    mavutil = None
    mavlink2 = None


GCS_SYSID = 255
GCS_COMPID = 190
TARGET_SYSID = 1
TARGET_COMPID = 1


class _Sink:
    """pymavlink 把发送字节写到该 sink，再交给串口写线程。"""

    def __init__(self):
        self.buf = bytearray()

    def write(self, data):
        self.buf.extend(data)
        return len(data)


class MavlinkService:
    """MAVLink 双向状态机；不直接打开串口。"""

    def __init__(self, target_sysid: int = TARGET_SYSID,
                 target_compid: int = TARGET_COMPID):
        if mavlink2 is None:
            raise RuntimeError('pymavlink 未安装（pip install pymavlink）')
        self.target_sys = target_sysid
        self.target_comp = target_compid
        self.sink = _Sink()
        self.mav = mavlink2.MAVLink(self.sink)
        self.mav.srcSystem = GCS_SYSID
        self.mav.srcComponent = GCS_COMPID
        self.mav.robust_parsing = True
        self.snap: dict[str, Any] = {}
        self.params: dict[str, float] = {}
        self.param_count: Optional[int] = None
        self.params_received = 0
        self.status_text: list[str] = []
        self.last_ack: Optional[tuple[int, int]] = None
        self.msg_counts: dict[int, int] = {}
        self.frames = 0
        self.last_rx_ts: Optional[float] = None
        self._last_hb = 0.0

    # ---------------- 接收 ----------------
    def feed(self, data: bytes):
        """喂入任意字节流；pymavlink 逐字节解析并累积快照。"""
        now = time.time()
        for b in data:
            try:
                msg = self.mav.parse_char(bytes([b]))
            except Exception:
                continue
            if msg is not None:
                self.frames += 1
                self.msg_counts[msg.get_type()] = self.msg_counts.get(msg.get_type(), 0) + 1
                self._handle(msg)
        if data:
            self.last_rx_ts = now
        self._maybe_heartbeat(now)

    def _maybe_heartbeat(self, now: float):
        if now - self._last_hb >= 1.0:
            self._last_hb = now
            self.send_heartbeat()

    def _handle(self, msg):
        t = msg.get_type()
        s = self.snap
        if t == 'HEARTBEAT':
            base_mode = getattr(msg, 'base_mode', 0)
            s['armed'] = bool(base_mode & 128)
            s['flight_mode'] = getattr(msg, 'custom_mode', 0)
            s['vehicle_type'] = getattr(msg, 'type', 0)
        elif t == 'SYS_STATUS':
            s['bat_voltage_v'] = getattr(msg, 'voltage_battery', 0) / 1000.0
            s['bat_current_a'] = getattr(msg, 'current_battery', -1) / 100.0
            s['cpu_load_pct'] = getattr(msg, 'load', 0) / 10.0
        elif t == 'ATTITUDE':
            s['roll_deg'] = _deg(msg.roll)
            s['pitch_deg'] = _deg(msg.pitch)
            s['yaw_deg'] = (_deg(msg.yaw) + 360.0) % 360.0
            s['roll_rate_dps'] = _deg(msg.rollspeed)
            s['pitch_rate_dps'] = _deg(msg.pitchspeed)
            s['yaw_rate_dps'] = _deg(msg.yawspeed)
        elif t == 'ATTITUDE_QUATERNION':
            s['q'] = [msg.q1, msg.q2, msg.q3, msg.q4]
        elif t == 'GLOBAL_POSITION_INT':
            s['lat_deg'] = msg.lat / 1e7
            s['lon_deg'] = msg.lon / 1e7
            s['alt_msl_m'] = msg.alt / 1000.0
            s['rel_alt_m'] = msg.relative_alt / 1000.0
            s['v_north_mps'] = msg.vx / 100.0
            s['v_east_mps'] = msg.vy / 100.0
            s['v_down_mps'] = msg.vz / 100.0
            s['heading_deg'] = msg.hdg / 100.0 if msg.hdg != 65535 else s.get('yaw_deg', 0.0)
        elif t == 'VFR_HUD':
            s['airspeed_mps'] = msg.airspeed
            s['groundspeed_mps'] = msg.groundspeed
            s['throttle_pct'] = msg.throttle
            s['baro_alt_m'] = msg.alt
            s['climb_mps'] = msg.climb
            s['heading_deg'] = msg.heading
        elif t == 'RAW_IMU':
            # mG → m/s²，mrad/s → deg/s
            for i, k in enumerate(('acc_x_ms2', 'acc_y_ms2', 'acc_z_ms2')):
                s[k] = getattr(msg, ('xacc', 'yacc', 'zacc')[i]) / 1000.0 * 9.80665
            for i, k in enumerate(('gyro_x_dps', 'gyro_y_dps', 'gyro_z_dps')):
                s[k] = _deg(getattr(msg, ('xgyro', 'ygyro', 'zgyro')[i]) / 1000.0)
        elif t == 'RC_CHANNELS_RAW':
            s['rc_channels'] = [getattr(msg, f'chan{i}_raw') for i in range(1, 9)]
        elif t == 'GPS_RAW_INT':
            s['fix_type'] = msg.fix_type
            s['sats'] = msg.satellites_visible
            s['gps_eph'] = msg.eph
        elif t == 'HOME_POSITION':
            s['home_lat_deg'] = msg.latitude / 1e7
            s['home_lon_deg'] = msg.longitude / 1e7
            s['home_alt_m'] = msg.altitude / 1000.0
        elif t == 'AUTOPILOT_VERSION':
            s['capabilities'] = msg.capabilities
            s['flight_sw_version'] = msg.flight_sw_version
        elif t == 'PARAM_VALUE':
            name = _param_name(msg.param_id)
            if name:
                self.params[name] = msg.param_value
                self.params_received += 1
                s['param_count'] = msg.param_count
                s['param_index'] = msg.param_index
                self.param_count = msg.param_count
        elif t == 'STATUSTEXT':
            text = msg.text
            if isinstance(text, bytes):
                text = text.split(b'\x00', 1)[0].decode('utf-8', 'replace')
            else:
                text = str(text).split('\x00', 1)[0]
            self.status_text.append(text)
            del self.status_text[:-20]
            s['statustext'] = text
        elif t == 'COMMAND_ACK':
            self.last_ack = (msg.command, msg.result)
            s['last_ack_command'] = msg.command
            s['last_ack_result'] = msg.result
        elif t == 'MISSION_ACK':
            s['mission_ack_type'] = msg.type

    # ---------------- 发送 ----------------
    def drain_tx(self) -> bytes:
        out = bytes(self.sink.buf)
        self.sink.buf.clear()
        return out

    def _send(self, msg):
        self.mav.send(msg)

    def send_heartbeat(self):
        self.mav.heartbeat_send(mavlink2.MAV_TYPE_GCS,
                                mavlink2.MAV_AUTOPILOT_INVALID,
                                0, 0, mavlink2.MAV_STATE_ACTIVE)

    def request_params(self):
        self.mav.param_request_list_send(self.target_sys, self.target_comp)

    def request_param(self, name: str):
        self.mav.param_request_read_send(self.target_sys, self.target_comp,
                                         name.encode('ascii')[:15], -1)

    def set_param(self, name: str, value: float):
        self.mav.param_set_send(self.target_sys, self.target_comp,
                                name.encode('ascii')[:15], float(value),
                                mavlink2.MAV_PARAM_TYPE_REAL32)

    def command_long(self, command: int, p1: float = 0, p2: float = 0,
                     p3: float = 0, p4: float = 0, p5: float = 0,
                     p6: float = 0, p7: float = 0):
        self.mav.command_long_send(self.target_sys, self.target_comp,
                                   command, 0, p1, p2, p3, p4, p5, p6, p7)

    def arm(self, arm: bool = True):
        self.command_long(400, 1 if arm else 0)

    def set_mode(self, mode: int):
        self.command_long(176, 1, mode)

    def reboot(self):
        self.command_long(246, 1)

    def request_autopilot_version(self):
        self.command_long(520)

    def mission_request_list(self):
        self.mav.mission_request_list_send(self.target_sys, self.target_comp)


def _deg(rad: float) -> float:
    return float(rad) * 57.29577951308232


def _param_name(pid) -> str:
    if isinstance(pid, bytes):
        raw = pid
    else:
        raw = str(pid).encode('ascii', 'ignore')
    return raw.split(b'\x00', 1)[0].decode('ascii', 'ignore')
