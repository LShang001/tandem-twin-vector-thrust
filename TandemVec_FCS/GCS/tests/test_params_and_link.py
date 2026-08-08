# -*- coding: utf-8 -*-
"""参数元数据 + 遥测链路（GCS 全局状态机）测试"""
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'server'))

import anocom
import params as pm


def test_param_count_matches_firmware():
    """参数总数 = 12 环 × 9 字段 + 9 滤波 alpha = 117（与固件 ANO_PARAMS_COUNT 一致）"""
    names = pm.expected_names()
    assert len(names) == 117
    assert len(set(names)) == 117
    # 首个与末个（线上名 ≤20B 协议限制：filter_alpha→falpha，滤波数组短名）
    assert names[0] == 'att_roll.kp'
    assert names[-1] == 'out_alpha[2]'
    assert 'att_roll.falpha' in names
    assert all(len(n) <= 20 for n in names), [n for n in names if len(n) > 20]


def test_param_meta_cover_all():
    """元数据覆盖全部固件参数名（防漂移）"""
    missing = [n for n in pm.expected_names() if n not in pm.META]
    assert missing == []


def test_param_group_order():
    """分组顺序完整"""
    order = pm.group_order()
    assert order[0] == '姿态外环（deg 域）'
    assert '其他' in order


def test_meta_ranges():
    m = pm.META['att_roll.kp']
    assert m['group'] == '姿态外环（deg 域）' and m['type'] == 'float'
    assert pm.META['att_yaw.enabled']['type'] == 'uint8'
    assert pm.META['att_roll.out_max']['unit'] == 'deg/s'
    assert pm.META['att_roll.falpha']['group'] == '姿态外环（deg 域）'
    assert pm.META['out_alpha[2]']['group'] == '内环输出滤波'


# ---------------- 遥测链路（GCS 全局状态机）----------------
# 注意：server.main 的处理函数操作模块级全局 g（单例），测试须注入/断言 m.g

def _reset_g():
    import server.main as m
    m.g.disconnect()
    m.g.snap = {}
    m.g.param_names = {}
    m.g.param_values = {}
    m.g.param_pending = {}
    m.g.tele_buf.clear()
    return m


def _drain():
    """消费 rx 队列中所有字节流"""
    import server.main as m
    while not m.g.rx_q.empty():
        kind, payload = m.g.rx_q.get()
        m._on_telemetry_rx(payload)

def _make_telemetry_frames():
    """构造一组模拟固件 4 组遥测帧"""
    frames = []
    # 组0：IMU + 欧拉 + 目标姿态
    frames.append(anocom.encode_frame(anocom.FUNC_IMU_DATA,
        struct.pack('<3h', 981, 0, 1000) + struct.pack('<3h', 0, 0, 0) + b'\x00'))
    frames.append(anocom.encode_frame(anocom.FUNC_ATTITUDE_EULER,
        struct.pack('<3hB', 1250, -300, 9000, 0x81)))
    frames.append(anocom.encode_frame(anocom.FUNC_TARGET_ATTITUDE,
        struct.pack('<3h', 1000, 0, 0)))
    # 组1：高度 + 模式 + 控制量
    frames.append(anocom.encode_frame(anocom.FUNC_ALTITUDE_DATA,
        struct.pack('<3iB', 32000, 33000, 32500, 1)))
    frames.append(anocom.encode_frame(anocom.FUNC_FLIGHT_MODE, bytes([1, 1, 0, 0, 0])))
    frames.append(anocom.encode_frame(anocom.FUNC_ATTITUDE_CONTROL,
        struct.pack('<4h', 100, -200, 500, 30)))
    # 组2：目标速度 + 飞行速度 + PWM + 执行器输出(0x40)
    frames.append(anocom.encode_frame(anocom.FUNC_TARGET_SPEED, struct.pack('<3h', 0, 0, 0)))
    frames.append(anocom.encode_frame(anocom.FUNC_FLIGHT_SPEED, struct.pack('<3h', 100, -50, 250)))
    frames.append(anocom.encode_frame(anocom.FUNC_PWM_OUTPUT, struct.pack('<8H', *range(1000, 1008))))
    frames.append(anocom.encode_frame(anocom.FUNC_ACTUATOR_OUT,
        struct.pack('<hh2HhBB', -1230, 855, 624, 587, 490, 0x05, 0)))
    # 组3：位置 + 电压电流(压力) + GPS
    frames.append(anocom.encode_frame(anocom.FUNC_POS_OFFSET, struct.pack('<3i', 10000, -20000, 32500)))
    frames.append(anocom.encode_frame(anocom.FUNC_VOLT_CURR, struct.pack('<5H', 1260, 1680, 1520, 1480, 0)))
    frames.append(anocom.encode_frame(anocom.FUNC_GPS_INFO1,
        bytes([3, 12]) + struct.pack('<2i', 116407325, 39904823) +
        struct.pack('<i', 4500) + struct.pack('<3h', 100, 200, 300) + bytes([15, 20, 25])))
    return frames


def test_telemetry_aggregation():
    """注入模拟固件字节流 → GCS 快照字段完整"""
    m = _reset_g()
    m.g.connect_fake(_make_telemetry_frames())
    _drain()
    s = m.g.snap
    assert abs(s['roll_deg'] - 12.5) < 1e-6
    assert abs(s['pitch_deg'] - (-3.0)) < 1e-6
    assert s['heading_deg'] == 90.0
    assert s['deta100_online'] is True
    assert abs(s['acc_x_ms2'] - 9.81) < 1e-6
    assert abs(s['alt_fu_m'] - 325.0) < 1e-6
    assert s['flight_mode'] == 1 and s['unlocked'] is True
    assert s['ctrl_thr_pct'] == 50.0
    assert abs(s['vel_n_ms'] - 1.0) < 1e-6 and s['vel_up_ms'] == 2.5
    assert abs(s['rel_n_m'] - 100.0) < 1e-6 and s['rel_h_m'] == 325.0
    assert abs(s['p1_mpa'] - 15.2) < 1e-6
    assert s['gps_sats'] == 12 and s['gps_fix'] == 3
    assert s['rc3'] == 1002
    # 执行器输出帧（0x40）：摆角 deg / 电机 % / 差速 / 饱和标记
    assert abs(s['tvc_upper_deg'] - (-12.3)) < 1e-6
    assert abs(s['tvc_lower_deg'] - 8.55) < 1e-6
    assert abs(s['motor_front_pct'] - 62.4) < 1e-6
    assert abs(s['motor_rear_pct'] - 58.7) < 1e-6
    assert abs(s['dw'] - 0.49) < 1e-6
    assert s['sat_df'] is True and s['sat_dt'] is False and s['sat_dw'] is True
    # 链路统计：13 帧全解码、无 CRC 失败、遥测时间戳已更新
    assert m.g.stat_frames == 13 and m.g.stat_bad == 0
    assert m.g.stat_bytes == sum(len(f) for f in _make_telemetry_frames())
    assert m.g.last_tele_ts is not None
    m.g.disconnect()


def test_half_frame_retained_across_chunks():
    """块尾半帧必须保留到下一拼接（旧实现整体 clear 会丢半帧）"""
    m = _reset_g()
    frame = anocom.encode_frame(anocom.FUNC_ATTITUDE_EULER,
        struct.pack('<3hB', 1250, -300, 9000, 0x81))
    cut = len(frame) - 3
    m.g.connect_fake([frame[:cut], frame[cut:]])
    _drain()
    assert abs(m.g.snap['roll_deg'] - 12.5) < 1e-6   # 半帧拼接后成功解码
    assert len(m.g.tele_buf) == 0                    # 消费干净无残留
    m.g.disconnect()


def test_garbage_stream_bounded():
    """乱码流（错波特率场景）：缓冲有界不膨胀、统计照常累计——防 O(n²) 卡死"""
    m = _reset_g()
    garbage = bytes((i * 37 + 11) & 0xFF for i in range(20000))
    garbage = garbage.replace(b'\xab', b'\xaa')      # 剔除帧头字节，保证无帧
    m.g.connect_fake([garbage[:9000], garbage[9000:18000], garbage[18000:]])
    _drain()
    assert len(m.g.tele_buf) <= m.TELE_BUF_CAP
    assert m.g.stat_frames == 0
    assert m.g.stat_bytes == len(garbage)
    assert m.g.snap == {}                            # 无有效帧 → 快照为空
    m.g.disconnect()


def test_realworld_chunking_brutal():
    """真实串口形态：12 帧拼成整流 + 前置垃圾 + 任意断点切块 → 全部解码。
    模拟 2M VCP 实际到包方式（多块拼帧、块间任意撕裂、帧间夹垃圾）"""
    m = _reset_g()
    blob = b''.join(_make_telemetry_frames())
    garbage = b'\x00\xff\x13\x37\xaa'                # 帧间垃圾（无 AB 头）
    stream = garbage + blob
    # 任意断点切块（7 / 100 / 33 / 剩余），制造跨块半帧
    chunks = [stream[:7], stream[7:107], stream[107:140], stream[140:]]
    m.g.connect_fake(chunks)
    _drain()
    s = m.g.snap
    assert abs(s['roll_deg'] - 12.5) < 1e-6          # 组0
    assert s['flight_mode'] == 1                     # 组1
    assert abs(s['vel_n_ms'] - 1.0) < 1e-6           # 组2
    assert s['gps_sats'] == 12                       # 组3
    assert m.g.stat_frames == 13                     # 13 帧无一丢失
    assert m.g.stat_tele == 13                       # 全部计入遥测帧统计
    m.g.disconnect()


def test_param_read_response_flow():
    """E2 信息帧 + E1 值回传 → param_names/values 填充"""
    m = _reset_g()
    frames = []
    # E2 信息帧：id=0 → att_roll.kp float
    e2 = anocom.encode_frame(anocom.FUNC_PARAM_INFO,
        struct.pack('<HB', 0, anocom.ANO_FLOAT) + b'att_roll.kp' + b'\x00' * 12)
    frames.append(e2)
    # E1 值回传：id=0 → 2.5f
    e1 = anocom.encode_frame(anocom.FUNC_PARAM_WRITE_READ, struct.pack('<Hf', 0, 2.5))
    frames.append(e1)
    m.g.connect_fake(frames)
    _drain()
    assert m.g.param_names[0]['name'] == 'att_roll.kp'
    assert m.g.param_names[0]['type'] == 'float'
    assert m.g.param_values[0] == 2.5
    m.g.disconnect()


def test_param_write_ack_matching():
    """0x00 校验帧匹配（SC/AC 一致 → ack）"""
    m = _reset_g()
    # 模拟发送写入帧并记录期望校验
    frame = anocom.cmd_write_param(1, 3.3)
    body = frame[:-2]
    m.g.param_pending[1] = (anocom.sum_check(body), anocom.add_check(body), 0.0)
    # 固件回 0x00 校验帧（ID_GET=0xE1 + 原帧 SC/AC）
    ack = anocom.encode_frame(anocom.FUNC_DATA_CHECK,
        bytes([anocom.FUNC_PARAM_WRITE_READ, anocom.sum_check(body), anocom.add_check(body)]))
    m.g.connect_fake([ack])
    _drain()
    assert 1 not in m.g.param_pending   # 已确认弹出
    m.g.disconnect()


def test_datalog_recorder_roundtrip(tmp_path):
    """CSV 记录 → 回放读取（字段保留）"""
    from datalog import CsvRecorder, ReplaySource
    p = tmp_path / 'rec.csv'
    rec = CsvRecorder(str(p))
    rec.start()
    rec.write({'t_ms': 1000.0, 'roll_deg': 12.5, 'unlocked': True, 'rc3': 1500})
    rec.write({'t_ms': 1020.0, 'roll_deg': 13.0, 'unlocked': True, 'rc3': 1498})
    rec.stop()
    r = ReplaySource(str(p))
    r.open()
    row1 = r.next_row()
    assert row1['t_ms'] == 1000.0 and row1['roll_deg'] == 12.5
    assert row1['unlocked'] == 1.0
    assert r.next_row()['rc3'] == 1498.0
    r.close()


def test_param_name_to_id_mapping():
    """名字→ID 解析：expected_names 下标 = 固件注册序 = 参数 ID。
    ★ 27/28/36 为实机在线验证值（2026-08-09）——防参数表重排/改名漂移"""
    names = pm.expected_names()
    assert names.index('rate_roll.kp') == 27
    assert names.index('rate_roll.ki') == 28
    assert names.index('rate_pitch.kp') == 36
    assert names.index('rate_pitch.ki') == 37
    assert names.index('att_roll.kp') == 0
    assert names.index('att_pitch.kp') == 9
    assert names.index('rate_roll.kd') == 29
    assert len(names) == 117
