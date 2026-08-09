# -*- coding: utf-8 -*-
"""AnoCom 协议编解码测试 — 与固件 lib/AnoComProtocol 逐字节对齐"""
import struct
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'server'))

import anocom


def test_frame_checksum_reference():
    """校验和算法对照：已知帧逐字节验证 SC/AC"""
    # 手工构造一帧：AB 05 FF 03 07 00 + payload(7B) + SC + AC
    payload = bytes([0x10, 0x27, 0xD0, 0xFF, 0x00, 0x00, 0x01])
    f = anocom.encode_frame(anocom.FUNC_ATTITUDE_EULER, payload)
    assert f[0] == 0xAB and f[1] == 0x05 and f[2] == 0xFF and f[3] == 0x03
    assert f[4] == 7 and f[5] == 0
    assert f[6:13] == payload
    # SC = sum(AB..payload) & 0xFF；AC = 累加和
    assert f[13] == sum(f[:13]) & 0xFF
    assert f[14] == anocom.add_check(f[:13])


def test_find_frames_valid_and_corrupt():
    """帧切分：有效帧校验通过；单字节损坏校验失败；垃圾前缀可同步"""
    payload = bytes([0x10, 0x27, 0xD0, 0xFF, 0x00, 0x00, 0x01])
    f = anocom.encode_frame(anocom.FUNC_ATTITUDE_EULER, payload)
    # 前缀垃圾 + 完整帧 + 尾部垃圾
    stream = b'\x00\xFF\x12' + f + b'\xAB\x05'
    frames = anocom.find_frames(stream)
    assert len(frames) == 1
    assert frames[0].valid and frames[0].func == anocom.FUNC_ATTITUDE_EULER

    # 损坏
    bad = bytearray(f)
    bad[8] ^= 0x01
    frames = anocom.find_frames(bytes(bad))
    assert len(frames) == 1 and not frames[0].valid


def test_decode_euler_scale():
    """0x03 姿态欧拉：int16×100 缩放 + fusion bit7"""
    payload = struct.pack('<3hB', 1250, -300, 9000, 0x81)
    f = anocom.encode_frame(anocom.FUNC_ATTITUDE_EULER, payload)
    dec = anocom.decode_frame(anocom.find_frames(f)[0])
    assert dec['roll_deg'] == 12.5
    assert dec['pitch_deg'] == -3.0
    assert dec['heading_deg'] == 90.0
    assert dec['fusion_sta'] == 0x81
    assert dec['deta100_online'] is True


def test_decode_imu_gyro_scale():
    """0x01 IMU：acc ÷100（cm/s²→m/s²），gyr ÷16.384（LSB→deg/s）"""
    payload = struct.pack('<3h', 981, -98, 1000) + struct.pack('<3h', 1638, -1638, 32767) + bytes([0])
    f = anocom.encode_frame(anocom.FUNC_IMU_DATA, payload)
    dec = anocom.decode_frame(anocom.find_frames(f)[0])
    assert dec['acc_x_ms2'] == 9.81
    assert dec['acc_y_ms2'] == -0.98
    assert dec['gyro_x_dps'] == 1638 / 16.384
    assert abs(dec['gyro_z_dps'] - 32767 / 16.384) < 1e-6   # 32767 为固件限幅上界


def test_decode_volt_curr_pressure_occupation():
    """0x0D：fc_voltage/fc_current 承载氧压 P1/P2（本工程占用约定）"""
    payload = struct.pack('<5H', 1260, 1680, 1520, 1480, 0)
    f = anocom.encode_frame(anocom.FUNC_VOLT_CURR, payload)
    dec = anocom.decode_frame(anocom.find_frames(f)[0])
    assert dec['bat_voltage_v'] == 12.6
    assert dec['p1_mpa'] == 15.2 and dec['p2_mpa'] == 14.8


def test_decode_speed_upward_positive():
    """0x07：固件取反后向下速度为向上为正"""
    payload = struct.pack('<3h', 100, -50, 250)
    f = anocom.encode_frame(anocom.FUNC_FLIGHT_SPEED, payload)
    dec = anocom.decode_frame(anocom.find_frames(f)[0])
    assert dec['vel_n_ms'] == 1.0
    assert dec['vel_e_ms'] == -0.5
    assert dec['vel_up_ms'] == 2.5


def test_decode_gps_scale():
    """0x30：经纬度 ÷1e7、高度 ÷100、精度 ÷10
    手册字节序 p[20]=PDOP p[21]=SACC p[22]=VACC（固件调用处 sacc/vacc 双交换后字节流恰与手册一致）"""
    payload = bytes([3, 12]) + struct.pack('<2i', 1164073250, 399048230) + \
        struct.pack('<i', 4500) + struct.pack('<3h', 100, 200, 300) + bytes([15, 20, 25])
    f = anocom.encode_frame(anocom.FUNC_GPS_INFO1, payload)
    dec = anocom.decode_frame(anocom.find_frames(f)[0])
    assert dec['gps_fix'] == 3 and dec['gps_sats'] == 12
    assert abs(dec['gps_lon'] - 116.407325) < 1e-9
    assert abs(dec['gps_lat'] - 39.904823) < 1e-9
    assert dec['gps_alt_m'] == 45.0
    assert dec['gps_pdop'] == 1.5 and dec['gps_sacc'] == 2.0 and dec['gps_vacc'] == 2.5


def test_decode_pwm_rc_raw():
    """0x20：8×u16 raw_rc_values"""
    payload = struct.pack('<8H', *range(1000, 1008))
    f = anocom.encode_frame(anocom.FUNC_PWM_OUTPUT, payload)
    dec = anocom.decode_frame(anocom.find_frames(f)[0])
    assert dec['rc1'] == 1000 and dec['rc8'] == 1007


def test_decode_attitude_control():
    """0x21：4×int16 ÷10（alpha_ref×10 / 油门%×10）"""
    payload = struct.pack('<4h', 100, -200, 500, 30)
    f = anocom.encode_frame(anocom.FUNC_ATTITUDE_CONTROL, payload)
    dec = anocom.decode_frame(anocom.find_frames(f)[0])
    assert dec['ctrl_roll'] == 10.0
    assert dec['ctrl_pitch'] == -20.0
    assert dec['ctrl_thr_pct'] == 50.0
    assert dec['ctrl_yaw'] == 3.0


def test_param_command_frames():
    """0xE0/0xE1 命令帧构造（与固件 ano_params.cpp 解析逻辑匹配）"""
    # 读参数个数：CMD=1
    f = anocom.cmd_read_param_count()
    assert f[3] == 0xE0 and f[6] == 0x01
    # 读参数值：CMD=2 + ID u16 LE
    f = anocom.cmd_read_param_value(0x2A)
    assert f[3] == 0xE0 and f[6] == 0x02 and f[7] == 0x2A and f[8] == 0x00
    # 写 float 参数：ID u16 + float LE
    f = anocom.cmd_write_param(5, 2.5)
    assert f[3] == 0xE1 and f[6:8] == b'\x05\x00'
    assert struct.unpack('<f', f[8:12])[0] == 2.5
    # 写 uint8 参数（enabled）
    f = anocom.cmd_write_param(8, True, is_float=False)
    assert f[3] == 0xE1 and f[8] == 1
    # 恢复默认
    f = anocom.cmd_param_restore_defaults()
    assert f[6] == 0x10 and f[7] == 0xAA


def test_param_value_parse():
    """0xE1 回传值解析"""
    payload = struct.pack('<Hf', 3, -1.25)
    assert anocom.parse_param_value(payload, True) == -1.25
    assert anocom.parse_param_value(b'\x00\x00\x01', False) == 1


def test_decode_device_info():
    """0xE3 设备信息"""
    payload = bytes([0x05]) + struct.pack('<4h', 1, 5, 0, 1) + b'VTVL_DualRotor_FCS' + b'\x00' * 8
    dec = anocom.decode_device_info(payload)
    assert dec['dev_id'] == 0x05 and dec['sw_ver'] == 5
    assert dec['dev_name'] == 'VTVL_DualRotor_FCS'
