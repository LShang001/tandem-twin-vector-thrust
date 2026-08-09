# -*- coding: utf-8 -*-
"""
anocom.py — AnoCom（匿名地面站协议）编解码

与固件 lib/AnoComProtocol 逐字节对齐：
  - 帧格式：AB | src(0x05) | dst(0xFF) | func | len_lo len_hi(LE) | data[len] | SC | AC
  - SC = 逐字节和 & 0xFF；AC = 累加和 & 0xFF（sum += b; add += sum）
  - 遥测字段缩放与 src/communication.cpp 发送实现一致（见各解码函数注释）
"""
import struct
from dataclasses import dataclass

FRAME_HEAD = 0xAB
LOCAL_ADDR = 0x05   # 飞控
GND_ADDR = 0xFF     # 地面站

# ---- 功能码 ----
FUNC_DATA_CHECK = 0x00
FUNC_IMU_DATA = 0x01
FUNC_MAG_PRESS_TEMP = 0x02
FUNC_ATTITUDE_EULER = 0x03
FUNC_ATTITUDE_QUAT = 0x04
FUNC_ALTITUDE_DATA = 0x05
FUNC_FLIGHT_MODE = 0x06
FUNC_FLIGHT_SPEED = 0x07
FUNC_POS_OFFSET = 0x08
FUNC_WIND_EST = 0x09
FUNC_TARGET_ATTITUDE = 0x0A
FUNC_TARGET_SPEED = 0x0B
FUNC_RETURN_INFO = 0x0C
FUNC_VOLT_CURR = 0x0D
FUNC_EXT_MODULE = 0x0E
FUNC_PWM_OUTPUT = 0x20
FUNC_ATTITUDE_CONTROL = 0x21
FUNC_GPS_INFO1 = 0x30
FUNC_RC_DATA = 0x40            # 遥控器数据（手册定义：10×int16 us）
FUNC_ACTUATOR_OUT = 0xF1       # 本工程自定义（手册灵活格式帧）：执行器输出（TVC 摆角/电机推力/差速）
FUNC_VARS_VALUE = 0xF2         # 本工程自定义（手册灵活格式帧）：通用变量值帧 [id u16 LE] + [float LE]
FUNC_VARS_LIST = 0xF3          # 本工程自定义（手册灵活格式帧）：变量清单请求/应答（仿 0xE0/0xE2）
FUNC_PARAM_CMD = 0xE0
FUNC_PARAM_WRITE_READ = 0xE1
FUNC_PARAM_INFO = 0xE2
FUNC_DEVICE_INFO = 0xE3

FUNC_NAMES = {
    0x00: '数据校验', 0x01: 'IMU数据', 0x02: '磁力/气压/温度', 0x03: '姿态欧拉',
    0x04: '姿态四元数', 0x05: '高度数据', 0x06: '飞行模式', 0x07: '飞行速度',
    0x08: '位置偏移', 0x09: '风速估计', 0x0A: '目标姿态', 0x0B: '目标速度',
    0x0C: '回航信息', 0x0D: '电压电流', 0x0E: '外接模块状态', 0x20: 'PWM输出',
    0x21: '姿态控制输出', 0x30: 'GPS信息1', 0x40: '遥控器数据', 0xF1: '执行器输出',
    0xF2: '变量值', 0xF3: '变量清单', 0xE0: '参数命令', 0xE1: '参数读写',
    0xE2: '参数信息', 0xE3: '设备信息',
}

# 参数类型（与固件 AnoDataType 枚举一致，E2 信息帧 PAR_TYPE）
ANO_UINT8 = 0
ANO_FLOAT = 8


@dataclass
class Frame:
    func: int
    payload: bytes
    sc: int
    ac: int
    valid: bool


def sum_check(data: bytes) -> int:
    """和校验 SC（与固件 calculateSumCheck 一致）"""
    s = 0
    for b in data:
        s = (s + b) & 0xFF
    return s


def add_check(data: bytes) -> int:
    """附加校验 AC（与固件 calculateAddCheck 一致：add += sum）"""
    s = 0
    a = 0
    for b in data:
        s = (s + b) & 0xFF
        a = (a + s) & 0xFF
    return a


def encode_frame(func: int, payload: bytes, dst: int = GND_ADDR) -> bytes:
    """构造完整发送帧（模拟固件 sendData）"""
    head = bytes([FRAME_HEAD, LOCAL_ADDR, dst, func,
                  len(payload) & 0xFF, (len(payload) >> 8) & 0xFF])
    body = head + payload
    return body + bytes([sum_check(body), add_check(body)])


def extract_frames(buf: bytes):
    """逐字节扫描 AB 05 FF 帧头切帧；返回 (frames, consumed)。

    consumed = 最后一个完整帧的结束偏移。调用方 del buf[:consumed] 即可
    保留尾部半帧——流式场景下避免整体清空缓冲导致半帧丢失，也避免
    缓冲无限膨胀后反复全量扫描的 O(n²) 退化（错波特率/DBG 文本流场景
    曾致上位机卡死）。帧头匹配但长度不足（半帧）时停在原地等下一块。"""
    frames = []
    i = 0
    n = len(buf)
    consumed = 0
    while i + 8 <= n:  # 最小帧 = 6B 头 + 0 数据 + 2B 校验
        if buf[i] == FRAME_HEAD and buf[i + 1] == LOCAL_ADDR and buf[i + 2] == GND_ADDR:
            func = buf[i + 3]
            length = buf[i + 4] | (buf[i + 5] << 8)
            end = i + 6 + length + 2
            if end <= n:
                payload = buf[i + 6:i + 6 + length]
                sc, ac = buf[end - 2], buf[end - 1]
                body = buf[i:i + 6 + length]
                valid = (sum_check(body) == sc and add_check(body) == ac)
                frames.append(Frame(func, payload, sc, ac, valid))
                i = end
                consumed = end
                continue
            break  # 半帧：保留在缓冲尾部，等后续字节
        i += 1
    return frames, consumed


def find_frames(buf: bytes):
    """兼容包装：逐字节扫描 AB 05 FF 帧头切帧；返回 Frame 列表（校验不过的标 valid=False）"""
    return extract_frames(buf)[0]


# ========================================================================
#  遥测帧解码 — 缩放与 src/communication.cpp 发送实现逐字节对齐
# ========================================================================

def decode_imu(p: bytes) -> dict:
    """0x01: acc 3×int16 (÷100 → m/s²), gyr 3×int16 (÷16.384 → deg/s), shock u8"""
    if len(p) < 13:
        return {}
    ax, ay, az = struct.unpack('<3h', p[0:6])
    gx, gy, gz = struct.unpack('<3h', p[6:12])
    return {
        'acc_x_ms2': ax / 100.0, 'acc_y_ms2': ay / 100.0, 'acc_z_ms2': az / 100.0,
        'gyro_x_dps': gx / 16.384, 'gyro_y_dps': gy / 16.384, 'gyro_z_dps': gz / 16.384,
        'shock': p[12],
    }


def decode_euler(p: bytes) -> dict:
    """0x03: roll/pitch/yaw 3×int16 (÷100 → deg), fusion u8（bit7=DETA100, bit6=数据源）"""
    if len(p) < 7:
        return {}
    r, p_, y = struct.unpack('<3h', p[0:6])
    fusion = p[6]
    return {
        'roll_deg': r / 100.0, 'pitch_deg': p_ / 100.0, 'heading_deg': y / 100.0,
        'fusion_sta': fusion,
        'deta100_online': bool(fusion & 0x80),
    }


def decode_altitude(p: bytes) -> dict:
    """0x05: 3×int32 (÷100 → m) alt_bar/alt_add/alt_fu, altSta u8"""
    if len(p) < 13:
        return {}
    ab, aa, af = struct.unpack('<3i', p[0:12])
    return {'alt_bar_m': ab / 100.0, 'alt_add_m': aa / 100.0,
            'alt_fu_m': af / 100.0, 'alt_sta': p[12]}


def decode_flight_mode(p: bytes) -> dict:
    """0x06: mode u8, sFlag u8, cId/cmd0/cmd1"""
    if len(p) < 5:
        return {}
    return {'flight_mode': p[0], 'unlocked': bool(p[1]),
            'mode_cid': p[2], 'mode_cmd0': p[3], 'mode_cmd1': p[4]}


def decode_speed(p: bytes) -> dict:
    """0x07: 3×int16 (÷100 → m/s)；固件已把向下速度取反（向上为正）"""
    if len(p) < 6:
        return {}
    vn, ve, vd = struct.unpack('<3h', p[0:6])
    return {'vel_n_ms': vn / 100.0, 'vel_e_ms': ve / 100.0, 'vel_up_ms': vd / 100.0}


def decode_pos(p: bytes) -> dict:
    """0x08: 3×int32 (÷100 → m)；固件已把向下取反（rel_d 为高度向上）"""
    if len(p) < 12:
        return {}
    pn, pe, pd = struct.unpack('<3i', p[0:12])
    return {'rel_n_m': pn / 100.0, 'rel_e_m': pe / 100.0, 'rel_h_m': pd / 100.0}


def decode_target_attitude(p: bytes) -> dict:
    """0x0A: 3×int16 (÷100)；roll/pitch 目标角 deg，yaw 目标角速率 deg/s"""
    if len(p) < 6:
        return {}
    tr, tp, ty = struct.unpack('<3h', p[0:6])
    return {'target_roll_deg': tr / 100.0, 'target_pitch_deg': tp / 100.0,
            'target_yaw_rate_dps': ty / 100.0}


def decode_target_speed(p: bytes) -> dict:
    """0x0B: 3×int16 (÷100 → m/s)"""
    if len(p) < 6:
        return {}
    tx, ty, tz = struct.unpack('<3h', p[0:6])
    return {'target_vx_ms': tx / 100.0, 'target_vy_ms': ty / 100.0,
            'target_vz_ms': tz / 100.0}


def decode_volt_curr(p: bytes) -> dict:
    """0x0D: 4×u16 (÷100) + power_state u16
    ★ 本工程占用约定（communication.cpp）：fc_voltage=P1氧压, fc_current=P2氧压；
      bat_voltage/bat_current 为占位常量 12.6/16.8（无真实采样）"""
    if len(p) < 10:
        return {}
    bv, bc, fv, fc, ps = struct.unpack('<5H', p[0:10])
    return {'bat_voltage_v': bv / 100.0, 'bat_current_a': bc / 100.0,
            'p1_mpa': fv / 100.0, 'p2_mpa': fc / 100.0, 'power_state': ps}


def decode_pwm(p: bytes) -> dict:
    """0x20: 8×u16 PWM 控制量（0.01% 油门，手册语义）——固件 2026-08-10 起传
    ch3_output/ch4_output 电机输出 %×100 → 0-10000，其余通道 0"""
    if len(p) < 16:
        return {}
    vals = struct.unpack('<8H', p[0:16])
    return {f'motor_pwm{i + 1}': int(v) for i, v in enumerate(vals)}


def decode_rc(p: bytes) -> dict:
    """0x40: 10×int16 遥控器数据（手册：THR/YAW/ROL/PIT/AUX1-6，1000-2000 us，0=无通信）
    ★ 2026-08-10 数据归位：0x40 恢复手册遥控帧，RC 通道显示数据源（原 0x20 承载）；
    字段名 rc1-10 保持与旧 decode_pwm 输出一致，前端 RC 条零改动"""
    if len(p) < 20:
        return {}
    vals = struct.unpack('<10h', p[0:20])
    return {f'rc{i + 1}': int(v) for i, v in enumerate(vals)}


def decode_attitude_control(p: bytes) -> dict:
    """0x21: 4×int16 (÷10)；roll/pitch/yaw = alpha_ref×10（rad/s²），thr = 油门%×10"""
    if len(p) < 8:
        return {}
    r, p_, t, y = struct.unpack('<4h', p[0:8])
    return {'ctrl_roll': r / 10.0, 'ctrl_pitch': p_ / 10.0,
            'ctrl_thr_pct': t / 10.0, 'ctrl_yaw': y / 10.0}


def decode_actuator(p: bytes) -> dict:
    """0xF1（本工程自定义，手册灵活格式帧）: 前/下摆角 int16 (÷100 → deg)，前/尾电机 u16 (÷10 → %)，
    差速 int16 (÷1000，归一化 Δω)，饱和标记 u8（bit0 δf / bit1 δt / bit2 Δω），预留 u8
    —— mix 输出级统一捕获，锁定/手动/自动全模式有效。2026-08-10 由 0x40 迁入"""
    if len(p) < 11:
        return {}
    tf, tr, dw = struct.unpack('<3h', p[0:2] + p[2:4] + p[8:10])
    m1, m2 = struct.unpack('<2H', p[4:8])
    sat = p[10]
    return {
        'tvc_upper_deg': tf / 100.0, 'tvc_lower_deg': tr / 100.0,
        'motor_front_pct': m1 / 10.0, 'motor_rear_pct': m2 / 10.0,
        'dw': dw / 1000.0,
        'sat_df': bool(sat & 0x01), 'sat_dt': bool(sat & 0x02), 'sat_dw': bool(sat & 0x04),
    }


def decode_gps(p: bytes) -> dict:
    """0x30: fix u8, sats u8, lng/lat int32 (÷1e7), alt int32 (÷100),
    n/e/d 速度 int16 (÷100), pdop/sacc/vacc u8 (÷10)
    ★ 字节位置按手册：p[21]=SACC(速度精度)、p[22]=VACC(垂直精度)。
    固件 sendGPSInfo1 的形参顺序为 (pdop, vacc, sacc) 且调用处实参 (sacc, vacc)
    双交换——字节流恰好与手册一致，仅两侧变量命名与实际内容相反"""
    if len(p) < 23:
        return {}
    fix, sats = p[0], p[1]
    lng, lat = struct.unpack('<2i', p[2:10])
    alt = struct.unpack('<i', p[10:14])[0]
    vn, ve, vd = struct.unpack('<3h', p[14:20])
    pdop, sacc, vacc = p[20], p[21], p[22]
    return {'gps_fix': fix, 'gps_sats': sats, 'gps_lon': lng / 1e7, 'gps_lat': lat / 1e7,
            'gps_alt_m': alt / 100.0, 'gps_vel_n': vn / 100.0, 'gps_vel_e': ve / 100.0,
            'gps_vel_d': vd / 100.0, 'gps_pdop': pdop / 10.0, 'gps_sacc': sacc / 10.0,
            'gps_vacc': vacc / 10.0}


def decode_device_info(p: bytes) -> dict:
    """0xE3: dev_id u8, hw/sw/bl/pt 4×int16, name 20B"""
    if len(p) < 29:
        return {}
    dev_id, hw, sw, bl, pt = p[0], *struct.unpack('<4h', p[1:9])
    name = p[9:29].split(b'\x00')[0].decode('ascii', errors='replace')
    return {'dev_id': dev_id, 'hw_ver': hw, 'sw_ver': sw, 'bl_ver': bl,
            'pt_ver': pt, 'dev_name': name}


DECODERS = {
    FUNC_IMU_DATA: decode_imu,
    FUNC_ATTITUDE_EULER: decode_euler,
    FUNC_ALTITUDE_DATA: decode_altitude,
    FUNC_FLIGHT_MODE: decode_flight_mode,
    FUNC_FLIGHT_SPEED: decode_speed,
    FUNC_POS_OFFSET: decode_pos,
    FUNC_TARGET_ATTITUDE: decode_target_attitude,
    FUNC_TARGET_SPEED: decode_target_speed,
    FUNC_VOLT_CURR: decode_volt_curr,
    FUNC_PWM_OUTPUT: decode_pwm,
    FUNC_ATTITUDE_CONTROL: decode_attitude_control,
    FUNC_RC_DATA: decode_rc,
    FUNC_ACTUATOR_OUT: decode_actuator,
    FUNC_GPS_INFO1: decode_gps,
    FUNC_DEVICE_INFO: decode_device_info,
}


def decode_frame(f: Frame) -> dict:
    """按功能码解码有效帧；未知/无效帧返回 {}"""
    if not f.valid:
        return {}
    dec = DECODERS.get(f.func)
    return dec(f.payload) if dec else {}


# ========================================================================
#  上行命令帧构造（地面站 → 飞控）
# ========================================================================

def cmd_read_param_count() -> bytes:
    """0xE0 CMD 0x01 读参数个数（手册：CMD u8 + VAL u16，VAL 无意义置 0）"""
    return encode_frame(FUNC_PARAM_CMD, bytes([0x01, 0x00, 0x00]))


def cmd_read_param_value(param_id: int) -> bytes:
    """0xE0 CMD 0x02 读参数值（VAL=ID u16 LE）"""
    return encode_frame(FUNC_PARAM_CMD, bytes([0x02, param_id & 0xFF, (param_id >> 8) & 0xFF]))


def cmd_read_param_info(param_id: int) -> bytes:
    """0xE0 CMD 0x03 读参数信息"""
    return encode_frame(FUNC_PARAM_CMD, bytes([0x03, param_id & 0xFF, (param_id >> 8) & 0xFF]))


def cmd_param_restore_defaults() -> bytes:
    """0xE0 CMD 0x10 VAL=0xAA 恢复默认（手册：VAL 为 U16，0xAA 低字节）"""
    return encode_frame(FUNC_PARAM_CMD, bytes([0x10, 0xAA, 0x00]))


def cmd_param_save() -> bytes:
    """0xE0 CMD 0x10 VAL=0xAB 保存（固件无持久化，仅确认）"""
    return encode_frame(FUNC_PARAM_CMD, bytes([0x10, 0xAB, 0x00]))


def cmd_vars_count() -> bytes:
    """0xF3 CMD 0x01 读变量个数（AnoVars 清单）"""
    return encode_frame(FUNC_VARS_LIST, bytes([0x01]))


def cmd_vars_info(var_id: int) -> bytes:
    """0xF3 CMD 0x02 读变量信息（id u16 → 回 [id, type u8, name 16B]）"""
    return encode_frame(FUNC_VARS_LIST, bytes([0x02, var_id & 0xFF, (var_id >> 8) & 0xFF]))


def cmd_write_param(param_id: int, value, is_float: bool = True) -> bytes:
    """0xE1 参数写入：ID u16 LE + PAR_VAL（float LE 4B / uint8 1B）"""
    if is_float:
        payload = struct.pack('<Hf', param_id & 0xFFFF, float(value))
    else:
        payload = struct.pack('<HB', param_id & 0xFFFF, 1 if value else 0)
    return encode_frame(FUNC_PARAM_WRITE_READ, payload)


def parse_param_value(payload: bytes, is_float: bool = True):
    """解析 0xE1 回传帧 PAR_VAL（payload 已含 ID u16）"""
    if is_float and len(payload) >= 6:
        return struct.unpack('<f', payload[2:6])[0]
    if not is_float and len(payload) >= 3:
        return payload[2]
    return None
