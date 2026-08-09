# -*- coding: utf-8 -*-
"""MAVLink 双向桥接实测脚本（模拟 QGC 全流程）

用法: py -3.12 tools/test_mavlink_bridge.py [COM口] [--set att_roll.kp 1.1]

流程:
  1. 发 HEARTBEAT（QGC 存在性握手）
  2. PARAM_REQUEST_LIST → 收 121 条 PARAM_VALUE（校验 count/index 连续 + 短名唯一性）
  3. PARAM_REQUEST_READ 按短名读单条（att_roll.kp → att_roll.kp 短名相同）
  4. PARAM_SET 写入（--set 参数）→ 读回确认
  5. COMMAND_LONG 246 重启（--reboot 时执行）
  6. COMMAND_LONG 400 ARM → 期望 UNSUPPORTED（安全拒绝验证）
  7. STATUSTEXT 接收（1Hz 队列轮发）
"""
import sys, time, threading
from collections import Counter

sys.path.insert(0, r'D:\纵列双发矢量推力飞行器\TandemVec_FCS\GCS\server')
import params as pm

from pymavlink import mavutil

PORT = sys.argv[1] if len(sys.argv) > 1 else 'COM10'
BAUD = 2000000

# ---- 与固件 mavlink_bridge.cpp 相同的短名算法（唯一性测试用）----
FIELD_SHORT = {'out_min': 'omin', 'out_max': 'omax', 'int_limit': 'ilim',
               'threshold': 'thr', 'enabled': 'en'}
SPECIAL = {'inertia_comp_mask': 'inertia_mask'}

def mav_name(name: str, limit=15) -> str:
    if name in SPECIAL:
        return SPECIAL[name]
    if '.' in name:
        loop, field = name.split('.', 1)
        field = FIELD_SHORT.get(field, field)
        cand = f'{loop}.{field}'
    else:
        cand = name
    return cand[:limit] if len(cand) > limit else cand


def main():
    print(f'连接 {PORT} @ {BAUD} ...')
    m = mavutil.mavlink_connection(PORT, baud=BAUD)
    m.wait_heartbeat(timeout=5)
    print(f'✓ 心跳收到 (sysid={m.target_system} compid={m.target_component})')

    fail = 0

    # ---- 1. PARAM_REQUEST_LIST：收全 121 条 ----
    m.mav.param_request_list_send(m.target_system, m.target_component)
    got = {}
    deadline = time.time() + 5
    while time.time() < deadline and len(got) < 121:
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if msg:
            got[msg.param_index] = (msg.param_id, msg.param_value)
    print(f'✓ PARAM_REQUEST_LIST: 收到 {len(got)}/121 条')
    if len(got) != 121:
        fail += 1
    else:
        # 连续性 + 短名唯一性
        idxs = sorted(got.keys())
        if idxs != list(range(121)):
            print(f'✗ param_index 不连续: {idxs[:5]}...{idxs[-3:]}')
            fail += 1
        short_names = [got[i][0] for i in idxs]
        if len(set(short_names)) != 121:
            print('✗ 短名有重复!')
            fail += 1
        else:
            print('✓ 121 条连续 + 短名全唯一')

    # ---- 2. 短名算法一致性（固件短名 vs Python 算法）----
    names = list(pm.expected_names())
    exp_short = {i: mav_name(names[i]) for i in range(121)}
    mismatch = [i for i in range(121) if got.get(i, ('',))[0] != exp_short[i]]
    if mismatch:
        print(f'✗ 短名不一致 {len(mismatch)} 个: '
              f'{[(i, exp_short[i], got.get(i, ("?",))[0]) for i in mismatch[:3]]}')
        fail += 1
    else:
        print('✓ 固件短名与 Python 算法 121/121 一致')

    # ---- 3. PARAM_REQUEST_READ 按短名读 ----
    m.mav.param_request_read_send(m.target_system, m.target_component,
                                  b'att_roll.kp', -1)
    msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
    if msg and msg.param_id == b'att_roll.kp':
        print(f'✓ PARAM_REQUEST_READ: att_roll.kp = {msg.param_value:.6f}')
    else:
        print(f'✗ PARAM_REQUEST_READ 失败: {msg}')
        fail += 1

    # ---- 4. PARAM_SET 写入 + 读回 ----
    set_arg = None
    if '--set' in sys.argv:
        i = sys.argv.index('--set')
        set_arg = (sys.argv[i + 1], float(sys.argv[i + 2]))
    if set_arg:
        name, val = set_arg
        short = mav_name(name)
        m.mav.param_set_send(m.target_system, m.target_component,
                             short.encode(), val, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if msg:
            print(f'✓ PARAM_SET {short} = {val} → 读回 {msg.param_value:.6f} '
                  f'({"一致" if abs(msg.param_value - val) < 1e-6 else "✗ 不一致"})')
            if abs(msg.param_value - val) >= 1e-6:
                fail += 1
        else:
            print('✗ PARAM_SET 无读回确认')
            fail += 1

    # ---- 5. COMMAND_LONG 400 ARM → 期望 UNSUPPORTED ----
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                            1, 0, 0, 0, 0, 0, 0)
    msg = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
    if msg and msg.result == mavutil.mavlink.MAV_RESULT_UNSUPPORTED:
        print('✓ COMMAND_LONG 400 ARM → UNSUPPORTED（安全拒绝正确）')
    else:
        print(f'✗ ARM 命令响应异常: {msg}')
        fail += 1

    # ---- 6. STATUSTEXT 接收（等 1Hz 队列轮发）----
    got_text = []
    deadline = time.time() + 4
    while time.time() < deadline and not got_text:
        msg = m.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
        if msg:
            got_text.append(msg.text)
    if got_text:
        print(f'✓ STATUSTEXT: {got_text[0]}')
    else:
        print('⚠ STATUSTEXT 4s 内未收到（队列空或 mavlink 模式外——非致命）')

    # ---- 7. COMMAND_LONG 246 重启（--reboot 时）----
    if '--reboot' in sys.argv:
        print('发送重启命令...')
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
                                1, 0, 0, 0, 0, 0, 0)
        msg = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
        print(f'  重启 ACK: {msg.result if msg else "超时"}（固件 300ms 后软复位）')
        time.sleep(3)
        # 复位后心跳应恢复
        try:
            m.wait_heartbeat(timeout=5)
            print('✓ 重启后心跳恢复')
        except Exception:
            print('✗ 重启后心跳未恢复')
            fail += 1

    print('=' * 50)
    print('FAILED' if fail else 'ALL PASSED', f'({fail} 失败)')
    m.close()
    sys.exit(1 if fail else 0)


if __name__ == '__main__':
    main()
