#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    led_probe.py
@brief   台架验收：WS2812 自动灯效与限时调试命令的实测探针
@module  工具链 / 实机调试
@details 单个串口会话内按时序发送灯效命令并读回 wsstat，验证：
          ws/wsoff 限时 3 s、wsseq 参数生效、wsstatic 2 s 自动恢复、
          wsfault 3 s 恢复、ws_tx 持续增长且 ws_fail 不增。
@theory  N/A（外设验收工具）
@frame   N/A
@units   时间 s
@note    只读 + 调试灯效，不触发解锁与动力。
@test    2026-11-26 荣耀本 COM12（CoaxialVTVL_ICM45686_Dbg）实测
@see     TandemVec_FCS/src/communication.cpp（handleDebugConsole 的 ws 系列）
"""
import re
import sys
import time

import serial

BAUD = 921600


def read_for(ser, seconds):
    """在给定时长内读空串口并返回文本。"""
    buf = bytearray()
    end = time.time() + seconds
    while time.time() < end:
        chunk = ser.read(4096)
        if chunk:
            buf += chunk
    return buf.decode('utf-8', 'replace')


def cmd(ser, text, wait=0.7):
    """发送一条命令并回收输出，过滤调试打印噪声。"""
    ser.write((text + '\n').encode('utf-8'))
    raw = read_for(ser, wait)
    lines = [ln.strip() for ln in raw.replace('\r', '').split('\n')
             if ln.strip().startswith('[DBG]')]
    print('$ %s' % text)
    for ln in lines:
        print('  %s' % ln)
    return raw


def wsstat(ser, label):
    """读一次 wsstat 并解析为字典。"""
    raw = cmd(ser, 'wsstat')
    got = {}
    for line in raw.replace('\r', '').split('\n'):
        line = line.strip()
        for key in ('ws_tx', 'ws_fail', 'ws_test', 'ws_frame', 'ws_rgb',
                    'ws_cells', 'ws_primary', 'ws_hint', 'driver'):
            m = re.search(key + r'=(\S+)', line)
            if m:
                got[key] = m.group(1)
    print('  [%s] %s' % (label, got))
    return got


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM12'
    ser = serial.Serial(port, BAUD, timeout=0.2)
    failures = []
    try:
        read_for(ser, 0.5)
        for _ in range(4):                 # 启动期打印可能吞掉首条 DBG
            ser.write(b'DBG\n')
            time.sleep(0.6)
            if '[DBG]' in read_for(ser, 0.8):
                print('entered DBG on %s' % port)
                break
        else:
            print('FAIL: 未进入 DBG')
            return 1

        base = wsstat(ser, '基线')
        if base.get('ws_test') != '0':
            failures.append('基线不在自动灯效')
        if base.get('ws_frame') != '1':
            failures.append('基线帧无效')

        # 1) ws 限时 3 s
        cmd(ser, 'ws 0 255 0')
        s1 = wsstat(ser, 'ws 后立即')
        if s1.get('ws_rgb') != '0x00FF00' or s1.get('ws_test') != '2':
            failures.append('ws 未生效或颜色不符: %s' % s1)
        time.sleep(3.4)
        s2 = wsstat(ser, 'ws 后 3.4 s')
        if s2.get('ws_test') != '0':
            failures.append('ws 未在 3 s 后恢复自动')

        # 2) wsseq 参数是否真的生效（旧实现恒 500 ms）
        cmd(ser, 'wsseq 100', wait=0.25)
        s3 = wsstat(ser, 'wsseq 100 后立即')
        if s3.get('ws_test') != '3':
            failures.append('wsseq 未进入序列: %s' % s3)
        time.sleep(0.9)                    # 5 段 x100 ms = 500 ms
        s4 = wsstat(ser, 'wsseq 后 0.9 s')
        if s4.get('ws_test') != '0':
            failures.append('wsseq 未按 100 ms 段结束')

        # 3) wsstatic 2 s 后自动恢复驱动
        cmd(ser, 'wsstatic 1', wait=0.6)
        s5 = wsstat(ser, 'wsstatic 后立即')
        if s5.get('ws_test') != '4':
            failures.append('wsstatic 未进入静态电平: %s' % s5)
        time.sleep(2.6)
        s6 = wsstat(ser, 'wsstatic 后 2.6 s')
        if s6.get('ws_test') != '0':
            failures.append('wsstatic 未自动结束')
        if s6.get('driver') != '1':
            failures.append('wsstatic 后驱动未恢复到 DMA: %s' % s6)
        if s6.get('ws_frame') != '1':
            failures.append('恢复后帧未重新提交: %s' % s6)

        # 4) wsfault 3 s 后恢复
        cmd(ser, 'wsfault 1')
        s7 = wsstat(ser, 'wsfault 1 后立即')
        if s7.get('ws_test') != '5':
            failures.append('wsfault 未进入故障测试: %s' % s7)
        time.sleep(3.4)
        s8 = wsstat(ser, 'wsfault 后 3.4 s')
        if s8.get('ws_test') != '0' or s8.get('ws_primary') != 'NO_RECEIVER':
            failures.append('wsfault 未恢复自动灯效: %s' % s8)

        # 5) 发送计数必须前进、失败不得增加
        try:
            tx_a = int(s8.get('ws_tx', '0'))
            fail_a = int(s8.get('ws_fail', '0'))
            time.sleep(2.0)
            s9 = wsstat(ser, '2 s 后')
            tx_b = int(s9.get('ws_tx', '0'))
            fail_b = int(s9.get('ws_fail', '0'))
            print('  [计数] ws_tx %d -> %d (+%d)，ws_fail %d -> %d'
                  % (tx_a, tx_b, tx_b - tx_a, fail_a, fail_b))
            if tx_b <= tx_a:
                failures.append('ws_tx 在 2 s 内未增长')
            if fail_b != fail_a:
                failures.append('ws_fail 增加 %d 次' % (fail_b - fail_a))
        except ValueError:
            failures.append('ws_tx/ws_fail 无法解析')

        ser.write(b'exit\n')
        time.sleep(0.4)
        read_for(ser, 0.4)
    finally:
        ser.close()

    if failures:
        print('\n===== 台架灯效验收 FAILED =====')
        for f in failures:
            print(' - %s' % f)
        return 1
    print('\n===== 台架灯效验收 ALL PASS =====')
    return 0


if __name__ == '__main__':
    sys.exit(main())