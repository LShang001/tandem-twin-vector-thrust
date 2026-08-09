#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cli.py — GCS 命令行调试接口（复用 server.main 处理链路，行为与 Web 后端一致）

用法（全部命令在 TandemVec_FCS/GCS 目录执行；每条命令独立连接，加 --port 自动连/断）：
  py -3.12 server/cli.py ports                          # 枚举串口
  py -3.12 server/cli.py --port COM10 telemetry -n 5    # 自动连接并收集 5 个遥测快照
  py -3.12 server/cli.py --port COM10 device            # 设备/固件版本探测（0xE3 + 参数个数）
  py -3.12 server/cli.py --port COM10 param list        # 读取全部参数（表格）
  py -3.12 server/cli.py --port COM10 param get 0       # 读单个参数（ID 或名称 att_roll.kp）
  py -3.12 server/cli.py --port COM10 param set att_roll.kp 2.6   # 写参数（等 0x00 校验帧）
  py -3.12 server/cli.py --port COM10 param restore     # 恢复出厂默认
  py -3.12 server/cli.py --port COM10 dbg on            # 进入调试模式（遥测暂停）
  py -3.12 server/cli.py --port COM10 dbg "flash stat"  # 发任意 DBG 命令
  py -3.12 server/cli.py --port COM10 flash findseg     # 飞行段列表
  py -3.12 server/cli.py --port COM10 flash export --latest -o seg.csv   # 最新段导出解析
  py -3.12 server/cli.py --port COM10 flash export --start 8 --count 32 -o seg.csv
  py -3.12 server/cli.py --port COM10 record start -o rec.csv   # 遥测记录（Ctrl+C 停）
  py -3.12 server/cli.py --port COM10 sniff -t 3        # 帧监控（主题: rc/att/axes/euler/raw/all）
  py -3.12 server/cli.py --port COM10 stick -t 10       # 打杆诊断（模式/8通道/控制输出实时）
  py -3.12 server/cli.py baudscan --port COM10          # 波特率扫描（帧头 0x00 间隔规律性评分）
  py -3.12 server/cli.py --port COM10 link            # 链路健康检查（2M 间歇丢帧诊断）
  py -3.12 server/cli.py --port COM10 dbg "ws 255 0 0"  # DBG 任意命令（ws/wsstat/wsstatic/reset/tasks/...）
  py -3.12 server/cli.py --port COM10 raw --hex 414205ff03070000   # 发送原始帧
  # 两步式（先 connect 保持会话——注意 CLI 进程退出即断开，建议直接用 --port）
"""
import argparse
import os
import queue
import struct
import sys
import time
from collections import Counter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import main as m          # 复用 GCS 全局 + 遥测/参数/DBG 处理链路
import anocom
import blackbox as bb
import params as pm
from datalog import CsvRecorder

DEFAULT_BAUD = 2000000


# ========================================================================
#  串口队列消费（同步版 rx_consumer，行为与 WS 后端一致）
# ========================================================================
def drain(duration=0.5):
    end = time.time() + duration
    while time.time() < end:
        try:
            kind, payload = m.g.rx_q.get(timeout=0.05)
        except queue.Empty:
            continue
        if kind == 'rx':
            m._on_telemetry_rx(payload)


def require_connected():
    if not (m.g.link and m.g.link.is_open):
        print('✗ 未连接。先执行: cli.py connect <COM口> [--baud N]')
        sys.exit(1)


# ========================================================================
#  命令实现
# ========================================================================
def cmd_ports(_):
    from serial_link import list_ports
    for p in list_ports():
        print(f'{p["port"]:<6} {p["desc"]}  [{p["hwid"]}]')


# ========================================================================
#  波特率扫描（原 tools/scan_baud.py + probe_2m.py + probe_com10.py）
#  原理：AnoCom 遥测帧头 0x00 在正确波特率下应显著成簇——统计 0x00
#  间隔众数占比作为规律性评分；错误波特率下字节流近似噪声，0x00 间隔分散。
#  注意：本命令自开串口（逐波特率重开），不依赖 --port 的预连接，
#  main() 中 auto_conn 对 baudscan 跳过。
# ========================================================================
def cmd_baudscan(a):
    import serial as _serial
    if not a.port:
        print('✗ baudscan 需要串口: cli.py baudscan --port COM10 [--dur 2] [--bauds 9600,...,2000000]')
        sys.exit(1)
    bauds = [int(b) for b in a.bauds.split(',')]
    results = []
    for baud in bauds:
        try:
            s = _serial.Serial(a.port, baud, timeout=0.3)
        except Exception as e:
            print(f'{baud:>8}: 打开失败 {e}')
            continue
        time.sleep(0.5)
        s.reset_input_buffer()
        data = b''
        t0 = time.time()
        while time.time() - t0 < a.dur:
            chunk = s.read(8192)
            if chunk:
                data += chunk
        s.close()
        if len(data) < 50:
            print(f'{baud:>8}: {len(data)} 字节（几乎无数据）')
            continue
        pos0 = [i for i, b in enumerate(data) if b == 0x00]
        gaps = [pos0[i + 1] - pos0[i] for i in range(len(pos0) - 1)] if len(pos0) > 1 else []
        mode = Counter(gaps).most_common(3) if gaps else []
        score = mode[0][1] / max(len(gaps), 1) if mode else 0
        results.append((score, baud, len(data), mode))
        print(f'{baud:>8}: {len(data)} 字节, 0x00×{len(pos0)}, 间隔众数 {mode[:2]}, 规律性 {score:.2f}')
    results.sort(reverse=True)
    if results:
        print(f'\n最可能波特率: {results[0][1]}（规律性 {results[0][0]:.2f}）')


def cmd_connect(a):
    m.g.connect(a.port, a.baud)
    print(f'✓ 已连接 {a.port} @ {a.baud}')
    # 自动拉取设备信息
    m.g.write(anocom.encode_frame(anocom.FUNC_PARAM_CMD, bytes([0x00, 0, 0, 0, 0])))
    drain(1.0)


def cmd_telemetry(a):
    require_connected()
    print(f'收集 {a.n} 个遥测快照（间隔 {a.i}s）…')
    last = 0
    for i in range(a.n):
        now = time.time()
        if now - last < a.i:
            time.sleep(a.i - (now - last))
        last = time.time()
        drain(0.15)
        s = m.g.snap
        if not s:
            print(f'[{i + 1}] （尚无遥测）')
            continue
        print(f'[{i + 1}] R={s.get("roll_deg", "-"):>6} P={s.get("pitch_deg", "-"):>6} '
              f'H={s.get("heading_deg", "-"):>7} | Vn={s.get("vel_n_ms", "-"):>5} '
              f'Ve={s.get("vel_e_ms", "-"):>5} Vu={s.get("vel_up_ms", "-"):>5} '
              f'| h={s.get("rel_h_m", "-"):>6} | 油门={s.get("ctrl_thr_pct", "-"):>4} '
              f'| mode={s.get("flight_mode", "-")} unloc={s.get("unlocked", "-")} '
              f'| RC3={s.get("rc3", "-")} P1={s.get("p1_mpa", "-")} P2={s.get("p2_mpa", "-")}')
    print(f'共 {a.n} 个快照（{len(m.g.snap)} 字段）')


def cmd_device(_):
    require_connected()
    m.g.write(anocom.encode_frame(anocom.FUNC_PARAM_CMD, bytes([0x00, 0, 0, 0, 0])))
    for _ in range(40):
        drain(0.1)
        if any(True for _ in m.g.ws_clients):  # 无 WS 时直接查 snap 之外的标志
            pass
        # 设备信息经 _on_telemetry_rx → send_all（无客户端则丢弃）→ 无法直接取。
        # 直接探测：读参数个数（新固件支持）判断版本
        break
    # 用参数个数探测固件版本：新固件回 0xE0 [0x01, count]
    m.g.write(anocom.cmd_read_param_count())
    got = [False]
    deadline = time.time() + 2.0
    while time.time() < deadline:
        drain(0.1)
        if m.g.param_count_seen:
            got[0] = True
            break
    if got[0]:
        print(f'✓ 新固件（ano_params 在位）：{m.g.param_count_seen} 参数')
    else:
        print('✗ 固件为旧版（参数命令无响应）—— 需烧录新固件（pio run -t upload）')


def _param_id_of(spec):
    """ID 数字或名称（att_roll.kp 等）→ id"""
    try:
        return int(spec)
    except ValueError:
        pass
    names = pm.expected_names()
    if spec in names:
        return names.index(spec)
    # 模糊：去除空格
    for i, n in enumerate(names):
        if n.replace(' ', '') == spec.replace(' ', ''):
            return i
    print(f'✗ 未知参数名: {spec}')
    sys.exit(1)


def cmd_param(a):
    require_connected()
    if a.action == 'list':
        _param_list(a)
    elif a.action == 'get':
        pid = _param_id_of(a.spec)
        if _param_read_single(pid):
            info = m.g.param_names.get(pid, {})
            val = m.g.param_values.get(pid)
            meta = pm.meta_for(info.get('name', '')) or {}
            if getattr(a, 'json', False):
                import json as _json
                print(_json.dumps({'ok': True, 'id': pid, 'name': info.get('name', '?'),
                                   'type': info.get('type', '?'), 'value': val,
                                   'group': meta.get('group', '?'),
                                   'unit': meta.get('unit', '')}, ensure_ascii=False))
            else:
                print(f'id={pid}  name={info.get("name", "?")}  type={info.get("type", "?")}  '
                      f'value={val}  group={meta.get("group", "?")}  unit={meta.get("unit", "")}')
        else:
            if getattr(a, 'json', False):
                import json as _json
                print(_json.dumps({'ok': False, 'id': pid, 'error': 'read timeout'}, ensure_ascii=False))
            else:
                print('✗ 读取 id=%d 超时（3 轮重试）——可能是固件处于 DBG 模式（参数链路暂停）或串口丢帧。先执行: cli.py dbg off' % pid)
    elif a.action == 'set':
        _param_set(a.spec, a.value, 'set', use_json=getattr(a, 'json', False))
    elif a.action == 'verify':
        _param_set(a.spec, a.value, 'verify', use_json=getattr(a, 'json', False))
    elif a.action == 'restore':
        m.g.write(anocom.cmd_param_restore_defaults())
        drain(0.5)
        print('✓ 已发送恢复默认（RAM 生效，重启回默认）')


def _val_close(a, b, is_float=True, tol=1e-4):
    """写后读回比对：float 用相对容差（0.1%），整型按类型精度（u8/u16 取整）。"""
    if a is None or b is None:
        return False
    if is_float:
        return abs(float(a) - float(b)) <= tol * max(1.0, abs(float(b)))
    return int(round(float(a))) == int(round(float(b)))


def _param_set(spec, value, mode, use_json=False):
    """写参数。★ 2026-08-10：0x00 校验帧在 2M 间歇丢帧下偶发丢失（假阴性——
    写入实际生效但确认帧没抓到）。set 与 verify 都加写后读回验证：
      set    确认帧 或 读回相符 任一即判成功（读回更权威）
      verify 写前读 → 写 → 写后读 三态报告（链路诊断）
    """
    pid = _param_id_of(spec)
    # 先取名称/类型（E2），确认 float/uint8——重试（丢帧/时序）
    info = {}
    for _ in range(3):
        m.g.write(anocom.cmd_read_param_info(pid))
        drain(0.4)
        if pid in m.g.param_names:
            info = m.g.param_names.get(pid, {})
            break
    is_float = info.get('type') == 'float' if info else True
    name = info.get('name', '?')

    # 写前读回（verify 模式必做；set 模式顺带检查当前值）
    before = None
    if mode == 'verify':
        if _param_read_single(pid):
            before = m.g.param_values.get(pid)
        if not use_json:
            print(f'写前读回: id={pid} ({name}) = {before}')

    # 写入 + 等 0x00 校验帧：2M 下 USB 串口偶发丢帧（CRC 失败固件静默丢弃）→ 重试 3 轮
    ok = False
    for attempt in range(3):
        frame = anocom.cmd_write_param(pid, value, is_float)
        body = frame[:-2]
        m.g.param_pending[pid] = (anocom.sum_check(body), anocom.add_check(body), time.time())
        m.g.write(frame)
        deadline = time.time() + 1.5
        while time.time() < deadline:
            drain(0.1)
            if pid not in m.g.param_pending:
                ok = True
                break
        if ok:
            break

    # 写后读回验证（读回比确认帧更权威——确认帧丢了不代表写入失败）
    after = None
    for _ in range(3):
        if _param_read_single(pid):
            after = m.g.param_values.get(pid)
            if after is not None:
                break

    if mode == 'verify':
        good = _val_close(after, value, is_float)
        if use_json:
            import json as _json
            print(_json.dumps({'ok': good, 'mode': 'verify', 'id': pid, 'name': name,
                               'before': before, 'after': after, 'target': value},
                              ensure_ascii=False))
        else:
            print(f'写后读回: id={pid} ({name}) = {after}')
            if good:
                print(f'✓ 写入生效: {before} → {after}（目标 {value}）')
            else:
                print(f'✗ 写入未生效: {before} → {after}（目标 {value}）——'
                      f'固件处于 DBG 模式（参数链路暂停）？先执行: cli.py dbg off')
    else:
        verified = after is not None and _val_close(after, value, is_float)
        success = ok or verified
        if use_json:
            import json as _json
            print(_json.dumps({'ok': success, 'mode': 'set', 'id': pid, 'name': name,
                               'value': value, 'ack': ok, 'readback': after,
                               'verified': verified}, ensure_ascii=False))
        elif ok and verified:
            print(f'✓ 写入 id={pid} ({name}) = {value}：确认帧 + 读回 {after} 双确认')
        elif verified:
            print(f'✓ 写入 id={pid} ({name}) = {value}：确认帧未到但读回 {after}（2M 丢帧假阴性，写入实际生效）')
        elif ok:
            print(f'✓ 写入 id={pid} ({name}) = {value}：确认帧确认（读回 {after} 超时）')
        else:
            print(f'✗ 写入 id={pid} ({name}) = {value}：确认帧未到且读回 {after}——'
                  f'固件处于 DBG 模式（参数链路暂停）？先执行: cli.py dbg off')


def cmd_diag(a):
    """一键系统诊断：固件版本 + 任务调度 + GNSS 协议 + 链路健康（走 dbg 通道）"""
    require_connected()
    if m.g.mode != 'dbg':
        _dbg_mode(True)
        time.sleep(0.4)
    # 清掉进入 DBG 模式前残留的二进制遥测帧（否则混入文本输出变乱码）
    try:
        while True:
            m.g.rx_q.get_nowait()
    except queue.Empty:
        pass
    for cmd in ('ver', 'tasks', 'gpsproto'):
        m.g.dbg_cmd(cmd)
        out = list(_dbg_collect(1.2))
        print(f'--- {cmd} ---')
        for ln in out:
            print(ln)
    print('--- link ---')
    _dbg_mode(False)          # 链路检查需要 AnoCom 遥测，退出 DBG 模式
    time.sleep(0.5)
    cmd_link(a)


def cmd_link(a):
    """链路健康检查（2M 间歇丢帧诊断）：
    1. 遥测帧率统计（1s 采样）
    2. 0xE0 0x01/0x02/0x03 参数命令成功率（各 5 次，按处理后状态检测）
    3. 0xE1 写确认率（读当前值写回，不改值）
    用法: cli.py --port COM10 link
    """
    require_connected()
    m.g.write(b'exit\n')   # 清 DBG 残留（handleAnoCom 短路会吞参数帧）
    time.sleep(0.3)
    # 清空 rx_q 积压（旧遥测在 FIFO 前段会淹没响应帧，导致成功率误判为 0）
    while True:
        try:
            m.g.rx_q.get_nowait()
        except queue.Empty:
            break

    # ---- 1. 遥测帧率统计 ----
    g0 = m.g.stat_frames
    t0 = time.time()
    while time.time() - t0 < 1.0:
        drain(0.1)
    frames = m.g.stat_frames - g0
    print(f'遥测帧率: {frames}/s，坏帧累计: {m.g.stat_bad}')

    # ---- 2. 参数命令成功率（字节流跨块累积检测响应帧头，每轮最多 3 发重试） ----
    def probe(frame, target_byte, tries=3):
        for _ in range(tries):
            m.g.write(frame)
            buf = b''
            deadline = time.time() + 0.6
            while time.time() < deadline:
                try:
                    kind, payload = m.g.rx_q.get(timeout=0.05)
                except queue.Empty:
                    continue
                if kind == 'rx':
                    buf += payload
                    if b'\xab\x05\xff' + bytes([target_byte]) in buf:
                        return True
        return False

    tests = [
        ('0x01 参数数量', anocom.cmd_read_param_count(), anocom.FUNC_PARAM_CMD),
        ('0x02 读值', anocom.cmd_read_param_value(0), anocom.FUNC_PARAM_WRITE_READ),
        ('0x03 读信息', anocom.cmd_read_param_info(0), anocom.FUNC_PARAM_INFO),
    ]
    for name, frame, target in tests:
        ok = sum(1 for _ in range(5) if probe(frame, target))
        print(f'{name} 成功率: {ok}/5（每轮最多 3 发）')

    # ---- 3. 写确认率（读当前值写回，不改值） ----
    val = None
    if _param_read_single(0):
        val = m.g.param_values.get(0)
    if val is not None:
        ok_w = 0
        for _ in range(5):
            m.g.param_pending.clear()
            frame = anocom.cmd_write_param(0, val)
            body = frame[:-2]
            m.g.param_pending[0] = (anocom.sum_check(body), anocom.add_check(body), time.time())
            m.g.write(frame)
            deadline = time.time() + 0.8
            while time.time() < deadline:
                drain(0.05)
                if 0 not in m.g.param_pending:
                    ok_w += 1
                    break
        print(f'写确认率（值写回）: {ok_w}/5')
    else:
        print('写确认率: 读参考值失败，跳过')

    # ---- 结论 ----
    if frames > 50:
        print('结论: 遥测正常；成功率低 = 2M 间歇丢帧（CLI param 已重试兜底）')
    else:
        print('结论: 遥测异常（<50 帧/s）——检查波特率/接线，或固件处于 DBG 模式（cli.py dbg off）')


def _param_read_single(pid, timeout=1.5, tries=3):
    """发 info+value 并阻塞等待该 ID 的 E2 名称与 E1 值都到达（无竞态）。
    2M 波特率下 USB 串口偶发丢字节（CRC 失败固件静默丢弃）→ 重试 tries 轮。"""
    for attempt in range(tries):
        m.g.write(anocom.cmd_read_param_info(pid))
        m.g.write(anocom.cmd_read_param_value(pid))
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                kind, payload = m.g.rx_q.get(timeout=0.05)
            except queue.Empty:
                continue
            if kind == 'rx':
                m._on_telemetry_rx(payload)
            if pid in m.g.param_names and pid in m.g.param_values:
                return True
    return False


def _param_list(a):
    """逐 ID 同步读取（2M 速率下 USB 串口偶发丢字节 → 失败补拉循环）。
    参数数量动态获取（0xE0 0x01），避免固件注册表扩容后硬编码漂移。"""
    m.g.param_names = {}
    m.g.param_values = {}
    m.g.write(anocom.cmd_read_param_count())
    drain(0.5)
    count = getattr(m.g, 'param_count_seen', 0) or 121   # 回退 = 当前固件注册表数量
    failed = []
    for pid in range(count):
        if not _param_read_single(pid):
            failed.append(pid)
        if pid % 20 == 0:
            print(f'…{pid + 1}/{count}（失败 {len(failed)}）', end='\r', flush=True)
    # 补拉循环：只重读失败 ID，直到全收或轮次耗尽
    for rnd in range(3):
        if not failed:
            break
        print(f'\n补拉第 {rnd + 1} 轮（{len(failed)} 个）…')
        failed = [pid for pid in failed if not _param_read_single(pid)]
    print()
    if failed:
        print(f'✗ 仍失败 {len(failed)} 个: {failed}（可单独 param get 重读）')
    if not m.g.param_names:
        print('✗ 固件无参数响应（旧版固件？）')
        return
    rows = []
    for pid in sorted(m.g.param_names):
        info = m.g.param_names[pid]
        val = m.g.param_values.get(pid, '?')
        meta = pm.meta_for(info.get('name', '')) or {}
        rows.append((pid, info.get('name', '?'), val, meta.get('group', '?')))
    cur_group = None
    for pid, name, val, group in rows:
        if group != cur_group:
            cur_group = group
            print(f'\n── {group} ──')
        print(f'  [{pid:>3}] {name:<28} {str(val):>12}')
    print(f'\n共 {len(rows)} 参数')


def _dbg_mode(on: bool):
    if on:
        if not m.g.dbg_enter():
            print('✗ 未连接')
            sys.exit(1)
        print('✓ 已进入调试模式（遥测暂停）')
    else:
        m.g.dbg_exit()
        print('✓ 已退出调试模式（遥测恢复）')


def _dbg_collect(dur):
    """消费 rx 并收集 DBG 文本——直接驱动 DbgSession（不经过 _on_dbg_rx，
    后者会抢先 drain_text 导致 CLI 读不到输出；export 状态机同样由此驱动）"""
    lines = []
    deadline = time.time() + dur
    while time.time() < deadline:
        try:
            kind, payload = m.g.rx_q.get(timeout=0.1)
        except queue.Empty:
            continue
        if kind != 'rx':
            continue
        m.g.dbgs.feed(payload)
        for ln in m.g.dbgs.drain_text().split('\n'):
            if ln.strip():
                lines.append(ln)
    return lines


def cmd_dbg(a):
    require_connected()
    if a.cmdline == 'on':
        _dbg_mode(True)
    elif a.cmdline == 'off':
        _dbg_mode(False)
    else:
        if m.g.mode != 'dbg':
            print('（自动进入调试模式）')
            _dbg_mode(True)
            time.sleep(0.4)
        m.g.dbg_cmd(a.cmdline)
        for ln in _dbg_collect(1.5):
            print(ln)


def cmd_flash(a):
    require_connected()
    if m.g.mode != 'dbg':
        print('（自动进入调试模式）')
        _dbg_mode(True)
        time.sleep(0.4)

    if a.action == 'stat':
        m.g.dbg_cmd('flash stat')
        for ln in _dbg_collect(1.5):
            print(ln)
    elif a.action == 'findseg':
        m.g.dbg_cmd('flash findseg')
        for ln in _dbg_collect(1.5):
            print(ln)
    elif a.action == 'export':
        _flash_export(a)


def _flash_export(a):
    # 段信息（findseg 输出）
    m.g.dbg_cmd('flash findseg')
    seg_out = _dbg_collect(1.0)

    if a.latest:
        # 解析 findseg 输出取最后一段起始页
        segs = []
        import re
        for line in seg_out:
            sm = re.search(r'seg=(\d+)', line)
            pm_ = re.search(r'page=(\d+)', line)
            if sm and pm_:
                segs.append((int(sm.group(1)), int(pm_.group(1))))
        if not segs:
            print('✗ 无飞行段记录（findseg 无输出）')
            return
        seg, start = segs[-1]
        count = a.count
        print(f'→ 导出最新段 seg={seg}（page {start}..{start + count}）')
    else:
        start, count = a.start, a.count
        print(f'→ 导出 page {start}..{start + count}')

    result = []
    # ★ 分片导出：DAP-Link VCP 2M 下大流量（>~40KB）丢字节（实测 100/256 页
    #   仅到 ~80%），小片（32 页=64KB）相对可靠。每片按 n×2048B 长度校验，
    #   不足重试最多 3 次，全部拼接后统一解析。
    CHUNK = 4   # DAP-Link VCP 2M 下小片 80% 完整，重试可达 ~99%
    total_raw = bytearray()
    page, remaining = start, count
    while remaining > 0:
        n = min(remaining, CHUNK)
        got = None
        for attempt in range(3):
            piece = []
            m.g.dbgs.start_export(page, n, piece.append)
            deadline = time.time() + 5 + n * 0.02
            while time.time() < deadline and not piece:
                _dbg_collect(0.2)
            if piece and len(piece[0]) == n * 2048:
                got = piece[0]
                break
            print(f'  片 {page}..{page + n - 1} 第 {attempt + 1} 次不完整'
                  f'（{len(piece[0]) if piece else 0}/{n * 2048}B），重试…', flush=True)
        if got is None:
            print(f'✗ 片 {page}..{page + n - 1} 导出失败（3 次后放弃）')
            break
        total_raw.extend(got)
        page += n
        remaining -= n
        print(f'  已导出 {len(total_raw) // 2048} 页…', flush=True)

    raw = bytes(total_raw)
    if not raw:
        print('✗ 导出失败：无数据')
        return
    frames = bb.extract_frames(raw)
    print(f'收到 {len(raw)} 字节 → {len(frames)} 帧 '
          f'(I={sum(1 for t, _ in frames if t == bb.TYPE_I)}, '
          f'P={sum(1 for t, _ in frames if t == bb.TYPE_P)}, '
          f'S={sum(1 for t, _ in frames if t == bb.TYPE_S)}, '
          f'E={sum(1 for t, _ in frames if t == bb.TYPE_E)})')
    if not frames:
        print('✗ 无有效帧（检查 start/count 或 Flash 未写入）')
        return
    rows, segments, columns = bb.decode_frames(frames)
    print(f'解析：{len(rows)} 行数据，{len(segments)} 个飞行段，{len(columns)} 列（列名自 S 帧）')
    for sg in segments:
        print(f'  段 {sg["num"]}: start_tms={sg["start_tms"]} dur={sg["dur_ms"]}ms '
              f'frames={sg["frames"]} cols={len(sg["columns"])}')
    if a.out:
        with open(a.out, 'w', newline='') as f:
            f.write(','.join(columns) + '\n')
            for r in rows:
                f.write(f'{r[0]},{r[1]},' + ','.join(f'{v:.4f}' for v in r[2]) + '\n')
        print(f'✓ CSV 已写出 {a.out}')
    if not a.quiet:
        print('预览（前 3 行）:')
        print(','.join(columns[:8]) + '…')
        for r in rows[:3]:
            print(f'{r[1]},' + ','.join(f'{v:.2f}' for v in r[2][:7]) + '…')


def cmd_record(a):
    require_connected()
    if a.action == 'start':
        path = a.out or f'output/rec_{time.strftime("%Y%m%d_%H%M%S")}.csv'
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        m.g.recorder = CsvRecorder(path)
        m.g.recorder.start()
        if a.duration > 0:
            # ★ 2026-08-09：定时记录（飞行场景：起记录→飞→到点自动停）
            print(f'● 记录中 → {path}（{a.duration}s 后自动停止）')
            t_end = time.time() + a.duration
            while time.time() < t_end:
                drain(0.2)
            m.g.recorder.stop()
            print(f'■ 已停止，共写入 {path}')
        else:
            print(f'● 记录中 → {path}（Ctrl+C 停止）')
            try:
                while True:
                    drain(0.2)
            except KeyboardInterrupt:
                m.g.recorder.stop()
                print(f'\n■ 已停止，共写入 {path}')
    elif a.action == 'stop':
        if m.g.recorder:
            m.g.recorder.stop()
            print('■ 记录已停止')
        else:
            print('（无活动记录）')


def cmd_sniff(a):
    """帧监控（原 tools/sniff_*.py + parse_anocom.py 散脚本并入）。
    主题:
      rc    0x06 模式/解锁 + 0x20 8 通道
      att   0x03 欧拉 + 0x21 控制输出
      axes  0x01 IMU 加速度/陀螺 + 0x03 欧拉（轴方向测试）
      euler 0x03 欧拉角统计（均值/标准差/范围——欧拉奇异检验）
      raw   原始字节统计（高频字节/帧头候选）
      all   全部遥测帧计数 + 最新值
    """
    require_connected()
    print(f'帧监控 {a.t}s（主题: {a.theme}）…')
    buf = bytearray()
    counters = Counter()
    samples = {}
    raw_data = b''
    euler_vals = []
    t_end = time.time() + a.t
    while time.time() < t_end:
        try:
            kind, payload = m.g.rx_q.get(timeout=0.05)
        except queue.Empty:
            continue
        if kind != 'rx':
            continue
        if a.theme == 'raw':
            raw_data += payload
            continue
        buf += payload
        frames, consumed = anocom.extract_frames(buf)
        if consumed:
            del buf[:consumed]
        for f in frames:
            if not f.valid:
                counters['bad'] += 1
                continue
            counters[f.func] += 1
            d = anocom.decode_frame(f)
            if d:
                samples[f.func] = d
                if a.theme == 'euler' and f.func == anocom.FUNC_ATTITUDE_EULER:
                    euler_vals.append((d['roll_deg'], d['pitch_deg'], d['heading_deg']))
    _sniff_report(a.theme, counters, samples, raw_data, euler_vals)


def _sniff_report(theme, counters, samples, raw_data, euler_vals):
    if theme == 'raw':
        if not raw_data:
            print('未收到任何字节（检查波特率/接线）')
            return
        cnt = Counter(raw_data)
        print(f'收到 {len(raw_data)} 字节')
        print('高频字节:', [(f'0x{b:02X}', n) for b, n in cnt.most_common(8)])
        pairs = Counter()
        for i in range(len(raw_data) - 1):
            if raw_data[i] == 0x00:
                pairs[raw_data[i + 1]] += 1
        print('0x00 后字节分布 top8:', [(f'0x{b:02X}', n) for b, n in pairs.most_common(8)])
        print('帧头候选:', {f'0x{h:02X}': raw_data.count(bytes([h])) for h in (0xAA, 0xA5, 0xFD, 0xC8, 0x55, 0x7E)})
        print('前 60 字节:', raw_data[:60].hex(' '))
        return
    if theme == 'euler':
        if not euler_vals:
            print('未收到 0x03 欧拉帧')
            return
        import statistics as st
        for idx, name in enumerate(('roll', 'pitch', 'Heading')):
            col = [v[idx] for v in euler_vals]
            print(f'{name:8s}: 均值={st.mean(col):+8.2f}  标准差={st.pstdev(col):6.2f}  '
                  f'范围=[{min(col):+.2f}, {max(col):+.2f}]')
        print(f'样本数 {len(euler_vals)}')
        return
    if theme == 'rc':
        if anocom.FUNC_FLIGHT_MODE in samples:
            d = samples[anocom.FUNC_FLIGHT_MODE]
            print(f'模式帧: mode={d["flight_mode"]} 解锁={d["unlocked"]}')
        if anocom.FUNC_RC_DATA in samples:
            d = samples[anocom.FUNC_RC_DATA]
            for i in range(1, 9):
                print(f'  CH{i}: {d[f"rc{i}"]}')
        else:
            print('未收到 0x40 遥控帧')
        return
    if theme == 'att':
        if anocom.FUNC_ATTITUDE_EULER in samples:
            d = samples[anocom.FUNC_ATTITUDE_EULER]
            print(f'欧拉: roll={d["roll_deg"]:+.2f} pitch={d["pitch_deg"]:+.2f} heading={d["heading_deg"]:+.2f}')
        if anocom.FUNC_ATTITUDE_CONTROL in samples:
            d = samples[anocom.FUNC_ATTITUDE_CONTROL]
            print(f'控制输出: roll={d["ctrl_roll"]:+.1f} pitch={d["ctrl_pitch"]:+.1f} '
                  f'thr={d["ctrl_thr_pct"]:+.1f} yaw={d["ctrl_yaw"]:+.1f}')
        return
    if theme == 'axes':
        if anocom.FUNC_IMU_DATA in samples:
            d = samples[anocom.FUNC_IMU_DATA]
            print(f'acc(g): x={d["acc_x_ms2"] / 9.81:+.3f} y={d["acc_y_ms2"] / 9.81:+.3f} z={d["acc_z_ms2"] / 9.81:+.3f}'
                  f' | gyro(dps): x={d["gyro_x_dps"]:+.3f} y={d["gyro_y_dps"]:+.3f} z={d["gyro_z_dps"]:+.3f}')
        if anocom.FUNC_ATTITUDE_EULER in samples:
            d = samples[anocom.FUNC_ATTITUDE_EULER]
            print(f'欧拉: roll={d["roll_deg"]:+.1f} pitch={d["pitch_deg"]:+.1f} heading={d["heading_deg"]:+.1f}')
        return
    # all：计数 + 最新快照
    if not counters:
        print('未收到任何帧（检查波特率/接线，或固件处于 DBG 模式: cli.py dbg off）')
        return
    total = sum(n for k, n in counters.items() if k != 'bad')
    print(f'收到 {total} 帧（坏帧 {counters.get("bad", 0)}）:')
    for func, n in sorted(counters.items()):
        if func == 'bad':
            continue
        name = anocom.FUNC_NAMES.get(func, hex(func))
        print(f'  0x{func:02X} {name:<20} ×{n}')
    if samples:
        print('最新快照:')
        for func, d in samples.items():
            name = anocom.FUNC_NAMES.get(func, hex(func))
            print(f'  0x{func:02X} {name}: ' + ' '.join(f'{k}={v:.3g}' for k, v in d.items()))


def cmd_stick(a):
    """打杆诊断（原 tools/diag_stick.py）：实时打印 模式/解锁 + 8 通道 + 控制输出。
    用法: cli.py --port COM10 stick -t 10 —— 期间请打杆（CH1滚转/CH2俯仰/CH4偏航）"""
    require_connected()
    print(f'开始采集 {a.t}s —— 请打杆（CH1滚转/CH2俯仰/CH4偏航）…')
    buf = bytearray()
    last = 0.0
    ch = {}
    mode = unlock = None
    t_end = time.time() + a.t
    while time.time() < t_end:
        try:
            kind, payload = m.g.rx_q.get(timeout=0.05)
        except queue.Empty:
            continue
        if kind != 'rx':
            continue
        buf += payload
        frames, consumed = anocom.extract_frames(buf)
        if consumed:
            del buf[:consumed]
        now = time.time()
        for f in frames:
            if not f.valid:
                continue
            d = anocom.decode_frame(f)
            if not d:
                continue
            if f.func == anocom.FUNC_FLIGHT_MODE:
                mode, unlock = d['flight_mode'], d['unlocked']
            elif f.func == anocom.FUNC_RC_DATA:
                ch = d
            elif f.func == anocom.FUNC_ATTITUDE_CONTROL and now - last > 0.7:
                chs = ' '.join(f'{ch[f"rc{i}"]:5d}' for i in range(1, 9)) if ch else '?'
                print(f'mode={mode} 解锁={unlock} | CH1-8: {chs} | '
                      f'控制输出(r/p/t/y): {d["ctrl_roll"]:6.1f} {d["ctrl_pitch"]:6.1f} '
                      f'{d["ctrl_thr_pct"]:6.1f} {d["ctrl_yaw"]:6.1f}')
                last = now
    print('采集结束')


def cmd_raw(a):
    require_connected()
    data = bytes.fromhex(a.hex)
    m.g.write(data)
    print(f'✓ 已发送 {len(data)}B: {a.hex}')
    drain(0.5)


# ========================================================================
#  入口
# ========================================================================
def main():
    ap = argparse.ArgumentParser(description='TandemVec GCS CLI（复用 server.main 链路）')
    # 全局自动连接：每条命令独立进程，提供 --port 则命令前后自动 连接/断开
    ap.add_argument('--port', help='自动连接串口（如 COM10），命令结束自动断开')
    ap.add_argument('--baud', type=int, default=DEFAULT_BAUD)
    ap.add_argument('--json', action='store_true', help='JSON 结构化输出（Agent 调用友好）')
    sub = ap.add_subparsers(dest='cmd', required=True)

    sub.add_parser('ports', help='枚举串口').set_defaults(fn=cmd_ports)

    p = sub.add_parser('diag', help='一键系统诊断（link + tasks + gpsproto + ver）')
    p.set_defaults(fn=cmd_diag)

    p = sub.add_parser('link', help='链路健康检查（遥测帧率 + 参数命令成功率）')
    p.set_defaults(fn=cmd_link)

    p = sub.add_parser('connect', help='连接串口')
    p.add_argument('port')
    p.add_argument('--baud', type=int, default=DEFAULT_BAUD)
    p.set_defaults(fn=cmd_connect)

    p = sub.add_parser('telemetry', help='收集遥测快照')
    p.add_argument('-n', type=int, default=10)
    p.add_argument('-i', type=float, default=0.2)
    p.set_defaults(fn=cmd_telemetry)

    sub.add_parser('device', help='探测设备/固件版本').set_defaults(fn=cmd_device)

    p = sub.add_parser('param', help='参数读写')
    p.add_argument('action', choices=['list', 'get', 'set', 'verify', 'restore'])
    p.add_argument('spec', nargs='?', help='参数 ID 或名称（att_roll.kp）')
    p.add_argument('value', nargs='?', type=float, help='写入值')
    p.set_defaults(fn=cmd_param)

    p = sub.add_parser('dbg', help='DBG 控制台')
    p.add_argument('cmdline', help='on / off / 任意命令（如 "flash stat"）')
    p.set_defaults(fn=cmd_dbg)

    p = sub.add_parser('flash', help='黑匣子命令')
    p.add_argument('action', choices=['stat', 'findseg', 'export'])
    p.add_argument('--start', type=int, default=0)
    p.add_argument('--count', type=int, default=2048)
    p.add_argument('--latest', action='store_true', help='自动定位最新飞行段')
    p.add_argument('-o', '--out', help='解析结果 CSV 输出')
    p.add_argument('--quiet', action='store_true', help='不打印预览')
    p.set_defaults(fn=cmd_flash)

    p = sub.add_parser('record', help='遥测记录')
    p.add_argument('action', choices=['start', 'stop'])
    p.add_argument('-o', '--out')
    p.add_argument('-d', '--duration', type=float, default=0,
                   help='记录秒数（0=直到 Ctrl+C；飞行场景建议给时长自动停止）')
    p.set_defaults(fn=cmd_record)

    p = sub.add_parser('sniff', help='帧监控（主题: rc/att/axes/euler/raw/all）')
    p.add_argument('theme', nargs='?', default='all',
                   choices=['all', 'rc', 'att', 'axes', 'euler', 'raw'],
                   help='监控主题（默认 all 全部帧计数）')
    p.add_argument('-t', type=float, default=3.0)
    p.set_defaults(fn=cmd_sniff)

    p = sub.add_parser('stick', help='打杆诊断（模式/通道/控制输出实时）')
    p.add_argument('-t', type=float, default=10.0)
    p.set_defaults(fn=cmd_stick)

    p = sub.add_parser('baudscan', help='波特率扫描（帧头 0x00 间隔规律性评分）')
    p.add_argument('--dur', type=float, default=2.0, help='每个波特率采集秒数')
    p.add_argument('--bauds', default='9600,19200,38400,57600,115200,230400,460800,921600,2000000',
                   help='候选波特率（逗号分隔）')
    p.set_defaults(fn=cmd_baudscan)

    p = sub.add_parser('raw', help='发送原始帧（hex）')
    p.add_argument('hex')
    p.set_defaults(fn=cmd_raw)

    a = ap.parse_args()
    # baudscan 自开串口逐波特率扫描，跳过预连接（否则串口被占用打不开）
    auto_conn = a.port is not None and a.cmd != 'baudscan'
    if auto_conn:
        # CLI 自消费（drain/_param_read_single），不启动后端消费线程——
        # 否则 DBG 文本/参数响应会被消费线程取走，CLI 读不到
        m.g.connect(a.port, a.baud, consume_thread=False)
        time.sleep(0.4)   # 等串口稳定 + 遥测首帧
    try:
        a.fn(a)
    except KeyboardInterrupt:
        print('\n（中断）')
    finally:
        if auto_conn and m.g.link and m.g.link.is_open:
            m.g.disconnect()


if __name__ == '__main__':
    main()
