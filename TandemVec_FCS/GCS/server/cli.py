#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cli.py — GCS 命令行调试接口（复用 server.main 处理链路，行为与 Web 后端一致）

用法（全部命令在 TandemVec_FCS/GCS 目录执行；每条命令独立连接，加 --port 自动连/断）：
  py -3.12 server/cli.py ports                          # 枚举串口
  py -3.12 server/cli.py --port COM10 telemetry -n 5    # 自动连接并收集 5 个遥测快照
  py -3.12 server/cli.py --port COM10 device            # 设备/固件版本探测（0xE3 + 参数个数）
  py -3.12 server/cli.py --port COM10 param list        # 读取全部 117 参数（表格）
  py -3.12 server/cli.py --port COM10 param get 0       # 读单个参数（ID 或名称 att_roll.kp）
  py -3.12 server/cli.py --port COM10 param set att_roll.kp 2.6   # 写参数（等 0x00 校验帧）
  py -3.12 server/cli.py --port COM10 param restore     # 恢复出厂默认
  py -3.12 server/cli.py --port COM10 dbg on            # 进入调试模式（遥测暂停）
  py -3.12 server/cli.py --port COM10 dbg "flash stat"  # 发任意 DBG 命令
  py -3.12 server/cli.py --port COM10 flash findseg     # 飞行段列表
  py -3.12 server/cli.py --port COM10 flash export --latest -o seg.csv   # 最新段导出解析
  py -3.12 server/cli.py --port COM10 flash export --start 8 --count 32 -o seg.csv
  py -3.12 server/cli.py --port COM10 record start -o rec.csv   # 遥测记录（Ctrl+C 停）
  py -3.12 server/cli.py --port COM10 sniff -t 3        # 帧监控
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
        m.g.write(anocom.cmd_read_param_info(pid))
        m.g.write(anocom.cmd_read_param_value(pid))
        drain(1.0)
        info = m.g.param_names.get(pid, {})
        val = m.g.param_values.get(pid)
        meta = pm.meta_for(info.get('name', '')) or {}
        print(f'id={pid}  name={info.get("name", "?")}  type={info.get("type", "?")}  '
              f'value={val}  group={meta.get("group", "?")}  unit={meta.get("unit", "")}')
    elif a.action == 'set':
        pid = _param_id_of(a.spec)
        value = a.value
        # 先取名称/类型（E2），确认 float/uint8
        m.g.write(anocom.cmd_read_param_info(pid))
        drain(0.5)
        info = m.g.param_names.get(pid, {})
        is_float = info.get('type') == 'float' if info else True
        frame = anocom.cmd_write_param(pid, value, is_float)
        body = frame[:-2]
        m.g.param_pending[pid] = (anocom.sum_check(body), anocom.add_check(body), time.time())
        m.g.write(frame)
        # 等待 0x00 校验帧（_on_param_check 弹出 pending）
        deadline = time.time() + 2.0
        ok = False
        while time.time() < deadline:
            drain(0.1)
            if pid not in m.g.param_pending:
                ok = True
                break
        print(f'写入 id={pid} ({info.get("name", "?")}) = {value}: ' + ('✓ 校验帧确认' if ok else '✗ 超时无确认'))
    elif a.action == 'restore':
        m.g.write(anocom.cmd_param_restore_defaults())
        drain(0.5)
        print('✓ 已发送恢复默认（RAM 生效，重启回默认）')


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
    """逐 ID 同步读取（2M 速率下 USB 串口偶发丢字节 → 失败补拉循环）"""
    m.g.param_names = {}
    m.g.param_values = {}
    failed = []
    for pid in range(117):
        if not _param_read_single(pid):
            failed.append(pid)
        if pid % 20 == 0:
            print(f'…{pid + 1}/117（失败 {len(failed)}）', end='\r', flush=True)
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
    require_connected()
    counter = Counter()
    samples = {}
    print(f'帧监控 {a.t}s …')
    deadline = time.time() + a.t
    while time.time() < deadline:
        drain(0.2)
        # 直接扫串口原始缓冲不可行（消费在 _on_telemetry_rx）→ 统计已处理帧：
        # 在 telemetry 快照上统计字段来源
        if m.g.snap:
            counter['telemetry_snapshot'] += 1
            samples = dict(m.g.snap)
    print('收到遥测快照:', counter.get('telemetry_snapshot', 0))
    if samples:
        print('最新字段数:', len(samples))


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
    sub = ap.add_subparsers(dest='cmd', required=True)

    sub.add_parser('ports', help='枚举串口')

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
    p.add_argument('action', choices=['list', 'get', 'set', 'restore'])
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

    p = sub.add_parser('sniff', help='帧监控')
    p.add_argument('-t', type=float, default=3.0)
    p.set_defaults(fn=cmd_sniff)

    p = sub.add_parser('raw', help='发送原始帧（hex）')
    p.add_argument('hex')
    p.set_defaults(fn=cmd_raw)

    a = ap.parse_args()
    auto_conn = a.port is not None
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
