# -*- coding: utf-8 -*-
"""
main.py — GCS 服务入口：FastAPI + WebSocket + pyserial

启动：python -m uvicorn main:app --host 127.0.0.1 --port 8091
前端：http://127.0.0.1:8091/

架构：
  串口读线程 → queue.Queue → asyncio 任务（切帧/解码/聚合/广播）
  WS 客户端消息 → 串口写（AnoCom 命令帧 / DBG 控制台命令）

模式：telemetry（AnoCom 遥测，默认）| dbg（调试控制台，黑匣子流程）。
DBG 模式与遥测互斥（固件行为），进入 DBG 后固件停止遥测轮发。
"""
import asyncio
import json
import logging
import os
import queue
import struct
import threading
import time
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

# 相对导入（python -m uvicorn server.main:app）；直接运行本文件时回退绝对导入
try:
    from . import anocom
    from . import blackbox as bb
    from . import params as pm
    from .datalog import CsvRecorder, ReplaySource, list_recordings
    from .serial_link import SerialLink, list_ports
except ImportError:
    import anocom
    import blackbox as bb
    import params as pm
    from datalog import CsvRecorder, ReplaySource, list_recordings
    from serial_link import SerialLink, list_ports

log = logging.getLogger('gcs')
logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(name)s: %(message)s')

ROOT = Path(__file__).resolve().parent.parent
WEB_DIR = ROOT / 'web'
OUTPUT_DIR = ROOT / 'output'
OUTPUT_DIR.mkdir(exist_ok=True)

DEFAULT_BAUD = 2000000      # = 固件 SERIAL6_BAUDRATE
TELEMETRY_PERIOD = 0.05     # 遥测推送节流 20Hz
PARAM_WRITE_TIMEOUT = 2.0   # 参数写入校验帧超时
TELE_BUF_CAP = 8192         # 无完整帧时遥测缓冲上限（防乱码流 O(n²) 膨胀卡死）
STAT_PERIOD = 1.0           # 链路统计推送周期
START_TS = time.time()      # 后端启动时间（/api/status 用）
HEX_FLUSH_PERIOD = 0.35     # hex 监听推送节流
HEX_FLUSH_MAX = 512         # 单次 hex 推送最大字节

app = FastAPI(title='TandemVec GCS', version='0.3.0')


class GCS:
    """全局状态机：串口 + 模式 + 遥测聚合 + 参数客户端 + 记录/回放"""

    def __init__(self):
        self.link = None
        self.mode = 'telemetry'          # telemetry | dbg
        self.dbgs = None
        self.tele_buf = bytearray()      # AnoCom 原始流缓冲
        self.snap = {}                   # 最新遥测快照（键 = datalog.RECORD_FIELDS + 附加）
        self.recorder = None
        self.replay = None               # ReplaySource（回放中）
        self.replay_task = None
        self.param_names = {}            # id → {name, type}
        self.param_values = {}           # id → value
        self.param_count_seen = None     # 固件回传的参数个数（0xE0 CMD 0x01）
        self.param_pending = {}          # id → (sc, ac, 时间) 等待校验帧
        self.vars_meta = {}              # 变量 id → {name, type}（0xF3 清单，AnoVars）
        self.vars_meta_ready = False     # 清单是否已拉全
        self.vars_count_seen = None      # 固件回传的变量个数（0xF3 CMD 0x01）
        self.vars_samples = {}           # 变量名 → 最近值环形样本（watch 统计用）
        self.rx_q = queue.Queue()
        self.ws_clients = set()
        self._loop = None
        self._stat_buf = []
        self._flash_export_cb = None
        self._stat_cb = None
        self._consumer_thread = None
        self._consumer_stop = threading.Event()
        # ---- 链路统计（消费线程累加，_stat_loop 周期取差值广播）----
        self.stat_bytes = 0            # 累计接收字节
        self.stat_frames = 0           # 累计完整帧
        self.stat_bad = 0              # 累计 CRC 失败帧
        self.stat_tele = 0             # 累计解出遥测字段的帧（区别于参数/设备响应帧）
        self.last_tele_ts = None       # 最近一次解码出遥测字段的时间
        # ---- hex 监听（链路原始字节监视，排障用）----
        self.hex_watch = False
        self._hex_buf = bytearray()
        self._hex_last_flush = 0.0

    def reset_stats(self):
        self.stat_bytes = 0
        self.stat_frames = 0
        self.stat_bad = 0
        self.stat_tele = 0
        self.last_tele_ts = None

    # ---------------- 串口 ----------------
    def connect(self, port, baud, consume_thread: bool = True):
        self.disconnect()
        self.link = SerialLink(port, baud, self._on_rx)
        self.link.connect()
        self.tele_buf.clear()
        self.snap = {}
        self.mode = 'telemetry'
        self.dbgs = bb.DbgSession(self.link.write)
        self.recorder = None
        self.param_names = {}
        self.param_values = {}
        self.vars_meta = {}
        self.vars_meta_ready = False
        self.reset_stats()
        # ★ 防"固件卡在 DBG 模式无遥测"：连接后自动发 exit——
        #   若固件处于调试模式则退出恢复遥测轮发；若本就在遥测模式，
        #   固件 DBG 入口检测会把这行非帧字节当无效行丢弃，无副作用。
        try:
            self.link.write(b'exit\n')
        except Exception:
            pass
        if consume_thread:
            self._start_consumer()

    def _start_consumer(self):
        """独立消费线程：持续从 rx_q 取字节流并处理（与读线程解耦，
        保证参数响应帧不被遥测积压淹没——asyncio 消费吞吐不足会丢参数）"""
        if self._consumer_thread and self._consumer_thread.is_alive():
            return
        self._consumer_stop.clear()
        self._consumer_thread = threading.Thread(target=self._consumer_loop,
                                                 daemon=True, name='gcs-rx-consumer')
        self._consumer_thread.start()
        log.info('rx consumer 已启动')

    def _consumer_loop(self):
        log.info('rx consumer 循环开始')
        while not self._consumer_stop.is_set():
            try:
                kind, payload = self.rx_q.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                if kind == 'link_down':
                    log.warning('link_down：读线程异常退出')
                    # ★ 勿调 disconnect()：那会 join 消费线程自身（RuntimeError
                    #   被静默吞掉、断线通知丢失）。只关链路、清状态。
                    self._close_link()
                    self.send_all({'type': 'conn', 'status': 'disconnected', 'reason': 'link_down'})
                elif kind == 'rx':
                    if self.hex_watch:
                        self._hex_feed(payload)
                    if self.mode == 'dbg' and self.dbgs:
                        _on_dbg_rx(payload)
                    else:
                        _on_telemetry_rx(payload)
            except Exception:
                log.warning('rx consumer error', exc_info=True)
        log.info('rx consumer 循环退出')

    def _hex_feed(self, payload: bytes):
        """hex 监听：累积原始字节，节流推送 hex+ASCII 转储（排障用，
        让"完全没数据 / 乱码 / DBG 文本 / AnoCom 帧"一眼可分）"""
        self._hex_buf.extend(payload)
        now = time.time()
        if len(self._hex_buf) >= HEX_FLUSH_MAX or now - self._hex_last_flush >= HEX_FLUSH_PERIOD:
            chunk = bytes(self._hex_buf[:HEX_FLUSH_MAX])
            del self._hex_buf[:HEX_FLUSH_MAX]
            self._hex_last_flush = now
            hexs = ' '.join(f'{b:02x}' for b in chunk)
            asc = ''.join(chr(b) if 32 <= b < 127 else '·' for b in chunk)
            self.send_all({'type': 'rx_hex', 'hex': hexs, 'ascii': asc, 'n': len(chunk)})

    def _close_link(self):
        """仅关闭串口链路并复位模式/缓冲（不碰消费线程，线程内安全）"""
        if self.link:
            self.link.close()
            self.link = None
        self.tele_buf.clear()
        self.mode = 'telemetry'

    def disconnect(self):
        self._consumer_stop.set()
        if self._consumer_thread and self._consumer_thread.is_alive():
            # 消费线程自身调用时跳过 join（防 join 当前线程）
            if threading.current_thread() is not self._consumer_thread:
                self._consumer_thread.join(timeout=2.0)
            self._consumer_thread = None
        self._close_link()

    def connect_fake(self, replay: list, consume_thread: bool = False):
        """测试专用：注入模拟串口字节流（FakeSerialLink，不依赖 pyserial/硬件）
        consume_thread=False：测试自行驱动消费（_drain），避免线程竞态"""
        from serial_link import FakeSerialLink
        self.disconnect()
        self.link = FakeSerialLink(replay=replay, on_data=self._on_rx)
        self.link.connect()
        self.tele_buf.clear()
        self.snap = {}
        self.mode = 'telemetry'
        self.dbgs = bb.DbgSession(self.link.write)
        self.param_names = {}
        self.param_values = {}
        self.vars_meta = {}
        self.vars_meta_ready = False
        self.reset_stats()
        if consume_thread:
            self._start_consumer()

    def write(self, data: bytes):
        if self.link and self.link.is_open:
            self.link.write(data)

    # ---------------- 串口读线程 → 队列 ----------------
    def _on_rx(self, chunk):
        if chunk:
            self.rx_q.put(('rx', chunk))
        else:
            self.rx_q.put(('link_down', None))

    # ---------------- 模式与命令 ----------------
    def dbg_enter(self):
        if not (self.link and self.link.is_open) or not self.dbgs:
            return False
        self.mode = 'dbg'
        self.tele_buf.clear()
        # 清空 rx 队列残留（进入 DBG 前固件仍在发遥测字节，
        # 残留会混入 DBG 文本输出）
        while not self.rx_q.empty():
            try:
                self.rx_q.get_nowait()
            except queue.Empty:
                break
        # 导出场景吞吐优先：读粒度提大（遥测恢复时还原）
        try:
            self.link.set_read_size(8192)
        except Exception:
            pass
        self.dbgs.enter()
        return True

    def dbg_exit(self):
        if not self.dbgs:
            return False
        self.dbgs.exit()
        self.mode = 'telemetry'
        try:
            self.link.set_read_size(1024)
        except Exception:
            pass
        return True

    def dbg_cmd(self, line: str):
        if self.dbgs:
            self.dbgs.send_cmd(line)

    # ---------------- WS 广播 ----------------
    def send_all(self, obj: dict):
        if not self.ws_clients:
            return
        text = json.dumps(obj, ensure_ascii=False)
        for ws in list(self.ws_clients):
            try:
                asyncio.run_coroutine_threadsafe(ws.send_text(text), self._loop)
            except Exception:
                pass


g = GCS()


# ========================================================================
#  串口数据处理（由 GCS._consumer_loop 独立线程驱动）
# ========================================================================


def _on_dbg_rx(chunk: bytes):
    """DBG 模式：文本行推送 + flash export 原始字节收集"""
    g.dbgs.feed(chunk)
    text = g.dbgs.drain_text()
    if text:
        _emit_dbg_text(text)
        # ★ 2026-08-09 修复回归：findseg 输出解析从未被接线（黑匣子页
        #   「列出飞行段」静默失效）；此处补上 → flash_segments 广播
        _on_findseg_text(text)


def _emit_dbg_text(text: str):
    for line in text.split('\n'):
        if line.strip():
            g.send_all({'type': 'dbg_out', 'line': line})


def _on_telemetry_rx(chunk: bytes):
    g.stat_bytes += len(chunk)
    g.tele_buf.extend(chunk)
    # ★ 2026-08-08 卡顿根因修复：旧实现 find_frames 全缓冲扫描 + 找到帧才
    #   整体 clear——错波特率/固件卡 DBG 发文本时缓冲无限膨胀，每块都对
    #   全缓冲 O(n) 扫描 + bytes() 拷贝，整体 O(n²) 退化至上位机卡死；
    #   且整体 clear 会把块尾半帧一并丢弃。现改为：只切完整帧、只删已消费
    #   前缀、半帧留尾；无帧且超上限时按垃圾流裁尾。
    frames, consumed = anocom.extract_frames(bytes(g.tele_buf))
    if consumed:
        del g.tele_buf[:consumed]
    elif len(g.tele_buf) > TELE_BUF_CAP:
        idx = g.tele_buf.rfind(bytes([anocom.FRAME_HEAD]))
        if idx > 0:
            del g.tele_buf[:idx]
        elif idx < 0:
            g.tele_buf.clear()
        else:
            del g.tele_buf[1:]   # idx == 0：帧头在起点但组不成帧，丢弃帧头字节
    if frames:
        g.stat_frames += len(frames)
        g.stat_bad += sum(1 for f in frames if not f.valid)
    changed = False
    for f in frames:
        # ---- 参数/设备帧先分流（decode_frame 无对应解码器）----
        if f.func == anocom.FUNC_DATA_CHECK:
            _on_param_check(f)
            continue
        if f.func == anocom.FUNC_PARAM_INFO:
            _on_param_info(f)
            continue
        if f.func == anocom.FUNC_PARAM_WRITE_READ:
            # 0xE1 均为读值回传（上位机写入请求不会触发固件回 0xE1）
            if len(f.payload) >= 2:
                _on_param_value(f.payload[0] | (f.payload[1] << 8), f.payload)
            continue
        if f.func == anocom.FUNC_PARAM_CMD:
            _on_param_cmd(f)
            continue
        if f.func == anocom.FUNC_DEVICE_INFO:
            dec = anocom.decode_device_info(f.payload)
            if dec:
                g.send_all({'type': 'device_info', **dec})
            continue
        # ---- AnoVars 通用变量帧（2026-08-10）----
        if f.func == anocom.FUNC_VARS_LIST:
            _on_vars_list(f)
            continue
        if f.func == anocom.FUNC_VARS_VALUE:
            _on_vars_value(f)
            continue
        # ---- 遥测帧 ----
        dec = anocom.decode_frame(f)
        if not dec:
            continue
        g.stat_tele += 1
        g.snap.update(dec)
        changed = True
    if changed:
        g.last_tele_ts = time.time()
        g.snap['t_ms'] = time.time() * 1000
        if g.recorder and g.recorder.active and not g.replay:
            g.recorder.write(g.snap)
        # 20Hz 节流推送
        now = time.time()
        last = getattr(g, '_last_tele_push', 0)
        if now - last >= TELEMETRY_PERIOD:
            g._last_tele_push = now
            g.send_all({'type': 'telemetry', 'data': g.snap})


async def _stat_loop():
    """1Hz 链路健康统计广播：字节率/帧率/CRC 失败率/遥测帧率/停滞时长。
    前端据此区分"未连接 / 固件不发 / 乱码 / CRC 灭 / 只有响应帧无遥测"。"""
    prev = (0, 0, 0, 0)
    while True:
        await asyncio.sleep(STAT_PERIOD)
        cur = (g.stat_bytes, g.stat_frames, g.stat_bad, g.stat_tele)
        connected = bool(g.link and g.link.is_open)
        tele_age = (time.time() - g.last_tele_ts) if g.last_tele_ts is not None else None
        g.send_all({
            'type': 'link_stat',
            'connected': connected,
            'mode': g.mode,
            'bytes_s': cur[0] - prev[0],
            'frames_s': cur[1] - prev[1],
            'bad_s': cur[2] - prev[2],
            'tele_s': cur[3] - prev[3],
            'tele_age_s': round(tele_age, 1) if tele_age is not None else None,
            'fields': len(g.snap),
        })
        prev = cur


# ---------------- 参数响应处理 ----------------
def _on_param_info(f: anocom.Frame):
    """E2 信息帧：payload = [ID u16, TYPE u8, NAME 20B]"""
    p = f.payload
    if len(p) < 23:
        return
    pid = p[0] | (p[1] << 8)
    ptype = p[2]
    name = p[3:23].split(b'\x00')[0].decode('ascii', errors='replace')
    g.param_names[pid] = {'name': name, 'type': 'float' if ptype == anocom.ANO_FLOAT else 'uint8'}


def _on_param_value(pid: int, payload: bytes):
    """E1 值回传：payload = [ID u16, PAR_VAL]
    ★ 类型按 payload 长度判断（float=6B / uint8=3B），不依赖 E2 信息帧到达时序
    —— 若等 name_info，E1 先到时 uint8 值会被误判 float 而丢失（enabled 全缺 bug）"""
    is_float = len(payload) >= 6
    if len(payload) < 3:
        return
    val = anocom.parse_param_value(payload, is_float)
    if val is not None:
        g.param_values[pid] = val
    name = g.param_names.get(pid, {}).get('name', f'id{pid}')
    g.send_all({'type': 'param_value', 'id': pid, 'name': name, 'value': val})


def _on_param_cmd(f: anocom.Frame):
    """0xE0 回传帧（参数个数等）"""
    p = f.payload
    if p and p[0] == 0x01 and len(p) >= 5:
        count = p[1] | (p[2] << 8) | (p[3] << 16) | (p[4] << 24)
        g.param_count_seen = count
        g.send_all({'type': 'param_count', 'count': count})


def _on_param_check(f: anocom.Frame):
    """0x00 校验帧：匹配参数写入确认"""
    p = f.payload
    if len(p) < 3:
        return
    id_get, sc_get, ac_get = p[0], p[1], p[2]
    for pid, (exp_sc, exp_ac, t0) in list(g.param_pending.items()):
        if id_get == anocom.FUNC_PARAM_WRITE_READ and exp_sc == sc_get and exp_ac == ac_get:
            g.param_pending.pop(pid, None)
            g.send_all({'type': 'param_written', 'id': pid, 'ok': True})
            return


def _on_vars_list(f: anocom.Frame):
    """0xF3 变量清单应答：CMD 0x01 个数 / CMD 0x02 变量信息 [id u16, type u8, name 16B]"""
    p = f.payload
    if not p:
        return
    if p[0] == 0x01 and len(p) >= 3:
        count = p[1] | (p[2] << 8)
        g.vars_count_seen = count
        g.send_all({'type': 'vars_count', 'count': count})
        return
    if p[0] == 0x02 and len(p) >= 20:  # [0x02, id u16, type u8, name 16B] = 20B
        vid = p[1] | (p[2] << 8)
        vtype = p[3]
        name = p[4:20].split(b'\x00')[0].decode('ascii', errors='replace')
        g.vars_meta[vid] = {'name': name, 'type': 'float' if vtype == 0 else f'u{vtype}'}
        g.send_all({'type': 'vars_info', 'id': vid, 'name': name, 'type': vtype})


def _on_vars_value(f: anocom.Frame):
    """0xF2 变量值帧 [id u16 LE] + [float LE]：查 vars_meta 映射名称进快照
    （字段名 vars_<name>，随 20Hz telemetry 推送 + record 落盘）。
    清单未拉全时仍可推（名退化为 vars_idN）——CLI vars watch 会先拉清单。"""
    p = f.payload
    if len(p) < 6:
        return
    vid = p[0] | (p[1] << 8)
    val = struct.unpack('<f', p[2:6])[0]
    meta = g.vars_meta.get(vid)
    if meta:
        key = 'vars_' + meta['name']
    else:
        key = f'vars_id{vid}'
    g.snap[key] = val
    # 记录样本（watch 统计用）——轻量环形，最多 256 个
    samples = g.vars_samples.setdefault(key, [])
    if len(samples) < 256:
        samples.append(val)


# ========================================================================
#  WebSocket 消息处理
# ========================================================================

async def handle_ws_message(ws: WebSocket, msg: dict):
    cmd = msg.get('cmd')
    if cmd == 'list_ports':
        await ws.send_json({'type': 'ports', 'ports': list_ports()})

    elif cmd == 'hex_on':
        g.hex_watch = True
        g._hex_buf.clear()
        await ws.send_json({'type': 'hex_state', 'on': True})

    elif cmd == 'hex_off':
        g.hex_watch = False
        await ws.send_json({'type': 'hex_state', 'on': False})

    elif cmd == 'connect':
        port, baud = msg.get('port'), int(msg.get('baud') or DEFAULT_BAUD)
        try:
            await asyncio.to_thread(g.connect, port, baud)
        except Exception as exc:
            await ws.send_json({'type': 'conn', 'status': 'error', 'reason': str(exc)})
            return
        await ws.send_json({'type': 'conn', 'status': 'connected', 'port': port, 'baud': baud})
        log.info('connected %s @ %d', port, baud)

    elif cmd == 'disconnect':
        g.disconnect()
        await ws.send_json({'type': 'conn', 'status': 'disconnected'})

    elif cmd == 'device_info':
        # 0xE0 CMD 0x00 读设备信息 → 固件回 0xE3（新旧固件均支持）
        g.write(anocom.encode_frame(anocom.FUNC_PARAM_CMD, bytes([0x00, 0, 0, 0, 0])))

    elif cmd == 'dbg_enter':
        if not g.dbg_enter():
            await ws.send_json({'type': 'log', 'level': 'warn', 'msg': '未连接串口，无法进入调试模式'})
        else:
            await ws.send_json({'type': 'dbg_mode', 'on': True})

    elif cmd == 'dbg_exit':
        g.dbg_exit()
        await ws.send_json({'type': 'dbg_mode', 'on': False})

    elif cmd == 'dbg_cmd':
        if g.mode == 'dbg':
            g.dbg_cmd(str(msg.get('line', '')))

    elif cmd == 'flash_stat':
        await _dbg_ensure()
        g.dbg_cmd('flash stat')

    elif cmd == 'flash_findseg':
        await _dbg_ensure()
        g.dbg_cmd('flash findseg')

    elif cmd == 'flash_export':
        await _dbg_ensure()
        start = int(msg.get('start', 0))
        count = int(msg.get('count', 2048))
        await _flash_export_chunked_async(ws, start, count)

    elif cmd == 'param_read_all':
        await _param_read_all(ws)

    elif cmd == 'param_read':
        pid, name = _param_id_of(msg)
        if pid is None:
            await ws.send_json({'type': 'log', 'level': 'error',
                                'msg': f'未知参数名: {name}'})
            return
        g.write(anocom.cmd_read_param_info(pid))
        g.write(anocom.cmd_read_param_value(pid))

    elif cmd == 'param_write':
        pid, name = _param_id_of(msg)
        if pid is None:
            await ws.send_json({'type': 'log', 'level': 'error',
                                'msg': f'未知参数名: {name}'})
            return
        value = msg.get('value')
        name_info = g.param_names.get(pid, {})
        # ★ 2026-08-09：名字寻址时类型按 params.py 元数据（未读 E2 也能写对 float/uint8）
        is_float = (name_info.get('type') == 'float' if name_info
                    else pm.is_float_name(name or ''))
        frame = anocom.cmd_write_param(pid, value, is_float)
        # 记录期望校验（回传帧 SC/AC 需与发送帧一致）
        body = frame[:-2]
        g.param_pending[pid] = (anocom.sum_check(body), anocom.add_check(body), time.time())
        g.write(frame)
        await ws.send_json({'type': 'param_written_pending', 'id': pid})

    elif cmd == 'param_restore':
        g.write(anocom.cmd_param_restore_defaults())

    elif cmd == 'param_save':
        g.write(anocom.cmd_param_save())

    elif cmd == 'record_start':
        fn = str(msg.get('file') or f'rec_{time.strftime("%Y%m%d_%H%M%S")}.csv')
        g.recorder = CsvRecorder(str(OUTPUT_DIR / fn))
        g.recorder.start()
        await ws.send_json({'type': 'record_state', 'on': True, 'file': fn})

    elif cmd == 'record_stop':
        if g.recorder:
            g.recorder.stop()
        await ws.send_json({'type': 'record_state', 'on': False})

    elif cmd == 'list_recordings':
        await ws.send_json({'type': 'recordings', 'files': list_recordings(str(OUTPUT_DIR))})

    elif cmd == 'replay_start':
        fn = str(msg.get('file', ''))
        path = OUTPUT_DIR / fn
        if not path.exists():
            await ws.send_json({'type': 'log', 'level': 'error', 'msg': f'文件不存在: {fn}'})
            return
        g.replay = ReplaySource(str(path))
        g.replay.open()
        if g.replay_task:
            g.replay_task.cancel()
        g.replay_task = asyncio.create_task(_replay_loop(ws))
        await ws.send_json({'type': 'replay_state', 'on': True, 'file': fn})

    elif cmd == 'replay_stop':
        if g.replay_task:
            g.replay_task.cancel()
            g.replay_task = None
        g.replay = None
        await ws.send_json({'type': 'replay_state', 'on': False})


async def _replay_loop(ws: WebSocket):
    """按 CSV 时间戳节奏回放遥测快照"""
    t_last = None
    t0 = time.time()
    while g.replay:
        row = g.replay.next_row()
        if row is None:
            g.send_all({'type': 'replay_state', 'on': False})
            g.replay = None
            return
        t_now = row.get('t_ms')
        if t_last is not None and t_now is not None:
            dt = (t_now - t_last) / 1000.0
            if dt > 0:
                await asyncio.sleep(max(0.0, dt * 0.5))  # 半速回放，便于观察
        t_last = t_now
        g.snap = {k: v for k, v in row.items() if v is not None}
        await asyncio.sleep(TELEMETRY_PERIOD)
        g.send_all({'type': 'telemetry', 'data': g.snap, 'replay': True})


async def _dbg_ensure():
    """确保处于 DBG 模式（不在则进入并等固件切换完成）"""
    if g.mode != 'dbg':
        g.dbg_enter()
        await asyncio.sleep(0.4)   # 固件 200Hz 检测 DBG\n + 串口回程
    g.send_all({'type': 'dbg_mode', 'on': True})


def _on_findseg_text(text: str):
    """解析 flash findseg 输出行：[DBG] seg=<n> page=<p> tms=<t>"""
    segs = []
    for line in text.split('\n'):
        import re
        sm = re.search(r'seg=(\d+)', line)
        pm_ = re.search(r'page=(\d+)', line)
        tm = re.search(r'tms=(\d+)', line)
        if sm:
            segs.append({'num': int(sm.group(1)),
                         'page': int(pm_.group(1)) if pm_ else 0,
                         'tms': int(tm.group(1)) if tm else 0})
    if segs:
        g.send_all({'type': 'flash_segments', 'segments': segs})


async def _flash_export_chunked_async(ws: WebSocket, start: int, count: int):
    """分片导出：DAP-Link VCP 2M 下大流量丢字节（实测 32 页+ 必丢），
    分片 16 页（32KB）摊薄每片固定开销（命令往返+状态机 ~0.5-1s），
    超时 2s+0.02s/页（8KB 实际 65ms 即传完，5s 基数是浪费）。
    ★ 2026-08-09 优化：CHUNK 4→16、超时 5→2s——512 页导出从 5-10 分钟降到 ~1 分钟。
    每片按 n×2048B 长度校验，不足重试 3 次。消费线程负责 feed，本函数仅等待完成标志。"""
    CHUNK = 16
    total = bytearray()
    page, remaining = start, count
    while remaining > 0:
        n = min(remaining, CHUNK)
        got = None
        for attempt in range(3):
            done = threading.Event()
            piece = []
            g.dbgs.start_export(page, n, lambda raw, p=piece, d=done: (p.append(raw), d.set()))
            await asyncio.to_thread(done.wait, 2 + n * 0.02)
            if piece and len(piece[0]) == n * 2048:
                got = piece[0]
                break
        if got is None:
            g.send_all({'type': 'log', 'level': 'warn',
                        'msg': f'片 {page}..{page + n - 1} 导出失败（链路丢包，3 次重试后放弃）'})
            break
        total.extend(got)
        page += n
        remaining -= n
        g.send_all({'type': 'flash_progress', 'done': len(total) // 2048})
    if total:
        _on_flash_export_done(bytes(total))
    else:
        g.send_all({'type': 'log', 'level': 'warn', 'msg': '导出失败：无数据'})


def _on_flash_export_done(raw: bytes):
    """export 完成：切帧解析 → 段/行/列 → 前端"""
    frames = bb.extract_frames(bytes(raw))
    g.send_all({'type': 'flash_progress', 'done': len(raw) // 2048})
    if not frames:
        g.send_all({'type': 'log', 'level': 'warn', 'msg': '导出数据无有效帧'})
        return
    rows, segments, columns = bb.decode_frames(frames)
    # 抽样预览（前端绘图/表格用，最多 3000 行）
    step = max(1, len(rows) // 3000)
    preview = [[r[0], r[1]] + [round(v, 4) for v in r[2]] + [r[3]] for r in rows[::step]]
    g.send_all({'type': 'flash_done',
                'rows': preview,
                'row_count': len(rows),
                'sample_step': step,
                'columns': columns,
                'segments': segments})


def _param_id_of(msg: dict):
    """param_read/param_write 参数寻址：name 优先（expected_names 下标 = 固件注册序
    = 参数 ID），无 name 时按 id。返回 (pid, name)；未知名字返回 (None, name)。
    ★ 2026-08-09：按名字寻址避免 ID 猜错（曾把 rate_roll.ki 当 rate_pitch.kp 写）"""
    name = msg.get('name')
    if name:
        try:
            return pm.expected_names().index(name), name
        except ValueError:
            return None, name
    return int(msg.get('id', 0)), None


async def _param_read_all(ws: WebSocket):
    """拉取固件全部 117 参数（逐 ID 发送 + 等待该 ID 双全，超时重试。
    ★ 2M 速率下 USB 串口偶发丢字节（固件 CRC 失败静默丢弃）——
      批量灌发会系统性丢参数，必须逐 ID 确认）"""
    if not (g.link and g.link.is_open):
        return
    g.param_names = {}
    g.param_values = {}
    failed = []
    for pid in range(117):
        if not await _param_read_single_async(pid):
            failed.append(pid)
    # 补拉循环
    for _rnd in range(3):
        if not failed:
            break
        failed = [pid for pid in failed if not await _param_read_single_async(pid)]
    await ws.send_json({'type': 'log',
                        'msg': f'参数读取完成：{117 - len(failed)}/117' +
                               (f'（失败 {failed}）' if failed else '')})


async def _param_read_single_async(pid, timeout=1.5, tries=3):
    for _attempt in range(tries):
        g.write(anocom.cmd_read_param_info(pid))
        g.write(anocom.cmd_read_param_value(pid))
        deadline = asyncio.get_running_loop().time() + timeout
        while asyncio.get_running_loop().time() < deadline:
            await asyncio.sleep(0.05)
            if pid in g.param_names and pid in g.param_values:
                return True
    return False


# ========================================================================
#  WebSocket 端点
# ========================================================================

@app.websocket('/ws')
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    g.ws_clients.add(ws)
    g._loop = asyncio.get_running_loop()
    try:
        # 版本握手：前端据此发现"新页面配旧后端"（进程残留）并提示重启
        await ws.send_json({'type': 'hello', 'version': app.version})
        # 连接状态快照
        if g.link and g.link.is_open:
            await ws.send_json({'type': 'conn', 'status': 'connected',
                                'port': g.link.port, 'baud': g.link.baud})
        while True:
            msg = await ws.receive_json()
            try:
                await handle_ws_message(ws, msg)
            except Exception as exc:
                log.warning('cmd %r failed: %r', msg.get('cmd'), exc)
                await ws.send_json({'type': 'log', 'level': 'error', 'msg': str(exc)})
    except WebSocketDisconnect:
        pass
    finally:
        g.ws_clients.discard(ws)


@app.get('/api/status')
def api_status():
    """后端服务状态总览（前端顶栏/外部探活用）"""
    return {
        'ok': True,
        'version': app.version,
        'uptime_s': round(time.time() - START_TS, 1),
        'ws_clients': len(g.ws_clients),
        'serial': {
            'connected': bool(g.link and g.link.is_open),
            'port': g.link.port if g.link else None,
            'baud': g.link.baud if g.link else None,
            'mode': g.mode,
        },
        'stats': {
            'bytes_total': g.stat_bytes,
            'frames_total': g.stat_frames,
            'crc_bad_total': g.stat_bad,
            'tele_age_s': round(time.time() - g.last_tele_ts, 1) if g.last_tele_ts else None,
        },
    }


@app.get('/download/{name}')
def download(name: str):
    """下载 output/ 下的 CSV（记录或黑匣子导出）"""
    safe = Path(name).name
    p = OUTPUT_DIR / safe
    if not p.exists():
        return JSONResponse({'error': 'not found'}, status_code=404)
    return FileResponse(str(p), filename=safe)


# 静态前端（开发阶段禁用缓存：浏览器刷新即取最新 JS，避免改代码后白屏）
app.mount('/', StaticFiles(directory=str(WEB_DIR), html=True), name='web')


@app.middleware('http')
async def no_cache_middleware(request, call_next):
    resp = await call_next(request)
    resp.headers['Cache-Control'] = 'no-store'
    return resp


@app.on_event('startup')
async def startup():
    g._loop = asyncio.get_running_loop()
    asyncio.create_task(_stat_loop())
    log.info('GCS 后端启动完成（状态接口 /api/status）')


if __name__ == '__main__':
    import uvicorn
    uvicorn.run(app, host='127.0.0.1', port=8091)
