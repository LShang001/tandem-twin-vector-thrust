# -*- coding: utf-8 -*-
"""
main.py — GCS 服务入口：FastAPI + WebSocket + pyserial

@file    main.py
@module  GCS/后端服务（FastAPI + WebSocket + 串口链路）
@brief   把串口 AnoCom 帧流解码为遥测快照，经 WS/HTTP 提供给前端与 CLI
@details 输入：串口读线程字节流、WS 客户端命令；处理：切帧→解码→聚合快照→
         按 TELEMETRY_PERIOD 节流广播，并驱动参数/变量/黑匣子/DBG 状态机；
         输出：WS 消息（hello/conn/telemetry/link_stat/param_*/vars_*/dbg_*/flash_*）、
         /api/status 与 /download。
         不变量：app.version 必须与前端 web/src/main.mjs 的 BACKEND_VERSION 一致；
         telemetry 与 dbg 模式互斥（固件行为，进入 DBG 后固件停发遥测）。
@theory  N/A
@ref     PROTOCOL-001 §2/§4、COMM-001 §3（推送 50Hz）、COMM-001 §4 C 层（read_size 256）、
         GCS-001 §2（参数在线读写）
@frame   N/A（姿态/速度/位置由 anocom.py 解码，速度与高度已为"向上为正"）
@units   快照字段自带单位后缀（_deg/_dps/_ms2/_ms/_m/_pct/_v），原样透传前端
@note    ★ 改后端后必须杀掉 8091 残留进程（AGENTS.md 踩坑记录）：WS hello 握手比对版本
@see     server/anocom.py、server/params.py、server/blackbox.py、web/src/main.mjs

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
import math
import os
import queue
import struct
import threading
import time
from pathlib import Path

from fastapi import FastAPI, File, UploadFile, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

# 相对导入（python -m uvicorn server.main:app）；直接运行本文件时回退绝对导入
try:
    from . import anocom
    from . import blackbox as bb
    from . import params as pm
    from .alarms import AlarmEngine
    from .calibration import MAG_WIZARD, assess_magcal, parse_magcal_line
    from .datalog import CsvRecorder, ReplaySource, TlogReplaySource, list_recordings
    from .serial_link import SerialLink, list_ports
    from .mavlink_service import MavlinkService
except ImportError:
    import anocom
    import blackbox as bb
    import params as pm
    from alarms import AlarmEngine
    from calibration import MAG_WIZARD, assess_magcal, parse_magcal_line
    from datalog import CsvRecorder, ReplaySource, TlogReplaySource, list_recordings
    from serial_link import SerialLink, list_ports
    from mavlink_service import MavlinkService

log = logging.getLogger('gcs')
logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(name)s: %(message)s')

ROOT = Path(__file__).resolve().parent.parent
WEB_DIR = ROOT / 'web'
OUTPUT_DIR = ROOT / 'output'
# 瓦片缓存目录：地图底图按 z/x/y 落盘，可离线复用
TILE_DIR = OUTPUT_DIR / 'tiles'
# 瓦片源：CartoCDN 深色底图。实测 OSM 官源与 OpenTopoMap 在本机都超时，
# CartoCDN 可达且本身就是深色，贴合工程风。坐标系是标准 Web Mercator/WGS-84，
# 没有 GCJ-02 偏移（国内厂商瓦片有，会与飞控坐标差几十米）。
OUTPUT_DIR.mkdir(exist_ok=True)

# ---- 瓦片源配置（2026-11-09 实测后的结论）----
# 实测：OSM 官源 / OpenTopoMap / openstreetmap.fr / openstreetmap.bzh 在本机
# 全部超时或 404；wikimedia 403；CartoCDN 能连但返回 "API KEY REQUIRED"
# 占位图。即**国内环境没有可直接用的境外免费瓦片源**，这不是代码能绕过的。
#
# 因此瓦片源做成可配置，三条路都留好：
#   ① GCS_TILE_SOURCE=tianditu + GCS_TILE_KEY=<天地图 key>
#      天地图是国内官方源，坐标系 CGCS2000（与 WGS-84 差米级，地面站可接受）。
#      key 免费申请：https://console.tianditu.gov.cn/
#   ② GCS_TILE_SOURCE=<自定义 URL 模板，含 {z}/{x}/{y}>
#   ③ 都不配 → 只用本地缓存（把离线瓦片包按 z/x/y 放进 output/tiles/ 即可），
#      没有缓存时地图是灰底 + 航点照常工作。
TILE_SOURCE = os.environ.get('GCS_TILE_SOURCE', '').strip()
TILE_KEY = os.environ.get('GCS_TILE_KEY', '').strip()

TIANDITU_TMPL = ('https://t{s}.tianditu.gov.cn/vec_w/wmts?SERVICE=WMTS'
                 '&REQUEST=GetTile&VERSION=1.0.0&LAYER=vec&STYLE=default'
                 '&TILEMATRIXSET=w&FORMAT=tiles'
                 '&TILEMATRIX={z}&TILEROW={y}&TILECOL={x}&tk={key}')


def _tile_url(z: int, x: int, y: int):
    """@brief 按配置生成瓦片 URL；未配置源时返回 None（只用本地缓存）"""
    if not TILE_SOURCE:
        return None
    if TILE_SOURCE == 'tianditu':
        if not TILE_KEY:
            return None
        return (TIANDITU_TMPL
                .replace('{s}', str((x + y) % 8))
                .replace('{z}', str(z)).replace('{x}', str(x)).replace('{y}', str(y))
                .replace('{key}', TILE_KEY))
    if '{z}' in TILE_SOURCE and '{x}' in TILE_SOURCE and '{y}' in TILE_SOURCE:
        return (TILE_SOURCE.replace('{z}', str(z))
                .replace('{x}', str(x)).replace('{y}', str(y))
                .replace('{key}', TILE_KEY))
    return None



DEFAULT_BAUD = 921600      # = 固件 SERIAL6_BAUDRATE（★ 2026-08-10 COMM-001：2M→921600）
TELEMETRY_PERIOD = 0.02    # 遥测推送节流 50Hz（★ COMM-001：20Hz→50Hz 跟上快环 100Hz）
PARAM_WRITE_TIMEOUT = 2.0   # 参数写入校验帧超时
TELE_BUF_CAP = 8192         # 无完整帧时遥测缓冲上限（防乱码流 O(n²) 膨胀卡死）
STAT_PERIOD = 1.0           # 链路统计推送周期
START_TS = time.time()      # 后端启动时间（/api/status 用）
HEX_FLUSH_PERIOD = 0.35     # hex 监听推送节流
HEX_FLUSH_MAX = 512         # 单次 hex 推送最大字节

# PWA 清单的 MIME 类型：Python 的 mimetypes 不认 .webmanifest，
# 不注册的话 StaticFiles 会按 application/octet-stream 返回，
# 部分浏览器会因此拒绝把页面识别为可安装的 PWA。
import mimetypes as _mimetypes

_mimetypes.add_type('application/manifest+json', '.webmanifest')
_mimetypes.add_type('text/javascript', '.js')
_mimetypes.add_type('text/javascript', '.mjs')

app = FastAPI(title='UniGNC GCS', version='0.3.0')


class GCS:
    """全局状态机：串口 + 模式 + 遥测聚合 + 参数客户端 + 记录/回放"""

    def __init__(self):
        self.link = None
        self.protocol = 'anocom'   # anocom | mavlink
        self.mav = None
        self.mode = 'telemetry'          # telemetry | dbg
        self.dbgs = None
        self.tele_buf = bytearray()      # AnoCom 原始流缓冲
        self.snap = {}                   # 最新遥测快照（键 = datalog.RECORD_FIELDS + 附加）
        # 报警引擎放服务端：告警必须与当前页面无关（切页就丢是最危险的失效），
        # 事件历史也要能持久保存，不能只活在前端内存里。
        self.alarms = AlarmEngine()
        # 统一时序缓冲（双协议通用，供阶跃响应分析/事后查数用）。
        # 为什么不用 mav.hist：那是 MAVLink 专属，AnoCom 模式下就没有；
        # 调参界面必须在两种协议下都能用。每条 deque 有界，总量可控。
        self.hist = {}
        self.hist_start = time.time()
        # 磁标定向导状态：累积固件回传的字段，供判读使用
        self.calib = {}
        # 回放倍速：0=最快（纯灌数据给分析），1.0=原始速率，2.0=两倍
        self.replay_speed = 0.5
        self.calib_step = 0
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
    def connect(self, port, baud, consume_thread: bool = True, protocol: str = 'anocom'):
        # 幂等：同一端口/协议/波特率已连接时直接复用，避免多标签页或重复点击
        # 的第二次 connect 把第一次的串口会话断开。
        protocol = protocol if protocol in ('anocom', 'mavlink') else 'anocom'
        if (self.link and self.link.is_open and
                getattr(self.link, 'port', None) == port and
                getattr(self.link, 'baud', None) == baud and
                self.protocol == protocol):
            return
        self.disconnect()
        self.protocol = protocol
        self.link = SerialLink(port, baud, self._on_rx)
        self.link.connect()
        self.mav = MavlinkService() if self.protocol == 'mavlink' else None
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

    def mav_pump(self):
        """把 MavlinkService 积累的 TX 字节真正写到串口。"""
        if self.mav is not None and self.link and self.link.is_open:
            data = self.mav.drain_tx()
            if data:
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
            # 顺手喂给校准解析器：向导不需要用户在控制台里自己找数字，
            # 而是把这些行变成进度与判读结论。
            try:
                if parse_magcal_line(line, g.calib):
                    g.send_all({'type': 'calib_state',
                                'data': dict(g.calib),
                                'assess': assess_magcal(g.calib)})
            except Exception:
                pass


def _record_hist(snap: dict, ts: float | None = None):
    """@brief 把一份快照写进时序缓冲 g.hist（实时与回放共用同一入口）

    @details 为什么必须共用：频谱/调参/选区统计全都读 g.hist。如果回放不写，
             那"导入 tlog 做频域分析"这条路根本走不通——数据只到 g.snap
             给界面看，分析类功能全空。抽成一个函数就不会漏。

    @note 默认用 time.time()，但**回放时必须传记录的原始时间戳**：
          回放是快速推进的墙钟，若用 time.time()，一秒钟的墙钟里灌进几千帧，
          算出来的采样率会是几百 Hz 的假值，频域分析全错。
          用原始时间戳则 fs 反映的是**记录时的真实采样率**，这才是分析要的。

          （早期版本用"距启动秒数"，后来为了让曲线页历史对齐改成 Unix 秒；
           回放传原始 t_ms 也是 Unix 秒，两者同量级，能混在一起画。）
    """
    try:
        now = time.time() if ts is None else float(ts)
        for k, v in snap.items():
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                continue
            dq = g.hist.get(k)
            if dq is None:
                from collections import deque as _deque
                # ★ 容量必须够装下"一次完整的回放"。原值 4000 按 200 Hz 只有
                #   20 s，看似够；但回放是**快速灌数据**，一份 40 s 的日志就是
                #   8000 帧，前一半会被静默挤掉——阶跃发生在开头时就再也找不到，
                #   表现为"阶跃数 0 个"且"低频能量 0%"，两个症状同源（实测踩过，
                #   为此查了四轮）。
                #   20000 @ 200 Hz = 100 s；实时 50 Hz 下约 400 s，内存约
                #   320 KB/通道，可接受。
                dq = _deque(maxlen=20000)
                g.hist[k] = dq
            dq.append((now, float(v)))
    except Exception:
        pass


def _on_telemetry_rx(chunk: bytes):
    g.stat_bytes += len(chunk)
    if g.protocol == 'mavlink' and g.mav is not None:
        g.mav.feed(chunk)
        g.mav_pump()
        if g.mav.snap:
            g.snap.update(g.mav.snap)
            g.last_tele_ts = time.time()
            g.snap['t_ms'] = time.time() * 1000
            g.stat_tele += 1
            for ev in g.mav.drain_param_events():
                g.send_all({'type': 'param_count', 'count': ev['count']})
                g.send_all({'type': 'param_info', 'id': ev['id'],
                            'type': 'float', 'name': ev['name']})
                g.send_all({'type': 'param_value', 'id': ev['id'],
                            'name': ev['name'], 'value': ev['value']})
                # MAVLink 的 PARAM_VALUE 回显即写入确认；AnoCom 有独立校验帧。
                g.send_all({'type': 'param_written', 'id': ev['id'],
                            'name': ev['name'], 'value': ev['value']})
            if g.recorder and g.recorder.active and not g.replay:
                g.recorder.write(g.snap)
            now = time.time()
            # 报警引擎：每条新快照都喂，但事件只在状态跳变时产生，
            # 所以 50 Hz 喂入不会产生 50 Hz 推送。
            _prev = getattr(g, '_alarm_prev_ts', 0)
            try:
                evs = g.alarms.feed(g.snap,
                                    link_age_s=(now - _prev) if _prev else None,
                                    now=now)
            except Exception:
                evs = []
            g._alarm_prev_ts = now
            _record_hist(g.snap)
            for ev in evs:
                g.send_all({'type': 'alarm', **ev})
            last = getattr(g, '_last_tele_push', 0)
            if now - last >= TELEMETRY_PERIOD:
                g._last_tele_push = now
                g.send_all({'type': 'telemetry', 'data': g.snap})
        return
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
            'sysid': (g.mav.active_sys if g.mav is not None else None),
            'n_vehicles': (len(g.mav.snaps) if g.mav is not None else 0),
            # 任务上传是请求-应答式，状态必须回传给界面，否则用户不知道
            # 卡在"等飞控来拉"还是"飞控已拒绝"
            'mission': ({
                'phase': g.mav.mission_phase,
                'ack': g.mav.mission_ack,
                'want': len(g.mav.mission_items),
            } if g.mav is not None else None),
        })
        prev = cur


# ---------------- 参数响应处理 ----------------
def _on_param_info(f: anocom.Frame):
    """E2 信息帧：payload = [ID u16, TYPE u8, NAME N]
    NAME 为变长（官方协议 N=实际长度；老版上位机上限 16、新版上限 20），
    统一按 \x00 截断兜底，兼容定长补 0 与变长两种发法。"""
    p = f.payload
    if len(p) < 3:
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
    if p and p[0] == 0x01 and len(p) >= 3:
        # 官方协议 VAL 为 u16（3 字节 DATA）；兼容早期 5 字节 u32 版本
        count = int.from_bytes(p[1:5], "little") if len(p) >= 5 else p[1] | (p[2] << 8)
        g.param_count_seen = count
        g.send_all({'type': 'param_count', 'count': count})


def _on_param_check(f: anocom.Frame):
    """0x00 校验帧：匹配参数写入确认"""
    p = f.payload
    if len(p) < 3:
        return
    id_get, sc_get, ac_get = p[0], p[1], p[2]
    now = time.time()
    # ★ 2026-08-10 全面审查修复：校验帧丢失（2M 丢帧/固件 DBG）时条目永不清理——
    #   顺带清理超时条目（PARAM_WRITE_TIMEOUT 此前定义但从未使用）
    for pid, (exp_sc, exp_ac, t0) in list(g.param_pending.items()):
        if now - t0 > PARAM_WRITE_TIMEOUT:
            g.param_pending.pop(pid, None)
            g.send_all({'type': 'param_written', 'id': pid, 'ok': False})
            continue
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
    （字段名 vars_<name>，随 50Hz telemetry 推送 + record 落盘）。
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
    """@brief 按 msg['cmd'] 分派一条 WS 客户端命令
    @param ws 该客户端连接；msg 已解析的 JSON 命令（connect/disconnect/param_*/
           vars_*/flash_*/dbg_*/record_*/replay_* 等）
    @note  命令处理异常由 ws_endpoint 捕获并回 log 消息，不关闭连接
    """
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
        # 演示模式下拒绝连真实串口：合成遥测与真实链路共用 g.protocol/g.mav，
        # 一旦被改成 anoCom，DEMO 的 MAVLink 帧就会走 AnoCom 解析路径而全部解析失败，
        # 表现为「数据还在流、波形却冻住」——这类静默降级极难排查，所以直接挡住。
        if DEMO_MODE:
            await ws.send_json({
                'type': 'conn', 'status': 'error',
                'reason': '当前是演示模式（GCS_DEMO=1）：请重启去掉该环境变量后再连真实设备',
            })
            return
        port, baud = msg.get('port'), int(msg.get('baud') or DEFAULT_BAUD)
        try:
            await asyncio.to_thread(g.connect, port, baud, True, str(msg.get('protocol', 'anocom')))
        except Exception as exc:
            await ws.send_json({'type': 'conn', 'status': 'error', 'reason': str(exc)})
            return
        await ws.send_json({'type': 'conn', 'status': 'connected', 'port': port, 'baud': baud, 'protocol': g.protocol})
        log.info('connected %s @ %d', port, baud)

    elif cmd == 'calib_wizard':
        await ws.send_json({'type': 'calib_wizard', 'steps': MAG_WIZARD})

    elif cmd == 'calib_step':
        step_id = str(msg.get('id') or '')
        step = next((s for s in MAG_WIZARD if s['id'] == step_id), None)
        if step is None:
            await ws.send_json({'type': 'log', 'level': 'error',
                                'msg': f'未知步骤 {step_id}'})
            return
        # 清空上一轮累积：否则上一次的 samples 会和这一次混在一起判读
        g.calib = {}
        await _dbg_ensure()
        g.dbg_cmd(step['cmd'])
        await ws.send_json({'type': 'calib_progress', 'id': step_id,
                            'phase': 'sent', 'cmd': step['cmd']})
        log.info('calib step %s -> %s', step_id, step['cmd'])

    elif cmd == 'calib_assess':
        await ws.send_json({'type': 'calib_assess',
                            'data': dict(g.calib),
                            'assess': assess_magcal(g.calib)})

    elif cmd == 'calib_reset':
        g.calib = {}
        await ws.send_json({'type': 'calib_state', 'data': {},
                            'assess': assess_magcal({})})

    elif cmd == 'spectrum':
        ch = str(msg.get('channel') or '')
        nperseg = int(msg.get('nperseg') or 256)
        dq = g.hist.get(ch)
        if dq is None or len(dq) < 64:
            await ws.send_json({'type': 'spectrum', 'channel': ch,
                                'ok': False, 'reason': f'通道 {ch} 样本不足'})
            return
        from .spectrum import spectrum_summary
        try:
            r = spectrum_summary(list(dq), nperseg=nperseg)
        except Exception as exc:
            await ws.send_json({'type': 'spectrum', 'channel': ch,
                                'ok': False, 'reason': str(exc)})
            return
        await ws.send_json({'type': 'spectrum', 'channel': ch, **r})

    elif cmd == 'spectrum_multi':
        names = msg.get('channels') or []
        nperseg = int(msg.get('nperseg') or 256)
        chans = {}
        for n in names:
            dq = g.hist.get(n)
            if dq is not None and len(dq) >= 64:
                chans[n] = list(dq)
        from .spectrum import compare_channels
        try:
            r = compare_channels(chans, nperseg=nperseg)
        except Exception as exc:
            await ws.send_json({'type': 'spectrum_multi', 'ok': False,
                                'reason': str(exc)})
            return
        await ws.send_json({'type': 'spectrum_multi', **r})

    elif cmd == 'spectrogram':
        ch = str(msg.get('channel') or '')
        nperseg = int(msg.get('nperseg') or 128)
        hop = int(msg.get('hop') or 64)
        max_cols = int(msg.get('max_cols') or 160)
        dq = g.hist.get(ch)
        if dq is None or len(dq) < 128:
            await ws.send_json({'type': 'spectrogram', 'channel': ch,
                                'ok': False, 'reason': f'通道 {ch} 样本不足'})
            return
        from .spectrum import spectrogram
        try:
            r = spectrogram(list(dq), nperseg=nperseg, hop=hop, max_cols=max_cols)
        except Exception as exc:
            await ws.send_json({'type': 'spectrogram', 'channel': ch,
                                'ok': False, 'reason': str(exc)})
            return
        await ws.send_json({'type': 'spectrogram', 'channel': ch, **r})

    elif cmd == 'scope_history':
        # 曲线页的数据源改走后端：前端自己攒的历史一切页/刷新就没了，
        # 而频谱/调参页用的都是 g.hist。共用一个源，才能「先看波形再切去分析」。
        chans = msg.get('channels') or []
        series = {}
        for n in chans:
            dq = g.hist.get(n)
            if dq:
                # 只回最近 2000 点（约 60 s@32Hz），够铺满 60 s 窗口
                series[n] = [[round(float(tt), 3), float(vv)]
                             for tt, vv in list(dq)[-2000:]]
        await ws.send_json({'type': 'scope_history', 'series': series})

    elif cmd == 'scope_stats':
        ch = str(msg.get('channel') or '')
        dq = g.hist.get(ch)
        if dq is None or len(dq) < 4:
            await ws.send_json({'type': 'scope_stats', 'channel': ch,
                                'ok': False, 'reason': f'通道 {ch} 样本不足'})
            return
        from .stats import series_stats
        series = list(dq)
        try:
            r = series_stats(series, msg.get('t0'), msg.get('t1'))
        except Exception as exc:
            await ws.send_json({'type': 'scope_stats', 'channel': ch,
                                'ok': False, 'reason': str(exc)})
            return
        # 顺手把该区间的频谱主峰也算出来：现场问"这段抖得厉害"时，
        # 下一个问题必然是"抖在哪个频率"——时域强度和频域来源要在一屏里。
        # 用同一段数据、同一次请求，避免用户再切页面、再选一次区间。
        if r.get('ok'):
            try:
                from .spectrum import spectrum_summary
                t0 = r['t0']
                t1 = r['t1']
                seg = [(tt, vv) for tt, vv in series if t0 <= tt <= t1]
                # 窗长按选区长度自适应：默认 256，但选区短时缩到 len//2，
                # 否则短选区永远报"样本不足"，而短选区恰恰是现场最常用的
                # （看到一个毛刺想立刻知道它是什么频率）。
                # 下限 64：再小频率分辨率太差，主频没有参考价值。
                nperseg = max(64, min(256, len(seg) // 2))
                sp = spectrum_summary(seg, nperseg=nperseg)
                if sp.get('ok'):
                    peaks = sp.get('peaks') or []
                    banks = sp.get('bands') or {}
                    low = banks.get('low_lt1hz_pct') or 0.0
                    # ★ 关键判断：如果绝大部分能量在 <1 Hz，说明主成分**低于频率
                    #   分辨率**（分辨率 = fs/nperseg ≈ 0.13 Hz）。这时第一个
                    #   局部峰其实是次峰，报出来会严重误导——实测 pitch_deg 的
                    #   0.072 Hz 慢摆被报成 4.9 Hz。宁可报"低频主导"也不给错频率。
                    if low >= 80.0:
                        r['main_freq_hz'] = None
                        r['dominant_band'] = 'low'
                        r['main_freq_note'] = (
                            f'能量 {low:.1f}% 在 <1 Hz（低于 '
                            f'{sp.get("resolution_hz")} Hz 分辨率，主成分是慢漂）')
                        r['main_freq_share_pct'] = low
                    else:
                        r['main_freq_hz'] = peaks[0]['freq_hz'] if peaks else None
                        r['main_freq_share_pct'] = peaks[0]['share_pct'] if peaks else None
                        r['dominant_band'] = 'peak'
                    r['bands'] = banks
                    r['spectrum_fs'] = sp.get('fs')
                else:
                    r['spectrum_reason'] = sp.get('reason')
            except Exception as exc:
                r['spectrum_reason'] = str(exc)
        await ws.send_json({'type': 'scope_stats', 'channel': ch, **r})

    elif cmd == 'tuning_channels':
        # 列出可用于分析的通道（有足够样本的）
        chans = []
        for k, dq in sorted(g.hist.items()):
            if len(dq) >= 20:
                chans.append({'name': k, 'n': len(dq)})
        await ws.send_json({'type': 'tuning_channels', 'channels': chans})

    elif cmd == 'tuning_analyze':
        cmd_ch = str(msg.get('cmd_ch') or '')
        fb_ch = str(msg.get('fb_ch') or '')
        min_delta = float(msg.get('min_delta') or 0.0)
        window_s = float(msg.get('window_s') or 3.0)
        if cmd_ch not in g.hist or fb_ch not in g.hist:
            await ws.send_json({'type': 'tuning_result',
                                'error': f'通道不存在或样本不足：{cmd_ch} / {fb_ch}'})
            return
        from .stepresponse import analyze_pair
        try:
            out = analyze_pair(list(g.hist[cmd_ch]), list(g.hist[fb_ch]),
                               min_delta=min_delta, window_s=window_s)
        except Exception as exc:
            await ws.send_json({'type': 'tuning_result', 'error': str(exc)})
            return
        # 回传一段原始波形，前端画图用（截取最近 20 s）
        waves = {}
        for name in (cmd_ch, fb_ch):
            dq = g.hist.get(name) or []
            waves[name] = [[t, v] for t, v in list(dq)[-4000:]]

        # ★ 顺带对反馈通道做一次频谱，把"超调是被什么污染的"直接算出来。
        #   动机：超调偏大有两条完全相反的处置——控制律增益过大（该降 kp），
        #   或结构/电气振动把峰值抬起来（降 kp 只会让系统变慢、问题依旧）。
        #   让用户自己去别的页面看频谱再拼结论，是把这个判断推给了人；
        #   这里直接在阶跃结果里给出候选频段。
        #   只报 >2 Hz 的峰：低频峰是阶跃响应本身的频谱成分，不是污染。
        try:
            from .spectrum import spectrum_summary
            fb_series = list(g.hist[fb_ch] or [])
            if len(fb_series) >= 128:
                sp = spectrum_summary(fb_series, nperseg=min(512, max(64, len(fb_series) // 4)))
                if sp.get('ok'):
                    cand = [pk for pk in (sp.get('peaks') or [])
                            if pk['freq_hz'] >= 2.0 and pk['share_pct'] >= 0.01]
                    if cand:
                        top = cand[0]
                        out['vibration'] = {
                            'freq_hz': top['freq_hz'],
                            'share_pct': top['share_pct'],
                            'n_candidates': len(cand),
                            'candidates': cand[:3],
                            'hint': (f'该段数据在 {top["freq_hz"]:.1f} Hz 有清晰成分'
                                     f'（占 {top["share_pct"]}%），超调可能被它抬高了'
                                     f'——先排除结构/电气振动，再考虑降增益'),
                        }
                    out['spectrum_bands'] = sp.get('bands')
        except Exception:
            pass
        await ws.send_json({'type': 'tuning_result', **out, 'waves': waves})

    elif cmd == 'alarm_state':
        await ws.send_json({'type': 'alarm_state', **g.alarms.snapshot()})

    elif cmd == 'alarm_clear_events':
        g.alarms.clear_events()
        await ws.send_json({'type': 'alarm_state', **g.alarms.snapshot()})

    elif cmd == 'vehicles':
        await ws.send_json({
            'type': 'vehicles',
            'vehicles': (g.mav.vehicles() if g.mav is not None else []),
        })

    elif cmd == 'vehicle_select':
        sysid = int(msg.get('sysid') or 0)
        ok = (g.mav is not None and g.mav.select_vehicle(sysid))
        if ok:
            # 清空旧机的通用状态：hist/calib 都是按"当前机"累积的，
            # 不清会让切换后的曲线里混进上一台的数据
            g.hist = {}
            g.hist_start = time.time()
            g.calib = {}
            g.alarms.reset()
            g.snap = dict(g.mav.snap)
        await ws.send_json({'type': 'vehicle_selected', 'sysid': sysid, 'ok': ok})

    elif cmd == 'mission_upload':
        if g.mav is None:
            await ws.send_json({'type': 'error',
                                'msg': 'MAVLink 未连接（任务上传需要 MAVLink 模式）'})
            return
        items = msg.get('items') or []
        if not items:
            await ws.send_json({'type': 'error', 'msg': '航点为空'})
            return
        n = g.mav.mission_upload(items)
        await ws.send_json({'type': 'mission_state', 'phase': 'count_sent', 'count': n})
        log.info('mission upload requested: %d waypoints', n)

    elif cmd == 'mission_clear':
        if g.mav is not None:
            g.mav.mission_clear()
        await ws.send_json({'type': 'mission_state', 'phase': 'cleared', 'count': 0})

    elif cmd == 'inspector':
        # MAVLink Inspector：消息清单（类型/帧数/频率/多久没更新/字段值）。
        # 不在这里带历史序列——字段历史总量随消息种类线性增长，
        # 一次全塞进 WS 会让低频客户端也被大包拖住，历史另走 inspector_series。
        if g.mav is None:
            await ws.send_json({'type': 'error', 'msg': 'MAVLink 未连接（Inspector 需要 MAVLink 模式）'})
            return
        data = g.mav.inspector()
        await ws.send_json({
            'type': 'inspector',
            'messages': data['messages'],
            'elapsed_s': data['elapsed_s'],
            'frames': data['frames'],
        })

    elif cmd == 'inspector_series':
        keys = msg.get('keys') or []
        if g.mav is None or not keys:
            await ws.send_json({'type': 'inspector_series', 'series': {}})
            return
        full = g.mav.inspector()['series']
        picked = {k: full.get(k, []) for k in keys}
        await ws.send_json({'type': 'inspector_series', 'series': picked})

    elif cmd == 'mav_param_list':
        if g.mav is None:
            await ws.send_json({'type': 'error', 'msg': 'MAVLink 未连接'})
        else:
            g.mav.request_params()
            g.mav_pump()
            await ws.send_json({'type': 'mav_tx', 'cmd': 'param_list'})
    elif cmd == 'mav_param_set':
        if g.mav is None:
            await ws.send_json({'type': 'error', 'msg': 'MAVLink 未连接'})
        else:
            g.mav.set_param(str(msg.get('name', '')), float(msg.get('value', 0)))
            g.mav_pump()
            await ws.send_json({'type': 'mav_tx', 'cmd': 'param_set', 'name': msg.get('name')})
    elif cmd == 'mav_cmd':
        if g.mav is None:
            await ws.send_json({'type': 'error', 'msg': 'MAVLink 未连接'})
        else:
            g.mav.command_long(int(msg.get('command', 0)),
                              float(msg.get('p1', 0)), float(msg.get('p2', 0)),
                              float(msg.get('p3', 0)), float(msg.get('p4', 0)),
                              float(msg.get('p5', 0)), float(msg.get('p6', 0)),
                              float(msg.get('p7', 0)))
            g.mav_pump()
            await ws.send_json({'type': 'mav_tx', 'cmd': 'command', 'command': msg.get('command')})
    elif cmd == 'mav_status':
        await ws.send_json({'type': 'mav_status', 'connected': g.mav is not None,
                            'params': len(g.mav.params) if g.mav else 0,
                            'frames': g.mav.frames if g.mav else 0,
                            'snap': g.mav.snap if g.mav else {}})
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
        # ★ 2026-11-06 审计修复：WebSocket 写参数也要过包线。
        #   此前只有 CLI（cli.py::_param_set）和前端（params.mjs）做校验，后端直接转发；
        #   固件侧 anoParamDeserialize 只拒 NaN/Inf、不做 min/max，超范围值会真的写进
        #   RAM 并立刻生效（如 sedf.vane_max_deg 直接改变控制权限）。
        #   这里复用 params.py 的 META，与 CLI 同一份规则。
        meta = pm.meta_for(name or '') or {}
        if 'min' in meta and 'max' in meta:
            try:
                v = float(value)
            except (TypeError, ValueError):
                await ws.send_json({'type': 'log', 'level': 'error',
                                    'msg': f'{name} 需要数值，收到 {value!r}'})
                return
            if v < meta['min'] or v > meta['max']:
                await ws.send_json({'type': 'log', 'level': 'warn',
                                    'msg': f'{name}={v} 超出包线 [{meta["min"]}, {meta["max"]}]，未发送'})
                return
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
        # 支持两种来源：①本机录制目录里的文件（原来只支持这种）；
        # ②外部导入的 tlog（QGC / Mission Planner 录的），可用绝对路径。
        # 外部路径只做存在性检查，不做目录限制——这是本地单用户工具，
        # 限制目录只会让"导入我自己录的日志"变得别扭。
        raw = Path(fn)
        if raw.is_absolute() and raw.is_file():
            path = raw
        else:
            path = OUTPUT_DIR / fn
        if not path.exists():
            await ws.send_json({'type': 'log', 'level': 'error', 'msg': f'文件不存在: {fn}'})
            return
        is_tlog = path.suffix.lower() in ('.tlog', '.bin')
        g._replay_n = 0
        # 开始回放前清空时序缓冲：不清的话上一段（可能是实时数据）会和
        # 这次回放的文件混在同一个缓冲里，频谱/统计结果没有意义。
        g.hist = {}
        g.hist_start = time.time()
        try:
            g.replay = TlogReplaySource(str(path)) if is_tlog else ReplaySource(str(path))
        except Exception as exc:
            await ws.send_json({'type': 'log', 'level': 'error', 'msg': f'打开失败: {exc}'})
            return
        g.replay.open()
        if g.replay_task:
            g.replay_task.cancel()
        g.replay_task = asyncio.create_task(_replay_loop(ws))
        await ws.send_json({'type': 'replay_state', 'on': True, 'file': fn})

    elif cmd == 'replay_speed':
        sp = float(msg.get('speed', 0.5))
        g.replay_speed = max(0.0, min(50.0, sp))
        await ws.send_json({'type': 'replay_speed', 'speed': g.replay_speed})

    elif cmd == 'replay_stop':
        if g.replay_task:
            g.replay_task.cancel()
            g.replay_task = None
        # ★ 2026-08-10 全面审查修复：关闭文件句柄（原实现置 None 不 close，
        #   反复开始/停止回放泄漏句柄，Windows 下记录文件被占用无法覆盖）
        if g.replay:
            g.replay.close()
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
        if t_last is not None and t_now is not None and g.replay_speed > 0:
            dt = (t_now - t_last) / 1000.0
            if dt > 0:
                # 按倍速缩放：speed=1 时按原始节奏，2 时快一倍。
                # 快速模式（speed=0）跳过这里，直接灌数据。
                await asyncio.sleep(max(0.0, dt / max(0.01, g.replay_speed)))
        t_last = t_now
        g.snap = {k: v for k, v in row.items() if v is not None}
        # 回放也写时序缓冲：不写的话「导入 tlog 做频域分析」走不通，
        # 数据只到界面、分析类功能（频谱/统计/调参）全空。
        # 用记录里的原始时间戳（t_ms）——见 _record_hist 的说明。
        _raw_ts = row.get('t_ms')
        _record_hist(g.snap, ts=(float(_raw_ts) / 1000.0) if _raw_ts else None)
        # 倍速：speed=0 表示"最快"（不等待，纯灌数据给分析用）；
        # 1.0 = 原始速率；2.0 = 两倍速。半速回放（原来的 0.5）保留为默认，
        # 因为它最适合"看画面"，而 0 适合"跑完再分析"。
        if g.replay_speed <= 0:
            await asyncio.sleep(0)
        else:
            await asyncio.sleep(TELEMETRY_PERIOD / max(0.01, g.replay_speed))
        g.send_all({'type': 'telemetry', 'data': g.snap, 'replay': True})
        # 进度每 10 帧发一次即可：50Hz 下每帧推进度纯属浪费带宽
        n = getattr(g, '_replay_n', 0) + 1
        g._replay_n = n
        if n % 10 == 0 and hasattr(g.replay, 'progress'):
            try:
                g.send_all({'type': 'replay_progress', **g.replay.progress()})
            except Exception:
                pass


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
    """拉取固件全部注册表参数（当前 169；逐 ID 发送 + 等待该 ID 双全，超时重试。
    ★ 2M 速率下 USB 串口偶发丢字节（固件 CRC 失败静默丢弃）——
      批量灌发会系统性丢参数，必须逐 ID 确认）"""
    if not (g.link and g.link.is_open):
        return
    g.param_names = {}
    g.param_values = {}
    failed = []
    # ★ 2026-08-10 全面审查修复：硬编码 117 → 动态（0xE0 CMD 0x01 已应答过则用之）
    # ★ 2026-11-05：兜底改用本地参数表长度（与固件同序镜像，现 169 项）并截断固件上报值——
    #   避免“count 未回退”时只读前 121 项、或未来固件扩容后越界读不存在的 ID。
    local_count = len(pm.expected_names())
    seen = int(g.param_count_seen) if g.param_count_seen else local_count
    total = min(seen, local_count)
    for pid in range(total):
        if not await _param_read_single_async(pid):
            failed.append(pid)
    # 补拉循环
    for _rnd in range(3):
        if not failed:
            break
        failed = [pid for pid in failed if not await _param_read_single_async(pid)]
    await ws.send_json({'type': 'log',
                        'msg': f'参数读取完成：{total - len(failed)}/{total}' +
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

# ---------------------------------------------------------------
# 无硬件演示模式（GCS_DEMO=1）
# 为什么做：没有数传/飞控时，波形、GNSS、Inspector 这些页面全是空的，
# 既没法验收也没法给人演示。这里合成一条标准 MAVLink 遥测——姿态缓慢摆动、
# GNSS 3D 定位绕小圈、星数波动——走的是和真实设备完全相同的解析与广播链路
# （_on_telemetry_rx），因此验证的是真链路，不是另写一套假前端。
# 只在环境变量显式置 1 时启用，不影响正常连接流程。
# ---------------------------------------------------------------
# 演示用二阶跟随状态。为什么需要：调参页要分析「指令→响应」，
# 如果反馈是正弦而指令是方波，两者没有因果关系，算出来的指标毫无意义。
# 这里用一个真正的二阶系统（可调 ζ/ωn）跟随方波指令，
# 于是超调、上升时间、调节时间都是这套"虚拟控制器"的真实表现。
_DEMO_2ND = {'y': 0.0, 'yd': 0.0}


def _demo_second_order(target: float, dt: float,
                       wn: float = 7.0, zeta: float = 0.55) -> float:
    """@brief 二阶系统一步跟随（y'' = wn²(target−y) − 2ζwn·y'）
    @note ζ=0.55 → 理论超调约 12.6%，正好让调参页算得出非零超调。
    """
    st = _DEMO_2ND
    err = target - st['y']
    acc = wn * wn * err - 2.0 * zeta * wn * st['yd']
    st['yd'] += acc * dt
    st['y'] += st['yd'] * dt
    return st['y']

DEMO_MODE = os.environ.get('GCS_DEMO') == '1'


async def _demo_loop():
    """@brief 以 50Hz 合成 MAVLink 遥测并喂进 _on_telemetry_rx"""
    import math

    from pymavlink import mavutil

    g.protocol = 'mavlink'
    g.mav = MavlinkService()

    buf = bytearray()

    class _Sink:
        def write(self, data):
            buf.extend(data)
            return len(data)

    v = mavutil.mavlink.MAVLink(_Sink())
    v.srcSystem = 1
    v.srcComponent = 1

    t0 = time.time()
    while True:
        # 回放进行中就跳过本轮合成：否则演示数据和回放数据会同时写进
        # g.hist，分析结果被两路数据污染（实测：记录 209 行的文件，回放后
        # 历史里却有 255 点——多出来的 46 点全是演示循环偷偷推的）。
        # 用 continue 而不是停掉任务：回放结束后自然恢复，没有"忘了重启"的风险。
        if g.replay is not None:
            await asyncio.sleep(0.2)
            continue
        t = time.time() - t0
        ms = int(t * 1000) & 0xFFFFFFFF

        # 姿态：两个不同频率的摆动，横滚与俯仰不同相（避免看起来像单轴）
        roll = math.radians(6.0 * math.sin(0.7 * t))
        pitch = math.radians(4.0 * math.sin(0.45 * t + 1.0))
        yaw = math.radians(25.0 * math.sin(0.12 * t))
        # 目标姿态方波（3 s 一个平台）+ 二阶跟随，供调参页做阶跃响应分析。
        # 顺序很重要：_roll_fb 必须先算再喂给 ATTITUDE，否则 UnboundLocalError
        # 会让整个演示循环挂掉（实测踩过，只报在 asyncio 的 task exception 里，
        # 表面现象只是"没有数据"）。
        _tgt = 8.0 if (int(t) % 6) < 3 else 0.0
        _roll_fb = _demo_second_order(_tgt, 0.02)

        # roll 用二阶跟随结果（与 ATTITUDE_TARGET 的方波构成指令→响应对），
        # pitch/yaw 仍保留各自的正弦摆动，便于其它页面观察多通道行为。
        v.attitude_send(ms, math.radians(_roll_fb), pitch, yaw,
                        0.01 * math.cos(0.7 * t),
                        0.01 * math.cos(0.45 * t),
                        0.002 * math.cos(0.12 * t))

        # GNSS：3D 定位，位置绕 4 mm 级小圈（看散布图能否收成一点）
        lat = 28.230000 + 0.0000040 * math.sin(0.30 * t)
        lon = 112.940000 + 0.0000040 * math.cos(0.30 * t)
        alt_mm = int((120.0 + 2.0 * math.sin(0.5 * t)) * 1000)
        # 卫星数周期起伏：让报警链路在演示模式下也能被真实触发，
        # 否则「报警页永远显示无告警」等于这条链路从未被验证过。
        sats = 12 if (int(t) % 30) < 22 else 4
        v.gps_raw_int_send(ms * 1000, 3,
                           int(lat * 1e7), int(lon * 1e7), alt_mm,
                           90, 120, 350, 90, sats)

        # 多机演示：交替用 sysid 1 / 2 发送，模拟一条链路承载两台机。
        # 这样顶栏的机号选择器会出现，切换后各视图数据确实换成另一台——
        # 单机演示无法验证多机链路是否真的按 sysid 分离。
        v.srcSystem = 1 + (int(t * 10) % 2)

        # 目标姿态：方波指令（3 s 一个平台），供调参页做阶跃响应分析。
        # 没有这段，调参页在演示模式下永远是"没有可评估的阶跃"，
        # 等于这条链路从未被验证过。
        _tgt = 8.0 if (int(t) % 6) < 3 else 0.0
        _half = math.radians(_tgt) * 0.5
        v.attitude_target_send(
            ms * 1000, 0,
            (math.cos(_half), math.sin(_half), 0.0, 0.0),   # 绕 x 转 target
            0.0, 0.0, 0.0, 0.5)
        # 相对位置与速度（LOCAL_POSITION_NED）：前端「相对北/东」「垂直速度」
        # 直接读这组字段。协议里 z 向下为正，这里给负值表示在上方。
        v.local_position_ned_send(
            ms,
            0.5 * math.sin(0.30 * t),
            0.5 * math.cos(0.30 * t),
            -120.0 - 2.0 * math.sin(0.5 * t),
            0.15 * math.cos(0.30 * t),
            -0.15 * math.sin(0.30 * t),
            -1.0 * math.cos(0.5 * t),
        )

        # 速度/高度：与姿态同节奏，便于肉眼核对曲线关系
        v.vfr_hud_send(3.5 + 0.4 * math.sin(0.6 * t),
                       4.2 + 0.3 * math.sin(0.6 * t),
                       int((25.0 * math.sin(0.12 * t)) % 360),
                       78, 120.0 + 2.0 * math.sin(0.5 * t), 0.4)

        if buf:
            _on_telemetry_rx(bytes(buf))
            buf.clear()
        await asyncio.sleep(0.02)


@app.on_event('startup')
async def _demo_boot():
    """启动钩子：DEMO 模式下起合成遥测任务"""
    if DEMO_MODE:
        log.info('GCS_DEMO=1：注入合成遥测（无硬件演示模式，连真实设备前请关掉）')
        asyncio.create_task(_demo_loop())


@app.websocket('/ws')
async def ws_endpoint(ws: WebSocket):
    """@brief WebSocket 端点：接受连接 → 发 hello 版本握手 → 循环处理命令
    @note  首个消息固定为 {'type':'hello','version':app.version}，前端据此识别
           残留旧后端（AGENTS.md 踩坑记录）
    """
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


@app.post('/api/upload_log')
async def api_upload_log(file: UploadFile = File(...)):
    """@brief 上传日志文件（tlog/bin/csv）到录制目录，供回放使用
    @details 为什么需要上传：浏览器 <input type=file> 出于安全只给 File 对象，
             拿不到本地绝对路径，而回放需要后端能打开的文件。与其让用户手输
             路径，不如直接传上来。
    @note   同名文件不覆盖，自动加序号——覆盖用户已有的飞行记录是不可接受的。
    """
    name = Path(file.filename or 'upload.log').name
    safe = ''.join(c for c in name if c.isalnum() or c in '._-') or 'upload.log'
    dest = OUTPUT_DIR / safe
    n = 1
    while dest.exists():
        dest = OUTPUT_DIR / f'{Path(safe).stem}_{n}{Path(safe).suffix}'
        n += 1
    size = 0
    try:
        with dest.open('wb') as f:
            while True:
                chunk = await file.read(1 << 20)
                if not chunk:
                    break
                f.write(chunk)
                size += len(chunk)
    except Exception as exc:
        return JSONResponse({'ok': False, 'error': str(exc)}, status_code=500)
    log.info('uploaded log: %s (%d bytes)', dest.name, size)
    return {'ok': True, 'name': dest.name, 'bytes': size}


@app.get('/video/test.mjpg')
async def video_test_stream():
    """@brief 内置测试视频流（MJPEG），用于无硬件时验证播放器链路
    @details 为什么后端自己生成：视频页要在没有图传硬件的情况下也能验证，
             否则这块功能"写完就没法验收"。画面内容是动态的（时间戳 +
             旋转指针 + 渐变背景），一眼能看出是实时流还是卡住的静帧——
             这是判断播放器是否真的在解码的最快办法。
    @note   输出是 multipart/x-mixed-replace（MJPEG 标准封装），
             浏览器 <img src> 直接就能播，不需要任何 JS 解码库。
    """
    boundary = 'unignc'

    async def gen():
        import io as _io

        from PIL import Image, ImageDraw
        t0 = time.time()
        for _ in range(600):     # 约 60 s（10 fps），够验证
            el = time.time() - t0
            img = Image.new('RGB', (480, 270), (12, 20, 34))
            d = ImageDraw.Draw(img)
            # 渐变背景条：随时间水平移动
            for x in range(0, 480, 8):
                shade = int(30 + 40 * math.sin((x / 480.0) * 6.28 + el * 2.0))
                d.rectangle([x, 0, x + 7, 270], fill=(shade // 3, shade // 2, shade))
            # 旋转指针：角度直接反映时间，肉眼可判是否在动
            ang = el * 1.2
            cx, cy, r = 240, 135, 70
            d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=(90, 140, 200), width=2)
            d.line([cx, cy, cx + r * math.cos(ang), cy + r * math.sin(ang)],
                   fill=(255, 93, 108), width=3)
            d.text((10, 8), f'UniGNC test pattern  t={el:6.2f}s', fill=(220, 235, 255))
            d.text((10, 250), 'MJPEG / multipart', fill=(140, 170, 200))
            buf = _io.BytesIO()
            img.save(buf, 'JPEG', quality=70)
            data = buf.getvalue()
            yield (b'--' + boundary.encode() + b'\r\n'
                   b'Content-Type: image/jpeg\r\n'
                   b'Content-Length: ' + str(len(data)).encode() + b'\r\n\r\n'
                   + data + b'\r\n')
            await asyncio.sleep(0.1)   # 10 fps
    return StreamingResponse(
        gen(),
        media_type=f'multipart/x-mixed-replace; boundary={boundary}')


@app.get('/tiles/{z}/{x}/{y}.png')
async def get_tile(z: int, x: int, y: int):
    """@brief 地图瓦片：本地缓存优先，未命中则从瓦片源拉取并缓存
    @details 为什么做代理而不是前端直连瓦片源：
             ① 前端直连会带 Referer 且无法缓存到本机，野外断网就彻底没有底图；
             ② 走代理可以把用过的瓦片全部落盘，一次去过的地方下次离线也能用；
             ③ 也便于换源（源不可达时只改后端一处）。
    @note   缓存键是 z/x/y 三元组，不做过期——地图底图变化极慢，过期策略
             带来的复杂度不值得。上层目录按 z 分，避免单目录几十万文件。
    """
    cache = TILE_DIR / str(z) / str(x) / f'{y}.png'
    if cache.is_file():
        return FileResponse(str(cache), media_type='image/png')

    url = _tile_url(z, x, y)
    if url is None:
        # 未配置瓦片源：直接 404 让前端进入「灰底 + 航点可用」降级态，
        # 而不是去连一个注定失败的地址再等超时。
        return Response(status_code=404)
    try:
        import urllib.request
        req = urllib.request.Request(url, headers={'User-Agent': 'UniGNC-GCS/0.3'})

        def _fetch():
            with urllib.request.urlopen(req, timeout=8) as resp:
                return resp.read()

        data = await asyncio.to_thread(_fetch)
    except Exception as exc:
        log.debug('tile %s/%s/%s fetch failed: %r', z, x, y, exc)
        # 返回 404 而不是 500：Leaflet 对 404 只是不画这块，控制台不会刷错误
        return Response(status_code=404)

    try:
        cache.parent.mkdir(parents=True, exist_ok=True)
        cache.write_bytes(data)
    except Exception:
        pass
    return Response(content=data, media_type='image/png')


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
    """@brief 给所有 HTTP 响应加 Cache-Control: no-store
    @note  前端改动后浏览器刷新即取最新 JS，避免"界面全新、逻辑全旧"
    """
    resp = await call_next(request)
    resp.headers['Cache-Control'] = 'no-store'
    return resp


@app.on_event('startup')
async def startup():
    """@brief 应用启动钩子：记录事件循环并启动链路统计任务"""
    g._loop = asyncio.get_running_loop()
    asyncio.create_task(_stat_loop())
    log.info('GCS 后端启动完成（状态接口 /api/status）')


if __name__ == '__main__':
    import uvicorn
    uvicorn.run(app, host='127.0.0.1', port=8091)
