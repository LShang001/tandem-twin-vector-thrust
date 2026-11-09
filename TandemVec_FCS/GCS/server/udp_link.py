"""
@file    udp_link.py
@module  GCS/UDP 链路
@brief   与 SerialLink 同接口的 UDP 数据报链路，供平板/电脑经局域网直连飞控
@details 为什么要有它：串口只解决"线接在本机"的场景。平板装上地面站后，
         要么插 OTG 数传（串口），要么走 WiFi/局域网（UDP）——两条路都要通，
         上层解析与业务代码才能完全不关心底层是线还是网。
         本类刻意只实现 SerialLink 已有的那组方法（connect/is_open/write/
         set_read_size/close + on_data 回调），因此 main.py 里除了"建对象"
         那一步，其余遥测/参数/DBG 代码一行都不用改。
@theory  N/A
@frame   N/A
@units   N/A
@ref     GCS-001 §通信链路、docs/GCS-扩展指南.md（链路即插件）
"""
from __future__ import annotations

import socket
import threading


def parse_udp_spec(spec: str) -> tuple[tuple[str, int], tuple[str, int] | None]:
    """@brief 解析 UDP 端点描述
    @param spec 形如 `udp:14550`（只监听，自动学习对端）或
                `udp:14550@192.168.1.100:14550`（监听并固定发往该地址）
    @return (监听地址, 固定对端地址或 None)
    @note  自动学习对端的价值：飞控侧的 WiFi 数传常常是"上电后主动往某个端口
           发遥测"，地面站并不知道它的 IP；先把来源记下来再回发，就不用让用户
           去查飞控 IP。这是地面站常见的"服务端模式"。
    """
    body = spec[4:] if spec.startswith('udp:') else spec
    peer = None
    if '@' in body:
        body, peer_s = body.split('@', 1)
        host, _, port = peer_s.rpartition(':')
        peer = (host or '0.0.0.0', int(port))
    host, _, port = body.rpartition(':')
    return (host or '0.0.0.0', int(port)), peer


class UdpLink:
    """UDP 链路（数据报）；读线程把每个数据报回调给 on_data
    ★ 与 SerialLink 的差异只在"发往谁"：UDP 无连接，必须先知道对端地址，
      未学习到对端前写入会丢弃并计数（不抛异常——上层 DBG/参数写入不该因为
      还没收到过遥测就崩掉整个连接流程）。
    """

    def __init__(self, spec: str, baud: int = 0, on_data=None, read_size: int = 2048):
        self.spec = spec
        self.port = spec
        self.baud = baud                 # UDP 无波特率；保留字段以兼容上层统一处理
        self.on_data = on_data
        self._read_size = read_size       # UDP 按数据报读，此值仅作 recvfrom 上限
        self._bind, self._fixed_peer = parse_udp_spec(spec)
        self._peer = self._fixed_peer     # 未固定时由收到的第一个包学习
        self._sock: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._write_lock = threading.Lock()
        self._closed = True
        self.dropped = 0                  # 无对端时丢弃的包数（可观测，不静默）
        self.learned_peer = False

    @property
    def is_open(self) -> bool:
        return self._sock is not None

    def connect(self):
        """@brief 建 socket 并 bind，随后启动读线程
        @note SO_REUSEADDR：反复 connect/disconnect 时端口能立即重用，
              否则要等 TIME_WAIT 才能再绑，表现为"断开后连不上"。
        """
        if self.is_open:
            return
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.settimeout(0.2)
        s.bind(self._bind)
        self._sock = s
        self._stop.clear()
        self._closed = False
        self._thread = threading.Thread(target=self._read_loop, daemon=True, name='udp-read')
        self._thread.start()

    def _read_loop(self):
        while not self._stop.is_set():
            try:
                data, addr = self._sock.recvfrom(self._read_size)
            except socket.timeout:
                continue
            except Exception:
                if not self._stop.is_set():
                    try:
                        self.on_data(b'')     # 通知上层"链路异常"，与 SerialLink 同语义
                    except Exception:
                        pass
                break
            if self._fixed_peer is None and addr != self._peer:
                # 自动学习对端：飞控侧 WiFi 数传上电即发遥测，来源就是它的地址
                self._peer = addr
                self.learned_peer = True
            if data and not self._closed:
                self.on_data(data)

    def write(self, data: bytes):
        """@brief 发送一个数据报
        @note 还没学习到对端时丢弃并计数：UDP 无法确认对端在线，抛异常会让
              "先改参数再等遥测"这类操作直接失败，体验更差。
        """
        if not self.is_open:
            raise RuntimeError('UDP 链路未打开')
        if self._peer is None:
            self.dropped += 1
            return
        with self._write_lock:
            self._sock.sendto(data, self._peer)

    def set_read_size(self, size: int):
        """@brief 兼容 SerialLink 的接口（UDP 侧按数据报读，此处只更新上限）"""
        self._read_size = max(256, int(size))

    def close(self):
        self._closed = True
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

    def info(self) -> dict:
        """@brief 链路自描述（顶栏/诊断接口显示"连到谁"）"""
        return {
            'kind': 'udp',
            'bind': f'{self._bind[0]}:{self._bind[1]}',
            'peer': f'{self._peer[0]}:{self._peer[1]}' if self._peer else None,
            'learned_peer': self.learned_peer,
            'dropped': self.dropped,
        }


def open_link(port: str, baud: int, on_data, read_size: int = 256):
    """@brief 按端点描述建链路：`udp:` 前缀走 UDP，其余走串口
    @param port 串口名（COM5）、`udp:14550` / `udp:14550@ip:port`，或 `fd:3`（Termux USB）
    @return Link 对象（未 connect）
    @note 唯一的"协议选择"入口——加 TCP/蓝牙/WebSocket 链路时只改这里，
          上层不需要知道有哪些链路类型。
    """
    if isinstance(port, str) and port.lower().startswith('udp:'):
        return UdpLink(port, baud, on_data, read_size=2048)
    if isinstance(port, str) and port.lower().startswith('fd:'):
        # Termux 平板场景：termux-usb 把授权的 USB 数传以 fd 形式交给本进程
        from .termux_usb_link import TermuxFdLink
        return TermuxFdLink(int(port[3:]), baud, on_data)
    from .serial_link import SerialLink
    return SerialLink(port, baud, on_data, read_size=read_size)
