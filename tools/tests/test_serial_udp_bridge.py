"""
@file    test_serial_udp_bridge.py
@module  工具/测试/串口↔UDP 桥
@brief  用假串口 + 真实 UDP 验证桥的双向转运（不需要真数传）

判据：字节真的两个方向都走通了，且统计计数与内容一致——不是"对象建出来了"。
"""
import socket
import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tools.serial_udp_bridge import SerialUdpBridge  # noqa: E402


class FakeSerial:
    """最小假串口：read 从缓冲取，write 记录到 sent"""

    def __init__(self, feed: bytes = b''):
        self._feed = bytearray(feed)
        self.sent = bytearray()
        self.closed = False
        self._lock = threading.Lock()

    def read(self, n: int) -> bytes:
        with self._lock:
            chunk = bytes(self._feed[:n])
            del self._feed[:n]
        if not chunk:
            time.sleep(0.01)          # 模拟读超时，避免空转吃满 CPU
        return chunk

    def feed(self, data: bytes):
        with self._lock:
            self._feed += data

    def write(self, data: bytes) -> int:
        with self._lock:
            self.sent += data
        return len(data)

    def close(self):
        self.closed = True


def _free_udp_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _make_bridge(fake: FakeSerial, sink_port: int, listen_port: int) -> SerialUdpBridge:
    br = SerialUdpBridge('FAKE', 115200, '127.0.0.1', sink_port,
                         listen_port=listen_port)
    br.ser = fake
    br.open_udp()
    return br


def test_serial_to_udp():
    """串口字节 → UDP 目的端收到"""
    sink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sink.bind(('127.0.0.1', 0))
    sink.settimeout(2.0)
    sink_port = sink.getsockname()[1]

    fake = FakeSerial()
    br = _make_bridge(fake, sink_port, _free_udp_port())
    t = threading.Thread(target=br.pump_serial_to_udp, daemon=True)
    t.start()
    try:
        fake.feed(b'\x01\x02\x03\x04')
        data, _ = sink.recvfrom(1024)
        assert data == b'\x01\x02\x03\x04'
        assert br.stat_to_udp >= 4
    finally:
        br._stop.set()
        br.close()
        sink.close()


def test_udp_to_serial():
    """UDP 进来 → 串口 write 收到（地面站下发命令的方向）"""
    listen_port = _free_udp_port()
    fake = FakeSerial()
    br = _make_bridge(fake, _free_udp_port(), listen_port)
    t = threading.Thread(target=br.pump_udp_to_serial, daemon=True)
    t.start()
    try:
        c = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        c.sendto(b'\xAA\xBB', ('127.0.0.1', listen_port))
        deadline = time.time() + 2.0
        while time.time() < deadline and len(fake.sent) < 2:
            time.sleep(0.02)
        assert bytes(fake.sent) == b'\xaa\xbb'
        assert br.peer is not None, '没学会发送方地址'
        c.close()
    finally:
        br._stop.set()
        br.close()


def test_bidirectional_roundtrip():
    """双向同时工作：设备数据出去、命令回来，互不干扰"""
    sink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sink.bind(('127.0.0.1', 0))
    sink.settimeout(2.0)
    sink_port = sink.getsockname()[1]
    listen_port = _free_udp_port()

    fake = FakeSerial()
    br = _make_bridge(fake, sink_port, listen_port)
    threads = [threading.Thread(target=br.pump_serial_to_udp, daemon=True),
               threading.Thread(target=br.pump_udp_to_serial, daemon=True)]
    for t in threads:
        t.start()
    try:
        fake.feed(b'TELEMETRY')
        data, sender = sink.recvfrom(1024)
        assert data == b'TELEMETRY'

        # 用同一个 socket 回发命令，验证回来的方向
        sink.sendto(b'CMD', ('127.0.0.1', listen_port))
        deadline = time.time() + 2.0
        while time.time() < deadline and len(fake.sent) < 3:
            time.sleep(0.02)
        assert bytes(fake.sent) == b'CMD'
    finally:
        br._stop.set()
        br.close()
        sink.close()
