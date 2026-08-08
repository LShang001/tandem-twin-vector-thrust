# -*- coding: utf-8 -*-
"""
serial_link.py — 串口抽象层

- list_ports(): 枚举可用串口（供前端下拉）
- SerialLink: 连接/断开/读写线程；读数据通过 on_data 回调推送（读线程持续 read 并回调）
- FakeSerialLink: 测试用内存回放串口（可注入字节流，验证解析链路）
"""
import threading
import time

try:
    import serial
    import serial.tools.list_ports as list_ports_mod
except ImportError:  # 无 pyserial 环境（如文档服务器）下仍可 import 本模块
    serial = None
    list_ports_mod = None


def list_ports():
    """返回 [{port, desc, hwid}] 列表"""
    if list_ports_mod is None:
        return []
    out = []
    for p in list_ports_mod.comports():
        out.append({'port': p.device, 'desc': p.description or '', 'hwid': p.hwid or ''})
    return out


class SerialLink:
    """pyserial 连接；读线程把字节流回调给 on_data（线程安全：回调内勿阻塞）
    ★ read_size 取小（1024）：AnoCom 200Hz×12 帧 ≈2400 帧/s，chunk 越小
      解码耗时越短，消费吞吐须远超产生速率——否则参数响应帧在队列中
      排队超时 → 客户端重发风暴 → 固件 RX 缓冲溢出（实测 2M 下根因）"""

    def __init__(self, port, baud, on_data, read_size=1024):
        self.port = port
        self.baud = baud
        self.on_data = on_data
        self._read_size = read_size
        self._ser = None
        self._thread = None
        self._stop = threading.Event()
        self._write_lock = threading.Lock()
        self._closed = True

    @property
    def is_open(self):
        return self._ser is not None and self._ser.is_open

    def connect(self):
        if serial is None:
            raise RuntimeError('pyserial 未安装（pip install pyserial）')
        if self.is_open:
            return
        self._ser = serial.Serial(self.port, self.baud, timeout=0.2)
        self._stop.clear()
        self._closed = False
        self._thread = threading.Thread(target=self._read_loop, daemon=True, name='serial-read')
        self._thread.start()

    def _read_loop(self):
        while not self._stop.is_set():
            try:
                data = self._ser.read(self._read_size)
                if data and not self._closed:
                    self.on_data(data)
            except Exception:
                if not self._stop.is_set():
                    # 读线程异常（如拔线）→ 通知一次后退出
                    try:
                        self.on_data(b'')
                    except Exception:
                        pass
                break

    def write(self, data: bytes):
        if not self.is_open:
            raise RuntimeError('串口未连接')
        with self._write_lock:
            self._ser.write(data)

    def set_read_size(self, size: int):
        """动态调整读粒度（DBG 导出场景提大提吞吐，遥测场景保持小块保解码流畅）"""
        self._read_size = size

    def close(self):
        self._closed = True
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None


class FakeSerialLink:
    """测试用串口：可预置回放字节流 + 捕获写入字节"""

    def __init__(self, port='FAKE', baud=2000000, on_data=None, replay=None):
        self.port = port
        self.baud = baud
        self.on_data = on_data
        self.replay = list(replay or [])   # 字节流切片列表，connect 后按顺序回调
        self.written = bytearray()
        self._sent = 0
        self._closed = True
        self._started = False

    @property
    def is_open(self):
        return not self._closed

    def connect(self):
        self._closed = False
        if self._started:
            return
        self._started = True
        for chunk in self.replay:
            if self.on_data:
                self.on_data(chunk)

    def write(self, data: bytes):
        self.written.extend(data)

    def close(self):
        self._closed = True
