"""
@file    chaquopy_usb_link.py
@module  GCS/USB 链路
@brief   Android 内经 Java USB Host API 访问 OTG 串口，与 SerialLink 同接口

Chaquopy 环境下的 USB 串口链路：Python 侧调用 Java 的 USB Host 桥。

为什么不能在 Android 上用 pyserial：普通应用看不到 /dev/ttyUSB0，
`serial.tools.list_ports.comports()` 返回空列表、直接 open 也无权限。
唯一合法路径是 Android USB Host API，而它只有 Java 能调——所以这里通过
Chaquopy 的 jclass 桥去调 UsbSerialBridge 的静态方法。

对上层完全透明：与 SerialLink 同接口（connect/is_open/write/set_read_size/close
+ on_data 回调），所以遥测解析、参数、DBG 代码一行都不用改。
"""
from __future__ import annotations

import json
import threading


def _bridge():
    """取 Java 侧的 UsbSerialBridge 类对象（非 Android 环境返回 None）"""
    try:
        from java import jclass
    except ImportError:
        return None
    try:
        return jclass('com.unignc.gcs.UsbSerialBridge')
    except Exception:
        return None


def is_android() -> bool:
    """当前是否跑在 Chaquopy（Android）里"""
    return _bridge() is not None


def list_usb_devices() -> list[dict]:
    """列出 USB 串口设备（Android 专用；其它平台返回空列表）"""
    br = _bridge()
    if br is None:
        return []
    try:
        raw = str(br.listDevices())
        return json.loads(raw)
    except Exception:
        return []


def request_permission() -> bool:
    """请求 USB 访问权限（会弹系统授权框，最多等 10 秒）"""
    br = _bridge()
    if br is None:
        return False
    try:
        return bool(br.requestPermission())
    except Exception:
        return False


class ChaquopyUsbLink:
    """通过 Java 桥访问 USB 串口的链路

    @note is_open 以 Java 侧的 sPort 是否为空为准：链路对象存活不代表串口
          还真在（可能被拔掉），所以每次查询都问一次 Java 侧。
    """

    def __init__(self, spec: str = 'usb:-1:-1', baud: int = 921600,
                 on_data=None, read_size: int = 512):
        # spec 形如 usb:<vid>:<pid>；vid/pid 为 -1 表示"第一个可用设备"
        self.port = spec
        self.baud = baud
        self.on_data = on_data
        self._read_size = read_size
        parts = spec.split(':')
        self.vid = int(parts[1]) if len(parts) > 2 else -1
        self.pid = int(parts[2]) if len(parts) > 2 else -1
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._br = _bridge()
        self.last_error = ''

    @property
    def is_open(self) -> bool:
        if self._br is None:
            return False
        try:
            return bool(self._br.isOpen())
        except Exception:
            return False

    def connect(self):
        """@brief 打开串口并起读线程
        @note 失败时抛 RuntimeError 并把 Java 侧给的中文原因原样带出来——
              界面能直接显示，不用猜是权限、线缆还是驱动。
        """
        if self._br is None:
            raise RuntimeError('USB 桥不可用：本链路只能在 Android APK 内使用')
        if self.is_open:
            return
        err = ''
        try:
            err = str(self._br.open(self.vid, self.pid, self.baud))
        except Exception as e:
            err = f'{type(e).__name__}: {e}'
        if err:
            self.last_error = err
            raise RuntimeError(err)
        self._stop.clear()
        self._thread = threading.Thread(target=self._read_loop, daemon=True,
                                        name='chaquopy-usb-read')
        self._thread.start()

    def _read_loop(self):
        import time
        while not self._stop.is_set():
            try:
                data = self._br.read(200)
            except Exception:
                break
            if data is None:
                time.sleep(0.01)
                continue
            try:
                buf = bytes(data)          # Java byte[] → Python bytes
            except Exception:
                buf = b''
            if buf:
                if not self._stop.is_set():
                    self.on_data(buf)
            else:
                time.sleep(0.005)          # 无数据：让出 CPU，同时给停止信号机会

    def write(self, data: bytes):
        if self._br is None:
            raise RuntimeError('USB 桥不可用')
        n = int(self._br.write(bytes(data)))
        if n < 0:
            raise RuntimeError('串口未打开或写入失败')

    def set_read_size(self, size: int):
        """兼容 SerialLink 的接口（Java 侧固定 512，此处仅记录）"""
        self._read_size = max(16, int(size))

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._br is not None:
            try:
                self._br.close()
            except Exception:
                pass

    def info(self) -> dict:
        return {'kind': 'android-usb-otg', 'vid': self.vid, 'pid': self.pid,
                'open': self.is_open, 'error': self.last_error}
