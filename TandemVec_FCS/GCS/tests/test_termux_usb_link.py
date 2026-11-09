"""
@file    test_termux_usb_link.py
@module  GCS/测试/Termux USB fd 链路
@brief   用管道验证 fd 链路的读写逻辑（不依赖真 Android 设备）

判据：字节真的经 fd 双向流动、无效 fd 立刻报错、写不满时能循环补齐、
关闭不误关别人给的 fd。**能验证的是本类逻辑**；"termux-usb 传进来的真实
USB 设备 fd 能否读到数传字节"必须在真平板上验证，见 docs/Android-平板部署.md。
"""
import os

import pytest

from server.termux_usb_link import TermuxFdLink
from server.udp_link import open_link


def test_read_from_fd():
    """写入管道 → 链路 on_data 收到同样的字节"""
    r, w = os.pipe()
    got = bytearray()
    link = TermuxFdLink(r, on_data=got.extend)
    link.connect()
    try:
        os.write(w, b'\x01\x02\x03')
        import time
        deadline = time.time() + 2.0
        while time.time() < deadline and len(got) < 3:
            time.sleep(0.01)
        assert bytes(got) == b'\x01\x02\x03'
    finally:
        link.close()
        os.close(w)


def test_write_to_fd():
    """链路 write → 能从管道另一端读到"""
    r, w = os.pipe()
    link = TermuxFdLink(w, on_data=lambda d: None)
    link.connect()
    try:
        link.write(b'hello-usb')
        assert os.read(r, 64) == b'hello-usb'
    finally:
        link.close()
        os.close(r)


def test_bidirectional():
    """双向同时可用（真实数传既要收遥测又要发命令）"""
    r_in, w_in = os.pipe()      # 模拟设备→程序
    r_out, w_out = os.pipe()    # 程序→设备
    got = bytearray()

    class DualFd(TermuxFdLink):
        """读端与写端是不同 fd 的变体（真实 USB 端点也是分开的）"""
        def __init__(self, *a, write_fd=None, **kw):
            super().__init__(*a, **kw)
            self._write_fd = write_fd

        def write(self, data):
            view = memoryview(data)
            while view:
                n = os.write(self._write_fd, view)
                view = view[n:]

    link = DualFd(r_in, on_data=got.extend, write_fd=w_out)
    link.connect()
    try:
        os.write(w_in, b'telemetry')
        link.write(b'command')
        import time
        time.sleep(0.2)
        assert bytes(got) == b'telemetry'
        assert os.read(r_out, 64) == b'command'
    finally:
        link.close()
        for fd in (w_in, r_out):
            os.close(fd)


def test_invalid_fd_raises_on_connect():
    """无效 fd 在 connect 时就报错，而不是连上后永远没数据"""
    link = TermuxFdLink(999999, on_data=lambda d: None)
    with pytest.raises(RuntimeError):
        link.connect()


def test_write_before_connect_raises():
    """未连接就写要报错（避免静默丢命令）"""
    r, w = os.pipe()
    link = TermuxFdLink(w, on_data=lambda d: None)
    with pytest.raises(RuntimeError):
        link.write(b'x')
    os.close(r)
    os.close(w)


def test_close_does_not_close_borrowed_fd():
    """默认不关闭借来的 fd（属主是 termux-usb）"""
    r, w = os.pipe()
    link = TermuxFdLink(r, on_data=lambda d: None)
    link.connect()
    link.close()
    try:
        os.fstat(r)          # 仍有效
    finally:
        os.close(r)
        os.close(w)


def test_close_can_close_fd_when_asked():
    """显式 close_fd=True 时才关"""
    r, w = os.pipe()
    link = TermuxFdLink(r, on_data=lambda d: None, close_fd=True)
    link.connect()
    link.close()
    with pytest.raises(OSError):
        os.fstat(r)
    os.close(w)


def test_open_link_dispatches_fd():
    """open_link 认 fd: 前缀（平板部署时链路由它统一创建）"""
    r, w = os.pipe()
    try:
        link = open_link(f'fd:{w}', 0, lambda d: None)
        assert isinstance(link, TermuxFdLink)
        assert link.fd == w
        assert not link.is_open        # 只建对象，不连接
    finally:
        os.close(r)
        os.close(w)
