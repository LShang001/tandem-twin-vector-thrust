"""
@file    termux_usb_link.py
@module  GCS/Termux USB 链路
@brief   基于文件描述符的链路：接收 termux-usb 传入的 USB 设备 fd，供平板独立运行
@details 为什么需要它：Android 上普通应用不能直接打开 /dev/ttyUSB0，必须经
         Android USB Host API。Termux 用 `termux-usb -r -e <命令> <设备>` 把设备
         授权后以 fd 形式交给子进程（fd 号作为最后一个参数传入），所以平板上的
         串口数传只能通过这个 fd 读写。

         ★ 阅读者注意：本类的读写逻辑用管道做过单元测试（tests/test_termux_fd_link.py），
           但"termux-usb 传进来的 fd 能不能正常读到数传字节"**必须在真平板 + OTG +
           真实数传上验证**，本机无法覆盖。未验证就当没做（见 docs/Android-平板部署.md）。

         与 SerialLink 同接口（connect/is_open/write/set_read_size/close + on_data），
         所以遥测解析、参数、DBG 全部代码不需要改。

@theory  N/A
@frame   N/A
@units   N/A
@ref     docs/Android-平板部署.md、termux-usb(1)
"""
from __future__ import annotations

import os
import threading


class TermuxFdLink:
    """用已就绪的 fd 收发的链路
    ★ 与 SerialLink 的关键差异：
      ① fd 由外部（termux-usb）提供，本类不负责打开/关闭设备授权；
      ② close() 默认**不关闭 fd**——fd 的属主是 termux-usb 的父进程，
         我们只是借用；关掉它会让 termux-usb 侧的设备句柄状态变脏。
    """

    def __init__(self, fd: int, baud: int = 0, on_data=None,
                 read_size: int = 256, close_fd: bool = False):
        self.fd = int(fd)
        self.port = f'fd:{self.fd}'      # 顶栏/幂等判断用
        self.baud = baud                  # USB 串口的波特率由设备固件定，此处仅兼容字段
        self.on_data = on_data
        self._read_size = read_size
        self._close_fd = close_fd
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._write_lock = threading.Lock()
        self._open = False

    @property
    def is_open(self) -> bool:
        return self._open

    def connect(self):
        """@brief 标记打开并启动读线程
        @note  fd 的有效性在这里做一次快速探测（fcntl 取状态），
               无效 fd 直接抛错，避免"连上了但永远没数据"这种最难查的故障。
        """
        if self._open:
            return
        try:
            os.fstat(self.fd)            # 无效 fd 会抛 OSError
        except OSError as e:
            raise RuntimeError(f'fd {self.fd} 不可用：{e}') from e
        self._open = True
        self._stop.clear()
        # ★ 必须设成非阻塞：阻塞的 os.read 会一直挂在读线程里，close() 的
        #   stop 标志它根本看不到，join 只能靠超时返回，线程和 fd 都悬着。
        #   改成非阻塞 + 短睡眠轮询，close() 才能在一个轮询间隔内真正退出。
        try:
            os.set_blocking(self.fd, False)
        except (OSError, AttributeError):
            pass          # 某些 fd 类型不支持设置，退回阻塞读（close 靠超时）
        self._thread = threading.Thread(target=self._read_loop, daemon=True,
                                        name='termux-usb-read')
        self._thread.start()

    def _read_loop(self):
        import time
        while not self._stop.is_set():
            try:
                data = os.read(self.fd, self._read_size)
            except BlockingIOError:
                time.sleep(0.005)        # 非阻塞读无数据：让出 CPU，并给 stop 机会
                continue
            except OSError:
                if not self._stop.is_set():
                    try:
                        self.on_data(b'')     # 通知上层链路异常，与 SerialLink 同语义
                    except Exception:
                        pass
                break
            if not data:
                # fd 上 EOF：设备被拔或授权被撤
                if not self._stop.is_set():
                    try:
                        self.on_data(b'')
                    except Exception:
                        pass
                break
            if not self._stop.is_set():
                self.on_data(data)

    def write(self, data: bytes):
        """@brief 向 fd 写字节
        @note 部分写要循环补齐：USB 端点一次写不完时 os.write 返回已写字节数，
              不补就会丢掉后半截，表现为"参数写入偶发失败"。
        """
        if not self._open:
            raise RuntimeError('USB fd 链路未打开')
        with self._write_lock:
            view = memoryview(data)
            while view:
                try:
                    n = os.write(self.fd, view)
                except BlockingIOError:
                    # 写端可能被设成非阻塞（同一个 fd 的 O_NONBLOCK 是共享的）：
                    # 缓冲满时短暂等待重试，不能把命令丢掉。
                    import time
                    time.sleep(0.002)
                    continue
                if n <= 0:
                    raise RuntimeError('fd 写入返回 0 字节')
                view = view[n:]

    def set_read_size(self, size: int):
        """@brief 读粒度（DBG 导出会调大）"""
        self._read_size = max(16, int(size))

    def close(self):
        """@brief 停止读线程；默认不关 fd（属主是 termux-usb）"""
        self._open = False
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._close_fd:
            try:
                os.close(self.fd)
            except OSError:
                pass

    def info(self) -> dict:
        return {'kind': 'termux-usb-fd', 'fd': self.fd, 'open': self._open}
