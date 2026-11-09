"""
@file    serial_udp_bridge.py
@module  工具/串口↔UDP 桥
@brief   把本机串口上的数传字节与局域网 UDP 互通，让地面站后端可以离串口很远

@details 解决什么问题：数传只能插在离飞控近、或者手边方便的那个设备上，
         而看界面的人可能在另一台机器（平板/笔记本）。本桥让"串口在这台机器、
         界面在另一台"成立，且不需要在那台机器上装 Python。

         链路形状：
             飞控 --无线--> 数传 --USB--> 【本桥所在机器】 --UDP--> 地面站后端
             地面站把命令发回 UDP → 本桥写回串口 → 数传 → 飞控

         与 GCS 内置 UDP 链路的分工：
         - GCS 已支持 `udp:14550`（自动学习对端），所以**平板/电脑跑后端时，
           飞控侧若有 WiFi 数传可直连 UDP，不需要本桥**；
         - 本桥用于"数传只有 USB 形态、且插在另一台机器"的场景。

@theory  N/A
@frame   N/A
@units   N/A
@ref     TandemVec_FCS/GCS/docs/Android-平板部署.md、server/udp_link.py
"""
from __future__ import annotations

import argparse
import socket
import sys
import threading
import time

try:
    import serial
except ImportError:                                   # pragma: no cover
    serial = None


class SerialUdpBridge:
    """串口 ↔ UDP 双向桥

    @note 设计上刻意"无状态"：不解析协议、不改字节，只搬字节。这样它对
          AnoCom 与 MAVLink 都成立，也不会因为协议升级而失效。
    """

    def __init__(self, port: str, baud: int, udp_host: str, udp_port: int,
                 listen_port: int = 0, verbose: bool = False):
        self.serial_name = port
        self.baud = baud
        self.udp_addr = (udp_host, udp_port)
        self.listen_port = listen_port or udp_port   # 默认与发送端口一致（对称）
        self.verbose = verbose
        self.ser = None
        self.sock = None
        self.peer = None            # 第一次收到回包时学习到的对端
        self.stat_to_udp = 0
        self.stat_to_ser = 0
        self._stop = threading.Event()

    # ------------------------------------------------------------------ 串口
    def open_serial(self):
        if serial is None:
            raise RuntimeError('pyserial 未安装：py -3.12 -m pip install pyserial')
        self.ser = serial.Serial(self.serial_name, self.baud, timeout=0.1)
        print(f'[桥] 串口已打开 {self.serial_name} @ {self.baud}')

    def open_udp(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('0.0.0.0', self.listen_port))
        s.settimeout(0.2)
        self.sock = s
        print(f'[桥] UDP 监听 0.0.0.0:{self.listen_port} → 转发到 {self.udp_addr[0]}:{self.udp_addr[1]}')

    # ------------------------------------------------------------------ 两条泵
    def pump_serial_to_udp(self):
        """串口数据 → UDP（每 256 字节一批，避免把一帧拆得太碎）"""
        while not self._stop.is_set():
            try:
                data = self.ser.read(256)
            except Exception as exc:
                if not self._stop.is_set():
                    print(f'[桥] 串口读错误：{exc}')
                break
            if not data:
                continue
            try:
                self.sock.sendto(data, self.udp_addr)
                self.stat_to_udp += len(data)
                if self.verbose:
                    print(f'[桥] 串口→UDP {len(data)}B')
            except Exception as exc:
                print(f'[桥] UDP 发送失败：{exc}')

    def pump_udp_to_serial(self):
        """UDP → 串口（地面站下发的命令走这里）"""
        while not self._stop.is_set():
            try:
                data, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except Exception as exc:
                if not self._stop.is_set():
                    print(f'[桥] UDP 收错误：{exc}')
                break
            if self.peer is None:
                self.peer = addr
                print(f'[桥] 学会地面站地址 {addr[0]}:{addr[1]}')
            try:
                self.ser.write(data)
                self.stat_to_ser += len(data)
                if self.verbose:
                    print(f'[桥] UDP→串口 {len(data)}B')
            except Exception as exc:
                print(f'[桥] 串口写失败：{exc}')

    # ------------------------------------------------------------------ 生命周期
    def run(self):
        self.open_serial()
        self.open_udp()
        t1 = threading.Thread(target=self.pump_serial_to_udp, daemon=True)
        t2 = threading.Thread(target=self.pump_udp_to_serial, daemon=True)
        t1.start()
        t2.start()
        print('[桥] 运行中，Ctrl+C 停止')
        try:
            while not self._stop.is_set():
                time.sleep(0.5)
        except KeyboardInterrupt:
            print('\n[桥] 收到中断，退出')
        finally:
            self.close()
        print(f'[桥] 统计：串口→UDP {self.stat_to_udp} B，UDP→串口 {self.stat_to_ser} B')

    def close(self):
        self._stop.set()
        for obj, name in ((self.ser, '串口'), (self.sock, 'UDP')):
            try:
                if obj is not None:
                    obj.close()
            except Exception:
                pass
        print(f'[桥] 已关闭（{name}）')


def main() -> int:
    ap = argparse.ArgumentParser(
        description='串口 ↔ UDP 双向桥：让数传插在这台机器、地面站跑在另一台')
    ap.add_argument('--port', required=True, help='串口名，如 COM5 或 /dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=921600, help='波特率，默认 921600')
    ap.add_argument('--to', required=True, help='地面站 UDP 地址，形如 192.168.0.208:14550')
    ap.add_argument('--listen', type=int, default=0,
                    help='本机监听端口（默认与 --to 的端口相同）')
    ap.add_argument('--verbose', action='store_true', help='打印每次转运')
    args = ap.parse_args()

    host, _, port_s = args.to.rpartition(':')
    if not host:
        print('--to 需要 host:port 形式', file=sys.stderr)
        return 2
    bridge = SerialUdpBridge(args.port, args.baud, host, int(port_s),
                             args.listen, args.verbose)
    try:
        bridge.run()
    except Exception as exc:
        print(f'[桥] 启动失败：{type(exc).__name__}: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
