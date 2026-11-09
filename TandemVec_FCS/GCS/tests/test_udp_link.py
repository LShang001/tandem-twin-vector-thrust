"""
@file    test_udp_link.py
@module  GCS/测试/UDP 链路
@brief   UDP 链路真实收发测试（本机回环，不用硬件）

判据：不是"对象建出来了"，而是字节真的通过 socket 收到了、对端地址真的学会了、
没有对端时写入真的被计数丢弃（而不是静默）。
"""
import socket
import time

import pytest

from server.udp_link import UdpLink, open_link, parse_udp_spec


def _free_port() -> int:
    """拿一个当前空闲的 UDP 端口（bind 0 让系统分配后立刻释放）"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


def test_parse_spec_plain():
    """只给端口：监听任意地址，不固定对端"""
    bind, peer = parse_udp_spec('udp:14550')
    assert bind == ('0.0.0.0', 14550)
    assert peer is None


def test_parse_spec_with_peer():
    """端口@对端：固定发往指定地址"""
    bind, peer = parse_udp_spec('udp:14550@192.168.1.100:14551')
    assert bind == ('0.0.0.0', 14550)
    assert peer == ('192.168.1.100', 14551)


def test_parse_spec_host_bind():
    """显式监听地址"""
    bind, peer = parse_udp_spec('udp:127.0.0.1:9000')
    assert bind == ('127.0.0.1', 9000)
    assert peer is None


def test_roundtrip_between_two_links():
    """两个链路互发字节：验证 write → sendto → recvfrom → on_data 全链真通"""
    port_a, port_b = _free_port(), _free_port()
    got_a, got_b = bytearray(), bytearray()

    # 回调绑到自己那侧的接收缓冲：a 的 on_data 收的是 b 发来的字节
    a = UdpLink(f'udp:{port_a}@127.0.0.1:{port_b}', on_data=got_a.extend)
    b = UdpLink(f'udp:{port_b}@127.0.0.1:{port_a}', on_data=got_b.extend)
    a.connect()
    b.connect()
    try:
        a.write(b'\x01\x02\x03')
        b.write(b'\xAA\xBB')
        deadline = time.time() + 2.0
        while time.time() < deadline and (not got_b or not got_a):
            time.sleep(0.02)
        assert bytes(got_b) == b'\x01\x02\x03', f'B 没收到 A 的字节：{bytes(got_b)!r}'
        assert bytes(got_a) == b'\xaa\xbb', f'A 没收到 B 的字节：{bytes(got_a)!r}'
    finally:
        a.close()
        b.close()


def test_learns_peer_when_not_pinned():
    """不固定对端时，收到第一个包就把来源记为对端（飞控上电主动推遥测的场景）"""
    port_srv, port_cli = _free_port(), _free_port()
    got = bytearray()
    srv = UdpLink(f'udp:{port_srv}', on_data=got.extend)
    srv.connect()
    try:
        # 模拟"飞控"：直接往 srv 端口发一包
        c = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        c.bind(('127.0.0.1', port_cli))
        c.sendto(b'hello', ('127.0.0.1', port_srv))
        deadline = time.time() + 2.0
        while time.time() < deadline and not got:
            time.sleep(0.02)
        assert bytes(got) == b'hello'
        assert srv.learned_peer is True
        assert srv.info()['peer'] == f'127.0.0.1:{port_cli}'

        # 学会之后回发要能到达对方
        back = bytearray()
        c.settimeout(2.0)
        srv.write(b'pong')
        data, _ = c.recvfrom(64)
        back.extend(data)
        assert bytes(back) == b'pong', '学习到对端后回发失败'
        c.close()
    finally:
        srv.close()


def test_write_without_peer_is_counted_not_raised():
    """还没学到对端时写入：丢弃并计数，不抛异常（否则连上就崩）"""
    port = _free_port()
    link = UdpLink(f'udp:{port}')
    link.connect()
    try:
        link.write(b'x')
        assert link.dropped == 1
        assert not link.learned_peer
    finally:
        link.close()


def test_open_link_dispatches_by_prefix():
    """open_link 是唯一的链路选择入口：udp: 前缀走 UDP，其余走串口"""
    udp = open_link(f'udp:{_free_port()}', 0, lambda d: None)
    assert isinstance(udp, UdpLink)
    serial_like = open_link('COM_不存在', 115200, lambda d: None)
    assert type(serial_like).__name__ == 'SerialLink'
    assert not serial_like.is_open      # 只建对象，不连接


def test_close_is_idempotent():
    """反复 close 不炸（断开/重连/退出都会调）"""
    port = _free_port()
    link = UdpLink(f'udp:{port}')
    link.connect()
    link.close()
    link.close()
    assert not link.is_open


def test_rebind_same_port_after_close():
    """关闭后能立即重绑同一端口（SO_REUSEADDR 的作用，否则断开后连不上）"""
    port = _free_port()
    for _ in range(3):
        link = UdpLink(f'udp:{port}')
        link.connect()
        assert link.is_open
        link.close()
