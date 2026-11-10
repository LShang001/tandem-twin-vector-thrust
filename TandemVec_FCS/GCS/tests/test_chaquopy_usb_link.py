"""
@file    test_chaquopy_usb_link.py
@module  GCS/测试/Android USB 链路
@brief  验证 Chaquopy USB 链路在非 Android 环境下的降级行为

判据：Windows/Linux 上不能崩，也不能假装有串口——必须明确告知"只能在 APK 内用"。
真正的 USB 读写只能在平板上验证（Java 侧代码本机无法执行）。
"""
import pytest

from server.chaquopy_usb_link import (ChaquopyUsbLink, is_android,
                                      list_usb_devices, request_permission)
from server.serial_link import list_ports


def test_is_android_false_on_desktop():
    """桌面环境必须识别为 False，否则会误走 USB 桥分支"""
    assert is_android() is False


def test_list_devices_empty_on_desktop():
    """没有 Java 桥时返回空列表，而不是抛异常"""
    assert list_usb_devices() == []


def test_request_permission_false_on_desktop():
    assert request_permission() is False


def test_link_raises_clear_error_off_android():
    """非 Android 下 connect 必须给出可读原因，不能是 AttributeError"""
    link = ChaquopyUsbLink('usb:-1:-1', 921600, on_data=lambda d: None)
    with pytest.raises(RuntimeError) as ei:
        link.connect()
    assert 'Android' in str(ei.value)


def test_link_is_open_false_off_android():
    link = ChaquopyUsbLink('usb:-1:-1', 921600, on_data=lambda d: None)
    assert link.is_open is False
    link.close()          # 未连接时关闭也不能炸


def test_parse_spec_vid_pid():
    """端点里的 vid/pid 要解析出来，连接时不再让用户选第二遍"""
    link = ChaquopyUsbLink('usb:6790:29987', 115200, on_data=lambda d: None)
    assert link.vid == 6790        # 0x1A86 CH340
    assert link.pid == 29987


def test_list_ports_still_works_on_desktop():
    """Android 分支不可用时，list_ports 仍走 pyserial（桌面正常路径不受影响）"""
    ports = list_ports()
    assert isinstance(ports, list)
    for p in ports:
        assert 'port' in p and 'desc' in p


def test_open_link_dispatches_usb():
    """open_link 认 usb: 前缀，交给 Chaquopy 链路"""
    from server.udp_link import open_link
    link = open_link('usb:6790:29987', 921600, lambda d: None)
    assert isinstance(link, ChaquopyUsbLink)
    assert not link.is_open       # 只建对象，不连接
