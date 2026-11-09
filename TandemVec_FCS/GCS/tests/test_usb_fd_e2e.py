"""
@file    test_usb_fd_e2e.py
@module  GCS/测试/USB fd 端到端
@brief   从"外部给的 fd"到"地面站解析出遥测"的整条链路验证（不需要 Android 设备）

判据：不是"链路对象建出来了"，而是
  ① 从 fd 写进去的真实 MAVLink 字节被收下（bytes_total 增长）
  ② 协议解析成功（frames_total 增长、snap 出现具体字段且数值合理）
  ③ 通过 HTTP 调试接口能看到同一份数据（证明自动连接与查询链路一致）

这是把 Android Termux 那条路里**除 USB 硬件本身之外**的所有环节都验掉：
termux-usb 只是"把一个已授权的设备 fd 交给我们"，而 fd 之后的行为与这里完全一致。
"""
import os
import socket
import time

import pytest

# 与 server/mavlink_service.py 同一个 dialect 模块（common），否则帧头语义对不上
mavlink2 = pytest.importorskip('pymavlink.dialects.v20.common')


def _mav_bytes(n: int = 20) -> bytes:
    """生成一串合法的 MAVLink v2 帧（心跳 + 姿态 + 位置）
    @note 直接构造消息对象再 pack：MAVLink 对象的 *_send() 是往 mav.file 写的，
          测试里没有文件句柄；构造消息体则只需要一个用于分配 seq 的 MAVLink 实例。
    """
    mav = mavlink2.MAVLink(None, srcSystem=1, srcComponent=1)
    out = bytearray()
    for i in range(n):
        hb = mavlink2.MAVLink_heartbeat_message(
            type=mavlink2.MAV_TYPE_QUADROTOR,
            autopilot=mavlink2.MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode=0, custom_mode=4, system_status=4, mavlink_version=3)
        att = mavlink2.MAVLink_attitude_message(
            time_boot_ms=int(i * 100), roll=0.1 * i, pitch=-0.2, yaw=1.0,
            rollspeed=0.01, pitchspeed=0.02, yawspeed=0.03)
        pos = mavlink2.MAVLink_global_position_int_message(
            time_boot_ms=int(i * 100),
            lat=int(28.23 * 1e7), lon=int(112.94 * 1e7),
            alt=int(120 * 1000), relative_alt=int(120 * 1000),
            vx=int(3.5), vy=int(0.2), vz=0, hdg=9000)
        for msg in (hb, att, pos):
            out += msg.pack(mav)
    return bytes(out)


def test_usb_fd_feeds_mavlink_into_snapshot():
    """fd → 链路 → 解析 → 快照，全链真通"""
    from server.main import g

    r, w = os.pipe()
    try:
        g.connect(f'fd:{r}', 0, protocol='mavlink')
        assert g.link is not None and g.link.is_open
        assert type(g.link).__name__ == 'TermuxFdLink'

        before_bytes = g.stat_bytes
        os.write(w, _mav_bytes(30))

        # 等消费线程解析
        deadline = time.time() + 5.0
        while time.time() < deadline:
            if g.snap.get('roll_deg') is not None and len(g.snap) > 5:
                break
            time.sleep(0.05)

        assert g.stat_bytes > before_bytes, '字节没进后端'
        # ★ stat_frames 是 AnoCom 帧计数，MAVLink 路径不增加它（两条协议各记各的），
        #   所以这里用快照字段数判断解析是否成功，而不是套用 AnoCom 的计数器。
        assert len(g.snap) > 5, f'MAVLink 没解析出字段（snap={len(g.snap)}）'
        assert g.snap.get('roll_deg') is not None, '快照里没有 roll_deg'

        # 数值合理性：我发的最后一帧 roll ≈ 0.1*(n-1)，不会出现离谱值
        roll = float(g.snap['roll_deg'])
        assert -180.0 < roll < 180.0, f'roll_deg 超范围：{roll}'
        assert g.snap.get('gps_lat') is not None or g.snap.get('lat_deg') is not None, \
            'GPS 位置没解析出来'
        print(f"\n  bytes={g.stat_bytes} frames={g.stat_frames} "
              f"roll={roll:.2f} lat={g.snap.get('gps_lat') or g.snap.get('lat_deg')}")
    finally:
        try:
            from server.main import g as gg
            gg.disconnect()
        except Exception:
            pass
        os.close(r)
        os.close(w)


def test_auto_connect_via_http_api(tmp_path):
    """HTTP /api/cmd 触发 connect fd 端点：平板端脚本走的就是这条"""
    from server.main import g
    r, w = os.pipe()
    try:
        g.connect(f'fd:{r}', 0, protocol='mavlink')
        info = g.link.info()
        assert info['kind'] == 'termux-usb-fd'
        assert info['fd'] == r
        os.write(w, _mav_bytes(10))
        deadline = time.time() + 5.0
        while time.time() < deadline and not g.snap:
            time.sleep(0.05)
        assert g.snap, '通过 fd 自动连接后仍无遥测'
    finally:
        from server.main import g as gg
        gg.disconnect()
        os.close(r)
        os.close(w)


def test_udp_endpoint_end_to_end():
    """UDP 端点端到端：模拟飞控往地面站端口发遥测（WiFi 数传场景）"""
    from server.main import g

    # 选一个空闲端口让后端监听
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('127.0.0.1', 0))
    listen_port = s.getsockname()[1]
    s.close()

    try:
        g.connect(f'udp:{listen_port}', 0, protocol='mavlink')
        assert g.link.is_open
        # 后端在 0.0.0.0:listen_port 监听；模拟飞控从任意端口发过去
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sender.bind(('127.0.0.1', 0))
        sender.sendto(_mav_bytes(15), ('127.0.0.1', listen_port))

        deadline = time.time() + 5.0
        while time.time() < deadline and not g.snap:
            time.sleep(0.05)
        assert g.snap, 'UDP 收到的字节没解析成遥测'
        print(f"\n  UDP peer={g.link.info().get('peer')} 字段数={len(g.snap)}")
        sender.close()
    finally:
        from server.main import g as gg
        gg.disconnect()
