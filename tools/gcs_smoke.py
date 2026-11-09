#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    gcs_smoke.py
@module  守卫 / GCS 冒烟测试
@brief   起一个真后端，验证资源可达、WS 命令有应答、关键数据链路通
@details 为什么需要它：前十几轮每次都是"起服务 → 开浏览器 → 点一遍 → 截图"，
         这套流程靠人做，一旦某轮改动把某个页面或某条 WS 命令弄坏，只有
         **下次恰好点到那一页**才会发现。这个脚本把"点一遍"变成可自动跑的判据。

         与浏览器实测的分工：
           · 这里验证**链路与协议**（资源 200、命令有正确类型的应答、数据在流）
           · 浏览器验证**渲染**（画出来的东西对不对、布局好不好看）
         两者不能互相替代：脚本看不到"曲线是空的"，截图也发现不了"某个 WS 命令
         返回了错误类型"。

         用 DEMO 模式起服务，所以不依赖任何硬件。

@theory  N/A
@frame   N/A
@units   N/A
@note    自己起自己关：用子进程 + finally 清理，不留下孤儿端口占用。
@test    本脚本自身即测试；退出码 0 = 全通过
@see     TandemVec_FCS/GCS/server/main.py、tools/verify-all.py
"""

from __future__ import annotations

import json
import os
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

GCS = Path(__file__).resolve().parents[1] / 'TandemVec_FCS' / 'GCS'
PORT = 8097


def _free_port(start: int = PORT) -> int:
    """找一个空闲端口，避免和正在运行的服务撞车"""
    for p in range(start, start + 40):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            if s.connect_ex(('127.0.0.1', p)) != 0:
                return p
    return start


def _http(url: str, timeout: float = 6.0):
    req = urllib.request.Request(url)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return resp.status, resp.read()


def _ws_roundtrip(port: int, commands: list[dict], timeout: float = 8.0) -> list[str]:
    """连一次 WS，依次发命令，收集所有收到的消息文本"""
    try:
        from websockets.sync.client import connect
    except Exception:
        return []
    got: list[str] = []
    with connect(f'ws://127.0.0.1:{port}/ws', open_timeout=timeout) as ws:
        for cmd in commands:
            ws.send(json.dumps(cmd))
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                msg = ws.recv(timeout=1.0)
            except Exception:
                continue
            if isinstance(msg, bytes):
                msg = msg.decode('utf-8', 'ignore')
            got.append(msg)
            if len(got) > 200:
                break
    return got


# 需要存在的静态资源：页面外壳 + 全部前端模块 + vendor 库
STATIC_PATHS = [
    '/',
    '/index.html',
    '/css/style.css',
    '/manifest.webmanifest',
    '/sw.js',
    '/src/main.mjs',
    '/src/widget.mjs',
    '/src/dashboard.mjs',
    '/src/view3d.mjs',
    '/src/scope.mjs',
    '/src/gnss.mjs',
    '/src/map.mjs',
    '/src/video.mjs',
    '/src/alarms.mjs',
    '/src/tuning.mjs',
    '/src/calib.mjs',
    '/src/inspector.mjs',
    '/src/blackbox.mjs',
    '/src/params.mjs',
    '/src/console.mjs',
    '/src/layout.mjs',
    '/vendor/leaflet-1.9.4/leaflet.js',
    '/vendor/leaflet-1.9.4/leaflet.css',
    '/vendor/hls/hls.min.js',
]

# 必须能被后端起响应的 WS 命令（只选不依赖硬件、不改变状态的）
WS_COMMANDS = [
    {'cmd': 'vehicles'},
    {'cmd': 'alarm_state'},
    {'cmd': 'tuning_channels'},
    {'cmd': 'calib_wizard'},
    {'cmd': 'inspector'},
]


def main() -> int:
    """命令行入口。"""
    port = _free_port()
    env = dict(os.environ)
    env['GCS_DEMO'] = '1'
    log_path = GCS.parents[1] / 'tmp' / 'gcs_smoke.log'
    log_path.parent.mkdir(exist_ok=True)
    logf = open(log_path, 'w', encoding='utf-8', errors='ignore')
    proc = None
    fails: list[str] = []
    try:
        proc = subprocess.Popen(
            [sys.executable, '-m', 'uvicorn', 'server.main:app',
             '--host', '127.0.0.1', '--port', str(port)],
            cwd=str(GCS), env=env, stdout=logf, stderr=subprocess.STDOUT)
        # 等后端起来（最多 25 s）
        base = f'http://127.0.0.1:{port}'
        up = False
        for _ in range(50):
            time.sleep(0.5)
            try:
                code, _body = _http(base + '/api/status', timeout=2.0)
                if code == 200:
                    up = True
                    break
            except Exception:
                continue
        if not up:
            print('[FAIL] 后端 25 s 内未就绪，日志见', log_path, file=sys.stderr)
            return 2

        # 1) 静态资源
        for p in STATIC_PATHS:
            try:
                code, body = _http(base + p)
            except Exception as exc:
                fails.append(f'资源 {p} 请求失败: {exc}')
                continue
            if code != 200:
                fails.append(f'资源 {p} 返回 {code}')
            elif p.endswith(('.mjs', '.js')) and len(body) < 40:
                fails.append(f'资源 {p} 内容过短（{len(body)} B），可能为空文件')

        # 2) MJPEG 测试流：只取前几 KB，确认是 multipart 且含 JPEG
        try:
            req = urllib.request.Request(base + '/video/test.mjpg')
            with urllib.request.urlopen(req, timeout=8.0) as resp:
                ctype = resp.headers.get('Content-Type', '')
                chunk = resp.read(4096)
            if 'multipart/x-mixed-replace' not in ctype:
                fails.append(f'测试视频流 Content-Type 异常: {ctype}')
            if b'Content-Type: image/jpeg' not in chunk:
                fails.append('测试视频流未包含 JPEG 分块')
        except Exception as exc:
            fails.append(f'测试视频流失败: {exc}')

        # 3) WS 命令应答
        try:
            msgs = _ws_roundtrip(port, WS_COMMANDS)
        except Exception as exc:
            msgs = []
            fails.append(f'WS 连接失败: {exc}')
        if not msgs:
            fails.append('WS 未收到任何消息')
        else:
            types = set()
            for m in msgs:
                try:
                    types.add(json.loads(m).get('type'))
                except Exception:
                    continue
            for want in ('vehicles', 'alarm_state', 'tuning_channels',
                         'calib_wizard', 'inspector'):
                if want not in types:
                    fails.append(f'WS 命令 {want} 无对应应答（收到类型: {sorted(t for t in types if t)}）')
            if 'telemetry' not in types:
                fails.append('WS 未推送 telemetry（演示数据链路断了）')

        # 4) 多机：DEMO 交替 sysid，应能看到至少 1 台
        try:
            msgs2 = _ws_roundtrip(port, [{'cmd': 'vehicles'}])
            veh = None
            for m in msgs2:
                try:
                    d = json.loads(m)
                except Exception:
                    continue
                if d.get('type') == 'vehicles':
                    veh = d.get('vehicles')
                    break
            if veh is not None and len(veh) == 0:
                fails.append('vehicles 返回空列表（演示模式应当有至少一台）')
        except Exception as exc:
            fails.append(f'多机查询失败: {exc}')

    finally:
        if proc is not None:
            proc.terminate()
            try:
                proc.wait(timeout=8)
            except Exception:
                proc.kill()
        logf.close()

    if fails:
        print(f'[FAIL] GCS 冒烟 {len(fails)} 项不通过：')
        for f in fails:
            print('   -', f)
        print('   日志：', log_path)
        return 1
    print(f'[OK] GCS 冒烟通过（{len(STATIC_PATHS)} 个资源 + '
          f'{len(WS_COMMANDS)} 条 WS 命令 + 视频流 + 多机）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
