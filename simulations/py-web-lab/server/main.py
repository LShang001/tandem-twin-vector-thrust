# -*- coding: utf-8 -*-
"""main.py — py-web-lab 服务入口：FastAPI + WebSocket + 静态前端。

@file    main.py
@module  analysis/py-web-lab 服务
@brief   暴露 /ws 命令通道与静态页面，把命令转给 Simulation 与 protocol。
@details 输入：WebSocket JSON 命令；处理：校验（protocol.handle）→ 推进/改状态 → 回包；
         输出：state/params/error 帧 + 静态文件。
         启动：python -m uvicorn main:app --host 127.0.0.1 --port 8090（或 ./start.bat）；
         关闭：页面「关闭服务」按钮（WebSocket shutdown → 进程退出）、Ctrl+C 或按端口杀进程。
@theory  N/A（服务装配，物理在 sim.py/controllers.py）
@frame  N/A
@units   N/A（dt 为秒，见 protocol）
@note    端口固定 8090；改后端后必须杀掉旧进程再启动，否则"界面全新、后端逻辑全旧"。
@see     sim.py、protocol.py、web/src/main.mjs、@ref AGENTS.md（GCS 残留进程踩坑）
"""
import os
from pathlib import Path

from fastapi import FastAPI, WebSocket
from fastapi.staticfiles import StaticFiles

from sim import Simulation
import protocol

app = FastAPI(title='py-web-lab', version='0.1.0')

WEB_DIR = Path(__file__).resolve().parent.parent / 'web'


@app.websocket('/ws')
async def ws_endpoint(ws: WebSocket):
    """WebSocket 命令通道：接收 JSON 命令 → 分发 → 回 state/params/error。"""
    await ws.accept()
    sim = Simulation()
    try:
        while True:
            msg = await ws.receive_json()
            if isinstance(msg, dict) and msg.get('cmd') == 'shutdown':
                # Web 关闭服务：先回确认，再退出进程（uvicorn 单进程模式）
                await ws.send_json({'type': 'bye', 'msg': 'server shutting down'})
                await ws.close()
                os._exit(0)
            resp = protocol.handle(sim, msg)
            if resp is not None:
                await ws.send_json(resp)
    except Exception as exc:  # 客户端断开 → 静默退出；服务端缺陷 → 打日志（review #5）
        import logging
        if exc.__class__.__name__ not in ('WebSocketDisconnect', 'ClientDisconnected'):
            logging.getLogger('py-web-lab').warning('ws error: %r', exc)
        try:
            await ws.close()
        except Exception:
            pass


app.mount('/', StaticFiles(directory=str(WEB_DIR), html=True), name='web')
