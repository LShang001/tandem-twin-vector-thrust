# -*- coding: utf-8 -*-
"""py-web-lab 服务入口：FastAPI + WebSocket + 静态前端。

启动：python -m uvicorn main:app --host 127.0.0.1 --port 8090
（或 ./start.bat；前端 http://127.0.0.1:8090/）
"""
from pathlib import Path

from fastapi import FastAPI, WebSocket
from fastapi.staticfiles import StaticFiles

from sim import Simulation
import protocol

app = FastAPI(title='py-web-lab', version='0.1.0')

WEB_DIR = Path(__file__).resolve().parent.parent / 'web'


@app.websocket('/ws')
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    sim = Simulation()
    try:
        while True:
            msg = await ws.receive_json()
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
