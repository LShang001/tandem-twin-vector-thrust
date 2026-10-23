"""repro_exit.py — 复现"退出/卡死"问题的最小 WebSocket 脚本。

@file    repro_exit.py
@module  analysis/py-web-lab 调试脚本
@brief   连 8090 服务，reset vtol 后给固定杆量推 157 帧，打印遥测末态。
@details 输入：本地 8090 服务；处理：reset → set(dt/dw/altHold) → step×157；
         输出：stdout 的 euler 与 S.dt/S.dw。
@theory  N/A（现象复现脚本）
@frame  S.quat 为 JS 顺序 (x,y,z,w)；euler 为 rad
@units   摆角 rad（脚本内用 度×0.0174533 换算）、dt s
@note    一次性复现脚本，不参与 CI；服务未启动时直接抛连接异常。
@see     server/protocol.py、tests/smoke_ws.py
"""
import asyncio, json, websockets


async def main():
    async with websockets.connect('ws://127.0.0.1:8090/ws') as ws:
        async def rpc(m):
            await ws.send(json.dumps(m))
            return json.loads(await ws.recv())
        await rpc({'cmd': 'reset', 'mode': 'vtol'})
        await rpc({'cmd': 'set', 'S': {'dt': 12*0.0174533, 'dw': 40*0.0174533, 'altHold': True}})
        for _ in range(157):
            r = await rpc({'cmd': 'step', 'dt': 0.016})
        print('悬停 2.5s:', r['F']['euler'], 'S.dt=', r['S']['dt'], 'dw=', r['S']['dw'])
        r = await rpc({'cmd': 'reset', 'mode': 'cruise'})
        print('reset 后立刻:', r['F']['euler'], 'S.dt=', r['S']['dt'], 'S.dw=', r['S']['dw'], 'thr=', r['S']['thr'])
        for _ in range(157):
            r = await rpc({'cmd': 'step', 'dt': 0.016})
        print('巡航 2.5s:', r['F']['euler'], 'thr=', r['S']['thr'])

asyncio.run(main())
