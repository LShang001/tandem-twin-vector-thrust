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
