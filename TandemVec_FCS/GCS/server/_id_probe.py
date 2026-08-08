"""临时诊断：读取在线辨识结果（DBG id 命令）+ 任务统计。"""
import asyncio
import re
import time
import websockets

async def cmd(ws, line, seconds=4):
    await ws.send(f'{{"cmd":"dbg_cmd","line":"{line}"}}')
    out = []
    t0 = time.time()
    try:
        while time.time() - t0 < seconds:
            msg = await asyncio.wait_for(ws.recv(), timeout=2)
            if '"dbg_out"' in msg:
                m = re.search(r'"line":\s*"((?:[^"\\]|\\.)*)"', msg)
                if m:
                    out.append(m.group(1).encode().decode('unicode_escape'))
    except asyncio.TimeoutError:
        pass
    return ''.join(out)

async def main():
    uri = 'ws://127.0.0.1:8091/ws'
    async with websockets.connect(uri, max_queue=0) as ws:
        await ws.recv()
        await ws.send('{"cmd":"connect","port":"COM10","baud":2000000}')
        await asyncio.sleep(3)
        await ws.send('{"cmd":"dbg_enter"}')
        await asyncio.sleep(1)
        print('===== id =====')
        print(await cmd(ws, 'id'))
        await ws.send('{"cmd":"dbg_exit"}')

asyncio.run(main())
