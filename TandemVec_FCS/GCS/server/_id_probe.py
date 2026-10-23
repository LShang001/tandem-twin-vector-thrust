"""临时诊断：读取在线辨识结果（DBG id 命令）+ 任务统计。

@file    _id_probe.py
@module  GCS/临时诊断脚本
@brief   通过 WS 发送 DBG `id` 命令，收集在线辨识结果与任务统计
@details 输入：固定 ws://127.0.0.1:8091/ws 与 COM10；处理：connect → dbg_enter →
         发送 id → 收集 dbg_out 若干秒 → dbg_exit；输出：stdout 文本。
         说明：一次性排查脚本，非生产路径，端口/串口写死。
@theory  N/A
@ref     AGENTS.md 踩坑记录（DBG 调试模式、在线辨识）
@frame   N/A
@units   N/A
"""
import asyncio
import re
import time
import websockets

async def cmd(ws, line, seconds=4):
    """@brief 发送一条 DBG 命令并收集若干秒内的 dbg_out 文本
    @param ws WebSocket 连接；line DBG 命令行；seconds 收集时长 [s]
    @return 拼接后的输出文本（超时未收到则为空串）
    """
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
    """@brief 连接后端 WS，进入 DBG，读取一次 `id` 在线辨识输出后退出"""
    uri = 'ws://127.0.0.1:8091/ws'
    async with websockets.connect(uri, max_queue=0) as ws:
        await ws.recv()
        await ws.send('{"cmd":"connect","port":"COM10","baud":921600}')
        await asyncio.sleep(3)
        await ws.send('{"cmd":"dbg_enter"}')
        await asyncio.sleep(1)
        print('===== id =====')
        print(await cmd(ws, 'id'))
        await ws.send('{"cmd":"dbg_exit"}')

asyncio.run(main())
