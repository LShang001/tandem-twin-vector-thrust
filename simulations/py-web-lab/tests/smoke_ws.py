# -*- coding: utf-8 -*-
"""协议冒烟测试：init → reset vtol → step 8s → set altHold → 验证遥测"""
import asyncio
import json

import websockets


async def main():
    async with websockets.connect('ws://127.0.0.1:8090/ws') as ws:
        # init
        await ws.send(json.dumps({'cmd': 'init'}))
        r = json.loads(await ws.recv())
        assert r['type'] == 'state' and 'params' in r, 'init 响应异常'
        P = r['params']
        print(f'[OK] init: params {len(P)} 个, quat={r["S"]["quat"]}, thr={r["S"]["thr"]:.4f}')

        # reset vtol
        await ws.send(json.dumps({'cmd': 'reset', 'mode': 'vtol'}))
        r = json.loads(await ws.recv())
        assert r['S']['vtolMode'] is True, 'vtolMode 未生效'
        print(f'[OK] reset vtol: thr={r["S"]["thr"]:.4f} (悬停配平), euler={r["F"]["euler"]}')

        # step 8s（逐帧 0.016）
        for _ in range(500):
            await ws.send(json.dumps({'cmd': 'step', 'dt': 0.016}))
            r = json.loads(await ws.recv())
        print(f'[OK] 8s 悬停: t={r["t"]:.1f} thr={r["S"]["thr"]:.4f} '
              f'omega=({r["S"]["omega"]["x"]:+.4f},{r["S"]["omega"]["y"]:+.4f},{r["S"]["omega"]["z"]:+.4f})')

        # 定高
        await ws.send(json.dumps({'cmd': 'set', 'S': {'altHold': True, 'altRef': 5.0}}))
        r = json.loads(await ws.recv())
        assert r['S']['altHold'] is True
        for _ in range(500):
            await ws.send(json.dumps({'cmd': 'step', 'dt': 0.016}))
            r = json.loads(await ws.recv())
        print(f'[OK] 定高 8s: h={-r["F"]["pos"]["z"]:.2f} m (目标 5.0)')

        # 俯仰扰动 → 收敛
        await ws.send(json.dumps({'cmd': 'pulse', 'omega': {'y': 0.3}}))
        r = json.loads(await ws.recv())
        for _ in range(500):
            await ws.send(json.dumps({'cmd': 'step', 'dt': 0.016}))
            r = json.loads(await ws.recv())
        err = max(abs(r['S']['omega'][k]) for k in ('x', 'y', 'z'))
        print(f'[OK] 扰动后 8s: omega 最大 {err:.4f} rad/s (应 <0.05)')
        assert err < 0.05, '扰动未收敛'

        # 巡航复位
        await ws.send(json.dumps({'cmd': 'reset', 'mode': 'cruise'}))
        r = json.loads(await ws.recv())
        print(f'[OK] reset cruise: thr={r["S"]["thr"]:.4f} euler.y={r["F"]["euler"]["y"]:+.4f}')
        await ws.send(json.dumps({'cmd': 'set', 'S': {'sasMode': 0}}))
        r = json.loads(await ws.recv())
        assert r['S']['sasMode'] == 0
        print('[OK] set sasMode=0')

        # 暂停：step 不推进
        await ws.send(json.dumps({'cmd': 'set', 'S': {'paused': True}}))
        r = json.loads(await ws.recv())
        t0 = r['t']
        await ws.send(json.dumps({'cmd': 'step', 'dt': 0.5}))
        r = json.loads(await ws.recv())
        assert abs(r['t'] - t0) < 1e-9, '暂停后仍推进'
        print(f'[OK] 暂停生效（t 保持 {r["t"]:.2f}）')
        await ws.send(json.dumps({'cmd': 'set', 'S': {'paused': False}}))
        await ws.recv()
        await ws.send(json.dumps({'cmd': 'step', 'dt': 0.1}))
        r = json.loads(await ws.recv())
        assert r['t'] > t0 + 0.09
        print(f'[OK] 恢复推进（t={r["t"]:.2f}）')

        # 注入拒绝：nan / inf / 越界 / 执行器字段 / 字符串布尔
        bad = [
            ({'cmd': 'set', 'S': {'thr': float('nan')}}, 'nan'),
            ({'cmd': 'set', 'S': {'thr': float('inf')}}, 'inf'),
            ({'cmd': 'set', 'S': {'dwAct': 1.5}}, 'dwAct 越界(白名单外)'),
            ({'cmd': 'set', 'S': {'aero': 'false'}}, '字符串布尔'),
            ({'cmd': 'pulse', 'omega': {'y': float('nan')}}, 'pulse nan'),
            ({'cmd': 'step', 'dt': float('nan')}, 'step dt nan'),
            ({'cmd': 'set', 'S': {'sasMode': 9}}, 'sasMode 越界'),
        ]
        for msg, tag in bad:
            await ws.send(json.dumps(msg))
            r = json.loads(await ws.recv())
            assert r['type'] == 'error', f'{tag} 未被拒绝: {r}'
            print(f'[OK] 拒绝 {tag}')

        print('\n=== 协议冒烟全部通过 ===')


asyncio.run(main())
