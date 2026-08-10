#!/usr/bin/env python3
"""bb_tool.py — 黑匣子读取与分析工具（驱动 GCS 后端 DBG flash 导出链路）

用法（在 TandemVec_FCS/GCS 下）:
  py -3.12 server/bb_tool.py --list              # 列出飞行段
  py -3.12 server/bb_tool.py --latest            # 导出最新段 → output/blackbox_segN.csv
  py -3.12 server/bb_tool.py --seg 3             # 导出指定段
  py -3.12 server/bb_tool.py --latest --analyze  # 导出并分析 roll/pitch 阶跃超调

数据链路：WS → dbg_enter → flash_findseg → flash_export → 后端解析
(blackbox.py decode_frames) → flash_done 广播（最多 3000 行时间抽样预览）。
"""
import argparse
import asyncio
import json
import re
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import anocom  # noqa: E402  (仅用于帧类型命名，实际解析在后端)

OUTPUT_DIR = Path(__file__).parent.parent / 'output'
WS_URL = 'ws://127.0.0.1:8091/ws'


def _field(msg: str, name: str):
    m = re.search(r'"' + name + r'":\s*("(?:[^"\\]|\\.)*"|[^,}\]]+)', msg)
    return m.group(1).strip('"') if m else None


class BbTool:
    def __init__(self):
        self.segments = []       # flash_segments: {num, page, tms}
        self.done = None         # flash_done 消息原文
        self.queue = []

    async def _recv_until(self, ws, want_type, timeout=240.0, verbose=False):
        """收消息直到出现 want_type（期间积累 segments/进度/告警）"""
        t0 = time.time()
        last_prog = 0
        while time.time() - t0 < timeout:
            try:
                msg = await asyncio.wait_for(ws.recv(), timeout=5)
            except asyncio.TimeoutError:
                continue
            if '"flash_segments"' in msg:
                try:
                    segs = json.loads(msg)['segments']
                    self.segments = segs
                except Exception:
                    pass
            if '"flash_done"' in msg:
                self.done = json.loads(msg)
                return self.done
            if '"flash_progress"' in msg:
                try:
                    done = json.loads(msg)['done']
                except Exception:
                    done = 0
                if verbose and done != last_prog:
                    print(f'  进度: {done} 页', end='\r')
                    last_prog = done
                continue
            if '"log"' in msg:
                try:
                    lv = json.loads(msg)
                    print(f"  [后端] {lv.get('level', '?')}: {lv.get('msg', '')}")
                except Exception:
                    pass
                continue
            if want_type and ('"' + want_type + '"') in msg:
                return msg
        raise TimeoutError(f'等待 {want_type} 超时')

    async def run(self, args):
        async with __import__('websockets').connect(WS_URL, max_queue=0) as ws:
            await ws.recv()  # hello
            # 等 conn 快照：已连接则跳过 connect（重复 connect 会触发串口重连，
            # 把 DBG 模式重置回 telemetry，导致后续 findseg 响应丢失）
            connected = False
            t0 = time.time()
            while time.time() - t0 < 5:
                try:
                    msg = await asyncio.wait_for(ws.recv(), timeout=2)
                except asyncio.TimeoutError:
                    continue
                if '"conn"' in msg:
                    connected = _field(msg, 'status') == 'connected'
                    break
            if not connected:
                await ws.send('{"cmd":"connect","port":"COM10","baud":921600}')
                await asyncio.sleep(3)
            await ws.send('{"cmd":"dbg_enter"}')
            await asyncio.sleep(1)

            await ws.send('{"cmd":"flash_findseg"}')
            await self._recv_until(ws, 'flash_segments', timeout=20)
            if not self.segments:
                print('未找到飞行段（flash findseg 无输出）')
                return 1
            print(f'飞行段: {len(self.segments)} 个')
            for s in self.segments:
                print(f"  seg {s['num']}: page={s['page']} tms={s.get('tms', '?')}")

            if args.list:
                return 0

            if args.seg is not None:
                seg = next((s for s in self.segments if s['num'] == args.seg), None)
            else:
                seg = max(self.segments, key=lambda s: s['num'])
            if seg is None:
                print(f'段 {args.seg} 不存在')
                return 1

            print(f'导出 seg {seg["num"]} (page {seg["page"]}, {args.pages} 页)...')
            await ws.send(json.dumps({'cmd': 'flash_export',
                                      'start': seg['page'], 'count': args.pages}))
            try:
                done = await self._recv_until(ws, 'flash_done',
                                              timeout=args.timeout, verbose=True)
                print()
            except TimeoutError:
                print('\n导出超时（链路丢包重试过多；可加 --timeout 或减 --pages）')
                return 1
            finally:
                # 无论成败都退出 DBG（避免后端 DbgSession/固件卡在导出态）
                await ws.send('{"cmd":"dbg_exit"}')

            rows = done.get('rows', [])
            columns = done.get('columns', [])
            row_count = done.get('row_count', len(rows))
            step = done.get('sample_step', 1)
            print(f'完成: {row_count} 行 (抽样 step={step}, 预览 {len(rows)} 行), '
                  f'{len(columns)} 通道')
            # ★ 2026-08-10 全面审查修复：明确警告 CSV 是抽样预览（长段丢 15/16 数据，
            #   超调/频谱分析精度大幅下降）——全量分析请用黑匣子分段参数或减段时长
            if step > 1:
                print(f'⚠ CSV 仅含抽样预览（step={step}，实际 {row_count} 行 → 写出 '
                      f'{len(rows)} 行）——定量分析（超调/频谱）精度受限，建议分段导出')

            out = args.out or (OUTPUT_DIR / f'blackbox_seg{seg["num"]}.csv')
            out.parent.mkdir(exist_ok=True)
            ncol = len(columns)
            with open(out, 'w', newline='', encoding='utf-8') as f:
                # 扁平结构: [seq, frame_t_ms, v0..v_{ncol-1}, seg]
                f.write('seq,frame_t_ms,' + ','.join(columns) + ',seg\n')
                for r in rows:
                    f.write(str(r[0]) + ',' + str(r[1]) + ',' +
                            ','.join(str(v) for v in r[2:2 + ncol]) + ',' +
                            str(r[-1]) + '\n')
            print(f'CSV → {out}')

            if args.analyze:
                analyze_overshoot(rows, columns)
            await ws.send('{"cmd":"dbg_exit"}')
            return 0


def analyze_overshoot(rows, columns):
    """基于目标姿态通道的阶跃超调分析（角度模式下直接对比 target vs actual）"""
    ncol = len(columns)
    axes = []
    for axis, act_c, tar_c in (('roll', 'roll_deg', 'target_roll_deg'),
                               ('pitch', 'pitch_deg', 'target_pitch_deg')):
        if act_c in columns and tar_c in columns:
            axes.append((axis, columns.index(act_c), columns.index(tar_c)))
    if not axes:
        print('\n无目标姿态通道（需含 target_roll_deg/target_pitch_deg 的角度模式数据）')
        return
    print('\n===== 姿态阶跃超调分析（target vs actual）=====')
    for axis, i_act, i_tar in axes:
        t = [r[1] / 1000.0 for r in rows]          # ms → s
        a = [r[2 + i_act] for r in rows]
        g = [r[2 + i_tar] for r in rows]
        found = 0
        i = 1
        n = len(a)
        while i < n and found < 10:
            dg = g[i] - g[i - 1]
            if abs(dg) >= 3.0:                     # 目标阶跃 ≥3°
                t0 = t[i]
                base, target = g[i - 1], g[i]
                # 实际响应峰值：阶跃后 2s 窗口内与目标同向的极值
                j = i
                while j < n and t[j] - t0 < 2.0:
                    j += 1
                window = a[i:j]
                peak = max(window) if dg > 0 else min(window)
                # 超调 = 越过目标的部分 / 阶跃幅度
                over = (peak - target) * (1 if dg > 0 else -1)
                os_pct = over / abs(dg) * 100.0
                if over > 0.05:                    # 只报真实过冲
                    found += 1
                    print(f"  {axis}: t={t0:6.2f}s  目标 {base:+6.1f}°→{target:+6.1f}° "
                          f"({dg:+5.1f}°)  实际峰值 {peak:+6.1f}°  超调 {os_pct:+5.1f}%")
                i = j
            else:
                i += 1
        if found == 0:
            print(f'  {axis}: 未检测到目标阶跃（角度模式下打杆并保持 ≥2s）或无非零超调')


def main():
    ap = argparse.ArgumentParser(description='黑匣子读取与分析')
    ap.add_argument('--list', action='store_true', help='只列飞行段')
    ap.add_argument('--latest', action='store_true', help='导出最新段')
    ap.add_argument('--seg', type=int, default=None, help='导出指定段号')
    ap.add_argument('--out', type=str, default=None, help='CSV 输出路径')
    ap.add_argument('--analyze', action='store_true', help='导出后分析超调')
    ap.add_argument('--pages', type=int, default=512, help='导出页数（512≈4分钟飞行数据，全量 2048 很慢）')
    ap.add_argument('--timeout', type=float, default=600.0, help='导出超时秒数')
    args = ap.parse_args()
    if not (args.list or args.latest or args.seg is not None):
        ap.error('需指定 --list / --latest / --seg')
    try:
        rc = asyncio.run(BbTool().run(args))
    except KeyboardInterrupt:
        rc = 130
    sys.exit(rc)


if __name__ == '__main__':
    main()
