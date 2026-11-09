"""
@file    gcs_ctl.py
@module  工具/GCS 调试入口
@brief   给人和 agent 用的地面站统一 CLI：查能力、读快照、调任意命令、起停演示后端
@details 为什么要有它：地面站有 46 个 WS 命令 + 若干 HTTP 接口，如果每次都要
         "开浏览器点点点"或手写 curl+WebSocket，自动化测试和排障就没法做。
         这里把"调一个命令"收敛成一行，并且默认输出机器可读 JSON——脚本和 agent
         直接吃，人也能看懂。

用法：
    py -3.12 tools/gcs_ctl.py caps                       # 有哪些能力
    py -3.12 tools/gcs_ctl.py modules                    # 按模块列命令
    py -3.12 tools/gcs_ctl.py status                     # 后端/链路状态
    py -3.12 tools/gcs_ctl.py snap                       # 当前遥测快照
    py -3.12 tools/gcs_ctl.py cmd spectrum channel=pitch_deg
    py -3.12 tools/gcs_ctl.py cmd scope_stats channel=roll_deg
    py -3.12 tools/gcs_ctl.py demo start                 # 起演示后端（GCS_DEMO=1）
    py -3.12 tools/gcs_ctl.py demo stop                  # 关掉它
    py -3.12 tools/gcs_ctl.py selftest                   # 端到端自检

约定：参数用 `key=value`，值按 JSON 解析（失败则当字符串）；数组写 `k=[a,b]`。
"""
from __future__ import annotations

import argparse
import json
import os
import pathlib
import socket
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
GCS = ROOT / 'TandemVec_FCS' / 'GCS'
DEFAULT_HOST = '127.0.0.1'
DEFAULT_PORT = int(os.environ.get('GCS_PORT', '8091'))


def _parse_kv(pairs: list[str]) -> dict:
    """@brief 把 key=value 列表解析成字典（值优先按 JSON 解析）"""
    out = {}
    for p in pairs:
        if '=' not in p:
            raise SystemExit(f'参数要写成 key=value：{p}')
        k, v = p.split('=', 1)
        try:
            out[k] = json.loads(v)
        except json.JSONDecodeError:
            out[k] = v
    return out


def _http(method: str, path: str, body: dict | None = None,
          host: str = DEFAULT_HOST, port: int | None = None, timeout: float = 20.0):
    """@brief 最小 HTTP 客户端（只用标准库，不引依赖；二进制/代理环境都稳）"""
    import urllib.error
    import urllib.request
    port = DEFAULT_PORT if port is None else port
    url = f'http://{host}:{port}{path}'
    data = json.dumps(body).encode('utf-8') if body is not None else None
    req = urllib.request.Request(url, data=data, method=method,
                                 headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            raw = r.read().decode('utf-8', 'replace')
    except urllib.error.HTTPError as e:
        return {'ok': False, 'http_status': e.code, 'error': e.read().decode('utf-8', 'replace')}
    except Exception as e:
        return {'ok': False, 'error': f'{type(e).__name__}: {e}'}
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        return {'ok': False, 'error': 'non-JSON', 'raw': raw[:400]}


def _port_open(host=DEFAULT_HOST, port: int | None = None, timeout=0.4) -> bool:
    port = DEFAULT_PORT if port is None else port
    with socket.socket() as s:
        s.settimeout(timeout)
        return s.connect_ex((host, port)) == 0


def cmd_caps(args):
    """列出能力清单"""
    r = _http('GET', '/api/caps')
    if args.raw:
        return _dump(r)
    if not r.get('modules'):
        return _dump(r)
    print(f"模块 {r['n_modules']} 个 / 命令 {r['n_cmds']} 条")
    for m in r['modules']:
        print(f"\n[{m['id']}] {m['name']} —— {m['desc']}")
        for c in m['cmds']:
            flag = '' if c['readonly'] else '  ★有副作用'
            print(f"  {c['cmd']:<20} {c['desc']}{flag}")
            for a in c['args']:
                req = '必填' if a['required'] else '可选'
                print(f"      {a['name']:<12} {a['type']:<7} {req}  {a['desc']}")
    print(f"\nHTTP 入口：")
    for h in r['http']:
        print(f"  {h['method']:<5} {h['path']:<22} {h['desc']}")


def cmd_modules(args):
    """只列模块与命令名（省屏幕）"""
    r = _http('GET', '/api/caps')
    for m in r.get('modules', []):
        names = ' '.join(c['cmd'] for c in m['cmds'])
        print(f"{m['id']:<10} {m['name']:<12} {names}")


def cmd_status(args):
    return _dump(_http('GET', '/api/status'))


def cmd_snap(args):
    r = _http('GET', '/api/snap')
    if args.raw or not r.get('snap'):
        return _dump(r)
    snap = r['snap']
    print(f"连接={r['connected']} 协议={r['protocol']} 模式={r['mode']} "
          f"字段={r['n_fields']} 链路={r.get('link')}")
    keys = args.keys.split(',') if args.keys else sorted(snap)
    for k in keys:
        if k in snap:
            print(f"  {k:<22} {snap[k]}")


def cmd_cmd(args):
    """调一条命令（核心用法）"""
    if not args.name:
        raise SystemExit('用法：gcs_ctl.py cmd <命令名> [key=value ...]')
    body = {'cmd': args.name, **_parse_kv(args.kv)}
    r = _http('POST', '/api/cmd', body)
    if args.raw or not r.get('ok'):
        return _dump(r)
    replies = r.get('replies') or []
    if not replies:
        print(f"OK {args.name}（无回复消息）")
        return
    for msg in replies:
        print(json.dumps(msg, ensure_ascii=False))


def cmd_demo(args):
    """起停演示后端（无硬件也能验证全部功能链路）"""
    if args.action == 'stop':
        n = _kill_port(DEFAULT_PORT)
        print(f'已停止 {n} 个占用 {DEFAULT_PORT} 的进程' if n else
              f'{DEFAULT_PORT} 无监听进程')
        return
    if _port_open():
        print(f'{DEFAULT_PORT} 已有后端在跑（不接管、不重启）')
        return _dump(_http('GET', '/api/status'))
    log = ROOT / 'tmp' / 'gcs_demo.log'
    log.parent.mkdir(exist_ok=True)
    env = dict(os.environ, GCS_DEMO='1')
    f = open(log, 'wb')
    subprocess.Popen([sys.executable, '-m', 'uvicorn', 'server.main:app',
                      '--host', DEFAULT_HOST, '--port', str(DEFAULT_PORT)],
                     cwd=str(GCS), env=env, stdout=f, stderr=subprocess.STDOUT,
                     start_new_session=True)
    for _ in range(60):
        time.sleep(0.25)
        if _port_open():
            print(f'演示后端已起：http://{DEFAULT_HOST}:{DEFAULT_PORT}  日志 {log}')
            return
    print(f'启动超时，看日志：{log}')
    sys.exit(1)


def cmd_selftest(args):
    """端到端自检：能力清单 → 遥测 → 分析命令 → 清理"""
    ok = 0
    total = 0

    def check(name, fn):
        nonlocal ok, total
        total += 1
        try:
            r = fn()
            good = bool(r)
            print(('  PASS  ' if good else '  FAIL  ') + name)
            ok += 1 if good else 0
        except Exception as e:
            print(f'  FAIL  {name}  {type(e).__name__}: {e}')

    started = False
    if not _port_open():
        print('后端未在跑，先起演示后端…')
        cmd_demo(argparse.Namespace(action='start'))
        started = True
        time.sleep(2.0)

    try:
        caps = _http('GET', '/api/caps')
        check('能力清单可读', lambda: caps.get('n_cmds', 0) >= 40)
        check('模块数 ≥10', lambda: caps.get('n_modules', 0) >= 10)

        snap = _http('GET', '/api/snap')
        got = (snap.get('snap') or {}).get('roll_deg') is not None
        print(f"  快照字段数 {snap.get('n_fields')}  roll_deg={got}")
        check('遥测快照有数据', lambda: got)

        sp = _http('POST', '/api/cmd', {'cmd': 'spectrum', 'channel': 'roll_deg'})
        rep = (sp.get('replies') or [{}])[0]
        print(f"  频谱 ok={rep.get('ok')} fs={rep.get('fs')} peaks={len(rep.get('peaks') or [])}")
        check('spectrum 命令返回结果', lambda: rep.get('ok'))

        ch = _http('POST', '/api/cmd', {'cmd': 'tuning_channels'})
        n = len(((ch.get('replies') or [{}])[0]).get('channels') or [])
        print(f"  可分析通道 {n} 个")
        check('tuning_channels 有通道', lambda: n > 0)

        bad = _http('POST', '/api/cmd', {'cmd': '不存在的命令'})
        check('未知命令被拒', lambda: bad.get('ok') is False)
    finally:
        if started and not args.keep:
            cmd_demo(argparse.Namespace(action='stop'))
            print('已关闭本次拉起的演示后端')
    print(f'\n自检 {ok}/{total} 通过')
    return 0 if ok == total else 1


def _kill_port(port: int) -> int:
    """@brief 杀掉监听该端口的进程（只用于关自己拉起的演示后端）"""
    try:
        out = subprocess.run(['netstat', '-ano'], capture_output=True, text=True).stdout
    except Exception:
        return 0
    pids = {ln.split()[-1] for ln in out.splitlines()
            if f':{port}' in ln and 'LISTENING' in ln}
    n = 0
    for pid in pids:
        subprocess.run(['taskkill', '/F', '/PID', pid], capture_output=True)
        n += 1
    return n


def _dump(obj):
    print(json.dumps(obj, ensure_ascii=False, indent=2))


def main() -> int:
    ap = argparse.ArgumentParser(prog='gcs_ctl', description='UniGNC 地面站调试入口')
    ap.add_argument('--port', type=int, default=DEFAULT_PORT, help='后端端口')
    sub = ap.add_subparsers(dest='action', required=True)

    p = sub.add_parser('caps', help='能力清单（模块/命令/参数）')
    p.add_argument('--raw', action='store_true', help='原始 JSON')
    p.set_defaults(fn=cmd_caps)

    p = sub.add_parser('modules', help='模块与命令名一览')
    p.set_defaults(fn=cmd_modules)

    p = sub.add_parser('status', help='后端与链路状态')
    p.set_defaults(fn=cmd_status)

    p = sub.add_parser('snap', help='当前遥测快照')
    p.add_argument('--keys', default='', help='只看这些字段（逗号分隔）')
    p.add_argument('--raw', action='store_true')
    p.set_defaults(fn=cmd_snap)

    p = sub.add_parser('cmd', help='调任意命令：cmd <名字> [key=value ...]')
    p.add_argument('name', nargs='?', help='命令名')
    p.add_argument('kv', nargs='*', help='参数 key=value')
    p.add_argument('--raw', action='store_true')
    p.set_defaults(fn=cmd_cmd)

    p = sub.add_parser('demo', help='起停演示后端')
    p.add_argument('action', choices=['start', 'stop'])
    p.set_defaults(fn=cmd_demo)

    p = sub.add_parser('selftest', help='端到端自检')
    p.add_argument('--keep', action='store_true', help='自检完不关后端')
    p.set_defaults(fn=cmd_selftest)

    args = ap.parse_args()
    globals()['DEFAULT_PORT'] = args.port   # 端口来自命令行，后续所有请求都用它
    r = args.fn(args)
    return r if isinstance(r, int) else 0


if __name__ == '__main__':
    sys.exit(main())
