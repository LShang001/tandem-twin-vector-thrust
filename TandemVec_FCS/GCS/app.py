# -*- coding: utf-8 -*-
"""
app.py — UniGNC GCS 桌面启动器（独立窗口，无需手动开浏览器）

@file    app.py
@module  GCS/桌面启动
@brief   后台拉起 uvicorn 后端并提供 pywebview 原生窗口，关窗即停
@details 输入：命令行 --port/--browser；处理：端口占用检测 → /api/status 版本比对，
         不符则重启残留后端 → 后台线程跑 uvicorn → 等待 HTTP 就绪 →
         pywebview 窗口加载页面；输出：进程内后端 + 前端窗口
         （pywebview 不可用时回退系统浏览器）。
         不变量：前端 web/src/main.mjs 的 BACKEND_VERSION 必须与 app.version 一致；
         app.py 直接 import 同一 app.version 校验残留后端（见 _ensure_fresh_backend）。
         （WS hello 握手与端口探活都靠它识别"残留旧进程"）。
@theory  N/A
@frame   N/A
@units   N/A
@ref     GCS-001 §上位机；AGENTS.md 踩坑记录「改 GCS 后端后必须杀掉旧进程再启动」
@note    ★ 8091 旧进程残留 + 前端 no-store = "界面全新、后端逻辑全旧"（2026-08-08 定位）
@see     server/main.py、web/src/main.mjs

流程：
  1. 后台线程启动 uvicorn（FastAPI 后端 + 静态前端）
  2. 等待 HTTP 就绪
  3. pywebview 原生窗口加载页面（Windows 走 Edge WebView2）
  4. 窗口关闭 → 停后端、退出进程

pywebview 不可用时回退：自动打开系统浏览器（仍免手动输地址）。

用法：
  py -3.12 app.py            # 桌面窗口模式（默认）
  py -3.12 app.py --browser  # 强制浏览器模式
  py -3.12 app.py --port 8092
"""
import argparse
import logging
import socket
import sys
import threading
import time
import urllib.request

HOST = '127.0.0.1'          # 默认只绑本机：不主动把飞控控制面暴露到局域网
LAN_HOST = '0.0.0.0'       # --lan 时绑全网卡：平板/手机通过局域网访问同一个界面
DEFAULT_PORT = 8091

log = logging.getLogger('gcs-app')
logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(name)s: %(message)s')


def port_free(port: int) -> bool:
    """@brief 检测本机端口是否空闲（能连上说明已被占用）
    @param[in] port TCP 端口
    @return True = 可绑定（connect_ex != 0）
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        return s.connect_ex((HOST, port)) != 0


def wait_http_ready(url: str, timeout: float = 15.0) -> bool:
    """@brief 轮询 URL 直到返回 200 或超时
    @param[in] url 探活地址；timeout 最长等待 [s]
    @return True = 后端已就绪
    """
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            with urllib.request.urlopen(url, timeout=1) as r:
                return r.status == 200
        except Exception:
            time.sleep(0.25)
    return False


def start_server(port: int):
    """后台线程跑 uvicorn；返回 (server, thread)"""
    import uvicorn
    from server.main import app  # noqa: 导入即就绪（模块级 g 单例）

    cfg = uvicorn.Config(app, host=HOST, port=port, log_level='warning')
    server = uvicorn.Server(cfg)
    # 桌面窗口内右键刷新很有用，但 uvicorn 默认装信号处理器——线程模式必须关掉
    cfg.install_signal_handlers = False
    th = threading.Thread(target=server.run, daemon=True, name='gcs-uvicorn')
    th.start()
    return server, th


def lan_urls(port: int) -> list[str]:
    """@brief 枚举本机所有可用于局域网访问的地址
    @return 形如 ['http://192.168.0.208:8091/']
    @note 为什么枚举而不是猜：一台机器常有多个网卡（有线/无线/WireGuard），
          用户不知道自己平板上该输哪个。全列出来，让用户直接试。
          UDP connect 只用来让系统选出主路由网卡的地址，不发包。
    """
    out = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(('8.8.8.8', 80))
            out.append(s.getsockname()[0])
        finally:
            s.close()
    except Exception:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if ip not in out and not ip.startswith('127.'):
                out.append(ip)
    except Exception:
        pass
    return [f'http://{ip}:{port}/' for ip in out]


def main():
    """@brief 解析参数并启动桌面窗口/浏览器（阻塞到窗口关闭后停后端）"""
    ap = argparse.ArgumentParser(description='UniGNC GCS 桌面启动器')
    ap.add_argument('--port', type=int, default=DEFAULT_PORT)
    ap.add_argument('--browser', action='store_true', help='强制系统浏览器模式（不用原生窗口）')
    ap.add_argument('--lan', action='store_true',
                    help='允许局域网访问（平板/手机用 http://<本机IP>:端口 打开同一个界面）')
    ap.add_argument('--host', default=None,
                    help='显式指定绑定地址（默认 127.0.0.1；--lan 等价于 0.0.0.0）')
    args = ap.parse_args()

    global HOST
    if args.host:
        HOST = args.host
    elif args.lan:
        HOST = LAN_HOST

    if not port_free(args.port):
        if not _ensure_fresh_backend(args.port):
            return
        # 旧实例已杀掉，落入正常启动流程

    server, _th = start_server(args.port)
    url = f'http://{HOST}:{args.port}/'
    if not wait_http_ready(url):
        print('[GCS] 后端启动超时，请检查依赖：py -3.12 -m pip install -r requirements.txt')
        server.should_exit = True
        sys.exit(1)
    print(f'[GCS] 后端已启动 {url}（关闭窗口即停止服务）')
    if HOST not in ('127.0.0.1', 'localhost'):
        urls = lan_urls(args.port)
        print('[GCS] 局域网访问已开启，平板/手机用下面任一地址打开（同一 WiFi/网段）：')
        for u in urls:
            print(f'         {u}')
        if not urls:
            print('         （未检测到非回环地址，检查网络连接）')
        print('[GCS] 提醒：此时局域网内任何设备都能看到遥测并操作连接/参数，不要在不可信网络开启。')

    try:
        _open_window(url, args.browser)
    finally:
        server.should_exit = True


def _ensure_fresh_backend(port: int) -> bool:
    """端口被占用时：确认是 GCS 后端 → 版本不符则杀掉重启（返回 True 继续启动）；
    版本相符 → 直接开窗口（返回 False）；不是 GCS → 警告并开窗口（返回 False）。

    ★ 2026-08-08 教训：旧 uvicorn 进程残留 8091，新前端（no-store）配旧后端
      逻辑，界面全新但数据全黑——排查半天根因是进程没重启。"""
    from server.main import app as gcs_app
    current = gcs_app.version
    base = f'http://{HOST}:{port}'
    try:
        with urllib.request.urlopen(base + '/', timeout=2) as r:
            html = r.read().decode('utf-8', 'ignore')
    except Exception:
        print(f'[GCS] 端口 {port} 被占用且不是 GCS 后端，请换端口（--port）或手动释放。')
        _open_window(base + '/', False)
        return False
    if 'UniGNC GCS' not in html:
        print(f'[GCS] 端口 {port} 被其他服务占用，请换端口（--port）。')
        _open_window(base + '/', False)
        return False
    # 是我们的后端：查版本
    try:
        import json as _json
        with urllib.request.urlopen(base + '/api/status', timeout=2) as r:
            ver = _json.loads(r.read()).get('version')
    except Exception:
        ver = None    # 旧版无 /api/status
    if ver == current:
        print(f'[GCS] 后端已在运行（v{ver}，与启动器一致）——直接打开界面。')
        _open_window(base + '/', False)
        return False
    print(f'[GCS] 检测到旧版后端残留（v{ver or "?"} < v{current}）——自动结束并重启…')
    pid = _pid_by_port(port)
    if pid:
        import subprocess
        subprocess.run(['taskkill', '/PID', str(pid), '/F'],
                       capture_output=True)
        for _ in range(20):
            if port_free(port):
                break
            time.sleep(0.25)
    if not port_free(port):
        print('[GCS] 旧进程未能结束，请手动 taskkill 后重试。')
        sys.exit(1)
    return True


def _pid_by_port(port: int):
    """Windows：netstat -ano 查 LISTENING PID"""
    import subprocess
    try:
        out = subprocess.run(['netstat', '-ano'], capture_output=True, text=True,
                             timeout=10).stdout
        for line in out.splitlines():
            if f':{port}' in line and 'LISTENING' in line:
                parts = line.split()
                if parts[1].endswith(f':{port}'):
                    return int(parts[-1])
    except Exception:
        pass
    return None


def _open_window(url: str, force_browser: bool):
    """优先 pywebview 原生窗口；不可用/失败时回退系统浏览器"""
    if not force_browser:
        try:
            import webview
            webview.create_window(
                'UniGNC GCS · UniGNC 通用飞控平台地面站',
                url, width=1440, height=900, min_size=(1100, 700),
                background_color='#0b1019',
            )
            webview.start()          # 阻塞，窗口关闭后返回
            print('[GCS] 窗口已关闭，后端停止。')
            return
        except ImportError:
            print('[GCS] 未安装 pywebview，回退浏览器模式。'
                  '安装：py -3.12 -m pip install pywebview')
        except Exception as exc:
            log.warning('pywebview 启动失败（%r），回退浏览器模式', exc)
    import webbrowser
    webbrowser.open(url)
    print('[GCS] 浏览器已打开。此窗口保持运行即后端在线；Ctrl+C 停止。')
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print('\n[GCS] 已停止。')


if __name__ == '__main__':
    main()
