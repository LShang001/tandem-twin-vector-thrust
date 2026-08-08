# -*- coding: utf-8 -*-
"""
app.py — TandemVec GCS 桌面启动器（独立窗口，无需手动开浏览器）

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

HOST = '127.0.0.1'
DEFAULT_PORT = 8091

log = logging.getLogger('gcs-app')
logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(name)s: %(message)s')


def port_free(port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        return s.connect_ex((HOST, port)) != 0


def wait_http_ready(url: str, timeout: float = 15.0) -> bool:
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


def main():
    ap = argparse.ArgumentParser(description='TandemVec GCS 桌面启动器')
    ap.add_argument('--port', type=int, default=DEFAULT_PORT)
    ap.add_argument('--browser', action='store_true', help='强制系统浏览器模式（不用原生窗口）')
    args = ap.parse_args()

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
    if 'TandemVec GCS' not in html:
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
                'TandemVec GCS · 纵列双发矢量推力飞行器地面站',
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
