"""
@file    gcs_android_main.py
@module  Android/APK 入口
@brief   APK 内的 Python 入口：在应用进程里把现有 FastAPI 后端跑起来

APK 内的 Python 入口。

为什么这样接：地面站后端（AnoCom/MAVLink 解析、频谱、参数、黑匣子）全是
Python，重写成 Java 等于维护第二套实现。Chaquopy 把 CPython 打进 APK 后，
这里只需把 uvicorn 起在一个后台线程，WebView 就能像浏览器一样访问它。
"""
import os
import threading

_started = False
_lock = threading.Lock()


def start_backend(port: int = 8091) -> int:
    """@brief 启动后端并返回端口（阻塞到 server 线程已创建）
    @param port 监听端口（只绑 127.0.0.1：平板自己就是后端）
    @return 实际使用的端口
    @note 必须绑 127.0.0.1 而不是 0.0.0.0：Android 上对外监听会弹网络权限提示，
          而且没有理由把控制面暴露给同网段设备。
    """
    global _started
    with _lock:
        if _started:
            return port
        # Chaquopy 把 python 包解压到应用私有目录，__file__ 就是这个目录下的文件，
        # 所以 server.main 里 `Path(__file__).parent.parent / 'web'` 能正常找到前端。
        import sys
        base = os.path.dirname(os.path.abspath(__file__))
        if base not in sys.path:
            sys.path.insert(0, base)

        import uvicorn
        from server.main import app

        config = uvicorn.Config(app, host='127.0.0.1', port=port,
                                log_level='warning', access_log=False)
        server = uvicorn.Server(config)
        # uvicorn 默认装信号处理器；在非主线程里必须关掉，否则启动即抛异常
        server.install_signal_handlers = lambda: None
        threading.Thread(target=server.run, daemon=True, name='uvicorn').start()
        _started = True
        return port
