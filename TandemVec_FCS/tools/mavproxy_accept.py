"""MAVProxy 交互式自动化验收（ConPTY + stdin 写入）。

关键点在上一版踩过：MAVProxy 的 --cmd 是**启动前**执行的，那时还没连上飞控，
所以 status/param/arm/mode 都会失效、exit 会报 Unknown command。
正确做法是启动后等心跳，再往 pty 的 stdin 写命令。
"""
from __future__ import annotations

import os
import re
import sys
import time

import winpty

PY = os.path.join(os.environ["USERPROFILE"], ".platformio", "penv", "Scripts", "python.exe")
MP = os.path.join(os.environ["USERPROFILE"], ".platformio", "penv", "Scripts", "mavproxy.py")

CMDS = [
    "status",
    "param show att_roll",
    "arm throttle",
    "mode guided",
    "exit",
]


def strip_ansi(s: str) -> str:
    s = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", s)
    s = re.sub(r"\x1b\][^\x07]*\x07", "", s)
    out = []
    for ln in s.splitlines():
        ln = ln.rstrip()
        if not ln:
            continue
        if "uxtheme.cpp" in ln or "GetThemeColor" in ln or ln.startswith("..\\"):
            continue
        if "FTP: request timed out" in ln:
            continue
        out.append(ln)
    return "\n".join(out)


def main() -> int:
    port = "COM6"
    if "--port" in sys.argv:
        port = sys.argv[sys.argv.index("--port") + 1]

    import serial
    ser = serial.Serial(port, 921600, timeout=0.1)
    time.sleep(2.0)
    ser.reset_input_buffer()
    ser.write(b"DBG\n"); time.sleep(1.5); ser.read(1024)
    ser.write(b"proto mavlink\n"); time.sleep(1.2); ser.read(512)
    ser.write(b"exit\n"); time.sleep(1.5)
    ser.close()
    print(f"[{port}] 已切 MAVLink 模式\n")

    args = [PY, MP, f"--master={port}", "--baudrate=921600"]
    proc = winpty.PtyProcess.spawn(args)

    buf = ""
    # 等连接（出现 Mode(x)> 或 Detected vehicle）
    t0 = time.time()
    connected = False
    while time.time() - t0 < 20.0:
        try:
            data = proc.read(4096)
        except EOFError:
            break
        if data:
            buf += data
            if "Detected vehicle" in buf or "Mode(" in buf:
                connected = True
                break
        else:
            time.sleep(0.1)

    print(f"连接状态: {'已连上' if connected else '未连上（继续尝试下发）'}\n")

    # 逐条下发命令，每条后等 1.5s 收输出
    for c in CMDS:
        proc.write(c + "\r")
        t1 = time.time()
        while time.time() - t1 < 2.0:
            try:
                data = proc.read(4096)
            except EOFError:
                data = None
                break
            if data:
                buf += data
            else:
                time.sleep(0.1)
        if c == "exit":
            break

    try:
        if proc.isalive():
            proc.terminate(force=True)
    except Exception:
        pass

    text = strip_ansi(buf)
    print("=" * 60)
    print(text)
    print("=" * 60)

    checks = {
        "检测到飞控": "Detected vehicle" in text,
        "收到 169 参数": "169 parameters" in text,
        "参数读取": "att_roll" in text.lower(),
        "ARM 被拒": "Arm via RC CH5" in text or "arm denied" in text.lower(),
        "模式被拒": "Mode via RC CH6" in text or "mode denied" in text.lower(),
    }
    print("\n关键判据:")
    ok = True
    for k, v in checks.items():
        print(f"  {'OK  ' if v else 'MISS'}  {k}")
        ok = ok and v
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
