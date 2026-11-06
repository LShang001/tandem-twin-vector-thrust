"""MAVLink 链路体检：pymavlink 做断言 + MAVProxy 做报文可视化。

为什么分成两条路（2026-11-07 实测结论）：
  · pymavlink 能精确控制每条消息、解析每个 ACK 字段 → 适合做断言
  · MAVProxy 的 command_long 命令实测不生效（ARM/SET_MODE/999 都没有 ACK），
    但它对**收到的**报文解析和打印非常完整 → 适合做人读的输出
  硬把 MAVProxy 掰成断言工具，得到的是「看起来跑了、其实没验证」的假绿灯。

用法：
    py tools/mavlink_check.py              # 两段都跑
    py tools/mavlink_check.py --assert-only
    py tools/mavlink_check.py --show-only
"""
from __future__ import annotations

import os
import re
import sys
import time

PY = os.path.join(os.environ["USERPROFILE"], ".platformio", "penv", "Scripts", "python.exe")
MP = os.path.join(os.environ["USERPROFILE"], ".platformio", "penv", "Scripts", "mavproxy.py")


def switch_mavlink(port: str) -> None:
    import serial
    ser = serial.Serial(port, 921600, timeout=0.1)
    time.sleep(2.0)
    ser.reset_input_buffer()
    ser.write(b"DBG\n"); time.sleep(1.5); ser.read(1024)
    ser.write(b"proto mavlink\n"); time.sleep(1.2); ser.read(512)
    ser.write(b"exit\n"); time.sleep(1.5)
    ser.close()


# ==================================================================
#  A. 断言段（pymavlink）
# ==================================================================
def run_asserts(port: str) -> bool:
    from pymavlink import mavutil

    switch_mavlink(port)
    print("=" * 62)
    print("A. 断言（pymavlink）")
    print("=" * 62)

    m = mavutil.mavlink_connection(port, baud=921600, source_system=255)
    checks: list[tuple[str, bool]] = []

    h = m.wait_heartbeat(timeout=10)
    checks.append(("心跳", h is not None))
    if h:
        checks.append((f"机型 type={h.type}（9=ROCKET）", h.type == 9))
        checks.append((f"固件 autopilot={h.autopilot}（3=ArduPilot）", h.autopilot == 3))

    # ★ 2026-11-07 实测：MAVLink 下行 ~384 帧/s，参数下载与命令 ACK 都会被
    #   周期遥测淹没。第一轮成功、第二轮 0 条就是队列积压程度不同导致的。
    #   对策：连接后先静置 2 秒让积压排空，参数与 ACK 的超时都放宽。
    time.sleep(2.0)
    while m.recv_match(blocking=False) is not None:
        pass

    m.mav.param_request_list_send(1, 1)
    params: dict[str, float] = {}
    t0 = time.time()
    while time.time() - t0 < 20.0 and len(params) < 200:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg:
            params[msg.param_id] = msg.param_value
    checks.append((f"参数数量 = {len(params)}", len(params) == 169))

    def cmd(cid: int, p1: float = 0, p2: float = 0) -> int | None:
        # 只丢 ACK，不丢参数：drain 全清会把并行的 PARAM_VALUE 一起丢掉
        while True:
            msg = m.recv_match(blocking=False)
            if msg is None:
                break
            if msg.get_type() == "PARAM_VALUE":
                params[msg.param_id] = msg.param_value
        m.mav.command_long_send(1, 1, cid, 0, p1, p2, 0, 0, 0, 0, 0)
        t1 = time.time()
        while time.time() - t1 < 8.0:
            msg = m.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.get_type() == "PARAM_VALUE":
                params[msg.param_id] = msg.param_value
            elif msg.get_type() == "COMMAND_ACK" and msg.command == cid:
                return msg.result
        return None

    checks.append(("ARM(400) → UNSUPPORTED(3)", cmd(400, 1) == 3))
    checks.append(("SET_MODE(176) → UNSUPPORTED(3)", cmd(176, 1) == 3))
    checks.append(("SET_MESSAGE_INTERVAL(511) → ACCEPTED(0)", cmd(511, 30, 100000) == 0))
    checks.append(("未知命令(999) → UNSUPPORTED(3)", cmd(999) == 3))
    m.close()

    for name, ok in checks:
        print(f"  {'OK  ' if ok else 'MISS'}  {name}")
    return all(ok for _, ok in checks)


# ==================================================================
#  B. 展示段（MAVProxy 的 status 报文）
# ==================================================================
def run_show(port: str) -> None:
    import winpty

    switch_mavlink(port)
    print("\n" + "=" * 62)
    print("B. 报文可视化（MAVProxy status）")
    print("=" * 62)

    proc = winpty.PtyProcess.spawn([PY, MP, f"--master={port}", "--baudrate=921600"])
    buf = ""
    t0 = time.time()
    while time.time() - t0 < 25.0:
        try:
            data = proc.read(4096)
        except EOFError:
            break
        if data:
            buf += data
            if "Saved" in buf:
                break
        else:
            time.sleep(0.05)

    proc.write("status\r")
    t1 = time.time()
    while time.time() - t1 < 6.0:
        try:
            data = proc.read(4096)
        except EOFError:
            break
        if data:
            buf += data
        else:
            time.sleep(0.05)

    try:
        proc.terminate(force=True)
    except Exception:
        pass

    s = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", buf)
    s = re.sub(r"\x1b\][^\x07]*\x07", "", s)
    keep = []
    for ln in s.splitlines():
        ln = ln.rstrip()
        if not ln or "uxtheme" in ln or "GetThemeColor" in ln or ln.startswith("..\\"):
            continue
        if "FTP: request timed out" in ln:
            continue
        keep.append(ln)
    print("\n".join(keep))


def main() -> int:
    port = "COM6"
    if "--port" in sys.argv:
        port = sys.argv[sys.argv.index("--port") + 1]

    mode = "both"
    if "--assert-only" in sys.argv:
        mode = "assert"
    elif "--show-only" in sys.argv:
        mode = "show"

    ok = True
    if mode in ("both", "assert"):
        ok = run_asserts(port)
    if mode in ("both", "show"):
        run_show(port)

    print("\n结论:", "全部通过" if ok else "有失败项")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
