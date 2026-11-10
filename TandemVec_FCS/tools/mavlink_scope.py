#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    mavlink_scope.py
@module  工具 / MAVLink 实时波形（示波器）
@brief   从 MAVLink 串口（或 .tlog 日志）读遥测并实时画滚动波形
@details 为什么需要它：QGC 的实时数据显示只有 MAVLink Inspector 的数字刷新，
         曲线只出现在 Analyze 里对已落地日志的离线分析——它没有实时示波器。
         Mission Planner 的 Quick 页有实时曲线，但那是 Windows 独占的地面站，
         调试时想「一边看曲线一边改固件」不方便。本项目已用 MAVLink v2 下行，
         直接用 pymavlink 解析成曲线最省事，也不需要再抢一个地面站。
         数据源两路：
           ① --port 实时串口（会被独占，此时 QGC/Mission Planner 必须先断开）；
           ② --log 读 .tlog / 已录数据，离线回放。
@theory  N/A（数据可视化工具，不做估计与控制）
@frame   显示的是 MAVLink 报文里飞控侧的 NED 语义量，脚本不做任何轴变换
@units   角度 deg、角速度 deg/s、位置 m、速度 m/s、高度 m、姿态角 deg
@note    串口是独占资源：波形工具和 QGC/Mission Planner 不能同时连同一个口。
         要同时看，用 MAVProxy 分流：
           mavproxy.py --master=COM12 --baudrate=921600 --out=udp:127.0.0.1:14550
         然后 QGC 连 UDP 14550，本脚本 --port udpin:127.0.0.1:14551（再分一路）。
@test    T1 连上后 3 s 内应出现曲线；T2 --list 能列出收到的消息类型；
         T3 --log 回放 .tlog 不报错
@see     TandemVec_FCS/src/mavlink_bridge.cpp、docs/memory/2026-11-09-ICM45686换型适配.md
"""

from __future__ import annotations

import argparse
import collections
import sys
import threading
import time
from pathlib import Path

import matplotlib

# 不在 import 阶段锁后端：--list 模式压根不画图，锁了 Agg 纯属多余；
# 真要画图时在 run_live 里按可用后端挑（有桌面用 TkAgg，否则给明确提示）。
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import animation  # noqa: E402

# 中文字体（Windows 自带 SimHei；缺字体时自动退回默认，只是显示成方框）
for _font in ("Microsoft YaHei", "SimHei", "SimSun"):
    try:
        matplotlib.font_manager.findfont(_font, fallback_to_default=False)
        plt.rcParams["font.sans-serif"] = [_font]
        break
    except Exception:  # noqa: BLE001
        continue
plt.rcParams["axes.unicode_minus"] = False

# 可画的字段：名字 → (数据源 key, 单位)
FIELDS: dict[str, tuple[str, str]] = {
    "roll": ("roll", "deg"),
    "pitch": ("pitch", "deg"),
    "yaw": ("yaw", "deg"),
    "p": ("rollspeed", "deg/s"),
    "q": ("pitchspeed", "deg/s"),
    "r": ("yawspeed", "deg/s"),
    "vn": ("vn", "m/s"),
    "ve": ("ve", "m/s"),
    "vd": ("vd", "m/s"),
    "x": ("x", "m"),
    "y": ("y", "m"),
    "z": ("z", "m"),
    "alt": ("alt", "m"),
    "groundspeed": ("groundspeed", "m/s"),
    "heading": ("heading", "deg"),
    "throttle": ("throttle", "%"),
}

DEFAULT_FIELDS = ["roll", "pitch", "yaw", "vn", "ve", "vd"]


class Recorder:
    """后台线程读 MAVLink，把关心的字段写进环形缓冲。"""

    def __init__(self, source: str, baud: int, window_s: float) -> None:
        self.source = source
        self.baud = baud
        self.window_s = window_s
        self.t: collections.deque[float] = collections.deque()
        self.series: dict[str, collections.deque[float]] = collections.defaultdict(collections.deque)
        self.msg_counts: collections.Counter[str] = collections.Counter()
        self.lock = threading.Lock()
        self.stop_flag = threading.Event()
        self.t0 = time.time()
        self.error: str | None = None
        self._conn = None
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        """建立连接并启动读线程。"""
        from pymavlink import mavutil

        if self.source.startswith("udpin:") or self.source.startswith("udpout:"):
            self._conn = mavutil.mavlink_connection(self.source)
        elif self.source.lower().startswith("com") or self.source.startswith("/dev"):
            self._conn = mavutil.mavlink_connection(self.source, baud=self.baud)
        else:
            # 当成 .tlog 文件回放
            p = Path(self.source)
            if not p.exists():
                self.error = f"既不是串口也不是存在的文件：{self.source}"
                return
            self._conn = mavutil.mavlink_connection(str(p))
        self._thread.start()

    def _run(self) -> None:
        """读线程主循环。"""
        try:
            while not self.stop_flag.is_set():
                msg = self._conn.recv_match(blocking=True, timeout=0.5)
                if msg is None:
                    continue
                name = msg.get_type()
                self.msg_counts[name] += 1
                d = msg.to_dict()
                now = time.time() - self.t0
                with self.lock:
                    self.t.append(now)
                    for key in list(self.series.keys()):
                        self.series[key].append(float(d.get(FIELDS[key][0], float("nan"))))
                    # 只保留窗口内的点
                    horizon = now - self.window_s
                    while self.t and self.t[0] < horizon:
                        self.t.popleft()
                        for dq in self.series.values():
                            dq.popleft()
        except Exception as exc:  # noqa: BLE001
            self.error = f"读取错误：{exc}"

    def register(self, fields: list[str]) -> None:
        """登记要画的字段（在启动后调用，保证线程里能按同一顺序 append）。"""
        with self.lock:
            for f in fields:
                if f not in self.series:
                    self.series[f] = collections.deque()
                    # 补历史长度，避免长度不齐
                    for _ in range(len(self.t)):
                        self.series[f].append(float("nan"))

    def snapshot(self) -> tuple[list[float], dict[str, list[float]]]:
        """取一份拷贝给绘图层用。"""
        with self.lock:
            t = list(self.t)
            return t, {k: list(v) for k, v in self.series.items()}

    def stop(self) -> None:
        """停止读线程。"""
        self.stop_flag.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)


def run_live(rec: Recorder, fields: list[str], window_s: float) -> int:
    """实时滚动绘图。"""
    if rec.error:
        print(f"[FAIL] {rec.error}", file=sys.stderr)
        return 1

    # 分组：角度/角速度一组、速度一组、位置一组，各自用独立子图
    groups: list[list[str]] = []
    for f in fields:
        unit = FIELDS[f][1]
        for g in groups:
            if FIELDS[g[0]][1] == unit:
                g.append(f)
                break
        else:
            groups.append([f])

    n = len(groups)
    fig, axes = plt.subplots(n, 1, figsize=(11, 2.6 * n), sharex=True)
    if n == 1:
        axes = [axes]
    fig.suptitle(f"MAVLink 实时波形 — {rec.source}", fontsize=12)

    lines: list[list] = []
    for ax, grp in zip(axes, groups):
        ls = []
        for f in grp:
            (ln,) = ax.plot([], [], label=f)
            ls.append(ln)
        ax.set_ylabel(FIELDS[grp[0]][1])
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=8, ncol=len(grp))
        lines.append(ls)
    axes[-1].set_xlabel("时间 / s")

    def update(_frame):
        t, data = rec.snapshot()
        for grp, ls in zip(groups, lines):
            for f, ln in zip(grp, ls):
                y = data.get(f, [])
                ln.set_data(t, y[: len(t)] if len(y) >= len(t) else y)
        if t:
            for ax in axes:
                ax.set_xlim(max(0.0, t[-1] - window_s), max(window_s, t[-1]))
                ax.relim()
                ax.autoscale_view(scalex=False, scaley=True)
        return [ln for ls in lines for ln in ls]

    ani = animation.FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)

    try:
        plt.show()
    except Exception as exc:  # noqa: BLE001
        print(f"[FAIL] 无法显示窗口（{exc}）。无桌面环境请用 --log 模式或保存图片。",
              file=sys.stderr)
        return 1
    finally:
        del ani
        rec.stop()
    return 0


def main() -> int:
    """命令行入口。"""
    ap = argparse.ArgumentParser(description="MAVLink 实时波形（示波器）")
    ap.add_argument("--port", help="串口（如 COM12）或 udpin:127.0.0.1:14551")
    ap.add_argument("--baud", type=int, default=921600, help="波特率，默认 921600")
    ap.add_argument("--log", help="回放 .tlog 文件（离线）")
    ap.add_argument("--fields", default=",".join(DEFAULT_FIELDS),
                    help=f"要画的字段，逗号分隔，可选：{', '.join(FIELDS)}")
    ap.add_argument("--window", type=float, default=10.0, help="时间窗长度（秒），默认 10")
    ap.add_argument("--list", action="store_true", help="只统计收到的消息类型，5 秒后退出")
    args = ap.parse_args()

    if not args.port and not args.log:
        ap.error("必须给 --port 或 --log")

    fields = [f.strip() for f in args.fields.split(",") if f.strip()]
    bad = [f for f in fields if f not in FIELDS]
    if bad:
        print(f"[FAIL] 未知字段：{bad}\n  可选：{', '.join(FIELDS)}", file=sys.stderr)
        return 1

    source = args.log or args.port
    rec = Recorder(source, args.baud, args.window)
    rec.start()
    time.sleep(0.5)
    rec.register(fields)

    if rec.error:
        print(f"[FAIL] {rec.error}", file=sys.stderr)
        return 1

    if args.list:
        print("[list] 统计 5 秒内收到的消息 ...")
        time.sleep(5.0)
        rec.stop()
        with rec.lock:
            items = rec.msg_counts.most_common()
        if not items:
            print("[FAIL] 5 秒内没收到任何 MAVLink 消息", file=sys.stderr)
            return 1
        for name, cnt in items:
            print(f"  {name:<28} x{cnt}")
        return 0

    print(f"[scope] {source} @ {args.baud}，字段={fields}，窗口={args.window}s")
    print("[scope] 提示：串口是独占的，QGC/Mission Planner 必须先断开；需要同时看请用 MAVProxy 分流")
    return run_live(rec, fields, args.window)


if __name__ == "__main__":
    sys.exit(main())
