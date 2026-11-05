# -*- coding: utf-8 -*-
"""@file    serial_check.py
@module  工具
@brief   串口链路健康检查：经 GCS CLI 逐条跑 DBG 命令并汇总输出
@details 通过 GCS CLI（AnoCom 协议的唯一实现）逐条执行 DBG 命令，汇总
         health/ubxstat/monver/gpsproto/tasks 输出；可选 --uplink-seconds 间隔
         采样两次 health 供人工比对上行帧数。只读查询，不写固件参数。
@frame   N/A（串口链路健康检查，不涉坐标系）
@units   s（采样窗口）、Hz（由上行帧数换算）
TandemVec FCS serial health check (COM port -> GCS CLI -> DBG).

Usage:
  py -3.12 TandemVec_FCS/tools/serial_check.py --port COM9
  py -3.12 TandemVec_FCS/tools/serial_check.py --port COM9 --uplink-seconds 10

It runs the GCS CLI (single implementation of the protocol) instead of
re-implementing the AnoCom framing here. Use this after flashing to verify:
  health  -> IMU/baro/GNSS/EKF/Serial5 uplink summary
  ubxstat -> PVT/EOE counts and last UBX message
  tasks   -> task timing / CPU load
"""
import argparse
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CLI = os.path.join(ROOT, "GCS", "server", "cli.py")


def run_cli(port, *args, timeout=45):
    """调用 GCS CLI 执行一条命令，返回合并后的 stdout+stderr 文本。

    参数：port 为串口名，*args 为 cli 子命令（如 "dbg" "health"），timeout 为秒级
    超时；强制 UTF-8 环境并用替换字符兜底解码，保证串口乱码不会抛异常。
    """
    cmd = [sys.executable, CLI, "--port", port] + list(args)
    env = dict(os.environ, PYTHONUTF8="1", PYTHONIOENCODING="utf-8")
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout,
                       env=env, encoding="utf-8", errors="replace")
    return (p.stdout or "") + (p.stderr or "")


def show(title, text, keep=16):
    """打印一段 CLI 输出的前 keep 条非空行（每行截断 180 字符），带标题分隔。

    用于把多条 DBG 查询压成一屏可读摘要，避免串口日志刷屏。
    """
    print(f"\n=== {title} ===")
    lines = [ln for ln in text.splitlines() if ln.strip()]
    for ln in lines[:keep]:
        print("  " + ln[:180])


def main():
    """命令行入口：按固定顺序查 health/ubxstat/monver/gpsproto/tasks 并打印摘要。

    --port 默认 COM9；--uplink-seconds>0 时前后各取一次 health，供人工比对上行计数
    估算帧率。只读查询，不修改任何固件状态。
    """
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM9")
    ap.add_argument("--uplink-seconds", type=float, default=0.0,
                    help="if >0, print uplink counters twice to estimate frame rate")
    a = ap.parse_args()

    show("health", run_cli(a.port, "dbg", "health"))
    show("ubxstat", run_cli(a.port, "dbg", "ubxstat"))
    show("monver", run_cli(a.port, "dbg", "ubxcfg monver"))
    show("gpsproto", run_cli(a.port, "dbg", "gpsproto"))
    show("tasks", run_cli(a.port, "dbg", "tasks"), keep=8)

    if a.uplink_seconds > 0:
        before = run_cli(a.port, "dbg", "health")
        time.sleep(a.uplink_seconds)
        after = run_cli(a.port, "dbg", "health")
        show("uplink before", before, keep=12)
        show("uplink after", after, keep=12)
        print(f"\nuplink window = {a.uplink_seconds:.1f} s "
              f"(compare 'uplink rx=' lines to get frames/s)")


if __name__ == "__main__":
    main()
