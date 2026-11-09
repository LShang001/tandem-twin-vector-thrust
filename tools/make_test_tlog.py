#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    make_test_tlog.py
@module  工具/GCS 测试日志生成
@brief   造一份采样率与频率成分都已知的 tlog，用来验证频域分析链路

@details 为什么需要：演示回放用的是 20 Hz 的 tlog，奈奎斯特只有 10 Hz，
         根本看不到桨叶通过频率（真实值常在 20–80 Hz）这类最需要诊断的成分。
         要验证"高频能不能被识别"，就得有一份**采样率已知、频率成分已知**的样本。

         本脚本生成的信号是刻意设计的：
           · 低频 0.5 Hz（≈体动/慢摆）
           · 高频 28 Hz 桨叶通过频率（幅值较小，模拟结构传递的振动）
           · 中频 6 Hz（模拟控制环振荡）
         频谱应当在这三个位置出峰，且 0.5 Hz 因低于一个 bin 会归入低频带。

@theory 奈奎斯特采样定理：fs=100 Hz 时可无混叠观测的最高频率为 50 Hz
@frame   N/A
@units   频率 Hz、时间 s
@note   必须每条消息前写 8 字节大端时间戳，否则 pymavlink 读出来全是 BAD_DATA。
@test    生成后由 tests/test_spectrum.py 的集成用例或手工回放验证
@see     TandemVec_FCS/GCS/server/spectrum.py
"""
from __future__ import annotations

import math
import struct
import sys
from pathlib import Path

from pymavlink import mavutil


def build(path: Path, fs_hz: int = 100, seconds: float = 60.0) -> dict:
    """@brief 生成 tlog
    @param fs_hz 采样率（决定奈奎斯特上限与频率分辨率）
    @param seconds 时长
    @return 生成摘要（含设计频率、帧数、时间戳间隔）
    """
    mav = mavutil.mavlink.MAVLink(None)
    mav.srcSystem = 1
    mav.srcComponent = 1

    n = int(seconds * fs_hz)
    dt = 1.0 / fs_hz
    t0_us = 1_690_000_000 * 1_000_000   # 固定起点，便于复现

    # 设计的频率成分（写进返回值，供断言用）
    design = {'low_hz': 0.5, 'mid_hz': 6.0, 'high_hz': 28.0}

    with open(path, 'wb') as f:
        for i in range(n):
            t = i * dt
            # ★ 幅值选择：28 Hz 幅值 2°，6 Hz 幅值 1°，0.5 Hz 幅值 5°。
            #   低频幅值最大是刻意的——真实飞机上慢摆确实比结构振动幅值大，
            #   但它低于频率分辨率，会被归进低频带；诊断关心的是 28 Hz 还在不在。
            roll_rad = math.radians(
                5.0 * math.sin(2 * math.pi * design['low_hz'] * t)
                + 1.0 * math.sin(2 * math.pi * design['mid_hz'] * t)
                + 2.0 * math.sin(2 * math.pi * design['high_hz'] * t))
            pitch_rad = math.radians(3.0 * math.sin(2 * math.pi * 0.8 * t))
            yaw_rad = math.radians(20.0 + 35.0 * math.sin(2 * math.pi * 0.05 * t))

            ts_us = t0_us + int(round(t * 1_000_000))
            # ★ 每条消息前都要有 8 字节大端时间戳；漏写会让 pymavlink 判 BAD_DATA
            f.write(struct.pack('>Q', ts_us))
            # ★ MAVLink(None) 没有写出目标（file=None），不能用 send；
            #   encode 拿消息对象再 pack 成字节，自己写进文件。
            msg = mav.attitude_encode(
                int(t * 1000) & 0xFFFFFFFF, roll_rad, pitch_rad, yaw_rad,
                0.0, 0.0, 0.0)
            f.write(msg.pack(mav))

    return {
        'path': str(path),
        'fs_hz': fs_hz,
        'seconds': seconds,
        'frames': n,
        'dt_ms': round(dt * 1000, 3),
        'nyquist_hz': fs_hz / 2,
        'design': design,
        'bytes': path.stat().st_size,
    }


def main(argv: list[str]) -> int:
    out = Path(argv[1]) if len(argv) > 1 else Path('output/test_100hz.tlog')
    fs = int(argv[2]) if len(argv) > 2 else 100
    secs = float(argv[3]) if len(argv) > 3 else 60.0
    out.parent.mkdir(parents=True, exist_ok=True)
    info = build(out, fs, secs)
    for k, v in info.items():
        print(f'  {k}: {v}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
