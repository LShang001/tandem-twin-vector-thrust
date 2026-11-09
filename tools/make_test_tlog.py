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




def build_dynamics(path: Path, fs_hz: int = 200, seconds: float = 20.0,
                   zeta: float = 0.55, wn: float = 7.0) -> dict:
    """@brief 生成带真实二阶动力学响应的 tlog（阶跃指令 + 结构振动）

    @param zeta 阻尼比（与 server 演示模式同一组：0.55）
    @param wn 自然频率 rad/s（7.0）
    @return 生成摘要，含理论超调与上升时间，供断言用

    @details 为什么要有这个场景：纯正弦只能证明"峰值频率识别得对"，
             证明不了"时域指标算得对"。这里让 roll 跟随一个**解析可算的
             二阶阶跃响应**，于是：
               · 调参页的阶跃分析应给出上升时间/超调/调节时间
               · 频谱页应看到叠加的 28 Hz 结构振动峰
               · 两边的数字都能和理论值对账

    @theory 二阶系统阶跃响应：
            y(t) = A·[1 − e^(−ζωn t)·(cos(ωd t) + ζ/√(1−ζ²)·sin(ωd t))]，
            ωd = ωn·√(1−ζ²)；超调 Mp = exp(−πζ/√(1−ζ²))；上升时间约 1.8/ωn
    """
    import math as _m

    mav = mavutil.mavlink.MAVLink(None)
    mav.srcSystem = 1
    mav.srcComponent = 1

    wd = wn * _m.sqrt(1.0 - zeta ** 2)
    n = int(seconds * fs_hz)
    dt = 1.0 / fs_hz
    t0_us = 1_690_000_000 * 1_000_000

    # 阶跃序列：2 s 时 0→10°，8 s 时 10→0°（来回各一次，便于验证双向）
    steps = [(2.0, 10.0), (8.0, 0.0)]
    amp = 0.0
    osc_hits = []      # 记录每次阶跃的时刻与幅度

    def resp(tau, a):
        """二阶阶跃响应（tau 为距阶跃时刻的秒数）"""
        if tau <= 0:
            return 0.0
        e = _m.exp(-zeta * wn * tau)
        return a * (1.0 - e * (_m.cos(wd * tau) + (zeta / _m.sqrt(1 - zeta ** 2))
                               * _m.sin(wd * tau)))

    with open(path, 'wb') as f:
        for i in range(n):
            tt = i * dt
            # 当前所处的阶跃段：把每个阶跃的响应叠起来（前一阶跃保持）
            y = 0.0
            for (ts, target) in steps:
                if tt >= ts:
                    prev = sum(v for (s2, v) in steps if s2 < ts)
                    y += resp(tt - ts, target - prev)
            # 叠加 28 Hz 结构振动（幅值 0.3°，模拟桨叶通过频率）
            vib = 0.3 * _m.sin(2 * _m.pi * 28.0 * tt)
            # 再叠加一点 50 Hz 电噪（幅值 0.1°）
            noise = 0.1 * _m.sin(2 * _m.pi * 50.0 * tt)

            roll_rad = _m.radians(y + vib + noise)
            pitch_rad = _m.radians(0.5 * _m.sin(2 * _m.pi * 0.5 * tt))

            ts_us = t0_us + int(round(tt * 1_000_000))
            f.write(struct.pack('>Q', ts_us))
            msg = mav.attitude_encode(
                int(tt * 1000) & 0xFFFFFFFF, roll_rad, pitch_rad, 0.0,
                0.0, 0.0, 0.0)
            f.write(msg.pack(mav))

    mp = _m.exp(-_m.pi * zeta / _m.sqrt(1 - zeta ** 2)) * 100.0
    return {
        'path': str(path),
        'scenario': 'dynamics',
        'fs_hz': fs_hz,
        'seconds': seconds,
        'frames': n,
        'dt_ms': round(dt * 1000, 3),
        'zeta': zeta,
        'wn_rad_s': wn,
        'wn_hz': round(wn / (2 * _m.pi), 4),
        'theory_overshoot_pct': round(mp, 2),
        'theory_rise_time_s': round(1.8 / wn, 3),
        'steps': steps,
        'vib': {'freq_hz': 28.0, 'amp_deg': 0.3},
        'bytes': path.stat().st_size,
    }


def main(argv: list[str]) -> int:
    out = Path(argv[1]) if len(argv) > 1 else Path('output/test_100hz.tlog')
    fs = int(argv[2]) if len(argv) > 2 else 100
    secs = float(argv[3]) if len(argv) > 3 else 60.0
    scenario = argv[4] if len(argv) > 4 else 'tone'
    out.parent.mkdir(parents=True, exist_ok=True)
    if scenario == 'dynamics':
        info = build_dynamics(out, fs, secs)
    else:
        info = build(out, fs, secs)
    for k, v in info.items():
        print(f'  {k}: {v}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
