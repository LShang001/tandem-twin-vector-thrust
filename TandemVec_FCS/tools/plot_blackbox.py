#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plot_blackbox.py — 黑匣子 CSV 可视化（飞行数据分析）

用法:
  python plot_blackbox.py <csv文件> [-o 输出目录]
  （CSV 由 tools/flash_export.py 导出）

图表：
  1. 姿态角（roll/pitch/heading）
  2. 加速度（三轴，含重力偏置）
  3. 角速度（三轴）
  4. 速度（NED）
  5. 相对位置（NED，水平轨迹）
  6. TVC 舵机角度 + 发动机压力

依赖: pip install matplotlib
"""

import sys
import os
import argparse

import matplotlib
matplotlib.use('Agg')   # 无显示环境
import matplotlib.pyplot as plt
import pandas as pd

def main():
    ap = argparse.ArgumentParser(description="Blackbox CSV plotter")
    ap.add_argument("csv", help="导出 CSV 文件")
    ap.add_argument("-o", "--outdir", default="output",
                    help="图表输出目录 (默认 output)")
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    os.makedirs(args.outdir, exist_ok=True)
    base = os.path.splitext(os.path.basename(args.csv))[0]

    t = (df['t_ms'] - df['t_ms'].iloc[0]) / 1000.0   # 秒

    # 1. 姿态
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t, df['roll_deg'], label='roll')
    ax.plot(t, df['pitch_deg'], label='pitch')
    ax.plot(t, df['heading_deg'], label='heading')
    ax.set_xlabel('t (s)'); ax.set_ylabel('deg')
    ax.set_title('Attitude'); ax.legend(); ax.grid(True)
    fig.tight_layout(); fig.savefig(f"{args.outdir}/{base}_attitude.png"); plt.close(fig)

    # 2. 加速度
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t, df['accel_x_ms2'], label='ax')
    ax.plot(t, df['accel_y_ms2'], label='ay')
    ax.plot(t, df['accel_z_ms2'], label='az')
    ax.axhline(-9.81, color='r', ls='--', lw=0.8, label='-g (static)')
    ax.set_xlabel('t (s)'); ax.set_ylabel('m/s²')
    ax.set_title('Acceleration'); ax.legend(); ax.grid(True)
    fig.tight_layout(); fig.savefig(f"{args.outdir}/{base}_accel.png"); plt.close(fig)

    # 3. 角速度
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t, df['gyro_x_dps'], label='gx')
    ax.plot(t, df['gyro_y_dps'], label='gy')
    ax.plot(t, df['gyro_z_dps'], label='gz')
    ax.set_xlabel('t (s)'); ax.set_ylabel('deg/s')
    ax.set_title('Angular rate'); ax.legend(); ax.grid(True)
    fig.tight_layout(); fig.savefig(f"{args.outdir}/{base}_gyro.png"); plt.close(fig)

    # 4. 速度
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t, df['vel_n_ms'], label='vn')
    ax.plot(t, df['vel_e_ms'], label='ve')
    ax.plot(t, df['vel_d_ms'], label='vd')
    ax.set_xlabel('t (s)'); ax.set_ylabel('m/s')
    ax.set_title('Velocity (NED)'); ax.legend(); ax.grid(True)
    fig.tight_layout(); fig.savefig(f"{args.outdir}/{base}_velocity.png"); plt.close(fig)

    # 5. 水平轨迹
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(df['rel_e_m'], df['rel_n_m'], marker='.', ms=1, lw=0.5)
    ax.plot(df['rel_e_m'].iloc[0], df['rel_n_m'].iloc[0], 'go', label='start')
    ax.plot(df['rel_e_m'].iloc[-1], df['rel_n_m'].iloc[-1], 'ro', label='end')
    ax.set_xlabel('East (m)'); ax.set_ylabel('North (m)')
    ax.set_title('Horizontal track'); ax.legend(); ax.grid(True); ax.axis('equal')
    fig.tight_layout(); fig.savefig(f"{args.outdir}/{base}_track.png"); plt.close(fig)

    # 6. TVC + 压力
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t, df['tvc1_deg'], label='tvc1')
    ax.plot(t, df['tvc2_deg'], label='tvc2')
    ax.set_xlabel('t (s)'); ax.set_ylabel('deg')
    ax.set_title('TVC servo angles'); ax.legend(); ax.grid(True)
    fig.tight_layout(); fig.savefig(f"{args.outdir}/{base}_tvc.png"); plt.close(fig)

    print(f"已生成 {args.outdir}/{base}_*.png (6 张图)")

if __name__ == "__main__":
    main()
