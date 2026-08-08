#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plot_blackbox.py — 黑匣子 CSV 可视化（飞行数据分析）

用法:
  python plot_blackbox.py <csv文件> [-o 输出目录]

★ 自适应列名：CSV 列名来自 S 帧通道名表（通道自定义后列名可变）。
  按列名前缀分组绘制——缺列自动跳过，新增通道自动纳入"其他"图。

依赖: pip install matplotlib pandas
"""

import os
import argparse

import matplotlib
matplotlib.use('Agg')   # 无显示环境
import matplotlib.pyplot as plt
import pandas as pd

# 标准分组：前缀 → (图文件名, 标题, y轴单位)
GROUPS = [
    ("roll_deg", "attitude", "Attitude (roll/pitch/heading)", "deg"),
    ("pitch_deg", None, None, None),
    ("heading_deg", None, None, None),
    ("accel_", "accel", "Acceleration (m/s²)", "m/s²"),
    ("gyro_", "gyro", "Angular rate (deg/s)", "deg/s"),
    ("vel_", "velocity", "Velocity NED (m/s)", "m/s"),
    ("rel_", None, None, None),   # rel_ 三轴由水平轨迹图覆盖
    ("tvc", "tvc", "TVC servo angles (deg)", "deg"),
]


def plot_group(df, t, cols, fname, title, ylabel, outdir, base):
    """绘制一组列；缺列返回 False。"""
    if not cols:
        return False
    fig, ax = plt.subplots(figsize=(12, 5))
    for c in cols:
        ax.plot(t, df[c], label=c)
    if fname == "accel" and "accel_z_ms2" in cols:
        ax.axhline(-9.81, color='r', ls='--', lw=0.8, label='-g (static)')
    ax.set_xlabel('t (s)'); ax.set_ylabel(ylabel)
    ax.set_title(title); ax.legend(); ax.grid(True)
    fig.tight_layout(); fig.savefig(f"{outdir}/{base}_{fname}.png"); plt.close(fig)
    return True


def main():
    ap = argparse.ArgumentParser(description="Blackbox CSV plotter (adaptive columns)")
    ap.add_argument("csv", help="导出 CSV 文件")
    ap.add_argument("-o", "--outdir", default="output",
                    help="图表输出目录 (默认 output)")
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    os.makedirs(args.outdir, exist_ok=True)
    base = os.path.splitext(os.path.basename(args.csv))[0]

    if 't_ms' not in df.columns:
        print(f"错误：CSV 缺少 t_ms 列（列: {list(df.columns)[:8]}...）")
        return
    t = (df['t_ms'] - df['t_ms'].iloc[0]) / 1000.0   # 秒

    plotted = []
    used_cols = set()

    # 标准组
    for prefix, fname, title, ylabel in GROUPS:
        if fname is None:
            continue
        cols = [c for c in df.columns if c.startswith(prefix) and c != 't_ms']
        if plot_group(df, t, cols, fname, title, ylabel, args.outdir, base):
            plotted.append(fname)
            used_cols.update(cols)

    # 水平轨迹（rel_e vs rel_n）
    if 'rel_e_m' in df.columns and 'rel_n_m' in df.columns:
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.plot(df['rel_e_m'], df['rel_n_m'], marker='.', ms=1, lw=0.5)
        ax.plot(df['rel_e_m'].iloc[0], df['rel_n_m'].iloc[0], 'go', label='start')
        ax.plot(df['rel_e_m'].iloc[-1], df['rel_n_m'].iloc[-1], 'ro', label='end')
        ax.set_xlabel('East (m)'); ax.set_ylabel('North (m)')
        ax.set_title('Horizontal track'); ax.legend(); ax.grid(True); ax.axis('equal')
        fig.tight_layout(); fig.savefig(f"{args.outdir}/{base}_track.png"); plt.close(fig)
        plotted.append('track')
        used_cols.update(['rel_e_m', 'rel_n_m'])

    # 其他未归类通道（自定义新增的自动画出来）
    others = [c for c in df.columns if c not in used_cols and c not in ('t_ms', 'seq')]
    if others:
        fig, ax = plt.subplots(figsize=(12, 5))
        for c in others:
            ax.plot(t, df[c], label=c)
        ax.set_xlabel('t (s)'); ax.set_ylabel('value')
        ax.set_title('Other channels'); ax.legend(); ax.grid(True)
        fig.tight_layout(); fig.savefig(f"{args.outdir}/{base}_others.png"); plt.close(fig)
        plotted.append('others')

    print(f"已生成 {len(plotted)} 张图: {', '.join(plotted)}")
    if not plotted:
        print("（无可用列，检查 CSV 内容）")


if __name__ == "__main__":
    main()
