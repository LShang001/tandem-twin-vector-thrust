# -*- coding: utf-8 -*-
"""plot_style.py — 论文仿真图统一样式（字体/配色/坐标轴约定）。

@file    plot_style.py
@module  analysis/图件样式
@brief   统一 matplotlib 样式：SimSun 宋体 + STIX 公式 + Okabe-Ito 色盲安全配色。
@details 输入：无（import 即配置 rcParams）；处理：字体、线宽、刻度、矢量导出；
         输出：apply_style()/finish()/ref_hline()/log_sci_ticks() 等绘图辅助。
         不变量：全部图件导出矢量 PDF、中文标注、配色语义固定（蓝=SAS、朱红=INDI）。
@theory  N/A（图件样式，不含物理模型）
@frame   N/A
@units   N/A（线宽 pt、字号 pt）
@note    matplotlib 不做逐字形回退：Times+SimSun 混排会丢中文字形，故统一 SimSun；
         绘图脚本不得另设 rcParams 覆盖本模块，否则图件风格漂移。
@see     plot_results.py、run_vtol.py、TandemVec-Paper/fig-sim
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Okabe-Ito 色盲安全调色板
C_BLUE = '#0072B2'    # 蓝 — SAS
C_VERM = '#D55E00'    # 朱红 — INDI
C_GREEN = '#009E73'   # 蓝绿 — LQR
C_ORANGE = '#E69F00'  # 橙
C_SKY = '#56B4E9'     # 天蓝
C_PURPLE = '#CC79A7'  # 紫红
C_GRAY = '#7F7F7F'    # 参考线灰

def apply_style():
    """应用全局 rcParams（字体/线宽/刻度/矢量导出），供所有图件复用。"""
    plt.rcParams.update({
        # 字体：SimSun 宋体（中西文全覆盖，与 ctex 正文一致），公式 STIX
        # 注：matplotlib 不做逐字形回退，Times+SimSun 列表会导致中文缺字形，故统一用 SimSun
        'font.family': 'serif',
        'font.serif': ['SimSun'],
        'mathtext.fontset': 'stix',
        'axes.unicode_minus': False,
        # 字号（适配 0.8\\textwidth 缩放）
        'font.size': 9,
        'axes.titlesize': 9.5,
        'axes.labelsize': 9.5,
        'legend.fontsize': 8,
        'xtick.labelsize': 8.5,
        'ytick.labelsize': 8.5,
        # 线宽与刻度
        'lines.linewidth': 1.1,
        'axes.linewidth': 0.8,
        'xtick.direction': 'in',
        'ytick.direction': 'in',
        'xtick.top': True,
        'ytick.right': True,
        'xtick.major.size': 3.5,
        'ytick.major.size': 3.5,
        # 网格
        'axes.grid': True,
        'grid.alpha': 0.28,
        'grid.linewidth': 0.5,
        # 图例
        'legend.framealpha': 0.9,
        'legend.edgecolor': '0.7',
        # 输出
        'figure.dpi': 150,
        'savefig.bbox': 'tight',
        'savefig.pad_inches': 0.02,
    })

def ref_hline(ax, y, label=None):
    """灰色参考虚线"""
    ax.axhline(y, color=C_GRAY, ls='--', lw=0.7, alpha=0.8, label=label)

def log_sci_ticks(ax, axis='y'):
    """对数轴科学计数标签：$10^{n}$ 走 STIX 数学字体。
       默认 LogFormatter 用 \\mathdefault(SimSun) 渲染 U+2212 会缺字形变成 ¤。"""
    import numpy as np
    from matplotlib.ticker import FuncFormatter
    fmt = FuncFormatter(lambda v, _: f'$10^{{{int(round(np.log10(v)))}}}$')
    if axis == 'y':
        ax.yaxis.set_major_formatter(fmt)
    else:
        ax.xaxis.set_major_formatter(fmt)

def finish(fig, path):
    """收紧坐标轴并按矢量格式保存图件。

    @param[in] fig  matplotlib Figure
    @param[in] path 输出路径（PDF/SVG）
    @note   统一走这里导出，避免各脚本用不同 dpi/bbox 导致图件尺寸漂移。
    """
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    print(f'  -> {path.split("/")[-1]}')
