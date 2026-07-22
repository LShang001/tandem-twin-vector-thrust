# -*- coding: utf-8 -*-
"""论文仿真图统一样式：Times New Roman(西文) + SimSun(中文) + STIX 公式
   配色采用 Okabe-Ito 色盲安全调色板，全部图件中文标注。"""

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
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    print(f'  -> {path.split("/")[-1]}')
