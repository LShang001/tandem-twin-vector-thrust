# -*- coding: utf-8 -*-
"""零偏阶跃扫描成图（时机/轴向/方向/长失联）。

@file    plot_att_aid_bias.py
@brief   读 sim-data/att-aid-mc-2026-11-01/mc_bias_sweep{,2}.json，产出
         “条件 × 模式 姿态中位柱状图”，生成后目视审查。
@module  TandemVec-EKF-Report/作图脚本（L2 研究设施）
"""
import json
import os
import statistics

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.normpath(os.path.join(HERE, "..", "sim-data", "att-aid-mc-2026-11-01"))
FIG = os.path.normpath(os.path.join(HERE, "..", "figs"))

LABEL = {
    "step05_xy": "阶跃@5 s\n(GNSS 有效)",
    "step20_x": "失联瞬间\n滚转轴",
    "step20_y": "失联瞬间\n俯仰轴",
    "step20_z": "失联瞬间\n航向轴",
    "step20_negx": "失联瞬间\n反向滚转",
    "step20_xz": "失联瞬间\n滚转+航向",
    "step20_yz": "失联瞬间\n俯仰+航向",
    "step20_xy": "失联瞬间\n双轴",
    "step20_xy_t": "瞬时阶跃\n(15 s 恢复)",
    "step20_xy_wind": "双轴+风",
    "step20_xy_vib": "双轴+1 g 振动",
    "step15_xy_60s": "双轴\n60 s 失联",
}
ORDER = list(LABEL.keys())


def setup_font():
    for name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC"):
        try:
            font_manager.findfont(name, fallback_to_default=False)
            plt.rcParams["font.family"] = name
            break
        except Exception:
            continue
    plt.rcParams["axes.unicode_minus"] = False


def load():
    recs = []
    for fn in ("mc_bias_sweep.json", "mc_bias_sweep2.json"):
        p = os.path.join(DATA, fn)
        if not os.path.exists(p):
            continue
        with open(p, encoding="utf-8") as fh:
            recs.extend(r for r in json.load(fh) if "_error" not in r)
    key = {}
    for r in recs:
        key[(r["_cond"], r["_mode"], r.get("seed"))] = r
    return list(key.values())


def med(recs, cond, mode, field):
    xs = [r[field] for r in recs if r["_cond"] == cond and r["_mode"] == mode
          and isinstance(r.get(field), (int, float))]
    return statistics.median(xs) if xs else float("nan")


def main():
    setup_font()
    os.makedirs(FIG, exist_ok=True)
    recs = load()
    print(f"[data] {len(recs)} records")
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.0), sharey=False)
    width = 0.38
    for ax, field, title in ((axes[0], "nav_att_rmse", "姿态 RMSE 中位 [deg]"),
                             (axes[1], "pos_rms", "水平位置 RMS 中位 [m]")):
        for mi, mode in enumerate((1, 5)):
            ys = [med(recs, c, mode, field) for c in ORDER]
            x = np.arange(len(ORDER)) + (mi - 0.5) * width
            ax.bar(x, ys, width=width * 0.95,
                   color=("#4c72b0" if mode == 1 else "#55a868"),
                   alpha=0.85, label=f"mode {mode}")
        ax.set_xticks(range(len(ORDER)))
        ax.set_xticklabels([LABEL[c] for c in ORDER], fontsize=8)
        ax.set_title(title)
        ax.grid(True, axis="y", alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle("陀螺零偏阶跃扫描（0.3 dps，10 seeds，VTOL mag_clean）")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_att_aid_bias_sweep.png"), dpi=150)
    plt.close(fig)
    print("[fig] nav_att_aid_bias_sweep.png")


if __name__ == "__main__":
    main()
