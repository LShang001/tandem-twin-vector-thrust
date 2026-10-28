# -*- coding: utf-8 -*-
"""高过载 8/16/32 g 量程试点图：协调转弯与拉杆。
@file    plot_highg_fs.py
@brief   pos/att vs 过载，按 IMU 量程分线。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
"""
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Arial"]
plt.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "sim-data")
FIGS = os.path.join(HERE, "..", "figs")


def load_rows(name, key="fs_g"):
    rows = [json.loads(l) for l in open(os.path.join(DATA, name), encoding="utf-8") if l.strip()]
    loads = sorted({r["load"] for r in rows})
    out = {}
    for load_v in loads:
        for fs in sorted({r[key] for r in rows}):
            rs = [r for r in rows if r["load"] == load_v and r[key] == fs]
            out[(load_v, fs)] = (
                float(np.median([r["pos_rmse"] for r in rs])),
                float(np.median([r["att_rmse_deg"] for r in rs])),
            )
    return loads, out


def main():
    turn_loads, turn = load_rows("highg_fs_2026-10-27.jsonl")
    pull = json.load(open(os.path.join(DATA, "highg_pull_2026-10-27.json"),
                          encoding="utf-8"))
    pull_loads = sorted({r["load"] for r in pull})
    pull_out = {}
    for load_v in pull_loads:
        for fs in (16.0, 32.0):
            rs = [r for r in pull if r["load"] == load_v and r["fs_g"] == fs]
            if rs:
                pull_out[(load_v, fs)] = (
                    float(np.median([r["pos_rmse"] for r in rs])),
                    float(np.median([r["att_rmse_deg"] for r in rs])),
                )

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    for ax, data, loads, title in (
            (axes[0, 0], turn, turn_loads, "协调转弯：位置 RMSE"),
            (axes[0, 1], turn, turn_loads, "协调转弯：姿态 RMSE"),
            (axes[1, 0], pull_out, pull_loads, "纯拉杆：位置 RMSE"),
            (axes[1, 1], pull_out, pull_loads, "纯拉杆：姿态 RMSE")):
        for fs, color in ((8.0, "#d95f5f"), (16.0, "#4f8fc0"),
                          (32.0, "#2e8b57")):
            xs, ys = [], []
            for load_v in loads:
                if (load_v, fs) in data:
                    xs.append(load_v)
                    ys.append(data[(load_v, fs)][0 if "位置" in title else 1])
            if xs:
                ax.plot(xs, ys, "o-", color=color, label=f"{fs:g} g 量程")
        ax.set_xlabel("过载 (g)")
        ax.set_yscale("symlog", linthresh=1.0)
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle("高过载 IMU 量程试点（协调转弯/拉杆，3 seeds，300 m/s）")
    fig.tight_layout()
    out = os.path.join(FIGS, "nav_highg_fs_2026-10-27.png")
    fig.savefig(out, dpi=150)
    print(out)


if __name__ == "__main__":
    main()
