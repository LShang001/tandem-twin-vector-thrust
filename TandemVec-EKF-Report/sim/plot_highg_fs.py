# -*- coding: utf-8 -*-
"""高过载 8/16/32 g 量程试点图：协调转弯与拉杆。
@file    plot_highg_fs.py
@brief   pos/att vs 过载，按 IMU 量程分线。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 读 highg_fs_2026-10-27.jsonl（协调转弯）与 highg_pull_2026-10-27.json（纯拉杆），
         按 (过载档, IMU 量程) 取各 seed 的 pos_rmse/att_rmse_deg 中位数，画 2×2 折线：
         上排转弯、下排拉杆，左位置、右姿态，纵轴 symlog 兼顾跨量级。只画图不判裁决。
@frame   NED（pos_rmse 为位置误差范数）；att_rmse_deg 为固件欧拉角误差范数，不做坐标换算
@units   g（过载/量程）、m（位置 RMSE）、deg（姿态 RMSE）、m/s（仿真空速 300 m/s）
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
    """读协调转弯 jsonl，返回 (过载档列表, {(过载, 量程): (位置 RMSE 中位, 姿态 RMSE 中位)})。

    key 是量程字段名（默认 fs_g），同格多 seed 取中位数。
    """
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
    """出 2×2 高过载量程试点图（转弯/拉杆 × 位置/姿态），写 figs/nav_highg_fs_2026-10-27.png。

    纯拉杆只有 16/32 g 两档数据，缺档自动少一条线；横轴过载档来自数据本身，不硬编码。
    """
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
