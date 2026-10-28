#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24q 图：速度子集触发时刻的位置残差与后果（反向判据的证据图）。

@file    plot_vel_subset_posres.py
@brief   画四张图：触发时刻位置残差 vs 后果散点（判据来源）、发生率柱状对照、
         20 Hz 逐 seed 配对、六臂 pos_rms 分布（CDF + 直方）
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 图 1 是本节的核心证据：它直接展示"触发时位置残差小 ⇒ 有益、大 ⇒ 有害"的分离带，
         以及 15 m 上限落在哪里。残差从 200 Hz 追踪里**重新计算**（不硬编码），
         后果 Δ 从 SIL JSON 的 pos_rms 差取。
         图 2–4 是判据 X2/X3 的图形化：两端（10 Hz 收益、20 Hz 伤害）必须同时可见。
@frame   NED；pos_rms [m]，触发时刻位置残差 [m]
@units   米 / 秒 / 个 seed
@note    若 X 组裁决为回退，图 1 仍成立（它是对旧数据的新读法），图 2–4 用于说明回退理由
@see     sim/check_vel_subset_posres.py（X1–X4 裁决）、报告 §4.24q
"""
import csv
import json
import os
import re

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.family"] = "sans-serif"
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
TR = os.path.join(SD, "vel20_traces")
FIG = os.path.abspath(os.path.join(HERE, "..", "figs"))
CAP_M = 15.0
# 追踪文件名 → (刷新率 Hz, 组标签)
TRACES = [("hz10_s%d.csv" % s, 10, "10 Hz 有益") for s in (6, 17, 19, 22, 25, 27)] + \
         [("fire_s%d_sub1.csv" % s, 20, "20 Hz 变好") for s in (3, 9, 11, 18, 29, 30)] + \
         [("fire_s%d_sub1.csv" % s, 20, "20 Hz 变差") for s in (16, 20, 28, 37, 45, 47)]
ARMS = [("10 Hz 子集关", "onset50"), ("10 Hz 旧口径", "vel50sub"),
        ("10 Hz +15m 上限", "q10"), ("20 Hz 子集关", "hz20only"),
        ("20 Hz 旧口径", "vel50sub_hz20"), ("20 Hz +15m 上限", "q20"),
        ("20 Hz 上限取消", "q20nocap")]


def load_jsonl(tag):
    out = {}
    with open(os.path.join(SD, "attrib305_2026-10-28_%s.jsonl" % tag), encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                r = json.loads(line)
                out[int(r["seed"])] = r
    return out


def seed_from(name):
    m = re.search(r"_s(\d+)(?:_|\.)", name)
    assert m, "文件名里取不到 seed：%s" % name
    return int(m.group(1))


def first_fire_residual(path):
    """返回 (首次触发时刻 t, 该帧位置残差 m, 触发次数)；无触发返回 None。"""
    if not os.path.exists(path):
        return None
    prev = 0
    first = None
    cnt = 0
    with open(path, encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            v = int(float(r["vel_sub_rec"] or 0))
            if v > prev:
                cnt += 1
                if first is None:
                    first = (float(r["t"]), float(r["gnss_resid_pos_ne"]))
            prev = v
    return (first[0], first[1], cnt) if first else None


def main():
    os.makedirs(FIG, exist_ok=True)
    data = {label: load_jsonl(tag) for label, tag in ARMS}
    off10 = data["10 Hz 子集关"]
    off20 = data["20 Hz 子集关"]

    # ---- 图 1：触发时刻位置残差 vs 后果 ----
    fig, ax = plt.subplots(figsize=(8.8, 5.2))
    colors = {"10 Hz 有益": "tab:green", "20 Hz 变好": "tab:blue", "20 Hz 变差": "crimson"}
    rows = []
    for fname, hz, grp in TRACES:
        s = seed_from(fname)
        info = first_fire_residual(os.path.join(TR, fname))
        if info is None:
            continue
        t, resid, cnt = info
        base = off10 if hz == 10 else off20
        on = data["10 Hz 旧口径"] if hz == 10 else data["20 Hz 旧口径"]
        if s not in base or s not in on:
            continue
        delta = float(on[s]["pos_rms"]) - float(base[s]["pos_rms"])
        rows.append((grp, hz, s, t, resid, cnt, delta))
        ax.scatter(resid, delta, s=70, color=colors[grp], marker="o" if hz == 10 else "s",
                   alpha=0.9)
        ax.annotate("s%d" % s, (resid, delta), fontsize=7, xytext=(4, 4),
                    textcoords="offset points")
    ax.axvline(CAP_M, color="black", ls="--", lw=1.2)
    ax.text(CAP_M + 0.4, ax.get_ylim()[1] * 0.6, "15 m 上限", fontsize=9)
    ax.axhline(0, color="gray", lw=1.0)
    ax.set_xlabel("首次触发时刻的 GNSS 位置残差 [m]")
    ax.set_ylabel("后果：pos_rms(子集开) − pos_rms(子集关) [m]")
    handles = [plt.Line2D([], [], marker="o", ls="", color=c, label=l)
               for l, c in colors.items()]
    ax.legend(handles=handles, fontsize=8)
    ax.grid(ls="dotted", alpha=0.4)
    ax.set_title("§4.24q 判据来源：残差 ≤12 m 全部有益，≥21 m 全部有害（分离带 12.0→21.3 m）")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_velsubset_posres_firetime_2026-10-28.png"), dpi=150)
    plt.close(fig)

    # ---- 图 2：发生率与尾部对照 ----
    labels = [l for l, _ in ARMS]
    fails = [sum(1 for v in d.values() if float(v["pos_rms"]) > 25) for _, d in data.items()]
    p95 = [float(np.percentile([float(v["pos_rms"]) for v in d.values()], 95))
           for _, d in data.items()]
    fig, ax = plt.subplots(figsize=(9.4, 4.8))
    x = np.arange(len(labels))
    ax.bar(x - 0.2, fails, 0.4, color="crimson", alpha=0.85, label=">25 m 的 seed 数")
    ax2 = ax.twinx()
    ax2.bar(x + 0.2, p95, 0.4, color="tab:blue", alpha=0.6, label="p95 [m]（右轴）")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=20, ha="right", fontsize=8)
    ax.set_ylabel("fail 数 / 50")
    ax2.set_ylabel("p95 [m]")
    for i, (f, p) in enumerate(zip(fails, p95)):
        ax.text(i - 0.2, f + 0.2, "%d" % f, ha="center", fontsize=8)
        ax2.text(i + 0.2, p + 0.5, "%.1f" % p, ha="center", fontsize=7)
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, fontsize=8, loc="upper left")
    ax.set_title("X2/X3：两端都要看（10 Hz 保住 0 fail，20 Hz 消掉 105 m 尾部）")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_velsubset_posres_incidence_2026-10-28.png"), dpi=150)
    plt.close(fig)

    # ---- 图 3：20 Hz 逐 seed 配对（上限 vs 子集关），标出 6 个受害 seed ----
    bad = [16, 20, 28, 37, 45, 47]
    keys = sorted(set(data["20 Hz +15m 上限"]) & set(off20))
    assert len(keys) == 50, "配对 %d != 50" % len(keys)
    fig, ax = plt.subplots(figsize=(6.6, 6.2))
    a = [float(off20[s]["pos_rms"]) for s in keys]
    b = [float(data["20 Hz +15m 上限"][s]["pos_rms"]) for s in keys]
    ax.scatter(a, b, s=24, alpha=0.8, label="全部 50 seeds")
    ax.scatter([float(off20[s]["pos_rms"]) for s in bad],
               [float(data["20 Hz +15m 上限"][s]["pos_rms"]) for s in bad],
               s=70, marker="x", color="crimson", label="§4.24p 的 6 个受害 seed")
    lim = max(max(a), max(b)) * 1.05
    ax.plot([0, lim], [0, lim], color="gray", ls="--", lw=1.0)
    ax.set_xlabel("20 Hz 子集关 pos_rms [m]")
    ax.set_ylabel("20 Hz +15 m 上限 pos_rms [m]")
    ax.legend(fontsize=8)
    ax.grid(ls="dotted", alpha=0.4)
    ax.set_title("X3：受害 seed 应回到 y=x 线或其下方")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_velsubset_posres_paired_2026-10-28.png"), dpi=150)
    plt.close(fig)

    # ---- 图 4：六臂分布（CDF）----
    fig, ax = plt.subplots(figsize=(8.4, 5.0))
    for label, key in (("10 Hz 子集关", "10 Hz 子集关"), ("10 Hz 旧口径", "10 Hz 旧口径"),
                       ("10 Hz +15m 上限", "10 Hz +15m 上限"),
                       ("20 Hz 旧口径", "20 Hz 旧口径"), ("20 Hz +15m 上限", "20 Hz +15m 上限")):
        v = np.sort([float(x["pos_rms"]) for x in data[key].values()])
        ax.plot(v, np.arange(1, len(v) + 1) / len(v), lw=1.6, label=label)
    ax.axvline(25, color="gray", ls=":", lw=1)
    ax.set_xscale("symlog", linthresh=1.0)
    ax.set_xlabel("pos_rms [m]（对数轴）")
    ax.set_ylabel("CDF")
    ax.legend(fontsize=8)
    ax.grid(ls="dotted", alpha=0.4)
    ax.set_title("分布对照：上限版应同时贴住 10 Hz 旧口径的低位与避开 20 Hz 旧口径的长尾")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "nav_velsubset_posres_cdf_2026-10-28.png"), dpi=150)
    plt.close(fig)

    print(json.dumps({"fire_table": [
        {"grp": g, "hz": hz, "seed": s, "t_first_fire": round(t, 2),
         "pos_resid_at_fire_m": round(r, 2), "n_fires": c,
         "delta_pos_rms_m": round(d, 2)} for g, hz, s, t, r, c, d in rows]},
        ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
