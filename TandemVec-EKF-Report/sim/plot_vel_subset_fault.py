#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24u/§4.24v 图：放宽触发上限（15→25 m）后的两类安全性验证。

@file    plot_vel_subset_fault.py
@brief   图 1（§4.24u 故障交互）：四故障格的发生率/配对数/分布；图 2（§4.24v 弱失稳档）：
         两档推力上的发生率与 max、逐 seed 配对
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 两图共同回答一个问题：**把上限放宽之后，除了 30.5 g 主格变好以外，别的地方有没有变坏。**
         ★ 图必须把"关臂"和"开臂"画在一起、同一坐标；只看开臂的绝对水平无法判断"是否被引入"。
         ★ 故障格的 pos_rms 分布常常是双峰（一部分 seed 正常、一部分被故障拖走），
           因此箱线只作形状参考，判读用散点与发生率。
@frame   NED；pos_rms [m]
@units   米 / 个 seed
@note    图内文字避免 U+21D2（微软雅黑缺字）
@see     sim/check_vel_subset_fault.py（Z1–Z3）、sim/check_vel_subset_lowerloads.py（V1–V3）、报告 §4.24u/§4.24v
"""
import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.family"] = "sans-serif"
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
FIG = os.path.abspath(os.path.join(HERE, "..", "figs"))
FAIL_M = 25.0
CELL_LABEL = {"nan1s": "IMU NaN 1 s", "out5": "GNSS 失联 5 s",
              "out20": "GNSS 失联 20 s", "vib1g": "振动 1 g@80 Hz"}
CELLS = ["nan1s", "out5", "out20", "vib1g"]


def load_fault():
    path = os.path.join(SD, "vel_subset_fault_2026-10-28_z.jsonl")
    if not os.path.exists(path):
        raise SystemExit("缺证据：%s" % path)
    d = {(c, a): {} for c in CELLS for a in ("off", "on")}
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                r = json.loads(line)
                d[(r["cell"], r["arm"])][int(r["seed"])] = float(r["pos_rms"])
    return d


def load_lowerloads():
    out = {}
    for arm, tag in (("off", "v_off"), ("on", "v_on")):
        path = os.path.join(SD, "attrib305_2026-10-28_%s.jsonl" % tag)
        if not os.path.exists(path):
            raise SystemExit("缺证据：%s" % path)
        m = {}
        with open(path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line:
                    r = json.loads(line)
                    m[(int(r["thrust_n"]), int(r["seed"]))] = float(r["pos_rms"])
        out[arm] = m
    return out


def main():
    # 图 1：故障交互
    fd = load_fault()
    fig, ax = plt.subplots(1, 3, figsize=(17.0, 5.2))
    x = np.arange(len(CELLS))
    r_off = [(np.array(list(fd[(c, "off")].values())) > FAIL_M).sum() for c in CELLS]
    r_on = [(np.array(list(fd[(c, "on")].values())) > FAIL_M).sum() for c in CELLS]
    ax[0].bar(x - 0.2, r_off, width=0.4, color="#7f7f7f", label="子集关")
    ax[0].bar(x + 0.2, r_on, width=0.4, color="#2ca02c", label="子集开（默认 25 m）")
    for i, (a, b) in enumerate(zip(r_off, r_on)):
        ax[0].text(i - 0.2, a + 0.12, str(a), ha="center", fontsize=8)
        ax[0].text(i + 0.2, b + 0.12, str(b), ha="center", fontsize=8)
    ax[0].axhline(len(next(iter(fd.values()))), color="k", ls=":", lw=0.8)
    ax[0].set_xticks(x)
    ax[0].set_xticklabels([CELL_LABEL[c] for c in CELLS], rotation=15)
    ax[0].set_ylabel("pos_rms > 25 m 的 seeds")
    ax[0].set_title("§4.24u 故障交互（30.5 g/20 Hz，20 seeds/格）：开臂不得高于关臂")
    ax[0].legend(fontsize=8)
    ax[0].grid(axis="y", alpha=0.3)

    colors = ["#1f77b4", "#d62728", "#ff7f0e", "#2ca02c"]
    for c, col in zip(CELLS, colors):
        off = np.sort(np.array(list(fd[(c, "off")].values())))
        on = np.sort(np.array(list(fd[(c, "on")].values())))
        ax[1].plot(off, np.arange(1, len(off) + 1) / len(off), color=col, lw=1.6,
                   ls="--", label="%s 关" % CELL_LABEL[c])
        ax[1].plot(on, np.arange(1, len(on) + 1) / len(on), color=col, lw=1.6,
                   label="%s 开" % CELL_LABEL[c])
    ax[1].axvline(FAIL_M, color="k", ls=":", lw=1)
    ax[1].set_xscale("log")
    ax[1].set_xlabel("pos_rms [m]（对数）")
    ax[1].set_ylabel("累积比例")
    ax[1].set_title("故障格 CDF：实线（开）不应整体右移")
    ax[1].legend(fontsize=6, ncol=2)
    ax[1].grid(alpha=0.3)

    for i, c in enumerate(CELLS):
        off = np.array(list(fd[(c, "off")].values()))
        on = np.array(list(fd[(c, "on")].values()))
        jit = (np.random.RandomState(0).rand(len(off)) - 0.5) * 0.25
        ax[2].scatter(np.full(len(off), i) + jit - 0.15, off, s=14, c="#7f7f7f", alpha=0.8)
        ax[2].scatter(np.full(len(on), i) + jit + 0.15, on, s=14, c="#2ca02c", alpha=0.8)
    ax[2].axhline(FAIL_M, color="k", ls=":", lw=1)
    ax[2].set_xticks(x)
    ax[2].set_xticklabels([CELL_LABEL[c] for c in CELLS], rotation=15)
    ax[2].set_ylabel("pos_rms [m]")
    ax[2].set_yscale("log")
    ax[2].set_title("逐 seed 散点（灰=关 / 绿=开），双峰时不看箱线")
    ax[2].grid(axis="y", alpha=0.3)
    fig.suptitle("§4.24u 放宽上限后的故障交互矩阵（同二进制 tv_32g_q2，core5）", y=1.0)
    fig.tight_layout()
    p1 = os.path.join(FIG, "nav_velsubset_fault_2026-10-28.png")
    fig.savefig(p1, dpi=140, bbox_inches="tight")
    plt.close(fig)

    # 图 2：弱失稳档
    ld = load_lowerloads()
    thr_list = sorted({t for t, _ in ld["off"]})
    fig, ax = plt.subplots(1, 3, figsize=(17.0, 5.2))
    labels = ["%.1f g" % (t / 781.8) for t in thr_list]
    x2 = np.arange(len(thr_list))
    f_off = [int(sum(1 for (t, _s), v in ld["off"].items() if t == th and v > FAIL_M))
             for th in thr_list]
    f_on = [int(sum(1 for (t, _s), v in ld["on"].items() if t == th and v > FAIL_M))
            for th in thr_list]
    ax[0].bar(x2 - 0.2, f_off, width=0.4, color="#7f7f7f", label="子集关")
    ax[0].bar(x2 + 0.2, f_on, width=0.4, color="#2ca02c", label="子集开（默认 25 m）")
    for i, (a, b) in enumerate(zip(f_off, f_on)):
        ax[0].text(i - 0.2, a + 0.08, "%d/50" % a, ha="center", fontsize=8)
        ax[0].text(i + 0.2, b + 0.08, "%d/50" % b, ha="center", fontsize=8)
    ax[0].set_xticks(x2)
    ax[0].set_xticklabels(labels)
    ax[0].set_ylabel("pos_rms > 25 m 的 seeds")
    ax[0].set_title("§4.24v 弱失稳档：没有失效可修时不得引入失效")
    ax[0].legend(fontsize=8)
    ax[0].grid(axis="y", alpha=0.3)

    m_off = [max(v for (t, _s), v in ld["off"].items() if t == th) for th in thr_list]
    m_on = [max(v for (t, _s), v in ld["on"].items() if t == th) for th in thr_list]
    ax[1].plot(x2, m_off, "o--", color="#7f7f7f", label="max，子集关")
    ax[1].plot(x2, m_on, "s-", color="#2ca02c", label="max，子集开")
    ax[1].axhline(FAIL_M, color="k", ls=":", lw=1)
    ax[1].set_xticks(x2)
    ax[1].set_xticklabels(labels)
    ax[1].set_ylabel("max pos_rms [m]")
    ax[1].set_title("两档 max：开臂不得超过关臂 +5 m")
    ax[1].legend(fontsize=8)
    ax[1].grid(alpha=0.3)

    thr = thr_list[0]
    off = np.array([v for (t, _s), v in ld["off"].items() if t == thr])
    on = np.array([v for (t, _s), v in ld["on"].items() if t == thr])
    ax[2].plot([0, 40], [0, 40], "k--", lw=0.8)
    ax[2].scatter(off, on, s=16, c="#1f77b4", alpha=0.8)
    ax[2].axvline(FAIL_M, color="#7f7f7f", ls=":", lw=1)
    ax[2].axhline(FAIL_M, color="#7f7f7f", ls=":", lw=1)
    ax[2].set_xlabel("%.1f g：子集关 pos_rms [m]" % (thr / 781.8))
    ax[2].set_ylabel("子集开 pos_rms [m]")
    ax[2].set_title("逐 seed 配对（%s）：对角线下方=变好" % labels[0])
    ax[2].grid(alpha=0.3)
    fig.suptitle("§4.24v 弱失稳档不添乱（同二进制 tv_32g_q2，core6，seeds 51–100）", y=1.0)
    fig.tight_layout()
    p2 = os.path.join(FIG, "nav_velsubset_lowerloads_2026-10-28.png")
    fig.savefig(p2, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print("written", p1)
    print("written", p2)


if __name__ == "__main__":
    main()


def load_tag(tag):
    path = os.path.join(SD, "attrib305_2026-10-28_%s.jsonl" % tag)
    if not os.path.exists(path):
        raise SystemExit("缺证据：%s" % path)
    out = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                r = json.loads(line)
                out[int(r["seed"])] = float(r["pos_rms"])
    return out


def plot_quota():
    """§4.24w/§4.24x：配额门在"有/无幅值门"两种配置下的效果。"""
    cfgs = [("无幅值门 · 配额 255", "xq_nocap_q255_s51", "xq_nocap_q255_s1", "#d62728"),
            ("无幅值门 · 配额 1", "xq_nocap_q1_s51", "xq_nocap_q1_s1", "#ff7f0e"),
            ("cap 25 · 配额 1（现行默认）", "w_q1_s51", "w_q1_s1", "#2ca02c")]
    data = [(lbl, load_tag(a51), load_tag(a1), col) for lbl, a51, a1, col in cfgs]
    fig, ax = plt.subplots(1, 3, figsize=(17.0, 5.2))
    x = np.arange(len(data))
    f51 = [int((np.array(list(d51.values())) > FAIL_M).sum()) for _, d51, _, _ in data]
    f1 = [int((np.array(list(d1.values())) > FAIL_M).sum()) for _, _, d1, _ in data]
    ax[0].bar(x - 0.2, f51, width=0.4, color="#1f77b4", label="seeds 51–100")
    ax[0].bar(x + 0.2, f1, width=0.4, color="#ff7f0e", label="seeds 1–50")
    for i, (a, b) in enumerate(zip(f51, f1)):
        ax[0].text(i - 0.2, a + 0.08, "%d/50" % a, ha="center", fontsize=8)
        ax[0].text(i + 0.2, b + 0.08, "%d/50" % b, ha="center", fontsize=8)
    ax[0].set_xticks(x)
    ax[0].set_xticklabels([t[0] for t in data], rotation=18)
    ax[0].set_ylabel("pos_rms > 25 m 的 seeds")
    ax[0].set_title("§4.24x 配额能否替代幅值门：能压掉大部分尾部，但留下 1 例单次超大触发")
    ax[0].legend(fontsize=8)
    ax[0].grid(axis="y", alpha=0.3)

    seg = "seeds 1–50（无幅值门）"
    q255 = data[0][2]
    q1 = data[1][2]
    off = np.array([q255[s] for s in sorted(q255)])
    on = np.array([q1[s] for s in sorted(q1)])
    lim = max(float(np.maximum(off, on).max()) * 1.05, FAIL_M * 1.4)
    ax[1].plot([0, lim], [0, lim], "k--", lw=0.8)
    ax[1].scatter(off, on, s=18, c="#ff7f0e", alpha=0.85)
    ax[1].axvline(FAIL_M, color="#7f7f7f", ls=":", lw=1)
    ax[1].axhline(FAIL_M, color="#7f7f7f", ls=":", lw=1)
    ax[1].set_xlabel("配额 255 pos_rms [m]")
    ax[1].set_ylabel("配额 1 pos_rms [m]")
    ax[1].set_title("%s 逐 seed 配对：对角线下方=配额门带来改善" % seg)
    ax[1].grid(alpha=0.3)

    for lbl, d51, d1, col in data:
        v = np.sort(np.array(list(d1.values())))
        ax[2].plot(v, np.arange(1, len(v) + 1) / len(v), lw=1.7, color=col, label=lbl)
    ax[2].axvline(FAIL_M, color="k", ls=":", lw=1)
    ax[2].set_xscale("log")
    ax[2].set_xlabel("pos_rms [m]（对数，seeds 1–50）")
    ax[2].set_ylabel("累积比例")
    ax[2].set_title("CDF：配额 1 只掐右尾，主体分布不动")
    ax[2].legend(fontsize=7)
    ax[2].grid(alpha=0.3)
    fig.suptitle("§4.24w/§4.24x 每历元配额门（同二进制 tv_32g_w2，core5，30.5 g/20 Hz）", y=1.0)
    fig.tight_layout()
    p = os.path.join(FIG, "nav_velsubset_quota_2026-10-28.png")
    fig.savefig(p, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print("written", p)


if __name__ == "__main__":
    plot_quota()
