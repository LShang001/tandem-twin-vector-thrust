#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24s/§4.24t 图：把"触发时位置残差上限"从单点量成区间。

@file    plot_vel_subset_capband.py
@brief   三张面板：发生率随 cap 的阶梯（两段 seeds 并列）、逐 seed 配对（cap15 vs cap25）、
         各 cap 的分布（CDF + 右尾放大）
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 这张图要一眼能看出两件事：
         ① **收紧比放宽更危险**——发生率在 cap 变小的一侧（12/13）反而升高，
            所以"阈值取小一点更安全"的直觉在这里是错的；
         ② 平台有多宽——cap15 到 cap25（以及 §4.24t 的 30/35）之间发生率一直是 0，
            直到"无上限"才出现尾部。区间宽度决定默认值该放在哪里。
         右尾必须单独放大画：主分布挤在 5–25 m，线性轴上看不出 44–105 m 的灾难来自哪一档。
@frame   NED；pos_rms [m]
@units   米 / 个 seed / 比例
@note    图内文字避免 U+21D2（微软雅黑缺字）
@see     sim/check_vel_subset_capband.py（Y5）、sim/check_vel_subset_capupper.py（Y6）、报告 §4.24s/§4.24t
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
# (cap 标签, seeds 51-100 的 tag, seeds 1-50 的 tag)；None 表示该段未跑该档
LADDER = [("12", "y20_c12", None), ("13", "y20_c13", None), ("14", "y20_c14", None),
          ("15", "y20_c15", "q20"), ("18", "y20_c18", None), ("21", "y20_c21", None),
          ("25", "y20_c25", "p20_c25"), ("30", "u30_s51", "u30_s1"),
          ("35", "u35_s51", "u35_s1"), ("40", "v40_s51", "v40_s1"),
          ("50", "v50_s51", "v50_s1"), ("无上限", "y20_nocap", "vel50sub_hz20"),
          ("子集关", "y20_off", "hz20only")]


def load(tag):
    path = os.path.join(SD, "attrib305_2026-10-28_%s.jsonl" % tag)
    if not os.path.exists(path):
        return None
    out = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                r = json.loads(line)
                out[int(r["seed"])] = r
    return out


def arr(d):
    return np.array([float(d[s]["pos_rms"]) for s in sorted(d)])


def main():
    fig, ax = plt.subplots(1, 3, figsize=(17.0, 5.4))
    labels, r51, r150, m51, m150 = [], [], [], [], []
    got = {}
    for cap, t51, t1 in LADDER:
        d51, d1 = load(t51), (load(t1) if t1 else None)
        if d51 is None:
            continue
        labels.append(cap)
        p51 = arr(d51)
        r51.append(int((p51 > FAIL_M).sum()))
        m51.append(float(p51.max()))
        got[t51] = d51
        if d1 is not None:
            p1 = arr(d1)
            r150.append(int((p1 > FAIL_M).sum()))
            m150.append(float(p1.max()))
            got[t1] = d1
        else:
            r150.append(None)
            m150.append(None)

    x = np.arange(len(labels))
    ax[0].bar(x - 0.2, r51, width=0.4, color="#1f77b4", label="seeds 51-100")
    r150p = [np.nan if v is None else v for v in r150]
    ax[0].bar(x + 0.2, r150p, width=0.4, color="#ff7f0e", label="seeds 1-50")
    for i, v in enumerate(r51):
        ax[0].text(i - 0.2, v + 0.15, str(v), ha="center", fontsize=8)
    for i, v in enumerate(r150):
        if v is not None:
            ax[0].text(i + 0.2, v + 0.15, str(int(v)), ha="center", fontsize=8)
    ax[0].set_xticks(x)
    ax[0].set_xticklabels(labels, rotation=40, ha="right")
    ax[0].set_xlabel("触发时位置残差上限 [m]")
    ax[0].set_ylabel(">25 m 的 seeds（共 50）")
    ax[0].set_title("收紧比放宽更危险：cap 越小发生率越高；放宽过头才再次恶化")
    ax[0].legend(fontsize=8)
    ax[0].grid(axis="y", alpha=0.3)

    ax[1].plot(x, m51, "o-", color="#1f77b4", label="max，seeds 51-100")
    ax[1].plot(x, m150, "s--", color="#ff7f0e", label="max，seeds 1-50")
    ax[1].axhline(FAIL_M, color="k", ls=":", lw=1)
    ax[1].text(2.0, FAIL_M * 1.06, "25 m 门", fontsize=8)
    ax[1].set_xticks(x)
    ax[1].set_xticklabels(labels, rotation=40, ha="right")
    ax[1].set_yscale("log")
    ax[1].set_ylabel("max pos_rms [m]（对数）")
    ax[1].set_title("右尾：平台档 max 平坦，放行 >41 m 的触发才出现 43-105 m 灾难")
    ax[1].legend(fontsize=8)
    ax[1].grid(alpha=0.3)

    for cap, t51, _t1 in [("12", "y20_c12", None), ("15", "y20_c15", None),
                          ("25", "y20_c25", None), ("40", "v40_s51", None),
                          ("50", "v50_s51", None), ("无上限", "y20_nocap", None)]:
        if t51 not in got:
            continue
        v = np.sort(arr(got[t51]))
        ax[2].plot(v, np.arange(1, len(v) + 1) / len(v), lw=1.8, label="cap %s" % cap)
    ax[2].axvline(FAIL_M, color="k", ls=":", lw=1)
    ax[2].set_xscale("log")
    ax[2].set_xlabel("pos_rms [m]（对数）")
    ax[2].set_ylabel("累积比例")
    ax[2].set_title("CDF（seeds 51-100）：平台档之间几乎重合，无上限档右尾分离")
    ax[2].legend(fontsize=8)
    ax[2].grid(alpha=0.3)

    fig.suptitle("§4.24s/§4.24t 速度子集触发上限：区间 [L, U) 的边界映射（30.5 g，20 Hz，同二进制）",
                 y=1.0)
    fig.tight_layout()
    p = os.path.join(FIG, "nav_velsubset_capband_2026-10-28.png")
    fig.savefig(p, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(json.dumps({"caps": labels, "rate_51_100": r51, "rate_1_50": r150,
                      "max_51_100": [round(v, 3) for v in m51],
                      "max_1_50": [None if v is None else round(v, 3) for v in m150]},
                     ensure_ascii=False, indent=1))
    print("written", p)


if __name__ == "__main__":
    main()
