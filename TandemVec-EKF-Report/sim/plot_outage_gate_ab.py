# -*- coding: utf-8 -*-
"""真实失联下的否决翻转与缺口门 A/B 可视化（§4.22 证据图）。

@file    plot_outage_gate_ab.py
@brief   把 outage_veto_ab 与 gap_gate_ab 两份 JSON 画成六面板多类型图
@module  TandemVec-EKF-Report/可视化（L2 研究设施）
@details 输入：sim-data/outage_veto_ab_2026-10-27.json（veto 开→关，20 seeds/档）、
         sim-data/gap_gate_ab_2026-10-27.json（缺口门 关/0.5/2.0 s）。
         输出：figs/nav_outage_gate_ab_2026-10-27.png。
         判读要点：①纯失联两档在取消否决后 20/20 一致变差（陈旧 τ̂ 应用），
         ②"失联+真延时"组合 10/10 大幅变好，③缺口门修好助推档却把
         "失联5s+延100ms"档推差 +37% ⇒ 门不是净收益。
"""
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager

ROOT = "D:/纵列双发矢量推力飞行器/TandemVec-EKF-Report"
OV = f"{ROOT}/sim-data/outage_veto_ab_2026-10-27.json"
GG = f"{ROOT}/sim-data/gap_gate_ab_2026-10-27.json"
OUT = f"{ROOT}/figs/nav_outage_gate_ab_2026-10-27.png"
ARM_ON = "veto开"
ARM_OFF = "veto关(默认)"


def setup_font():
    for name in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC"):
        try:
            font_manager.findfont(name, fallback_to_default=False)
            plt.rcParams["font.family"] = name
            break
        except Exception:
            continue
    plt.rcParams["axes.unicode_minus"] = False


def main():
    setup_font()
    ov = json.load(open(OV, encoding="utf-8"))["rows"]
    gg = json.load(open(GG, encoding="utf-8"))["rows"]
    names = list(ov.keys())
    fig, ax = plt.subplots(2, 3, figsize=(16.5, 9.2))

    # (a) 条形：veto 开→关 的 pos_rms 中位
    x = np.arange(len(names))
    on = [np.median([r["pos_rms"] for r in ov[n]["arms"][ARM_ON]]) for n in names]
    off = [np.median([r["pos_rms"] for r in ov[n]["arms"][ARM_OFF]])
           for n in names]
    a = ax[0, 0]
    a.bar(x - 0.2, on, width=0.38, label=ARM_ON, color="#4c72b0")
    a.bar(x + 0.2, off, width=0.38, label=ARM_OFF, color="#dd8452")
    for i, (u, v) in enumerate(zip(on, off)):
        a.annotate(f"{(v - u) / u * 100:+.0f}%", (i, max(u, v) * 1.02),
                   ha="center", fontsize=8,
                   color=("#c0392b" if v > u else "#1e8449"))
    a.set_xticks(x)
    a.set_xticklabels(names, rotation=18, ha="right", fontsize=8)
    a.set_ylabel("位置 RMSE 中位 [m]")
    a.set_yscale("log")
    a.set_title("(a) 取消 vib 否决：真实失联下并非无害")
    a.legend(fontsize=8)
    a.grid(alpha=0.3, axis="y")

    # (b) 逐 seed 配对（veto 开 vs 关）
    a = ax[0, 1]
    mk = ["o", "s", "^", "D", "v", "P", "X"]
    lim = max(max(on), max(off)) * 1.15
    for i, n in enumerate(names):
        u = [r["pos_rms"] for r in ov[n]["arms"][ARM_ON]]
        v = [r["pos_rms"] for r in ov[n]["arms"][ARM_OFF]]
        a.scatter(u, v, s=22, marker=mk[i % len(mk)], alpha=0.75, label=n)
    a.plot([0.1, lim], [0.1, lim], "k--", lw=0.9)
    a.set_xscale("log")
    a.set_yscale("log")
    a.set_xlabel(f"{ARM_ON} pos_rms [m]")
    a.set_ylabel(f"{ARM_OFF} pos_rms [m]")
    a.set_title("(b) 逐 seed 配对：纯失联全在对角线上方，组合档全在下方")
    a.legend(fontsize=7, loc="upper left")
    a.grid(alpha=0.3)

    # (c) ad_app：失联段反而更多应用（陈旧 valid 未被撤销的直接体现）
    a = ax[0, 2]
    app_on = [np.median([r["ad_app"] for r in ov[n]["arms"][ARM_ON]])
              for n in names]
    app_off = [np.median([r["ad_app"] for r in ov[n]["arms"][ARM_OFF]])
               for n in names]
    a.bar(x - 0.2, app_on, width=0.38, label=ARM_ON, color="#4c72b0")
    a.bar(x + 0.2, app_off, width=0.38, label=ARM_OFF, color="#dd8452")
    a.set_xticks(x)
    a.set_xticklabels(names, rotation=18, ha="right", fontsize=8)
    a.set_ylabel("ad_app 中位 [次]")
    a.set_title("(c) 应用次数：20 s 失联段从 2 涨到 139（陈旧 τ 估计仍在生效）")
    a.legend(fontsize=8)
    a.grid(alpha=0.3, axis="y")

    # (d) 缺口门三档条形
    gnames = list(gg.keys())
    garms = ["门关", "门0.5s", "门2.0s"]
    gx = np.arange(len(gnames))
    a = ax[1, 0]
    for j, arm in enumerate(garms):
        vals = [np.median([r["pos_rms"] for r in gg[n]["arms"][arm]])
                for n in gnames]
        a.bar(gx + (j - 1) * 0.25, vals, width=0.24, label=arm)
    a.set_xticks(gx)
    a.set_xticklabels(gnames, rotation=18, ha="right", fontsize=8)
    a.set_ylabel("位置 RMSE 中位 [m]")
    a.set_yscale("log")
    a.set_title("(d) 缺口门：修好助推档，却把「失联5s+延100ms」推差 +37%")
    a.legend(fontsize=8)
    a.grid(alpha=0.3, axis="y")

    # (e) 门的逐 seed 差值
    a = ax[1, 1]
    for i, n in enumerate(gnames):
        u = [r["pos_rms"] for r in gg[n]["arms"]["门关"]]
        v = [r["pos_rms"] for r in gg[n]["arms"]["门0.5s"]]
        k = min(len(u), len(v))
        d = [v[t] - u[t] for t in range(k)]
        a.scatter([i] * k, d, s=26, alpha=0.7,
                  color=["#c0392b" if np.median(d) > 0 else "#1e8449"][0])
    a.axhline(0, color="k", lw=0.9)
    a.set_xticks(range(len(gnames)))
    a.set_xticklabels(gnames, rotation=18, ha="right", fontsize=8)
    a.set_ylabel("Δ pos_rms（门0.5s − 门关）[m]")
    a.set_title("(e) 缺口门逐 seed 差值：符号按工况分裂，非一致改善")
    a.grid(alpha=0.3, axis="y")

    # (f) 热图：seed × 工况 的 veto 开→关 Δpos
    a = ax[1, 2]
    nseed = max(len(ov[n]["arms"][ARM_ON]) for n in names)
    mat = np.full((len(names), nseed), np.nan)
    for i, n in enumerate(names):
        u = ov[n]["arms"][ARM_ON]
        v = ov[n]["arms"][ARM_OFF]
        for t in range(min(len(u), len(v))):
            # 用相对变化而非绝对米：组合档的 −7 m 会把纯失联档的 +0.25 m 压成白色
            mat[i, t] = (v[t]["pos_rms"] - u[t]["pos_rms"]) /                         u[t]["pos_rms"] * 100.0
    im = a.imshow(mat, aspect="auto", cmap="RdBu_r", vmin=-25, vmax=25)
    a.set_xticks(range(nseed))
    a.set_xticklabels([str(t + 1) for t in range(nseed)], fontsize=6.5)
    a.set_yticks(range(len(names)))
    a.set_yticklabels(names, fontsize=8)
    a.set_xlabel("seed")
    a.set_ylabel("Δ pos_rms [%]")
    a.set_title("(f) Δpos% 热图（红=取消否决更差）：纯失联行整行同号")
    fig.colorbar(im, ax=a, fraction=0.046, pad=0.04, label="Δ pos_rms [%]")

    fig.suptitle("真实 GNSS 失联下的自动延时补偿：vib 否决翻转与拟合窗缺口门 "
                 "（导弹 6DOF SIL，core2，20 seeds/档，注入自证据 age 峰全部 [OK]）",
                 fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(OUT, dpi=115)
    print("saved:", OUT)


if __name__ == "__main__":
    main()
