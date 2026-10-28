# -*- coding: utf-8 -*-
"""GNSS 垂直 spoof 与气压重锚门控：时序 + 10-seed 汇总。
@file    plot_vspoof_gate.py
@brief   GNSS 垂直 spoof 门控可视化。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 绘制台阶/慢偏与门控前后结果。
@frame   N/A
@units   N/A
"""
import json, os, re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "sim-data")
FIG = os.path.join(HERE, "..", "figs")
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

LINE = re.compile(r"t=\s*([0-9.]+) alt=\s*([-0-9.]+) nav_alt=\s*([-0-9.]+)"
                  r".*?roll=\s*([-0-9.]+)")


def parse(path):
    t, alt, nal, roll = [], [], [], []
    for line in open(path, encoding="utf-8", errors="ignore"):
        m = LINE.search(line)
        if m:
            t.append(float(m.group(1)))
            alt.append(float(m.group(2)))
            nal.append(float(m.group(3)))
            roll.append(float(m.group(4)))
    return map(np.array, (t, alt, nal, roll))


t0, alt0, nal0, roll0 = parse(os.path.join(
    DATA, "trace-vspoof-z100-gate0-2026-10-26.log"))
t1, alt1, nal1, roll1 = parse(os.path.join(
    DATA, "trace-vspoof-z100-gate1-2026-10-26.log"))

g0 = json.load(open(os.path.join(DATA, "gnss_vspoof_2026-10-26_gate0.json"),
                    encoding="utf-8"))
g1 = json.load(open(os.path.join(DATA, "gnss_vspoof_2026-10-26_gate1.json"),
                    encoding="utf-8"))
reg = json.load(open(os.path.join(DATA, "gnss_vgate_reg_2026-10-26.json"),
                     encoding="utf-8"))

fig, ax = plt.subplots(2, 2, figsize=(12.8, 8.6), dpi=160,
                       constrained_layout=True)
col0, col1 = "#c0392b", "#1f6fb4"

a = ax[0, 0]
a.plot(t0, alt0, color=col0, lw=1.6, label="无门控（默认）")
a.plot(t1, alt1, color=col1, lw=1.6, label="气压重锚门控")
a.axvline(25, color="gray", ls="--", lw=1)
a.axhline(10, color="gray", ls=":", lw=1)
a.text(25.4, 11.0, "GNSS 高度 +100 m 欺骗", fontsize=7.5, color="gray")
a.set_xlabel("时间 [s]")
a.set_ylabel("真值高度 [m]")
a.set_ylim(-0.5, 12)
a.set_title("(a) 100 m 高度欺骗：默认降到地面，门控保持 10 m", fontsize=10)
a.grid(alpha=0.3)
a.legend(fontsize=8)

b = ax[0, 1]
b.plot(t0, nal0, color=col0, lw=1.6, label="无门控")
b.plot(t1, nal1, color=col1, lw=1.6, label="门控")
b.axvline(25, color="gray", ls="--", lw=1)
b.set_xlabel("时间 [s]")
b.set_ylabel("EKF 相对高度 [m]")
b.set_title("(b) EKF 高度：默认被拉到 ~60 m，门控保持在 ~10 m", fontsize=10)
b.grid(alpha=0.3)
b.legend(fontsize=8)

c = ax[1, 0]
arms = ["step_up_50", "step_up_100", "step_dn_50", "step_dn_100"]
x = np.arange(len(arms))
for i, (d, col, lab) in enumerate(((g0, col0, "无门控"), (g1, col1, "门控"))):
    vals = [d[a]["n_landed"] for a in arms]
    c.bar(x + (i - 0.5) * 0.34, vals, 0.32, color=col, label=lab)
    for xx, v in zip(x + (i - 0.5) * 0.34, vals):
        c.text(xx, v + 0.2, str(v), ha="center", fontsize=8)
c.set_xticks(x)
c.set_xticklabels(["+50 m", "+100 m", "−50 m", "−100 m"])
c.set_ylabel("落地例数 / 10")
c.set_ylim(0, 12)
c.set_title("(c) 高度台阶：默认 10/10 落地（上偏）", fontsize=10)
c.grid(axis="y", alpha=0.3)
c.legend(fontsize=8)

d = ax[1, 1]
names = ["nominal", "outage5", "outage20", "wind"]
labs = ["正常", "5 s 失联", "20 s 失联", "风"]
for i, (key, lab) in enumerate(zip(names, labs)):
    r0 = reg[f"g0_{key}"]
    r1 = reg[f"g1_{key}"]
    d.plot([i - 0.18, i + 0.18], [r0["hold_end_rms"]["med"],
                                  r1["hold_end_rms"]["med"]],
           "o-", color="#555", lw=1)
    d.text(i + 0.22, r0["hold_end_rms"]["med"],
           f'{r0["hold_end_rms"]["med"]:.2f}', fontsize=7, va="center")
d.axhline(1.5, color="gray", ls="--", lw=1)
d.set_xticks(range(len(names)))
d.set_xticklabels(labs)
d.set_ylabel("hold_end 中位 [m]")
d.set_title("(d) 无欺骗回归：门控 0/1 逐 seed 相同（气压健康）", fontsize=10)
d.grid(axis="y", alpha=0.3)

fig.suptitle("GNSS 垂直 spoof 与气压一致性重锚门控（SIL L2，mag_clean + flow，10 seeds）",
             fontsize=12)
out = os.path.join(FIG, "nav_vspoof_gate_2026-10-26.png")
fig.savefig(out, bbox_inches="tight")
print("wrote", out)
