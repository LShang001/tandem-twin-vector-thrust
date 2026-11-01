#!/usr/bin/env python3
"""CMA-ES 优化结果可视化：收敛曲线 + 最优参数对照 + 参数敏感度扫描。

输入：tmp/opt_cma_log.csv（eval,gen,best,mean）
      tmp/opt_cma_best.json（cost/params）
输出：tmp/opt_convergence.png、tmp/opt_params.png
"""
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
LOG = os.path.join(REPO, "tmp", "opt_cma_log.csv")
BEST = os.path.join(REPO, "tmp", "opt_cma_best.json")
OUT1 = os.path.join(REPO, "tmp", "opt_convergence.png")
OUT2 = os.path.join(REPO, "tmp", "opt_params.png")

if not os.path.exists(LOG):
    raise SystemExit(f"缺少 {LOG}，先运行 opt_nav_params.py")

data = np.loadtxt(LOG, delimiter=",", skiprows=1)
ev, gen, best, mean = data[:, 0], data[:, 1], data[:, 2], data[:, 3]

fig, ax = plt.subplots(figsize=(10, 5), constrained_layout=True)
ax.plot(ev, best, "r-", lw=1.5, label="每代最优")
ax.plot(ev, mean, "b--", lw=1.0, alpha=0.7, label="每代均值")
ax.set_xlabel("评估次数")
ax.set_ylabel("cost（越小越好）")
ax.set_title("CMA-ES 8 维参数优化收敛")
ax.set_yscale("log")
ax.grid(True, alpha=0.3)
ax.legend()
fig.savefig(OUT1, dpi=150)
print(f"图已保存：{OUT1}")
print(f"最优 cost {best.min():.4f}，末代 {best[-1]:.4f}")

if os.path.exists(BEST):
    b = json.load(open(BEST, encoding="utf-8"))
    p = b["params"]
    defaults = {
        "IMUPERF_LPF_GYRO": 200.0, "IMUPERF_LPF_ACCEL": 100.0,
        "IMUPERF_HACC": 1e9, "IMUPERF_Q_ACCEL": 2.0,
        "IMUPERF_EKF_ACCEL_STD": 0.25, "IMUPERF_EKF_GYRO_STD": 0.0015,
        "IMUPERF_EKF_ACCEL_MARKOV": 0.010, "IMUPERF_EKF_GYRO_MARKOV": 0.00524,
    }
    keys = list(p.keys())
    fig, ax = plt.subplots(figsize=(11, 5), constrained_layout=True)
    x = np.arange(len(keys))
    w = 0.38
    ax.bar(x - w / 2, [defaults.get(k, 0) for k in keys], w, label="默认值")
    ax.bar(x + w / 2, [p.get(k, 0) for k in keys], w, label="优化值")
    ax.set_xticks(x)
    ax.set_xticklabels([k.replace("IMUPERF_", "") for k in keys], rotation=30,
                       ha="right", fontsize=9)
    ax.set_ylabel("数值（原始量纲）")
    ax.set_title(f"最优参数对照（cost {b['cost']:.4f}）")
    ax.legend()
    ax.grid(True, axis="y", alpha=0.3)
    fig.savefig(OUT2, dpi=150)
    print(f"图已保存：{OUT2}")
    print(json.dumps(p, ensure_ascii=False, indent=2))
