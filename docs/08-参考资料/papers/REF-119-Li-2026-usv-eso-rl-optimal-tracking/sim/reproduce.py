# -*- coding: utf-8 -*-
"""
REF-119 (Li et al., JAI 2026) 复现仿真
论文: Adaptive optimal tracking control for underactuated surface vessels
      using extended state observer and reinforcement learning

复现范围:
  - 3-DOF USV 动力学 (论文 §5 参数, M=diag(25,24,3.7), 1:67 缩比船模)
  - 非线性 ESO (式 11-12, sfal 双模态, k=(5,8,10), h=0.5, q=0.001, gamma=0.005)
  - AOTC 控制器: 两通道四级反步 + DSC 一阶滤波 + actor-critic RBF 网络
    (β1=6, β2=1.2, β3=5.5, β4=1.2, c1=c2=0.1, γc=(1,2.7,3,2.7), γa=(8,10,12,10))
  - 场景 1 (圆轨迹) / 场景 2 (Lissajous 轨迹)
  - 全状态对照版 (RLOTC 精神复现: 同结构控制器 + 真实状态, 无 ESO)

重建约定 (原文排版损坏/歧义处的选择, 见 reproduction.md):
  - D 矩阵: D22 系数 4.1 (原文印 41, 疑漏小数点); D23=-0.1-2.5|v|+2.5|r| (pdfplumber 读法)
  - û 由 ESO 惯性系速度估计旋转到体轴: û = x̂2[0]cosψ + x̂2[1]sinψ (原文未说明)
  - z1 角度缠绕到 (-π,π]
  - 制导符号按原文印刷: xe = x - xd, ψd = atan2(ye, xe)
    (论文 Fig.3 实测稳态 u<0 佐证该读法: 船尾朝前追踪)

输出: figs/*.png + repro_summary.json
"""
import json
import os
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FIGS = os.path.join(HERE, "figs")
os.makedirs(FIGS, exist_ok=True)

# ---------------- 模型参数 (论文 §5) ----------------
M11, M22, M33 = 25.0, 24.0, 3.7
MINV = np.diag([1.0 / M11, 1.0 / M22, 1.0 / M33])
# D22 次线性阻尼系数: 原文印 "41|v|", 默认按 4.1 重建 (疑漏小数点);
# 环境变量 REF119_D22V=41 可切换为原印刷值做对照实验
D22_LIN = float(os.environ.get("REF119_D22V", "4.1"))


def plant_deriv(b, psi, tau):
    """M ḃ = -C(b)b - D(b)b + tau, g(a)=0 (仿真未给恢复力, 取 0)"""
    u, v, r = b
    tu, _, tr = tau
    Cb = np.array(
        [
            -(24.0 * v + 0.72 * r) * r,
            25.0 * u * r,
            (24.0 * v + 0.72 * r) * u - 25.0 * u * v,
        ]
    )
    D11 = 0.72 + 1.8 * abs(u) + 6.3 * u * u
    D22 = 0.86 + D22_LIN * abs(v) + 3.5 * abs(r)
    D23 = -0.1 - 2.5 * abs(v) + 2.5 * abs(r)
    D32 = -0.1 - 5.5 * abs(v) + 3.5 * abs(r)
    D33 = 6.0 + 4.5 * abs(v) + 4.5 * abs(r)
    Db = np.array([D11 * u, D22 * v + D23 * r, D32 * v + D33 * r])
    bd = MINV @ (-Cb - Db + tau)
    ad = np.array(
        [u * np.cos(psi) - v * np.sin(psi), u * np.sin(psi) + v * np.cos(psi), r]
    )
    return ad, bd


def theta_true(b, psi):
    """真实集总项 Θ = ä - Bτ = J̇b + J M⁻¹(-Cb-Db+g) (与 τ 无关), 全状态对照版用"""
    u, v, r = b
    _, bd0 = plant_deriv(b, psi, np.zeros(3))
    Jdotb = r * np.array(
        [-u * np.sin(psi) - v * np.cos(psi), u * np.cos(psi) - v * np.sin(psi), 0.0]
    )
    J = np.array([[np.cos(psi), -np.sin(psi), 0],
                  [np.sin(psi), np.cos(psi), 0],
                  [0, 0, 1.0]])
    return Jdotb + J @ bd0


# ---------------- ESO (式 11-12) ----------------
K1, K2, K3 = 5.0, 8.0, 10.0
SFAL_H, SFAL_Q, SFAL_G = 0.5, 0.001, 0.005


def sfal(e):
    a = abs(e)
    if a <= SFAL_Q:
        return e / SFAL_Q ** (1.0 - SFAL_H)
    return a * SFAL_Q ** (SFAL_H - 1.0 + SFAL_G * (a - SFAL_Q)) * np.sign(e)


# ---------------- RBF 网络 ----------------
CENTERS = np.linspace(-6.0, 6.0, 30)
SIGMA = 2.0


def rbf(z):
    return np.exp(-((z - CENTERS) ** 2) / (2.0 * SIGMA ** 2))


class NNPair:
    """critic: ω̇̂c = -γc B ω̂c ; actor: ω̇̂a = -B[γa(ω̂a-ω̂c)+γc ω̂c]"""

    def __init__(self, gc, ga, d):
        self.wc = 0.2 * np.ones(30)
        self.wa = 0.1 * np.ones(30)
        self.gc, self.ga, self.d = gc, ga, d

    def update(self, z, dt):
        th = rbf(z)
        Bm = np.outer(th, th) + self.d * np.eye(30)
        self.wc -= self.gc * (Bm @ self.wc) * dt
        self.wa -= (Bm @ (self.ga * (self.wa - self.wc) + self.gc * self.wc)) * dt
        return th


def wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


# ---------------- 仿真主函数 ----------------
def run(scenario, full_state=False, T=200.0, dt=1e-3):
    """scenario: 1=圆, 2=Lissajous; full_state: True=全状态对照(RLOTC 精神)"""
    w = 0.01 * np.pi

    def ref(t):
        if scenario == 1:
            return np.array([30 * np.sin(w * t), 30 * np.sin(w * t - np.pi / 2)])
        return np.array(
            [30 * np.sin(w * t), 30 * np.sin(w * t) * np.cos(w * t)]
        )

    # 控制器参数
    B1, B2, B3, B4 = 6.0, 1.2, 5.5, 1.2
    C1, C2 = 0.1, 0.1
    nn1 = NNPair(1.0, 8.0, 0.01)
    nn2 = NNPair(2.7, 10.0, 0.01)
    nn3 = NNPair(3.0, 12.0, 0.01)
    nn4 = NNPair(2.7, 10.0, 0.01)
    afr, afu = 0.0, 0.0  # 滤波初值 = α̂(0) 由首步同步

    # 状态初值
    a = np.zeros(3)  # [x, y, ψ]
    b = np.zeros(3)  # [u, v, r]
    xh1, xh2, xh3 = a.copy(), np.zeros(3), np.zeros(3)  # ESO
    psi = 0.0

    n = int(T / dt)
    keep = max(1, int(0.01 / dt))  # 100Hz 记录
    log = {k: [] for k in
           ["t", "x", "y", "psi", "u", "v", "r", "u_hat", "v_hat", "r_hat",
            "xh1", "xh2", "xh3", "e1", "z1", "z3", "tau_u", "tau_r",
            "nnorm", "xd", "yd", "cost"]}
    cost = 0.0
    t = 0.0
    t0 = time.time()
    for k in range(n):
        x, y = a[0], a[1]
        rd = ref(t)
        xe, ye = x - rd[0], y - rd[1]
        z3 = np.hypot(xe, ye)
        psig = np.arctan2(ye, xe) if z3 > 1e-9 else psi
        z1 = wrap(psi - psig)
        cpe = np.cos(z1)

        # --- ESO/全状态 状态源 ---
        if full_state:
            r_hat, u_hat = b[2], b[0]
            tau_probe = np.zeros(3)  # Θ 真值在控制计算前用 τ=0 先取不了, 下面回填
        else:
            u_hat = xh2[0] * np.cos(psi) + xh2[1] * np.sin(psi)
            v_hat = -xh2[0] * np.sin(psi) + xh2[1] * np.cos(psi)
            r_hat = xh2[2]

        # --- yaw 通道 Step1 ---
        th1 = rbf(z1)
        alpha_r = -B1 * z1 - 0.5 * nn1.wa @ th1
        nn1.update(z1, dt)
        afr = alpha_r if k == 0 else afr + dt / C1 * (alpha_r - afr)
        zh2 = r_hat - afr

        # --- surge 通道 Step1 ---
        th3 = rbf(z3)
        alpha_u = (-B3 * z3 - 0.5 * nn3.wa @ th3) * cpe
        nn3.update(z3, dt)
        afu = alpha_u if k == 0 else afu + dt / C2 * (alpha_u - afu)
        zh4 = u_hat - afu

        # --- 集总项 (ESO 估计或真值) ---
        if full_state:
            th_true = theta_true(b, psi)
            lump3, lump1, lump2 = th_true[2], th_true[0], th_true[1]
        else:
            lump3, lump1, lump2 = xh3[2], xh3[0], xh3[1]

        # --- yaw Step2 / surge Step2 (式 46 / 78) ---
        th2 = rbf(zh2)
        tau_rm = -B2 * zh2 - lump3 - 0.5 * nn2.wa @ th2
        nn2.update(zh2, dt)
        tau_r = M33 * tau_rm

        th4 = rbf(zh4)
        tau_um = -B4 * zh4 - lump1 * np.cos(psi) - lump2 * np.sin(psi) - 0.5 * nn4.wa @ th4
        nn4.update(zh4, dt)
        tau_u = M11 * tau_um

        tau = np.array([tau_u, 0.0, tau_r])

        cost += (z1 ** 2 + alpha_r ** 2 + zh2 ** 2 + tau_rm ** 2
                 + z3 ** 2 + alpha_u ** 2 + zh4 ** 2 + tau_um ** 2) * dt

        # --- 记录 ---
        if k % keep == 0:
            log["t"].append(t)
            log["x"].append(x)
            log["y"].append(y)
            log["psi"].append(psi)
            log["u"].append(b[0])
            log["v"].append(b[1])
            log["r"].append(b[2])
            log["u_hat"].append(u_hat)
            log["v_hat"].append(v_hat if not full_state else b[1])
            log["r_hat"].append(r_hat)
            log["xh1"].append(xh1.copy())
            log["xh2"].append(xh2.copy())
            log["xh3"].append(xh3.copy())
            log["e1"].append(a - xh1)
            log["z1"].append(z1)
            log["z3"].append(z3)
            log["tau_u"].append(tau_u)
            log["tau_r"].append(tau_r)
            log["nnorm"].append([np.linalg.norm(p.wc) for p in (nn1, nn2, nn3, nn4)]
                                + [np.linalg.norm(p.wa) for p in (nn1, nn2, nn3, nn4)])
            log["xd"].append(rd[0])
            log["yd"].append(rd[1])
            log["cost"].append(cost)

        # --- ESO 积分 (式 11) ---
        e1 = a - xh1
        sf = np.array([sfal(e) for e in e1])
        Btau = np.array([[np.cos(psi), -np.sin(psi), 0],
                         [np.sin(psi), np.cos(psi), 0],
                         [0, 0, 1.0]]) @ MINV @ tau
        xh1 += (xh2 + K1 * e1) * dt
        xh2 += (xh3 + K2 * sf + Btau) * dt
        xh3 += (K3 * sf) * dt

        # --- 被控对象积分 ---
        ad, bd = plant_deriv(b, psi, tau)
        a = a + ad * dt
        b = b + bd * dt
        psi = a[2]
        t += dt

        if not np.isfinite(b).all() or not np.isfinite(a).all():
            print(f"  !! 发散于 t={t:.2f}s")
            break

    out = {k: np.array(v) for k, v in log.items()}
    out["elapsed"] = time.time() - t0
    return out


# ---------------- 统计与绘图 ----------------
def summarize(out, label):
    t = out["t"]
    m = t > 50  # 稳态段
    stats = {
        "run": label,
        "T_end": float(t[-1]),
        "z3_init": float(out["z3"][0]),
        "z3_ss_mean": float(out["z3"][m].mean()),
        "z3_ss_max": float(out["z3"][m].max()),
        "z1_ss_rms": float(np.sqrt((out["z1"][m] ** 2).mean())),
        "u_ss_mean": float(out["u"][m].mean()),
        "u_min": float(out["u"].min()),
        "tau_u_min": float(out["tau_u"].min()),
        "tau_u_min_t": float(t[np.argmin(out["tau_u"])]),
        "tau_r_max": float(np.abs(out["tau_r"]).max()),
        "e1_ss_rms": float(np.sqrt((out["e1"][m] ** 2).mean(0).sum())),
        "nnorm_end": [round(float(x), 4) for x in out["nnorm"][-1]],
        "cost_end": float(out["cost"][-1]),
    }
    return stats


def plots(scn, prop, full):
    tag = f"s{scn}"
    t = prop["t"]

    # ESO 速度估计 (对应论文 Fig.3)
    fig, axs = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    for ax, key, hat, name in [
        (axs[0], "u", "u_hat", "u [m/s]"),
        (axs[1], "v", "v_hat", "v [m/s]"),
        (axs[2], "r", "r_hat", "r [rad/s]"),
    ]:
        ax.plot(t, prop[key], "b-", label="actual")
        ax.plot(t, prop[hat], "r--", label="ESO estimate")
        ax.set_ylabel(name)
        ax.legend()
        ax.grid(alpha=0.3)
    axs[0].set_title(f"Scenario {scn}: ESO velocity estimation (cf. paper Fig.3)")
    axs[2].set_xlabel("Time (s)")
    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, f"{tag}_eso_vel.png"), dpi=110)
    plt.close(fig)

    # ESO 位置/航向估计 (Fig.2)
    actual = np.column_stack([prop["x"], prop["y"], wrap_arr(prop["psi"])])
    fig, axs = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    for ax, i, name in [(axs[0], 0, "x [m]"), (axs[1], 1, "y [m]"), (axs[2], 2, "psi [rad]")]:
        ax.plot(t, actual[:, i], "b-", label="actual")
        ax.plot(t, prop["xh1"][:, i], "r--", label="ESO estimate")
        ax.set_ylabel(name)
        ax.legend()
        ax.grid(alpha=0.3)
    axs[0].set_title(f"Scenario {scn}: ESO position/heading estimation (cf. paper Fig.2)")
    axs[2].set_xlabel("Time (s)")
    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, f"{tag}_eso_pos.png"), dpi=110)
    plt.close(fig)

    # 轨迹 (Fig.4)
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(prop["xd"], prop["yd"], "k--", lw=1, label="reference")
    ax.plot(prop["x"], prop["y"], "b-", lw=1, label="proposed (ESO)")
    ax.plot(full["x"], full["y"], "g-", lw=0.8, alpha=0.7, label="full-state")
    ax.plot(prop["x"][0], prop["y"][0], "ko", ms=5)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"Scenario {scn}: trajectory (cf. paper Fig.4)")
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, f"{tag}_traj.png"), dpi=110)
    plt.close(fig)

    # 误差 z1/z3 (Fig.5)
    fig, axs = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    axs[0].plot(t, prop["z1"], "b-", label="proposed")
    axs[0].plot(full["t"], full["z1"], "g-", alpha=0.7, label="full-state")
    axs[0].set_ylabel("z1 [rad]")
    axs[1].plot(t, prop["z3"], "b-", label="proposed")
    axs[1].plot(full["t"], full["z3"], "g-", alpha=0.7, label="full-state")
    axs[1].set_ylabel("z3 [m]")
    axs[1].set_xlabel("Time (s)")
    for ax in axs:
        ax.legend()
        ax.grid(alpha=0.3)
    axs[0].set_title(f"Scenario {scn}: tracking errors (cf. paper Fig.5)")
    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, f"{tag}_err.png"), dpi=110)
    plt.close(fig)

    # 控制量 (Fig.6)
    fig, axs = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    axs[0].plot(t, prop["tau_u"], "b-", label="proposed")
    axs[0].plot(full["t"], full["tau_u"], "g-", alpha=0.7, label="full-state")
    axs[0].set_ylabel("tau_u [N]")
    axs[1].plot(t, prop["tau_r"], "b-", label="proposed")
    axs[1].plot(full["t"], full["tau_r"], "g-", alpha=0.7, label="full-state")
    axs[1].set_ylabel("tau_r [N·m]")
    axs[1].set_xlabel("Time (s)")
    for ax in axs:
        ax.legend()
        ax.grid(alpha=0.3)
    axs[0].set_title(f"Scenario {scn}: control inputs (cf. paper Fig.6)")
    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, f"{tag}_ctrl.png"), dpi=110)
    plt.close(fig)

    # 代价 (Fig.7) 与 NN 范数 (Fig.8)
    fig, axs = plt.subplots(1, 2, figsize=(12, 5))
    axs[0].plot(t, prop["cost"], "b-", label="proposed")
    axs[0].plot(full["t"], full["cost"], "g-", alpha=0.7, label="full-state")
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel("cumulative cost")
    axs[0].set_title(f"Scenario {scn}: cost (cf. paper Fig.7)")
    axs[0].legend()
    axs[0].grid(alpha=0.3)
    nn = prop["nnorm"]
    for i in range(4):
        axs[1].plot(t, nn[:, i], "-", label=f"|wc{i+1}|")
        axs[1].plot(t, nn[:, 4 + i], "--", label=f"|wa{i+1}|")
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel("NN weight norm")
    axs[1].set_title(f"Scenario {scn}: AC weight norms (cf. paper Fig.8)")
    axs[1].legend(fontsize=7, ncol=2)
    axs[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, f"{tag}_cost_nn.png"), dpi=110)
    plt.close(fig)


def wrap_arr(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


def main():
    results = {}
    for scn in (1, 2):
        print(f"[Scenario {scn}] proposed (ESO) running ...", flush=True)
        prop = run(scn, full_state=False)
        print(f"  done in {prop['elapsed']:.1f}s", flush=True)
        print(f"[Scenario {scn}] full-state comparison running ...", flush=True)
        full = run(scn, full_state=True)
        print(f"  done in {full['elapsed']:.1f}s", flush=True)
        plots(scn, prop, full)
        results[f"s{scn}_proposed"] = summarize(prop, f"S{scn}-ESO")
        results[f"s{scn}_fullstate"] = summarize(full, f"S{scn}-full")
    with open(os.path.join(HERE, "repro_summary.json"), "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, indent=2)
    print(json.dumps(results, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
