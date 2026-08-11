#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cascade_closedform.py — 级联架构标称闭环符号推导与不确定建模
================================================================
对应论文 TandemVec-Paper/ch/15-cascade-control.tex §15.4–15.6

目的：
  1. 单通道（B_true 对角化解耦后）2 状态线性化模型的符号 + 数值闭式
  2. 把 B_true 模型误差 + 惯量不确定集总成范数有界不确定 A(δ)=A0+BΔ·δ·CΔ
  3. 标称特征值 / 判别式自检（应过阻尼稳定）

符号约定（与仿真 control.mjs + 论文 ch/15 对齐）：
  状态 x = [e_R, ω]ᵀ
    e_R = 短弧旋转向量误差（小角度 ≈ θ_err）
    ω = 角速度（rad/s）

  外环（当前固件几何误差约定）：ω_des = vtolAttKp·e_R
  内环（control.mjs:142-147，理想 B·B⁻¹=I 精确对角化，论文 eq:cascade_inner_dot）：
    ė_R = -ω，ω̇ = btrueK·(ω_des - ω)

  论文符号对应（ch/15 eq:cascade_inner_tf / eq:cascade_outer_tf）：
    K_p（论文，ω_ref = K_p·e_R）= vtolAttKp
    K_ω/I（论文）= btrueK   ⟹   K_ω = I·btrueK

参数来源：models/aircraft-model.json → parameters.mjs（sync-params.py 生成）
  vtolAttKp = 2.5, btrueK = 60, Iy = Iz = 0.022, Ix = 0.0021
"""
import numpy as np
import sympy as sp

# ============================================================
#  仿真侧现行增益（parameters.mjs，单一事实源 aircraft-model.json）
# ============================================================
VTOL_ATT_KP = 2.5     # 外环悬停姿态增益 [1/s]
BTRUE_K = 60.0        # 内环角加速度增益 = K_ω/I [1/s]
I_YZ = 0.022          # 俯仰(Iy)/偏航(Iz)惯量 [kg·m²]
I_X = 0.0021          # 滚转惯量 [kg·m²]

# 论文符号等效值
K_P_PAPER = VTOL_ATT_KP              # = 2.5，当前 e_R 坐标下
K_OMEGA_OVER_I = BTRUE_K             # = 60


# ============================================================
#  标称闭环 A0
# ============================================================
def nominal_A(vtol_att_kp: float = VTOL_ATT_KP,
              btrue_k: float = BTRUE_K) -> np.ndarray:
    """标称闭环 A0（单通道，状态 [e_R, ω]），用仿真参数。

    ė_R = -ω
    ω̇ = btrue_k·(vtol_att_kp·e_R - ω)
    """
    return np.array([
        [0.0,                          -1.0],
        [btrue_k * vtol_att_kp,        -btrue_k],
    ], dtype=float)


def symbolic_A():
    """符号 A0，供论文推导核对（用论文符号 K_p, K_ω/I）。

    ė_R = -ω
    ω̇ = (K_ω/I)·(K_p·e_R - ω)

    返回 (A_sym, Kp, Kw)，Kp=K_p(论文)=vtolAttKp, Kw=K_ω/I=btrueK。
    """
    Kp, Kw = sp.symbols('K_p K_omega_over_I', positive=True)
    A = sp.Matrix([
        [0,        -1],
        [Kw * Kp, -Kw],
    ])
    return A, Kp, Kw


def hermes_A(kp_r: float, kp_w: float) -> np.ndarray:
    """Hermes 重构后的 A0（论文 ch/15 §15.6.2 当前符号，旋转向量 e_R 约定）。

    A0 = [[0, -1], [k_p,ω·k_p,R, -k_p,ω]]
      ė_R = -ω（外环：ω_ref = k_p,R·e_R → ė_R = ω_ref - ω = k_p,R·e_R - ω，
            注意 Hermes 论文行文取 ė_R ≈ -ω 的符号约定，矩阵第 2 行符号相应）
      ω̇ = k_p,ω·(ω_ref - ω) = k_p,ω·(k_p,R·e_R - ω)

    nominal_A() 与本函数使用同一 e_R 坐标；两者特征多项式完全相同。
    """
    return np.array([
        [0.0,    -1.0],
        [kp_w * kp_r, -kp_w],
    ])


def symbolic_charpoly():
    """符号特征多项式 det(λI - A0) = λ² + Kw·λ + Kw·Kp。"""
    A, Kp, Kw = symbolic_A()
    lam = sp.Symbol('lambda')
    p = sp.det(lam * sp.eye(2) - A).expand()
    return p, Kp, Kw


# ============================================================
#  范数有界不确定分解 A(δ) = A0 + BΔ·δ·CΔ
# ============================================================
def uncertainty_decomposition(vtol_att_kp: float = VTOL_ATT_KP,
                              btrue_k: float = BTRUE_K):
    """集总范数有界不确定分解。

    物理来源（一阶展开，δ 集总）：
      δ ≈ δ_B - δ_I
        δ_B：B_true 模型误差（k_T/k_Q 标定误差，论文 ch/15:485 定性 ±10% ⟹ γ_B≈0.1）
        δ_I：惯量不确定（实测 b=0.35/0.36/0.90 不可反推；量级估计 ±20–30%）
      合成上界 γ = γ_B + γ_I

    影响：内环有效增益 btrueK·(1+δ) ⟹ A 第 2 行整体乘 (1+δ)。
          即 (2,1) 与 (2,2) 元素同时受 δ 调制。

    返回 (A0, BΔ[2×1], CΔ[1×2])，满足 A(δ)=A0 + BΔ·δ·CΔ。
    """
    A0 = nominal_A(vtol_att_kp, btrue_k)
    B_Delta = np.array([[0.0], [1.0]])                              # 2×1
    C_Delta = np.array([[btrue_k * vtol_att_kp, -btrue_k]])       # 1×2
    return A0, B_Delta, C_Delta


def reconstruct(A0, B_D, C_D, delta: float) -> np.ndarray:
    """从分解重建 A(δ)，供蒙特卡洛与 LMI 交叉验证用。"""
    return A0 + B_D @ np.array([[delta]]) @ C_D


def direct_A_perturbed(vtol_att_kp: float, btrue_k: float, delta: float) -> np.ndarray:
    """直接构造含 δ 的 A（独立路径，用于交叉校验 reconstruct）。"""
    eff = btrue_k * (1.0 + delta)
    return np.array([
        [0.0, -1.0],
        [eff * vtol_att_kp, -eff],
    ])


# ============================================================
#  辅助分析量
# ============================================================
def time_scale_separation(vtol_att_kp: float = VTOL_ATT_KP,
                          btrue_k: float = BTRUE_K) -> float:
    """时间尺度分离比 ω_BW,i/ω_BW,o = btrueK/vtolAttKp（论文 eq:tss_condition_ch）。

    注：外环带宽 ω_BW,o ≈ K_p = vtolAttKp；内环带宽 ω_BW,i = K_ω/I = btrueK。
    """
    return btrue_k / vtol_att_kp


def over_damped_condition(vtol_att_kp: float = VTOL_ATT_KP,
                          btrue_k: float = BTRUE_K) -> bool:
    """过阻尼判据：判别式 = Kw² - 4·Kw·Kp = btrueK·(btrueK - 4·vtolAttKp) > 0。"""
    return btrue_k * (btrue_k - 4.0 * vtol_att_kp) > 0.0


# ============================================================
#  自检 main
# ============================================================
def _main():
    print("=" * 68)
    print(" 级联架构标称闭环 — 符号 + 数值推导自检")
    print("=" * 68)

    # ---- 符号推导 ----
    A_sym, Kp_s, Kw_s = symbolic_A()
    charpoly, _, _ = symbolic_charpoly()
    print("\n[1] 符号 A0（论文符号 K_p, K_ω/I）：")
    sp.pprint(A_sym)
    print("\n    特征多项式 det(λI - A0) =")
    sp.pprint(charpoly)
    print("    ⟹  λ² + (K_ω/I)·λ + (K_ω/I)·K_p = 0")
    disc_sym = sp.discriminant(charpoly, sp.Symbol('lambda'))
    print("\n    判别式（符号）=")
    sp.pprint(sp.factor(disc_sym))
    print("    过阻尼条件：K_ω/I > 4·K_p  ⟺  btrueK > 4·vtolAttKp")

    # ---- 数值（y/z 通道）----
    print("\n[2] 数值实例化（Iy=Iz 通道）:")
    print(f"    vtolAttKp = {VTOL_ATT_KP}, btrueK = {BTRUE_K}, Iy = Iz = {I_YZ}")
    print(f"    论文 K_p      = vtolAttKp     = {K_P_PAPER}")
    print(f"    论文 K_ω/I    = btrueK        = {K_OMEGA_OVER_I}")
    A0 = nominal_A()
    print(f"\n    A0 =\n{A0}")

    eigs = np.sort(np.linalg.eigvals(A0).real)
    print(f"\n    特征值 λ₁ = {eigs[0]:.4f}, λ₂ = {eigs[1]:.4f}")
    stable = (eigs[0] < 0) and (eigs[1] < 0)
    print(f"    ⟹ {'两个负实根，渐近稳定 ✓' if stable else '存在非负实部特征值 ✗'}")
    print(f"    慢模态 τ_slow  = {-1.0 / eigs[1]:.4f} s（外环主导）")
    print(f"    快模态 τ_fast = {-1.0 / eigs[0]:.4f} s（内环）")

    # ---- 符号 vs 数值一致性自检 ----
    charpoly_num = charpoly.subs({Kp_s: K_P_PAPER, Kw_s: K_OMEGA_OVER_I})
    coeffs_sym = [float(charpoly_num.coeff(sp.Symbol('lambda'), i)) for i in range(3)]
    A0_num = np.array(A0, dtype=float)
    coeffs_num = np.round(np.poly(A0_num), 9)  # [λ²系数, λ系数, 常数]
    consistent = np.allclose(coeffs_sym[::-1], coeffs_num, atol=1e-9)
    print(f"\n[3] 符号 vs 数值特征多项式一致性:")
    print(f"    符号代值系数 [常数, λ, λ²] = {coeffs_sym}")
    print(f"    numpy.poly 系数 [λ², λ, 常数] = {coeffs_num.tolist()}")
    print(f"    {'一致 ✓' if consistent else '不一致 ✗——检查 symbolic_A'}")

    # ---- 过阻尼 / 分离比 ----
    ratio = time_scale_separation()
    overdamped = over_damped_condition()
    disc_val = BTRUE_K ** 2 - 4.0 * BTRUE_K * K_P_PAPER
    print(f"\n[4] 动力学性质:")
    print(f"    判别式 = {disc_val:.2f}  ⟹  {'过阻尼（无振荡）' if overdamped else '欠阻尼（有振荡）'}")
    print(f"    时间尺度分离比 btrueK/vtolAttKp = {ratio:.1f}")
    print(f"    论文 eq:tss_condition_ch 要求 ≥ 3–5  ⟹  {'满足 ✓' if ratio >= 3 else '不满足 ✗'}")

    # ---- 不确定分解 ----
    A0d, B_D, C_D = uncertainty_decomposition()
    print(f"\n[5] 范数有界不确定分解 A(δ) = A0 + BΔ·δ·CΔ:")
    print(f"    BΔ = {B_D.ravel()}")
    print(f"    CΔ = {C_D.ravel()}")

    # 交叉校验：reconstruct vs direct_A_perturbed
    ok_all = True
    print(f"\n[6] reconstruct vs direct 交叉校验（多 δ）:")
    for d in (-0.3, -0.1, 0.0, 0.1, 0.3, 0.5):
        a_rec = reconstruct(A0d, B_D, C_D, d)
        a_dir = direct_A_perturbed(VTOL_ATT_KP, BTRUE_K, d)
        err = float(np.abs(a_rec - a_dir).max())
        ok = err < 1e-12
        ok_all &= ok
        print(f"    δ={d:+.2f}  max|err|={err:.2e}  {'✓' if ok else '✗'}")
    print(f"    ⟹ {'全通过 ✓' if ok_all else '存在不一致 ✗'}")

    # ---- δ 扰动下特征值变化（直觉）----
    print(f"\n[7] δ 扰动下特征值（鲁棒直觉）:")
    for d in (-0.5, -0.2, 0.0, 0.2, 0.5):
        a = reconstruct(A0d, B_D, C_D, d)
        ev = np.sort(np.linalg.eigvals(a).real)
        still_stable = ev[1] < 0
        note = "稳定" if still_stable else "失稳"
        print(f"    δ={d:+.2f}  λ={ev}  {note}")

    print("\n" + "=" * 68)
    print(" 自检完成。结果供 robust_lmi_analysis.py / robust_montecarlo.py 复用。")
    print("=" * 68)
    return consistent and ok_all and stable


if __name__ == "__main__":
    import sys
    ok = _main()
    sys.exit(0 if ok else 1)
