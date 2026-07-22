# -*- coding: utf-8 -*-
r"""
纵列双发矢量推力飞行器 — 高精度多维仿真分析
===========================================
运行: py -3.12 simulations/high-fidelity-analysis/run_all.py

产出:
  1. 配平分析 (24 m/s 纵向平衡点)
  2. 数值线性化 + 特征值/模态分析
  3. SAS 闭环阶跃响应
  4. INDI vs SAS 对比 (扰动抑制)
  5. LaTeX 表格数据 → docs/03-理论推导/THY-004/sim-data/
"""

import sys, os, json
from pathlib import Path
import numpy as np

# 添加路径
sys.path.insert(0, str(Path(__file__).parent))
from core import load_params, aero_forces, quat_rotate, quat_conj, euler_from_quat
from trim_analysis import trim_longitudinal, linearize_at_trim, analyze_modes
from simulate import SASController, INDIController, compare_controllers, simulate
from export_latex import modes_table, trim_table, step_response_metrics

OUT = Path(__file__).parent.parent.parent / "docs" / "03-理论推导" / "THY-004" / "sim-data"
OUT.mkdir(parents=True, exist_ok=True)

def main():
    P = load_params()
    print("=" * 60)
    print(" 纵列双发矢量推力飞行器 — 高精度多维仿真分析")
    print("=" * 60)
    print(f"  MODEL-DEFAULT 参数: kT={P['kT']:.1e}, kQ={P['kQ']:.1e}, "
          f"m={P['m']}kg, Iy={P['Iy']}kg·m²")
    print()

    # ========== 1. 配平 ==========
    print("[1/5] 纵向配平分析 (V=24 m/s)")
    V_target = 24.0
    trim = trim_longitudinal(V_target, P, verbose=True)
    # 写配平表
    (OUT / "trim_table.tex").write_text(trim_table(trim, P), encoding="utf-8")
    print(f"  → {OUT / 'trim_table.tex'}")

    # ========== 2. 线性化 + 特征值 ==========
    print("[2/5] 数值线性化 + 模态分析")
    A, A_long, A_lat = linearize_at_trim(trim, P)
    modes = analyze_modes(A_long, A_lat, P)
    print(f"  纵向特征值: {np.linalg.eigvals(A_long)}")
    print(f"  横航向特征值: {np.linalg.eigvals(A_lat)}")
    (OUT / "modes_table.tex").write_text(modes_table(modes, trim, P), encoding="utf-8")
    (OUT / "eigenvalues.json").write_text(json.dumps({
        "longitudinal": [complex(e.real, e.imag) for e in np.linalg.eigvals(A_long)],
        "lateral": [complex(e.real, e.imag) for e in np.linalg.eigvals(A_lat)],
    }, indent=2, default=lambda o: str(o)), encoding="utf-8")
    print(f"  → {OUT / 'modes_table.tex'}")

    # ========== 3. SAS 阶跃响应 ==========
    print("[3/5] SAS 俯仰阶跃响应")
    sas = SASController(P, trim)
    data = simulate(sas, P, trim, T_total=15, disturbance=(3, 3.1, "pitch", 5.0))
    metrics = step_response_metrics(data, 3.0)
    print(f"  峰值: {np.degrees(metrics['peak']):.2f}°, 稳态: {np.degrees(metrics['steady']):.2f}°, "
          f"调节时间: {metrics['settling_s']:.2f}s, 超调: {metrics['overshoot_pct']:.1f}%")
    # 10s 配平稳定性（全新控制器实例, 避免阶跃仿真的积分器残留污染）
    trim_data = simulate(SASController(P, trim), P, trim, T_total=20, disturbance=None)
    v_start = trim_data["u"][0]
    v_end = np.mean(trim_data["u"][-50:])
    drift_pct = (v_end - v_start) / v_start * 100
    print(f"  20s 配平漂移: {drift_pct:.3f}%")

    # ========== 4. INDI vs SAS ==========
    print("[4/5] INDI vs SAS 扰动抑制对比")
    for dist_name, dist in [("俯仰扰动 5°", (3, 3.1, "pitch", 5.0)),
                             ("滚转扰动 0.25", (3, 3.1, "roll", 0.25))]:
        data_s, data_i = compare_controllers(P, trim, T_total=12, disturbance=dist)
        n_s = np.sum(np.abs(data_s["theta"] - data_s["theta"][0]))
        n_i = np.sum(np.abs(data_i["theta"] - data_i["theta"][0]))
        improvement = (1 - n_i/n_s) * 100 if n_s > 0 else 0
        print(f"  {dist_name}: INDI 累积误差相对 SAS 改善 {improvement:.1f}%")

    # ========== 5. 积分方法精度验证 ==========
    print("[5/5] RK4 vs 显式欧拉精度对比")
    from core import quat_norm, quat_multiply
    # 基准旋转 (恒定 1 rad/s 绕 z 轴, 60s → 60 rad = ~9.55 圈)
    q_euler = np.array([1.0, 0.0, 0.0, 0.0])
    q_rk4_q = np.array([1.0, 0.0, 0.0, 0.0])
    omega = np.array([0.0, 0.0, 1.0])
    dt = 0.004; N = 15000  # 60s
    for _ in range(N):
        dq_e = 0.5*dt*quat_multiply(q_euler, np.array([0,omega[0],omega[1],omega[2]]))
        q_euler = quat_norm(q_euler + dq_e)
        def f_q(qq): return 0.5*quat_multiply(qq, np.array([0,omega[0],omega[1],omega[2]]))
        d1=f_q(q_rk4_q); q2=quat_norm(q_rk4_q+0.5*dt*d1); d2=f_q(q2)
        q3=quat_norm(q_rk4_q+0.5*dt*d2); d3=f_q(q3)
        q4=quat_norm(q_rk4_q+dt*d3); d4=f_q(q4)
        q_rk4_q = quat_norm(q_rk4_q + dt/6*(d1+2*d2+2*d3+d4))
    # 理论: 60 rad → q = [cos(30), 0, 0, sin(30)]
    # 用四元数范数偏差衡量
    q_theory = np.array([np.cos(30.0), 0.0, 0.0, np.sin(30.0)])
    err_euler = np.linalg.norm(q_euler - q_theory)
    err_rk4 = np.linalg.norm(q_rk4_q - q_theory)
    print(f"  显式欧拉 60s 四元数偏差: {err_euler:.6e}")
    print(f"  RK4 60s 四元数偏差:      {err_rk4:.6e} (RK4/Euler={err_rk4/err_euler*100:.3f}%)")

    print(f"\n{'='*60}")
    print(f" 全部分析完成 → {OUT}")
    print(f"{'='*60}")

if __name__ == "__main__":
    main()
