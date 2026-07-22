# -*- coding: utf-8 -*-
"""导出 LaTeX 表格数据"""

import numpy as np

def modes_table(results, trim, P):
    """生成纵向/横航向模态特征值 LaTeX 表格 — 每个模态一对共轭特征值"""
    lines = []
    lines.append(r"\begin{table}[H]")
    lines.append(r"\centering")
    lines.append(r"\caption{配平点线化模态特征值 ($V="
                 f"{trim['V']:.0f}" r"\,\text{m/s}$)""}")
    lines.append(r"\label{tab:modes}")
    lines.append(r"\small")
    lines.append(r"\begin{tabularx}{\textwidth}{l c c c c}")
    lines.append(r"\toprule")
    lines.append(r"\textbf{模态} & \textbf{特征值} & \textbf{频率 (Hz)} & "
                 r"\textbf{阻尼比} & \textbf{时间常数/周期} \\")
    lines.append(r"\midrule")

    # 收集已处理的共轭对
    eigs = np.array([complex(e.real, e.imag) for e in results["modes"]])
    seen_pairs = set()
    for ev in results["modes"]:
        re, im = np.real(ev["eigenvalue"]), np.imag(ev["eigenvalue"])
        key = (round(re, 3), round(abs(im), 3))
        if key in seen_pairs:
            continue
        seen_pairs.add(key)
        freq = ev.get("freq_hz", 0)
        damp = ev.get("damping", 0)
        abs_val = abs(complex(re, im))
        if damp > 0.7:
            tau = 1/abs(re) if abs(re) > 1e-6 else 0
            period_str = f"$\\tau={tau:.3f}$\,s"
        elif abs(im) > 1e-4:
            period = 2*np.pi/abs(im) if abs(im) > 1e-6 else 0
            period_str = f"$T={period:.3f}$\,s"
        else:
            tau = 1/abs(re) if abs(re) > 1e-6 else 0
            period_str = f"$\\tau={tau:.3f}$\,s"
        if abs(im) < 1e-4:
            ev_str = f"${re:.3f}$"
        else:
            ev_str = f"${re:.3f} \\pm {abs(im):.3f}i$"
        if freq > 1.0 and damp > 0.3:
            label = "短周期 (SP)"
        elif freq < 0.2 and damp < 0.7:
            label = "长周期 (Ph)"
        elif abs(re) > 10 and abs(im) < 1:
            label = "滚转收敛 (RR)"
        elif freq > 0.5 and damp < 0.7:
            label = "荷兰滚 (DR)"
        elif abs(re) < 0.2 and abs(im) < 1e-4:
            label = "螺旋 (Sp)"
        else:
            label = "—"
        lines.append(f"  {label} & {ev_str} & {freq:.3f} & {damp:.3f} & {period_str} \\\\")
    lines.append(r"\bottomrule")
    lines.append(r"\end{tabularx}")
    lines.append(r"\end{table}")
    return "\n".join(lines)

def trim_table(trim, P):
    """生成配平状态摘要表"""
    return (
        r"\begin{table}[H]" "\n"
        r"\centering" "\n"
        r"\caption{纵向配平状态 ($V=" f"{trim['V']:.0f}" r"\,\text{m/s}$)}" "\n"
        r"\label{tab:trim}" "\n"
        r"\small" "\n"
        r"\begin{tabular}{l c c}" "\n"
        r"\toprule" "\n"
        r"\textbf{参数} & \textbf{符号} & \textbf{值} \\" "\n"
        r"\midrule" "\n"
        f"迎角 & $\\alpha$ & ${np.degrees(trim['alpha']):.3f}^\\circ$ \\\\" "\n"
        f"尾摆偏置 & $\\delta_{{t0}}$ & ${np.degrees(trim['delta_t']):.3f}^\\circ$ \\\\" "\n"
        f"基准转速 & $\\omega_0$ & ${trim['omega0']:.1f}\\,\\text{{rad/s}}$ \\\\" "\n"
        f"油门百分比 & — & ${trim['omega0']/P['wMax']*100:.1f}\\%$ \\\\" "\n"
        r"\bottomrule" "\n"
        r"\end{tabular}" "\n"
        r"\end{table}"
    )

def step_response_metrics(data, event_t, settle_window=2.0):
    """从时域数据提取阶跃响应指标."""
    t = data["t"]; y = data["theta"]  # default: pitch
    idx_event = np.argmin(np.abs(t - event_t))
    # 稳态
    steady = np.mean(y[t > t[-1] - settle_window])
    peak = np.max(np.abs(y[idx_event:]))
    settling = t[-1]  # simplified
    for i in range(idx_event, len(t)):
        if np.all(np.abs(y[i:] - steady) < 0.05 * peak):
            settling = t[i] - event_t; break
    overshoot = (peak - steady) / max(abs(steady), 1e-4) * 100 if steady != 0 else 0
    return {"peak": peak, "steady": steady, "settling_s": settling,
            "overshoot_pct": overshoot}
