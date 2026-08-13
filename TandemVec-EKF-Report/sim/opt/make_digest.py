# -*- coding: utf-8 -*-
"""EKF 优化研究结果汇总 —— 读取 baseline/sweep/variants JSON 生成对比表
输出 sim-data/opt/digest.md"""
import json
import os

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                   "sim-data", "opt")
os.makedirs(OUT, exist_ok=True)


def load(name):
    p = os.path.join(OUT, name)
    if not os.path.exists(p):
        return None
    with open(p, encoding="utf-8") as f:
        return json.load(f)


def main():
    lines = ["# EKF 优化研究 —— 数据摘要（自动生成）", ""]
    base = load("baseline_mc.json")
    if base:
        lines.append("## 基线蒙特卡洛（固件现值语义，20 种子）")
        lines.append("")
        lines.append("| 场景 | pos_ne RMSE | pos_d RMSE | vel_ne RMSE | 航向 RMSE | 姿态 RMSE | GNSS NIS | GNSS 接受率 |")
        lines.append("|---|---|---|---|---|---|---|---|")
        for scn, d in base.items():
            m = d["metrics"]
            lines.append(
                f"| {scn} | {m['pos_ne_rmse']['mean']:.3f} | {m['pos_d_rmse']['mean']:.3f} | "
                f"{m['vel_ne_rmse']['mean']:.3f} | {m.get('yaw_rmse_deg', {}).get('mean', float('nan')):.1f}° | "
                f"{m.get('att_rmse_deg', {}).get('mean', float('nan')):.1f}° | "
                f"{m.get('gnss_nis_mean', {}).get('mean', float('nan')):.1f} | "
                f"{m.get('gnss_accept', {}).get('mean', float('nan')):.2f} |")
        lines.append("")

    for g in ("gnss", "qnoise", "static", "outage"):
        sw = load(f"sweep_{g}.json")
        if not sw:
            continue
        lines.append(f"## 参数扫描组：{g}")
        for name, d in sw.items():
            lines.append(f"\n### {name} — {d['desc']}")
            scns = d["scenarios"]
            lines.append("| 配置 | " + " | ".join(
                f"{s} (pos_ne/pos_d/vel/yaw/NIS/accept)" for s in scns) + " |")
            lines.append("|---|" + "---|" * len(scns))
            for row in d["rows"]:
                cells = []
                for s in scns:
                    m = row.get(s, {})
                    cells.append(
                        f"{m.get('pos_ne_rmse', float('nan')):.3f}/"
                        f"{m.get('pos_d_rmse', float('nan')):.3f}/"
                        f"{m.get('vel_ne_rmse', float('nan')):.3f}/"
                        f"{m.get('yaw_rmse_deg', float('nan')):.1f}/"
                        f"{m.get('gnss_nis_mean', float('nan')):.1f}/"
                        f"{m.get('gnss_accept', float('nan')):.2f}")
                lines.append(f"| {row['label']} | " + " | ".join(cells) + " |")
        lines.append("")

    var = load("variants.json")
    if var:
        lines.append("## 算法变体 A/B（相对同窗口基准的差值）")
        lines.append("")
        lines.append("| 变体 | 场景 | pos_ne Δ | pos_d Δ | vel_ne Δ | 航向 Δ | GNSS NIS Δ |")
        lines.append("|---|---|---|---|---|---|---|")
        for vname, d in var["variants"].items():
            for scn, m in d["results"].items():
                if scn.endswith("_delta_vs_base"):
                    continue
                dd = d["results"].get(scn + "_delta_vs_base", {})
                lines.append(
                    f"| {vname} ({d['desc']}) | {scn} | "
                    f"{dd.get('pos_ne_rmse', float('nan')):+.3f} | "
                    f"{dd.get('pos_d_rmse', float('nan')):+.3f} | "
                    f"{dd.get('vel_ne_rmse', float('nan')):+.3f} | "
                    f"{dd.get('yaw_rmse_deg', float('nan')):+.1f} | "
                    f"{dd.get('gnss_nis_mean', float('nan')):+.1f} |")
        lines.append("")

    hifi = load("hifi_variants.json")
    if hifi:
        lines.append("## 高保真闭环验证")
        lines.append("")
        lines.append("| 变体 | H1 | H2 | H3 | H4 | H5 |")
        lines.append("|---|---|---|---|---|---|")
        for vname, res in hifi["variants"].items():
            cells = [f"{res[s].get('pos_ne_rmse', float('nan')):.2f}/"
                     f"{res[s].get('pos_d_rmse', float('nan')):.2f}/"
                     f"{res[s].get('att_q_rmse_deg', float('nan')):.1f}°"
                     for s in ("H1_hover_gust", "H2_althold", "H3_gps_loss",
                               "H4_accel_jump", "H5_tilt20")]
            lines.append(f"| {vname} | " + " | ".join(cells) + " |")
    with open(os.path.join(OUT, "digest.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("[完成] 写入", os.path.join(OUT, "digest.md"))


if __name__ == "__main__":
    main()
