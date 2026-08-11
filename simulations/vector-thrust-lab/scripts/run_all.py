#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_all.py — 鲁棒性/相位裕度分析工具集统一入口
================================================
一键运行全部 7 个分析脚本，输出汇总 JSON（供论文同步/交接使用）。

用法：
  py -3.12 run_all.py                 # 全量运行 + 汇总
  py -3.12 run_all.py --quick         # 只跑核心 4 个（跳过二维扫描绘图）
  py -3.12 run_all.py --json out.json # 汇总写入指定文件

固件带宽覆盖：
  修改 pm_sensitivity.py 顶部 K_FW 常量（当前 rate.kp 直接为 16.042818 s^-1），
  或直接传参：py -3.12 run_all.py --fw-k 16.04
"""
import argparse
import json
import subprocess
import sys
import os
from datetime import datetime

SCRIPTS = [
    "cascade_closedform.py",       # D1 符号推导自检
    "robust_lmi_analysis.py",      # D1 LMI 鲁棒界（γ̄）
    "robust_montecarlo.py",        # D1 蒙特卡洛验证
    "phase_margin_analysis.py",    # 相位裕度（摆角 vs 差速）
    "pm_sensitivity.py",           # 量纲统一 + PM(K) 敏感度
    "pm_observer_compensation.py", # τm 观测器补偿量化
    "pm_2d_sweep.py",              # α×τm 二维扫描
]
QUICK = SCRIPTS[:6]                # --quick 跳过二维扫描

KEY_PATTERNS = {
    "cascade_closedform.py":    ["符号 vs 数值特征多项式", "全通过"],
    "robust_lmi_analysis.py":   ["γ̄ =", "标称二次稳定"],
    "robust_montecarlo.py":     ["LMI 保守度", "5000 样本失稳率"],
    "phase_margin_analysis.py": ["差速通道连续 PM", "相位裕度 PM"],
    "pm_sensitivity.py":        ["固件等效", "旧论文"],
    "pm_observer_compensation.py": ["失稳窗口", "补偿的 PM 增益"],
    "pm_2d_sweep.py":           ["交叉核查", "边界带内"],
}


def extract_key_lines(log: str, script: str):
    """从脚本输出抽取关键行（供汇总 JSON）。"""
    keys = KEY_PATTERNS.get(script, [])
    hits = []
    for line in log.splitlines():
        if any(k in line for k in keys) and len(line) < 200:
            hits.append(line.strip())
    return hits[:4]


def main():
    ap = argparse.ArgumentParser(description="鲁棒性分析工具集统一入口")
    ap.add_argument("--quick", action="store_true", help="只跑核心 6 个脚本")
    ap.add_argument("--json", default=None, help="汇总 JSON 输出路径")
    ap.add_argument("--fw-k", type=float, default=None,
                    help="固件等效带宽（rad/s），默认读 pm_sensitivity.K_FW")
    args = ap.parse_args()

    scripts = QUICK if args.quick else SCRIPTS
    here = os.path.dirname(os.path.abspath(__file__))

    results = {"timestamp": datetime.now().isoformat(), "scripts": []}
    all_ok = True
    print("=" * 60)
    print(" 鲁棒性/相位裕度分析工具集 — 统一回归")
    print("=" * 60)

    for s in scripts:
        path = os.path.join(here, s)
        if not os.path.exists(path):
            print(f"  [缺失] {s}"); all_ok = False; continue
        r = subprocess.run([sys.executable, path], capture_output=True, text=True,
                           cwd=here, encoding='utf-8', errors='replace')
        ok = (r.returncode == 0)
        all_ok &= ok
        log = r.stdout + r.stderr
        keys = extract_key_lines(log, s)
        entry = {"script": s, "exit": r.returncode, "key_lines": keys}
        results["scripts"].append(entry)
        status = "✓" if ok else "✗"
        print(f"  [{status}] {s} (exit={r.returncode})")
        for k in keys[:2]:
            print(f"          {k}")

    print("=" * 60)
    print(f" 总体: {'全部通过 ✓' if all_ok else '存在失败 ✗'}")
    if args.fw_k is not None:
        print(f" 注: 固件等效带宽覆盖为 {args.fw_k} rad/s（pm_sensitivity 内 K_FW）")
        print("     —— 请同时手动核对 pm_sensitivity.py 顶部 K_FW 常量")
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as f:
            json.dump(results, f, ensure_ascii=False, indent=2)
        print(f" 汇总已写入: {args.json}")
    print("=" * 60)
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
