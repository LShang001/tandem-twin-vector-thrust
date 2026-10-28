#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""§4.24o：30.5 g 条件性失稳的 onset 细扫——在 22→30.5 g 之间找分叉点。

@file    run_305g_onset_scan.py
@brief   固定 §4.24 的 30.5 g 单元格参数，只把 `--ms-thrust` 换成等比阶梯，
         逐档跑 N seeds，记录 pos_rms / 拒帧 / 硬重锚 / 逐历元 τ̂ 中位，
         用于回答"分叉发生在哪个过载值、哪个量先越界"
@module  TandemVec-EKF-Report/仿真驱动（L2 研究设施）
@details 已知事实（报告 §4.24/§4.24b/§4.24g/§4.24i）：
         0) **§4.24i 已给过一条发生率曲线**（旧二进制 `tv_32g` `12b4235e…`、core2）：
            ≤22 000 N 0/20 → 22 500/23 000/23 500 N 各 1/20 → 23 900 N 18/50。
            本脚本的新增内容只有两项：① 在**当前源码二进制 + 固定 core5** 上复现该曲线
            （SIL 有核相关双吸引子史，跨二进制/跨核的尾部数字必须重测才算数）；
            ② 填 23 500→23 900 N 之间的空档（`--ladder` 可给细阶梯），
            判定"陡增"是悬崖还是陡坡。
         1) 30.5 g/10 Hz 生产路径是**双峰**（≈42% 正常 6–11 m / ≈58% 失效 22–33 m），
            中位 23.39 m 无物理含义 ⇒ 本脚本按"发生率 + 簇"报告，不报中位；
         2) 22 g 时 τ̂ 已膨胀到 38 ms 却零拒帧（⇒ τ̂ 幅值不是充分判据）；
         3) `--imu-fs-g 40` 去掉削顶后逐位不变（⇒ 削顶不是原因）；
         4) 驱动量已证为"加计软件 LPF 的时变群延迟被自动延时估计吸收后回放"。
         本脚本不测新机制，只把 22→30.5 g 之间的**分叉位置**量化，
         并检查失效是否与"整帧拒绝 / 硬重锚"同序。
         ★ 口径纪律：JSON 的 `ad_tau_ms` 是**末帧**值、已被判不可作判据
            （§4.20.3/§4.23b 末帧坑），本脚本因此**不统计 τ̂**，只用累计量
            `ad_app`（τ̂ 真的被应用的历元数）与 `outlier_rejects`/`reacquire_resets`。
@frame   NED；指标取 SIL JSON 的 pos_rms/pos_max/nav_att_rmse/outlier_rejects/
         reacquire_resets/ad_app
@units   长度 m、姿态 deg、推力 N、计数帧/次
@note    预登记判据（跑前写死）：
         O1 每档 seeds 全收（无 NO JSON），否则该档作废；
         O2 给出每档 `>25 m` 发生率曲线：若发生率在某一档从 ≈0 跳到 ≈0.5 以上 ⇒ 分叉确认；
            若逐档平滑上升 ⇒ 不是分叉而是渐变，§4.24 的"双吸引子"表述需要修正；
         O3 记录每档 `ad_app>0` 的比例与拒帧中位：若"拒帧首次非零"的档**晚于**
            发生率起跳档 ⇒ 整帧拒绝不是起始事件（与 §4.24 的"先触发拒绝、后放大误差"
            矛盾，需登记为口径修正）。
"""
import argparse
import json
import os
import re
import subprocess

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SD = os.path.abspath(os.path.join(HERE, "..", "sim-data"))
JSON_RE = re.compile(r"SIL_REAL_JSON (\{.*\})")
BASE = ["--real-nav", "--missile", "--seconds", "45", "--imu-fs-g", "32",
        "--imu-health-accel-range", "313.8", "--profile", "mag_clean",
        "--imu-grade", "icm", "--gnss-rate-hz", "10"]
# 标定：§4.24g 已测 17 200 N≈22 g、20 000 N≈25.5 g、22 000 N≈28.1 g；
# §4.24 主格 23 900 N≈30.5 g ⇒ 线性换算 ≈781.8 N/g。阶梯取 22→30.5 g 之间 10 档。
N_PER_G = 23900.0 / 30.5
THRUST_LADDER_N = [17200, 18500, 19500, 20500, 21500, 22000, 22500, 23000, 23500, 23900]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", required=True)
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--seed-start", type=int, default=1,
                    help="起始 seed（§4.24q 的阈值由 seeds 1–50 事后选出，"
                         "独立复核要跑 seeds 51–100 必须能指定起点）")
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--tag", default="2026-10-28_onset")
    ap.add_argument("--bind-core", type=int, default=5)
    ap.add_argument("--ladder", default="",
                    help="逗号分隔的推力阶梯 [N]；缺省用 THRUST_LADDER_N")
    ap.add_argument("--extra-args", default="",
                    help="追加到每次 SIL 命令的运行时开关（同二进制 A/B 用，"
                         "如 '--gnss-vel-subset 1'）；缺省空 ⇒ 与基线臂完全同参数")
    args = ap.parse_args()
    ladder = ([int(x) for x in args.ladder.split(",") if x] if args.ladder
              else list(THRUST_LADDER_N))
    assert ladder, "--ladder 解析为空"
    if args.bind_core >= 0:
        import psutil
        psutil.Process().cpu_affinity([args.bind_core])

    from concurrent.futures import ThreadPoolExecutor

    def one(job):
        thr, seed = job
        cmd = [args.exe] + BASE + ["--ms-thrust", str(thr), "--seed", str(seed)]
        if args.extra_args:
            cmd += args.extra_args.split()
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
        m = JSON_RE.findall(r.stdout + r.stderr)
        if not m:
            return None
        d = json.loads(m[-1].replace("nan", "NaN").replace("inf", "Infinity"))
        d["thrust_n"] = thr
        d["seed"] = seed
        return d

    jobs = [(t, s) for t in ladder
            for s in range(args.seed_start, args.seed_start + args.seeds)]
    out_path = os.path.join(SD, f"attrib305_{args.tag}.jsonl")
    rows = []
    with ThreadPoolExecutor(max_workers=args.workers) as ex:
        for d in ex.map(one, jobs):
            if d is not None:
                rows.append(d)
    with open(out_path, "w", encoding="utf-8") as f:
        for d in rows:
            f.write(json.dumps(d, ensure_ascii=False) + "\n")

    summary = {}
    for thr in ladder:
        sub = [r for r in rows if r["thrust_n"] == thr]
        if len(sub) != args.seeds:
            summary[thr] = {"O1_ok": False, "n": len(sub)}
            continue
        p = np.array([float(r["pos_rms"]) for r in sub])
        app = np.array([float(r.get("ad_app", 0.0)) for r in sub])
        rej = np.array([float(r.get("outlier_rejects", 0.0)) for r in sub])
        rst = np.array([float(r.get("reacquire_resets", 0.0)) for r in sub])
        good = p <= 15.0
        bad = p > 25.0
        summary[thr] = {
            "O1_ok": True, "n": len(sub), "g_est": round(thr / N_PER_G, 2),
            "rate_gt25": round(float(bad.mean()), 3),
            "n_gt25": int(bad.sum()),
            "cluster_good_med": round(float(np.median(p[good])), 3) if good.any() else None,
            "cluster_bad_med": round(float(np.median(p[bad])), 3) if bad.any() else None,
            "p90": round(float(np.percentile(p, 90)), 3),
            "ad_app_seeds": int((app > 0).sum()), "ad_app_med": float(np.median(app)),
            "rej_med": float(np.median(rej)), "rej_max": float(rej.max()),
            "reanchor_seeds": int((rst > 0).sum()),
            "att_med": round(float(np.median([float(r["nav_att_rmse"]) for r in sub])), 3),
        }
    verdict = {
        "exe": args.exe, "seeds": args.seeds, "seed_start": args.seed_start,
        "seed_range": [args.seed_start, args.seed_start + args.seeds - 1],
        "bind_core": args.bind_core,
        "base": BASE, "ladder_N": ladder, "extra_args": args.extra_args,
        "per_thrust": summary,
        "O1_all_cells_full": all(v.get("O1_ok") for v in summary.values()),
        "note": "发生率按 >25 m 计；双峰格不得用中位当指标（§4.24）",
    }
    with open(os.path.join(SD, f"attrib305_{args.tag}_verdict.json"), "w",
              encoding="utf-8") as f:
        json.dump(verdict, f, ensure_ascii=False, indent=1)
    print(json.dumps(verdict["per_thrust"], ensure_ascii=False, indent=1))
    print("O1_all_cells_full:", verdict["O1_all_cells_full"])


if __name__ == "__main__":
    main()
