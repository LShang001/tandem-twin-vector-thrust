# -*- coding: utf-8 -*-
"""GNSS 延时搜索网格上界的二维扫描：放宽到多少安全、哪里撞上回放缓冲。

@file    run_delay_grid_sweep.py
@brief   网格上界 × 真实接收机延迟 的 5×5 扫描（同二进制 runtime 开关），
         为"是否把 MAX_TAU_S 从 0.150 提高、提高到多少"给出带边界的证据
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 三臂 A/B（`sim/run_delay_grid_widen_ab.py`）已显示：把网格与 CAP 一起放宽到
         300 ms，延 150 ms 档位置 RMSE 降 72%、延 200 ms 降 41%，而 ≤100 ms 与洁净档
         逐位不变 ⇒ §4.19"不放宽"的决定是在缺少①端点判无效②残差门之前做的，需要重估。
         但 300 ms 不是随便选的：回放缓冲 = 64 × 5 ms = **320 ms**，τ̂ 超过它就没有
         对应快照，GNSS 整帧被拒（§4.19 实测 ≥330 ms 时 399/399 全拒、位置 244–296 m）。
         所以要的不是"越大越好"，而是**收益曲线与安全边界**。

扫描：网格上界 ∈ {150(默认), 200, 250, 300, 330} ms × 真实延迟 ∈ {150, 200, 250, 300, 330} ms，
     CAP 始终 = 网格上界（两者必须成对放宽），20 seeds/格 = 500 次。

预登记判据（跑前写死）：
  Q1 命中：每格 `ad_tau_ms` 中位与注入延迟之差 ≤ 一个网格步长(5 ms) 记为"命中"；
  Q2 收益：对每个真实延迟，取"命中且 pos_rms 最小"的网格档；
  Q3 边界：任何一格出现 `outlier_rejects` 或 `reacquire_resets` 相对同延迟最小可用档
     显著增加（>20%）⇒ 该网格档标为"撞缓冲"，不得进默认；
  Q4 结论形式：给出"网格上界 = min(真延迟上限, 缓冲−裕量)"的推荐值，
     并明确它只在 SIL 的"数据+iTOW 同时后移"模型下成立；
  默认值：只有存在一档同时满足 Q1–Q3 且在全部 5 个延迟档都不劣于默认档，
     才把 `BFS_NAVIGATION_GNSS_DELAY_AUTO_MAX_TAU_S` 提上去；否则保留 0.150 并记录边界。
"""
import concurrent.futures as cf
import hashlib
import json
import os
import re
import statistics as st
import subprocess
import sys

import psutil

JRE = re.compile(r"SIL_REAL_JSON (\{.*\})")
EXE = os.environ.get(
    "EXE", r"C:/Users/12631/AppData/Local/Temp/tv_hf3/fw_real.exe")
BASE = ["--real-nav", "--missile", "--seconds", "45", "--gnss-rate-hz", "10",
        "--profile", "mag_clean", "--imu-grade", "icm"]
GRIDS = [150, 200, 250, 300, 330]
LATENCIES = [150, 200, 250, 300, 330]
NSEED = 20
BIND = int(os.environ.get("BIND", "2"))
KEYS = ["pos_rms", "pos_max", "nav_att_rmse", "ad_tau_ms", "ad_app", "ad_blk",
        "outlier_rejects", "reacquire_resets"]


def run(grid_ms, lat_ms, seed):
    cmd = [EXE] + BASE + ["--gnss-latency-ms", str(lat_ms)]
    if grid_ms != 150:
        cmd += ["--delay-grid-max-ms", str(grid_ms), "--delay-cap-ms", str(grid_ms)]
    cmd += ["--seed", str(seed)]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                         text=True, env=os.environ.copy())
    try:
        psutil.Process(p.pid).cpu_affinity([BIND])
    except Exception:
        pass
    try:
        out, err = p.communicate(timeout=900)
    except subprocess.TimeoutExpired:
        p.kill()
        return None
    m = JRE.findall((out or "") + (err or ""))
    return json.loads(m[-1]) if m else None


def med(rows, key):
    vals = [r.get(key, 0) or 0 for r in rows]
    return st.median(vals) if vals else float("nan")


def main():
    if not os.path.exists(EXE):
        print("MISSING EXE:", EXE)
        return 2
    md5 = hashlib.md5(open(EXE, "rb").read()).hexdigest()
    print("exe md5", md5[:8], "grid x lat =", len(GRIDS), "x", len(LATENCIES),
          "x", NSEED, "seeds")
    data = {}
    for lat in LATENCIES:
        for grid in GRIDS:
            with cf.ThreadPoolExecutor(max_workers=8) as pool:
                rows = [r for r in pool.map(
                    lambda s: run(grid, lat, s), range(1, NSEED + 1)) if r]
            data[(lat, grid)] = rows
            hit = med(rows, "ad_tau_ms")
            print(f"lat {lat:>4} ms  grid {grid:>4} ms | pos_rms {med(rows,'pos_rms'):>8.2f}"
                  f"  att {med(rows,'nav_att_rmse'):>6.2f}  tau {hit:>6.1f}"
                  f"  app {med(rows,'ad_app'):>6.1f}  rej {med(rows,'outlier_rejects'):>5.1f}"
                  f"  rst {med(rows,'reacquire_resets'):>4.1f}", flush=True)
        print("", flush=True)

    # 预登记判据
    print("=== Q1 命中判定（|τ̂ − 注入| ≤ 5 ms）===")
    hit_tab = {}
    for lat in LATENCIES:
        line = []
        for grid in GRIDS:
            t = med(data[(lat, grid)], "ad_tau_ms")
            ok = abs(t - lat) <= 5.0
            hit_tab[(lat, grid)] = ok
            line.append(f"{grid}:{'命中' if ok else '未中'}")
        print(f"  lat {lat:>4} ms  " + "  ".join(line))
    print("\n=== Q3 撞缓冲判定（rej/rst 相对该延迟最小可用档 >20%）===")
    unsafe = set()
    for lat in LATENCIES:
        usable = [g for g in GRIDS if hit_tab[(lat, g)]]
        if not usable:
            print(f"  lat {lat}: 无任何档命中（该延迟超出可补偿范围）")
            continue
        g0 = min(usable)
        r0 = med(data[(lat, g0)], "outlier_rejects") + med(data[(lat, g0)], "reacquire_resets")
        for g in usable:
            r = med(data[(lat, g)], "outlier_rejects") + med(data[(lat, g)], "reacquire_resets")
            if r0 > 0 and r > 1.2 * r0:
                unsafe.add(g)
                print(f"  lat {lat} grid {g}: rej+rst {r0:.1f} -> {r:.1f}  ★撞缓冲")
    if not unsafe:
        print("  未发现 rej+rst 显著增加的档")
    print("\n=== Q2/Q4 每档最优网格（命中且 pos_rms 最小、且不在撞缓冲集合）===")
    best = {}
    for lat in LATENCIES:
        cand = [g for g in GRIDS if hit_tab[(lat, g)] and g not in unsafe]
        if not cand:
            print(f"  lat {lat}: 无可用档")
            continue
        b = min(cand, key=lambda g: med(data[(lat, g)], "pos_rms"))
        best[lat] = b
        d = med(data[(lat, 150)], "pos_rms")
        print(f"  lat {lat:>4} ms: 最优 grid {b:>4} ms  pos_rms "
              f"{med(data[(lat,b)],'pos_rms'):.2f}（默认档 {d:.2f}，"
              f"{(med(data[(lat,b)],'pos_rms')-d)/d*100 if d else 0:+.1f}%）")
    path = ("D:/纵列双发矢量推力飞行器/TandemVec-EKF-Report/sim-data/"
            "delay_grid_sweep_2026-10-28.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump({"exe_md5": md5, "grids": GRIDS, "latencies": LATENCIES,
                   "nseed": NSEED,
                   "med": {f"{lat}_{g}": {k: med(data[(lat, g)], k) for k in KEYS}
                           for (lat, g) in data},
                   "hit": {f"{lat}_{g}": hit_tab[(lat, g)] for (lat, g) in hit_tab},
                   "unsafe_grids": sorted(unsafe), "best_grid_by_latency": best},
                  fh, ensure_ascii=False, indent=1)
    print("json:", path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
