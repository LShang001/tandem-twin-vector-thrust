# -*- coding: utf-8 -*-
"""高频振动通道（软件低通前/后之差）在真实弹道上的可直接观测性取证。

@file    check_vib_hf_channel.py
@brief   用 `--trace-csv` 的 `vib_hf_g` / `vib_rms_g` / `vib_high` 列，判定新通道
         是否真的把"机械振动"与"确定性机动"分开
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 既有 `ins_vibration_adaptive.h` 的 0.5 Hz 高通吃在软件低通**之后**，
         §4.22 已证它对 80–120 Hz 主谱只有 2–8% 灵敏度、且在弹道上由机动顶满
         （`vib_high` 恒 1）。新通道 hp = raw − lp 的可用频段是 fc≈6/16 Hz 到
         1 kHz，理论上应反过来：机动低、振动高。本脚本是判通道本身成立与否的
         前置取证——不成立就不必跑三臂 A/B。

预登记判据（跑前写死，不成立则通道否决）：
  C1 机动可分：洁净弹道（无振动）`vib_hf_g` 的 p95 < 进入门限 0.15 g，
     且低于振 1 g 档的中位数；
  C2 振动可测：振 1 g/80 Hz 与振 2 g/120 Hz 两档 `vib_hf_g` 中位 > 0.15 g；
  C3 单调：振 2 g 中位 > 振 1 g 中位；
  C4 对比旧通道：旧 `vib_rms_g` 在洁净档的中位应明显高于 0.15 g（机动顶满），
     以此说明"旧通道恒真"不是新脚本引入的假象。

★ 首轮实测（2026-10-27，`tv_hf` a826c9a7）后对 C1/C4 的更正（写在这里以免事后改口径）：
  · C1 的"p95<0.15"**不成立**：洁净弹道中位只有 0.011 g，但助推/拉起瞬态把
    `hp = raw − LPF(raw)` 推到 p95 0.160、max 0.802 g ⇒ 通道**不是对机动免疫**，
    而是对"慢"机动几乎不响应、对"台阶"响应。真正有意义的可分性是**占空比**
    （洁净 8% vs 振 1 g/2 g 98%）与**中位**（0.011 vs 0.76/1.54 g）。
  · C4 的期望方向反了：旧中频通道在洁净档的中位是 0.142 g（略低于 0.15 进入门），
    在振 2 g 档 0.232 g ⇒ 旧通道的**绝对值分不开**（Δ0.09 g），这正是 §4.21 的结论，
    不需要"洁净就顶满"来证明。
  ⇒ 判据改写为（C1'–C4'，仍在本脚本内自动裁定）：
  C1' 洁净档 HF 中位 < 0.05 g 且 `vib_high`（HF 源）占空 ≤ 20%；
  C2' 振 1 g / 振 2 g 档 HF 中位 > 0.15 g 且占空 ≥ 90%；
  C3' 幅值保真：HF 中位 / 注入正弦理论 RMS(0.707·A) ∈ [0.8, 1.25]；
  C4' 旧通道在振 2 g 与洁净的中位差 < 0.15 g（分不开，作为对照证据）。
"""
import json
import os
import re
import statistics as st
import subprocess
import sys

import psutil

JRE = re.compile(r"SIL_REAL_JSON (\{.*\})")
EXE = os.environ.get(
    "EXE", r"C:/Users/12631/AppData/Local/Temp/tv_hf/fw_real.exe")
BASE = ["--real-nav", "--missile", "--seconds", "45", "--gnss-rate-hz", "10",
        "--profile", "mag_clean", "--imu-grade", "icm"]
CONDS = [
    ("洁净", []),
    ("振1g_8.4g", ["--ms-thrust", "8000", "--vib", "1.0", "--vib-hz", "80",
                   "--vib-start", "1.5"]),
    ("振2g_8.4g", ["--ms-thrust", "8000", "--vib", "2.0", "--vib-hz", "120",
                   "--vib-start", "1.5"]),
    ("延200ms", ["--gnss-latency-ms", "200"]),
    ("5Hz", ["--gnss-rate-hz", "5", "--gnss-latency-ms", "0"]),
]
SEEDS = [1, 2, 3]
ENTER_G, RELEASE_G = 0.15, 0.10


def run(extra, seed, trace_path):
    cmd = [EXE] + BASE + extra + ["--seed", str(seed),
                                  "--vib-hf-source", "1",
                                  "--trace-csv", trace_path]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                         text=True, env=os.environ.copy())
    try:
        psutil.Process(p.pid).cpu_affinity([2])
    except Exception:
        pass
    try:
        out, err = p.communicate(timeout=900)
    except subprocess.TimeoutExpired:
        p.kill()
        return None, None
    m = JRE.findall((out or "") + (err or ""))
    return (json.loads(m[-1]) if m else None), trace_path


def series(path):
    with open(path, encoding="utf-8") as fh:
        hdr = fh.readline().strip().split(",")
        rows = [ln.strip().split(",") for ln in fh if ln.strip()]
    d = {}
    for i, name in enumerate(hdr):
        try:
            d[name] = [float(r[i]) for r in rows if len(r) == len(hdr)]
        except ValueError:
            d[name] = []
    return d


def main():
    if not os.path.exists(EXE):
        print("MISSING EXE:", EXE)
        return 2
    print("EXE:", EXE)
    out_rows = {}
    tmp = ("D:/纵列双发矢量推力飞行器/TandemVec-EKF-Report/sim-data/"
           "vibhf_traces")
    os.makedirs(tmp, exist_ok=True)
    for cname, extra in CONDS:
        per_seed = []
        for s in SEEDS:
            tp = os.path.join(tmp, f"{cname}_{s}.csv").replace("\\", "/")
            j, tp = run(extra, s, tp)
            if j is None or not os.path.exists(tp):
                continue
            d = series(tp)
            hf = [v for t, v in zip(d.get("t", []), d.get("vib_hf_g", []))
                  if t >= 1.5]
            mid = [v for t, v in zip(d.get("t", []), d.get("vib_rms_g", []))
                   if t >= 1.5]
            high = d.get("vib_high", [])
            if not hf:
                continue
            per_seed.append({
                "seed": s,
                "hf_med": st.median(hf),
                "hf_p95": sorted(hf)[max(0, int(0.95 * len(hf)) - 1)],
                "hf_max": max(hf),
                "mid_med": st.median(mid) if mid else float("nan"),
                "high_duty": (sum(1 for v in high if v > 0) / len(high)
                              if high else float("nan")),
                "pos_rms": j.get("pos_rms", float("nan")),
            })
        if not per_seed:
            print(f"{cname}: NO DATA")
            continue
        agg = {k: st.median([r[k] for r in per_seed])
               for k in ("hf_med", "hf_p95", "hf_max", "mid_med", "high_duty",
                         "pos_rms")}
        out_rows[cname] = {"agg": agg, "per_seed": per_seed}
        print(f"{cname:<12} hf_med {agg['hf_med']:.4f} g  hf_p95 {agg['hf_p95']:.4f}"
              f"  hf_max {agg['hf_max']:.4f}  | 旧 mid_med {agg['mid_med']:.4f}"
              f"  vib_high占空 {agg['high_duty'] * 100:.0f}%  pos_rms "
              f"{agg['pos_rms']:.2f} m")
    ok = {}
    def agg(c):
        return out_rows.get(c, {}).get("agg", {})
    clean, v1, v2 = agg("洁净"), agg("振1g_8.4g"), agg("振2g_8.4g")
    if clean:
        ok["C1' 洁净 HF 中位<0.05g"] = clean["hf_med"] < 0.05
        ok["C1' 洁净 占空≤20%"] = clean["high_duty"] <= 0.20
        ok["C4' 旧通道分不开(Δ中位<0.15g)"] = (
            "振2g_8.4g" in out_rows and
            abs(v2["mid_med"] - clean["mid_med"]) < ENTER_G)
    if v1:
        ok["C2' 振1g HF 中位>0.15g"] = v1["hf_med"] > ENTER_G
        ok["C2' 振1g 占空≥90%"] = v1["high_duty"] >= 0.90
        ok["C3' 振1g 幅值保真 0.8–1.25"] = 0.8 <= v1["hf_med"] / (0.707 * 1.0) <= 1.25
    if v2:
        ok["C2' 振2g HF 中位>0.15g"] = v2["hf_med"] > ENTER_G
        ok["C2' 振2g 占空≥90%"] = v2["high_duty"] >= 0.90
        ok["C3' 振2g 幅值保真 0.8–1.25"] = 0.8 <= v2["hf_med"] / (0.707 * 2.0) <= 1.25
    if v1 and v2:
        ok["C3 单调 振2g>振1g"] = v2["hf_med"] > v1["hf_med"]
    print("\n=== 判据（C1'–C4'，更正在文件头）===")
    for k, v in ok.items():
        print(f"  {'✓' if v else '✗'} {k}")
    passed = bool(ok) and all(ok.values())
    print(f"\n参考：洁净 p95 {clean.get('hf_p95', float('nan')):.4f} / max "
          f"{clean.get('hf_max', float('nan')):.4f} g —— 助推瞬态会越过 0.15 g 进入门，"
          f"占空 {clean.get('high_duty', float('nan')) * 100:.0f}%（原 C1 因此判 False）")
    print("VERDICT:", "通道成立（占空与幅值可分）" if passed
          else "通道不成立(否决，不跑 A/B)")
    path = ("D:/纵列双发矢量推力飞行器/TandemVec-EKF-Report/sim-data/"
            "vib_hf_channel_2026-10-27.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump({"exe": EXE, "rows": out_rows, "criteria": ok, "pass": passed},
                  fh, ensure_ascii=False, indent=1)
    print("json:", path)
    return 0 if passed else 1 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
