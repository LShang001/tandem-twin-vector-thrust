# -*- coding: utf-8 -*-
"""垂直重锚门控回归：正常/失联/风下 gate 0 vs 1（同二进制）。
@file    run_vgate_regression.py
@brief   垂直 spoof 门控回归。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 驱动门控开/关的常规场景 A/B。
@frame   N/A（只驱动门控开 / 关场景并汇总 SIL 汇总标量，不做坐标变换）
@units   m（高度 / 位置）、°（滚转角）、s（着陆时间）
"""
import json, os, re, subprocess
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TAG = os.environ.get("GATEREG_TAG", "2026-10-26")
OUT = os.path.join(HERE, "..", "sim-data", f"gnss_vgate_reg_{TAG}.jsonl")
SUM = os.path.join(HERE, "..", "sim-data", f"gnss_vgate_reg_{TAG}.json")
EXE = os.environ.get("GATEREG_EXE", "/tmp/tv_fcs_vgate/fw_real.exe")
JSON_RE = re.compile(r"SIL_REAL_JSON (\{.*\})")
SEEDS = list(range(1, int(os.environ.get("GATEREG_SEEDS", "11"))))
SCEN = {
    "nominal": (40.0, []),
    "outage5": (40.0, ["--gnss-outage", "20:25"]),
    "outage20": (50.0, ["--gnss-outage", "20:40"]),
    "wind": (40.0, ["--wind-window", "20:30", "--wind-force", "1.0"]),
}


def one(gate, scen, seed):
    """跑一个门控档 × 场景的单 seed，返回 SIL_REAL_JSON 指标字典。

    gate 为 --gnss-v-gate 取值（0/1），scen 为 SCEN 中的场景名（决定时长与追加参数），
    seed 为随机种子。输出无 JSON 时返回带 no_json=True 的占位行。
    """
    secs, extra = SCEN[scen]
    cmd = [EXE, "--real-nav", "--profile", "mag_clean", "--pos-hold",
           "--flow", "--gnss-v-gate", str(gate), "--seed", str(seed),
           "--seconds", str(secs)] + extra
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=240)
    mm = JSON_RE.findall(r.stdout + r.stderr)
    if not mm:
        return dict(gate=gate, scen=scen, seed=seed, no_json=True)
    d = json.loads(mm[-1].replace("nan", "NaN"))
    d.update(gate=gate, scen=scen, seed=seed)
    return d


def stats(v):
    """对一组值算 med/p90/max；剔除 NaN/Inf 与非数值，空集各值返回 NaN。"""
    v = np.array([x for x in v if isinstance(x, (int, float)) and np.isfinite(x)],
                 float)
    if not len(v):
        return dict(med=float("nan"), p90=float("nan"), mx=float("nan"))
    return dict(med=float(np.median(v)), p90=float(np.percentile(v, 90)),
                mx=float(v.max()))


def main():
    """命令行入口：gate 0/1 × 4 场景 × SEEDS，打印逐条摘要并落盘 jsonl/json。

    场景为正常/失联 5 s/失联 20 s/风扰；汇总 final_alt、pos_rms、hold_end_rms、
    max_roll、land_t 与重锚次数，并统计各格落地次数与 vgsus 触发计数。
    """
    rows = []
    for gate in (0, 1):
        for scen in SCEN:
            for seed in SEEDS:
                d = one(gate, scen, seed)
                rows.append(d)
                if not d.get("no_json"):
                    print(f"g{gate} {scen:8s} s{seed:2d} alt={d['final_alt']:6.2f} "
                          f"pos={d['pos_rms']:.2f} hold={d['hold_end_rms']:.2f} "
                          f"roll={d['max_roll']:5.1f} land={d['land_t']:5.1f} "
                          f"rst={d['reacquire_resets']} vg={d['vgsus']}/{d['vgfr']}",
                          flush=True)
    with open(OUT, "w", encoding="utf-8") as f:
        for d in rows:
            f.write(json.dumps(d, ensure_ascii=False) + "\n")
    keys = ["final_alt", "pos_rms", "hold_end_rms", "max_roll", "land_t",
            "reacquire_resets"]
    summary = {}
    for gate in (0, 1):
        for scen in SCEN:
            rs = [d for d in rows if d["gate"] == gate and d["scen"] == scen
                  and not d.get("no_json")]
            summary[f"g{gate}_{scen}"] = {k: stats([d.get(k) for d in rs])
                                          for k in keys}
            summary[f"g{gate}_{scen}"]["n_landed"] = int(
                sum(1 for d in rs if d.get("land_t", -1) >= 0))
            summary[f"g{gate}_{scen}"]["vgsus_seen"] = int(
                sum(1 for d in rs if d.get("vgsus", 0) == 1))
            summary[f"g{gate}_{scen}"]["n"] = len(rs)
    json.dump(summary, open(SUM, "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    print("wrote", OUT)


if __name__ == "__main__":
    main()
