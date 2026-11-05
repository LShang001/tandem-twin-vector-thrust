# -*- coding: utf-8 -*-
"""SIL：GNSS 垂直 spoof（高度台阶/慢偏）对高度保持与姿态的影响。
@file    run_gnss_vspoof.py
@brief   GNSS 垂直 spoof 矩阵驱动。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 驱动 fw_real.exe 的 --gnss-spoof-z 组合。
@frame   N/A（只驱动 fw_real.exe 并统计垂直通道标量；水平 spoof 固定为 0）
@units   m（高度 / 垂直偏移）、°（滚转角）、s（时间）、m/s（垂向速度）

垂直偏移正值 = 报得更高；水平 spoof 设为 0 以隔离垂直通道。
"""
import json, os, re, subprocess
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TAG = os.environ.get("VSPOOF_TAG", "2026-10-26")
OUT = os.path.join(HERE, "..", "sim-data", f"gnss_vspoof_{TAG}.jsonl")
SUM = os.path.join(HERE, "..", "sim-data", f"gnss_vspoof_{TAG}.json")
EXE = os.environ.get("VSPOOF_EXE", "/tmp/tv_fcs_vspoof/fw_real.exe")
JSON_RE = re.compile(r"SIL_REAL_JSON (\{.*\})")
SEEDS = list(range(1, int(os.environ.get("VSPOOF_SEEDS", "11"))))
START = 25.0
SECONDS = 50.0
GATE = os.environ.get("VSPOOF_GATE", "")
ARMS = {
    "step_up_30": ["--gnss-spoof-z", "30"],
    "step_up_50": ["--gnss-spoof-z", "50"],
    "step_up_100": ["--gnss-spoof-z", "100"],
    "step_dn_30": ["--gnss-spoof-z", "-30"],
    "step_dn_50": ["--gnss-spoof-z", "-50"],
    "step_dn_100": ["--gnss-spoof-z", "-100"],
    "ramp_up_0.5": ["--gnss-spoof-z-ramp", "0.5"],
    "ramp_dn_0.5": ["--gnss-spoof-z-ramp", "-0.5"],
}


def one(arm, seed):
    """跑一个垂直 spoof 臂的单 seed，返回 SIL_REAL_JSON 指标字典。

    固定 --pos-hold --flow 且水平 spoof=0 以隔离垂直通道，spoof 从 START 秒开始；
    VSPOOF_GATE 非空时追加 --gnss-v-gate。输出无 JSON 时返回带 no_json=True 的占位。
    """
    cmd = [EXE, "--real-nav", "--profile", "mag_clean", "--pos-hold",
           "--flow", "--gnss-spoof", "0", "--gnss-spoof-start", f"{START:g}",
           "--seed", str(seed), "--seconds", f"{SECONDS:g}"]
    if GATE:
        cmd += ["--gnss-v-gate", GATE]
    cmd += ARMS[arm]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=240)
    mm = JSON_RE.findall(r.stdout + r.stderr)
    if not mm:
        return dict(arm=arm, seed=seed, no_json=True)
    d = json.loads(mm[-1].replace("nan", "NaN"))
    d.update(arm=arm, seed=seed)
    return d


def stats(v):
    """对一组数值算 med/p10/p90/min/max；剔除 NaN/Inf 与非数值，空集各值返回 NaN。"""
    v = np.array([x for x in v if isinstance(x, (int, float)) and np.isfinite(x)],
                 float)
    if not len(v):
        return dict(med=float("nan"), p10=float("nan"), p90=float("nan"),
                    mn=float("nan"), mx=float("nan"))
    return dict(med=float(np.median(v)), p10=float(np.percentile(v, 10)),
                p90=float(np.percentile(v, 90)), mn=float(v.min()),
                mx=float(v.max()))


def main():
    """命令行入口：8 个 spoof 臂 × SEEDS 逐条跑，打印逐条摘要并落盘 jsonl/json。

    臂为 ±30/±50/±100 m 高度台阶与 ±0.5 m/s 慢偏；汇总 final_alt、nav_alt_mean、
    pos_rms、max_roll、land_t、拒帧/重锚计数等分位数，并统计各臂落地次数与有效样本数。
    """
    rows = []
    for arm in ARMS:
        for seed in SEEDS:
            d = one(arm, seed)
            rows.append(d)
            if not d.get("no_json"):
                print(f"{arm:12s} s{seed:2d} alt={d['final_alt']:6.2f} "
                      f"nav_alt={d['nav_alt_mean']:6.2f} pos={d['pos_rms']:.2f} "
                      f"roll={d['max_roll']:5.2f} land={d['land_t']:6.2f} "
                      f"rej={d['outlier_rejects']} rst={d['reacquire_resets']}",
                      flush=True)
    with open(OUT, "w", encoding="utf-8") as f:
        for d in rows:
            f.write(json.dumps(d, ensure_ascii=False) + "\n")
    keys = ["final_alt", "nav_alt_mean", "pos_rms", "max_roll", "land_t",
            "land_vz_down", "outlier_rejects", "reacquire_resets",
            "max_desc_vz_down"]
    summary = {}
    for arm in ARMS:
        rs = [d for d in rows if d["arm"] == arm and not d.get("no_json")]
        summary[arm] = {k: stats([d.get(k) for d in rs]) for k in keys}
        summary[arm]["n_landed"] = int(sum(1 for d in rs if d.get("land_t", -1) >= 0))
        summary[arm]["n"] = len(rs)
    json.dump(summary, open(SUM, "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    print("wrote", OUT)


if __name__ == "__main__":
    main()
