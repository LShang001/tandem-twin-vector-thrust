# -*- coding: utf-8 -*-
"""高频振动否决的进入门限扫描：能否找到"只在真振动时否决、机动与 1 g 不否决"的档位。

@file    run_vib_hf_enter_scan.py
@brief   同一二进制只差 runtime 的 `--vib-veto/--vib-hf-source/--vib-hf-enter`，
         在 8 个工况 × 4 个门限臂 × 20 seeds 上找占优档
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 三臂 A/B（`sim/run_vib_hf_ab.py`，`tv_hf` a826c9a7，20 seeds）已证：
         把 `vib_high` 换成高频通道后，"否决开启"能完整恢复对振 2 g 的保护
         （22.37→3.04 m，与历史否决逐位相同），并保住延时收益（延 100 ms 4.89→4.69）；
         唯一代价是振 1 g 档 2.68→3.31 m（+23%，20/20 变差），因为默认进入门限
         0.15 g 在 1 g 振动（HF RMS 0.76 g）下也置位。
         通道取证给出可选档位的物理依据：洁净中位 0.011 g、助推瞬态 max 0.802 g、
         振 1 g 0.763 g、振 2 g 1.539 g ⇒ 门限放在 (0.80, 1.54) 之间理论上可把
         "只挡强振动"做实。★ 这是**看过通道数据之后**加的参数，不是预登记假设；
         因此本脚本只问"是否占优于当前默认"，不拿同批 seeds 当独立验证。

预登记选择规则（跑前写死）：
  S1 保护仍在：候选臂在振 2 g 的 pos_rms 中位 ≤ 1.2 × 历史否决臂（≤3.6 m）；
  S2 零代价：候选臂在 洁净/延100ms/延200ms/5Hz/巡航失联5s 五格的 pos_rms 中位
     ≤ 1.05 × A 臂；
  S3 加分项：振 1 g 也 ≤1.05 × A（严格占优）；
  S4 在满足 S1+S2 的臂里取**进入门限最大**者（离助推瞬态 0.80 g 裕度最大）；
  S5 若无臂满足 S1+S2 ⇒ 默认保持"否决关"，高频通道仅作遥测；
  无论选到哪一档，默认值决策都必须由独立配置（21 格 × 40 seeds）确认，
  不得仅凭本表翻转固件默认。
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
    "EXE", r"C:/Users/12631/AppData/Local/Temp/tv_hf2/fw_real.exe")
BASE = ["--real-nav", "--missile", "--seconds", "45", "--gnss-rate-hz", "10",
        "--profile", "mag_clean", "--imu-grade", "icm"]
ARMS = [
    ("A veto关", ["--vib-veto", "0"], None),
    ("E15 veto开HF", ["--vib-veto", "1", "--vib-hf-source", "1",
                      "--vib-hf-enter", "0.15"], 0.15),
    ("E50 veto开HF", ["--vib-veto", "1", "--vib-hf-source", "1",
                      "--vib-hf-enter", "0.5"], 0.5),
    ("E100 veto开HF", ["--vib-veto", "1", "--vib-hf-source", "1",
                       "--vib-hf-enter", "1.0"], 1.0),
    ("C veto开中频", ["--vib-veto", "1", "--vib-hf-source", "0"], None),
]
CONDS = [
    ("洁净", [], 20),
    ("延100ms", ["--gnss-latency-ms", "100"], 20),
    ("延200ms", ["--gnss-latency-ms", "200"], 20),
    ("5Hz", ["--gnss-rate-hz", "5"], 20),
    ("振1g_8.4g", ["--ms-thrust", "8000", "--vib", "1.0", "--vib-hz", "80",
                   "--vib-start", "1.5"], 20),
    ("振2g_8.4g", ["--ms-thrust", "8000", "--vib", "2.0", "--vib-hz", "120",
                   "--vib-start", "1.5"], 20),
    ("巡航失联5s", ["--gnss-outage", "20:25"], 20),
    ("IMU冻结1s", ["--imu-fault", "stuck", "--imu-fault-window", "12:13"], 20),
]
FREE_CELLS = ["洁净", "延100ms", "延200ms", "5Hz", "巡航失联5s"]
KEYS = ["pos_rms", "pos_max", "nav_att_rmse", "nav_vel_rms", "ad_app", "ad_blk"]


def run(extra, arm_args, seed):
    cmd = [EXE] + BASE + extra + arm_args + ["--seed", str(seed)]
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
    print("exe md5", md5[:8], "arms", len(ARMS), "x cells", len(CONDS))
    res = {}
    for cname, extra, nseed in CONDS:
        per = {}
        for label, arm_args, enter in ARMS:
            with cf.ThreadPoolExecutor(max_workers=8) as pool:
                per[label] = [r for r in pool.map(
                    lambda s: run(extra, arm_args, s),
                    range(1, nseed + 1)) if r]
        print(f"\n=== {cname} ===")
        hdr = "  {:<16}".format("metric") + "".join(
            f"{lab:>18}" for lab, _, _ in ARMS)
        print(hdr)
        for key in KEYS:
            row = "  {:<16}".format(key)
            row += "".join(f"{med(per[lab], key):>18.3f}" for lab, _, _ in ARMS)
            print(row)
        pa = med(per["A veto关"], "pos_rms")
        row = "  {:<16}".format("pos_rms vs A%")
        row += "".join(
            f"{(med(per[lab], 'pos_rms') - pa) / pa * 100 if pa else 0:>17.1f}%"
            for lab, _, _ in ARMS)
        print(row)
        res[cname] = {"med": {lab: {k: med(per[lab], k) for k in KEYS}
                              for lab, _, _ in ARMS},
                      "rows": {lab: per[lab] for lab, _, _ in ARMS}}
    # 预登记规则 S1–S5
    def m(cell, arm, key="pos_rms"):
        return res[cell]["med"][arm][key]
    c_ref = m("振2g_8.4g", "C veto开中频")
    chosen, notes = None, {}
    for label, _, enter in ARMS[1:-1]:
        s1 = m("振2g_8.4g", label) <= 1.2 * c_ref
        s2 = all(m(cell, label) <= 1.05 * m(cell, "A veto关")
                 for cell in FREE_CELLS)
        s3 = m("振1g_8.4g", label) <= 1.05 * m("振1g_8.4g", "A veto关")
        notes[label] = {"S1 振2g保护": s1, "S2 五格零代价": s2, "S3 振1g也无代价": s3,
                        "enter_g": enter}
        print(f"\n{label}: S1={s1} S2={s2} S3={s3} (enter={enter} g)")
        if s1 and s2:
            if chosen is None or enter > chosen[1]:
                chosen = (label, enter)
    print("\n=== 选择 ===")
    if chosen:
        print(f"按 S4 选中 {chosen[0]}（enter={chosen[1]} g）—— 仍须 #143 的 "
              f"21 格 × 40 seeds 独立确认才能改默认")
    else:
        print("无臂满足 S1+S2 ⇒ 默认保持 veto 关，高频通道仅作遥测（S5）")
    path = ("D:/纵列双发矢量推力飞行器/TandemVec-EKF-Report/sim-data/"
            "vib_hf_enter_scan_2026-10-28.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump({"exe_md5": md5, "criteria": notes,
                   "chosen": chosen[0] if chosen else None,
                   "chosen_enter_g": chosen[1] if chosen else None,
                   "rows": res}, fh, ensure_ascii=False)
    print("json:", path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
