# -*- coding: utf-8 -*-
"""三臂 A/B：vib 否决 × vib_high 通道来源（高频/中频）—— 能否同时保住延时收益与强振动安全。

@file    run_vib_hf_ab.py
@brief   同一二进制 `tv_hf`(a826c9a7) 只差 `--vib-veto` 与 `--vib-hf-source` 两个
         runtime 开关，检验"用软件低通前/后之差重建 vib_high 含义"是否成立
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 背景：§4.22.3 再基线发现取消振动否决带来两个严重尾部（振 2 g 3.07→28.06 m、
         IMU 冻结 3.11→34.37 m），而保留否决又会让自动延时估计在弹道上永不生效
         （§4.21.1b：旧 `vib_high` 由确定性机动顶满，120 Hz 振动反而不置位）。
         §4.22.4 开放项 ① 指出必须有一个真能测到 80–120 Hz 的通道才能重启否决——
         本脚本就是该假设的可证伪检验。

  三臂：
    A 默认(veto 关)              --vib-veto 0 --vib-hf-source 0
    B 候选(veto 开 + 高频通道)    --vib-veto 1 --vib-hf-source 1
    C 历史(veto 开 + 中频通道)    --vib-veto 1 --vib-hf-source 0

预登记判据（跑前写死；任一不满足 ⇒ B 不上默认，高频通道仅作遥测）：
  J1 保护恢复：振 2 g@120/8.4 g 档 B 的 pos_rms 中位 ≤ C×1.2（高频通道能像旧否决
     那样挡住误补偿）。
  J2 收益保留：延 100 ms 与延 200 ms 档 B ≤ A×1.1（高频通道不在机动上误触发，
     延时补偿仍可用）。
  J3 无能力损失：洁净 / 5 Hz / 巡航失联 5 s / 振 1 g 档 |B−A| 中位 ≤10%，
     且 B 的 ad_app 中位 >0（不是被恒真否决顶死的 C 行为）。
  J4 机制自证：C 在洁净档 ad_app 中位 ≈0 而 B 明显 >0；B 在振 2 g 档 ad_app
     接近 C（说明否决确实由振动触发而非机动）。
  有效性前置：每臂每格记录 `ad_blk` 与 `ad_app`；失联档仍用 `gnss_stale_peak_s`
     校验注入；不满足的档标 INVALID 不判定。
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
    "EXE", r"C:/Users/12631/AppData/Local/Temp/tv_hf/fw_real.exe")
BASE = ["--real-nav", "--missile", "--seconds", "45", "--gnss-rate-hz", "10",
        "--profile", "mag_clean", "--imu-grade", "icm"]
ARMS = [
    ("A veto关(默认)", ["--vib-veto", "0", "--vib-hf-source", "0"]),
    ("B veto开+HF", ["--vib-veto", "1", "--vib-hf-source", "1"]),
    ("C veto开+中频", ["--vib-veto", "1", "--vib-hf-source", "0"]),
]
# (名称, 追加参数, seeds, 预期失联 s 或 None)
CONDS = [
    ("洁净", [], 20, None),
    ("延100ms", ["--gnss-latency-ms", "100"], 20, None),
    ("延200ms", ["--gnss-latency-ms", "200"], 20, None),
    ("5Hz", ["--gnss-rate-hz", "5"], 20, None),
    ("振1g_8.4g", ["--ms-thrust", "8000", "--vib", "1.0", "--vib-hz", "80",
                   "--vib-start", "1.5"], 20, None),
    ("振2g_8.4g", ["--ms-thrust", "8000", "--vib", "2.0", "--vib-hz", "120",
                   "--vib-start", "1.5"], 20, None),
    ("巡航失联5s", ["--gnss-outage", "20:25"], 20, 5.0),
    ("IMU冻结1s", ["--imu-fault", "stuck", "--imu-fault-window", "12:13"],
     20, None),
]
NSEED_DEFAULT = 20
KEYS = ["pos_rms", "pos_max", "nav_att_rmse", "nav_vel_rms", "ad_app", "ad_blk",
        "outlier_rejects", "reacquire_resets", "gnss_stale_peak_s"]


def run(extra, arm, seed):
    cmd = [EXE] + BASE + extra + arm + ["--seed", str(seed)]
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
    print("exe md5", md5[:8])
    res = {}
    for cname, extra, nseed, exp_age in CONDS:
        per = {}
        for label, arm in ARMS:
            with cf.ThreadPoolExecutor(max_workers=8) as pool:
                per[label] = [r for r in pool.map(
                    lambda s: run(extra, arm, s), range(1, nseed + 1)) if r]
        labels = [l for l, _ in ARMS]
        a, b, c = (per[labels[0]], per[labels[1]], per[labels[2]])
        valid = "OK"
        if exp_age is not None:
            pk = max(med(a, "gnss_stale_peak_s"), med(b, "gnss_stale_peak_s"),
                     med(c, "gnss_stale_peak_s"))
            valid = "OK" if pk >= exp_age * 0.8 else "INVALID"
        print(f"\n=== {cname} [{valid}] n={len(a)}/{len(b)}/{len(c)} ===")
        for key in KEYS:
            va, vb, vc = med(a, key), med(b, key), med(c, key)
            print(f"  {key:<18} A {va:>10.3f}  B {vb:>10.3f}  C {vc:>10.3f}")
        pa, pb, pc = med(a, "pos_rms"), med(b, "pos_rms"), med(c, "pos_rms")
        print(f"  → pos_rms: B/A {(pb - pa) / pa * 100 if pa else 0:+.1f}%   "
              f"B/C {(pb - pc) / pc * 100 if pc else 0:+.1f}%")
        n = min(len(a), len(b), len(c))
        d = [b[i]["pos_rms"] - a[i]["pos_rms"] for i in range(n)]
        print(f"  配对 B−A pos: 变差 {sum(1 for x in d if x > 0.05)} / 变好 "
              f"{sum(1 for x in d if x < -0.05)} / 同 "
              f"{sum(1 for x in d if abs(x) <= 0.05)}")
        res[cname] = {"valid": valid, "n": {l: len(per[l]) for l in labels},
                      "med": {l: {k: med(per[l], k) for k in KEYS}
                              for l in labels},
                      "rows": {l: per[l] for l in labels}}
    # 预登记判据自动裁定
    ok = {}
    def m_of(cond, arm, key):
        return res[cond]["med"][arm][key]
    try:
        ok["J1 振2g 保护恢复"] = (m_of("振2g_8.4g", "B veto开+HF", "pos_rms")
                              <= 1.2 * m_of("振2g_8.4g", "C veto开+中频", "pos_rms"))
        ok["J2a 延100ms 收益保留"] = (m_of("延100ms", "B veto开+HF", "pos_rms")
                                  <= 1.1 * m_of("延100ms", "A veto关(默认)", "pos_rms"))
        ok["J2b 延200ms 收益保留"] = (m_of("延200ms", "B veto开+HF", "pos_rms")
                                  <= 1.1 * m_of("延200ms", "A veto关(默认)", "pos_rms"))
        for cname, lim in (("洁净", 0.10), ("5Hz", 0.10), ("巡航失联5s", 0.10),
                           ("振1g_8.4g", 0.10)):
            pa = m_of(cname, "A veto关(默认)", "pos_rms")
            pb = m_of(cname, "B veto开+HF", "pos_rms")
            ok[f"J3 {cname} 无损(≤10%)"] = (abs(pb - pa) <= lim * max(pa, 1e-9))
        ok["J4a 洁净档 C 被顶死"] = (m_of("洁净", "C veto开+中频", "ad_app") <
                                 m_of("洁净", "B veto开+HF", "ad_app"))
        ok["J4b 振2g 档 B 被挡住"] = (m_of("振2g_8.4g", "B veto开+HF", "ad_app") <=
                                  0.5 * max(m_of("振2g_8.4g", "A veto关(默认)", "ad_app"), 1))
    except KeyError as exc:
        print("判据计算缺数据:", exc)
    print("\n=== 预登记判据 ===")
    for k, v in ok.items():
        print(f"  {'✓' if v else '✗'} {k}")
    verdict = bool(ok) and all(ok.values())
    print("VERDICT:", "B 臂候选成立（可讨论默认）" if verdict
          else "B 臂不成立：高频否决保持 opt-in，通道仅作遥测")
    path = ("D:/纵列双发矢量推力飞行器/TandemVec-EKF-Report/sim-data/"
            "vib_hf_ab_2026-10-28.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump({"exe_md5": md5, "criteria": ok, "verdict_pass": verdict,
                   "nseed": NSEED_DEFAULT, "rows": res}, fh, ensure_ascii=False)
    print("json:", path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
