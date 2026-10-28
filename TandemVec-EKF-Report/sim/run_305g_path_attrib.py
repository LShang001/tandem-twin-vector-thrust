# -*- coding: utf-8 -*-
"""30.5 g / 32 g 在生产传感器路径下的退化归因：LPF × AHRS 2×2 + 20 Hz 对照。

@file    run_305g_path_attrib.py
@brief   同一 32 g 二进制，用 `--no-imu-lpf` / `--no-sil-ahrs` 两个运行时开关
         做 2×2 因子分解，判断 §4.15/§4.16 的"30.5 g 中位 6.79 m、7/50 尾部"
         到当前 HEAD 变成"中位 ~23 m、50/50 系统离群"的退化是否由生产传感器
         路径（2 kHz 软件 LPF + 真实 Madgwick）引起
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 起因：§4.15/§4.16 的 30.5 g 群体证据（`sil32g_velrec_rel8`，二进制
         `c95db271…`，中位 6.787 m / p95 37.94 m / 尾部 7/50 / 中位 0 拒帧）
         产出于**生产传感器路径更正（§4.9/§4.10）之前**——当时 SIL 既没有
         2 kHz 软件 LPF，也没有链接真实 Madgwick。同一天报告里其余导弹数字
         已经因这个原因作废重测，唯独 30.5 g 群体没有重跑。本轮把它补齐。

         噪声序列不受这两个开关影响（LPF/AHRS 不消耗 RNG），因此四臂是严格
         同 seed 配对；A/B 用同一二进制 runtime 开关。

预登记判据（跑前写死，跑后不改口径）：
  F1 复现锚：臂 D（LPF 关 + AHRS 关 = 旧口径）中位 pos_rms 应回到 6.79 m 的
     ±20% 且中位拒帧 0。若不成立 ⇒ 退化不是生产路径造成的，本归因作废，
     必须先定位真正来源。
  F2 LPF 份额：臂 B（关 LPF、留 AHRS）。若 B≈A ⇒ LPF 不是驱动项；若 B≈D ⇒ LPF 是驱动项。
  F3 AHRS 份额：臂 C（留 LPF、关 AHRS），判读同 F2。
  F4 交互：若 B、C 都接近 A 而只有 D 接近 7 m ⇒ 两通道共同作用（LPF 群延迟
     改变 AHRS 收敛节奏），不能单独归因给某一个。
  F5 20 Hz：臂 E（默认路径 + 20 Hz）。若中位 ≤10 m ⇒ §4.15"≥20 Hz 可消除尾部"
     在生产路径下仍成立；若仍 ~20 m ⇒ 该建议必须降级为"生产路径下 20 Hz 不够"。
  F6 修复候选（导航增量绕过控制 LPF）：臂 G 中位 pos_rms ≤ 0.5 × A 且拒帧中位 0
     ⇒ 证实"加计群延迟 a·τ（300 m/s² × ~25 ms ≈ 7.5 m/s）是主因、绕过 LPF 有效"；
     不成立则机制假设需要修正（例如实为削顶/幅值衰减而非延迟）。
  F7 代价复核：臂 I（绕过 LPF + 注入 200 ms 真实接收机延迟）相对臂 J（默认 + 同延迟）
     不得更差。若更差 ⇒ 绕过 LPF 只是**有条件**的修复（无接收机延迟时才赚），
     必须做过载/延迟自适应或先标定延迟，不能无条件写进默认。
  F8 群延迟剔除（解析值 24.5 ms，α=0.02@2 kHz）：臂 M 相对 A 中位下降 ≥50% 且拒帧
     归零 ⇒ 支持"τ̂ 含 IMU 群延迟、回放时过补偿"这一机制；若几乎不降 ⇒ 剔除量与
     实际膨胀不匹配，转 F9。
  F9 匹配剔除（38.5 ms = 逐历元实测 τ̂ 膨胀 63.8−25.8）：臂 N 若回到 B 量级（≈5 m）
     ⇒ 机制闭合。但**常数只在被标定的动态下成立**（匀速段群延迟≈0，常数剔除会
     反向过剔除），因此默认仍保持 0；正确的结构性修复是让估计器的纯 IMU 参考改用
     未滤波增量（本轮未做，登记为下一步）。若 N 仍 >10 m ⇒ 机制未闭合，不得写"已修复"。
  F10 网格放宽在高过载大延迟格：臂 K（生产路径 + 200 ms + 网格/CAP 300 ms）相对 J
     若中位下降 ≥50% ⇒ §4.23 的放宽在 30 g 格也有收益；若逐位相同 ⇒ 该格估计器根本
     未参与（ad_app=0），放宽的收益范围仅限"估计器被应用"的格。
  边界：全部 L2 SIL，不写实机；两开关只是研究用的反事实口径，固件生产配置
     始终是 LPF+Madgwick 全开。
"""
import argparse
import concurrent.futures as cf
import hashlib
import json
import math
import os
import re
import statistics as st
import subprocess

import psutil

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.normpath(os.path.join(HERE, "..", "sim-data"))
JRE = re.compile(r"SIL_REAL_JSON (\{.*\})")

BASE = ["--real-nav", "--missile", "--seconds", "45", "--imu-fs-g", "32",
        "--imu-health-accel-range", "313.8",
        "--ms-thrust", "23900", "--profile", "mag_clean", "--imu-grade", "icm"]

ARMS = [
    ("A_prod", ["--gnss-rate-hz", "10"]),                   # 生产路径（LPF+AHRS 全开）
    ("B_nolpf", ["--gnss-rate-hz", "10", "--no-imu-lpf"]),
    ("C_noahrs", ["--gnss-rate-hz", "10", "--no-sil-ahrs"]),
    ("D_neither", ["--gnss-rate-hz", "10", "--no-imu-lpf", "--no-sil-ahrs"]),
    ("E_hz20", ["--gnss-rate-hz", "20"]),                   # 生产路径 + 20 Hz
    ("F_hz20_nolpf", ["--gnss-rate-hz", "20", "--no-imu-lpf"]),
    # 候选修复：导航增量绕过控制 LPF（消除加计群延迟 a·τ 项）
    ("G_navraw", ["--gnss-rate-hz", "10", "--nav-raw-imu"]),
    ("H_navraw_hz20", ["--gnss-rate-hz", "20", "--nav-raw-imu"]),
    # 已知代价复核：绕过 LPF 后在真实接收机延迟下是否仍变差（§4.9 旧负结果）
    ("I_navraw_lat200", ["--gnss-rate-hz", "10", "--nav-raw-imu",
                         "--gnss-latency-ms", "200"]),
    ("J_prod_lat200", ["--gnss-rate-hz", "10", "--gnss-latency-ms", "200"]),
    # 与 §4.23 网格放宽的交汇：30.5 g + 真延 200 ms 时，估计器被 150 ms 网格挡住
    # （ad_app=0/ad_blk=38、位置 86 m），放宽网格+CAP 到 300 ms 能不能救？
    ("K_prod_lat200_wide", ["--gnss-rate-hz", "10", "--gnss-latency-ms", "200",
                            "--delay-grid-max-ms", "300", "--delay-cap-ms", "300"]),
    ("L_navraw_lat200_wide", ["--gnss-rate-hz", "10", "--gnss-latency-ms", "200",
                              "--nav-raw-imu",
                              "--delay-grid-max-ms", "300", "--delay-cap-ms", "300"]),
    # 原 M_gd245 / N_gd385（应用侧常数剔除 IMU 群延迟）已试并**否证撤回**（§4.24b）：
    # 拒帧 21→0 但位置 16.85→24.1 m，且代价对"应用量"非单调 ⇒ 接缝与 CLI 已删。
]

KEYS = ["pos_rms", "pos_max", "rms_pitch", "max_roll", "outlier_rejects",
        "reacquire_resets", "vel_sub_rec", "ad_tau_ms", "ad_app", "ad_blk"]


def run_one(exe, arm, seed, nseed, log):
    extra = dict(ARMS).get(arm, [])
    cmd = [exe] + BASE + ["--seed", str(seed)] + list(extra)
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                         text=True)
    try:
        proc = psutil.Process(p.pid)
        if hasattr(proc, "cpu_affinity"):
            for _ in range(50):
                proc.cpu_affinity([CORE])
                if proc.cpu_affinity() == [CORE]:
                    break
    except Exception:
        pass
    out, err = p.communicate(timeout=900)
    m = JRE.findall(out + err)
    if not m:
        log.write(f"[{arm} seed={seed}] NO JSON rc={p.returncode}\n")
        log.write(out[-1200:] + "\n" + err[-600:] + "\n")
        return None
    d = json.loads(m[-1].replace("nan", "NaN").replace("inf", "Infinity"))
    d["arm"], d["seed"] = arm, seed
    return d


CORE = 2


def pct(vals, q):
    """线性插值分位数（vals 已升序）。"""
    if not vals:
        return float("nan")
    pos = q * (len(vals) - 1)
    lo = int(pos)
    hi = min(lo + 1, len(vals) - 1)
    return vals[lo] + (pos - lo) * (vals[hi] - vals[lo])


def stats(rows):
    s = {"n": len(rows)}
    for k in KEYS:
        vals = [r[k] for r in rows if isinstance(r.get(k), (int, float))
                and math.isfinite(r[k])]
        if not vals:
            continue
        vals.sort()
        s[k + "_med"] = round(st.median(vals), 3)
        s[k + "_mean"] = round(sum(vals) / len(vals), 3)
        s[k + "_p95"] = round(pct(vals, 0.95), 3)
        s[k + "_max"] = round(vals[-1], 3)
    pr = [r.get("pos_rms", 0.0) for r in rows]
    s["n_tail_gt25m"] = sum(1 for x in pr if x > 25.0)
    s["n_tail_gt15m"] = sum(1 for x in pr if x > 15.0)
    s["n_reject_gt0"] = sum(1 for r in rows if r.get("outlier_rejects", 0) > 0)
    s["bad_nav"] = sum(1 for r in rows if not r.get("nav_init", 0))
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", default="/tmp/tv_32g/fw_real.exe")
    ap.add_argument("--seeds", type=int, default=50)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--tag", default="2026-10-28")
    ap.add_argument("--only", default="",
                    help="只跑这些臂（逗号分隔），配对参考取其中第一个存在的臂")
    ap.add_argument("--base-extra", default="",
                    help="追加到 BASE 的 token（SIL 同名参数后出现者生效，可换弹道档）")
    args = ap.parse_args()
    if args.base_extra:
        BASE.extend(args.base_extra.split())

    arms = ([a for a in ARMS if a[0] in set(args.only.split(","))]
            if args.only else list(ARMS))

    exe = args.exe.replace("/tmp/", os.path.join(
        os.environ.get("LOCALAPPDATA", ""), "Temp") + "/")
    md5 = hashlib.md5(open(exe, "rb").read()).hexdigest()
    out_jsonl = os.path.join(DATA, f"attrib305_{args.tag}.jsonl")
    open(out_jsonl, "w", encoding="utf-8").close()
    jobs = [(a, s) for a, _ in arms for s in range(1, args.seeds + 1)]
    rows = []
    with open(out_jsonl + ".log", "w", encoding="utf-8") as log:
        log.write(f"exe={exe} md5={md5} core={CORE} jobs={len(jobs)}\n")
        with cf.ThreadPoolExecutor(max_workers=args.workers) as pool:
            futs = {pool.submit(run_one, exe, a, s, args.seeds, log): (a, s)
                    for a, s in jobs}
            done = 0
            for f in cf.as_completed(futs):
                d = f.result()
                done += 1
                if done % 25 == 0:
                    log.write(f"done {done}/{len(jobs)}\n")
                    log.flush()
                if d:
                    rows.append(d)
                    with open(out_jsonl, "a", encoding="utf-8") as fh:
                        fh.write(json.dumps(d, ensure_ascii=False) + "\n")

    summary = {"exe": exe, "md5": md5, "core": CORE, "seeds": args.seeds,
               "arms": {}}
    for a, _ in arms:
        sel = [r for r in rows if r["arm"] == a]
        if sel:
            summary["arms"][a] = stats(sel)
    # 逐 seed 配对差（相对参考臂，默认 A_prod，否则取选中臂的第一个）
    ref = "A_prod" if any(a == "A_prod" for a, _ in arms) else arms[0][0]
    by = {(r["arm"], r["seed"]): r for r in rows}
    for a, _ in arms:
        if a == ref:
            continue
        diffs = [by[(a, s)]["pos_rms"] - by[(ref, s)]["pos_rms"]
                 for s in range(1, args.seeds + 1)
                 if (a, s) in by and (ref, s) in by]
        if diffs:
            diffs.sort()
            summary["arms"][a][f"paired_delta_vs_{ref}"] = {
                "med": round(st.median(diffs), 3),
                "mean": round(sum(diffs) / len(diffs), 3),
                "min": round(diffs[0], 3), "max": round(diffs[-1], 3),
                "n_improved": sum(1 for x in diffs if x < -0.05),
                "n_worse": sum(1 for x in diffs if x > 0.05),
            }
    with open(os.path.join(DATA, f"attrib305_{args.tag}.json"), "w",
              encoding="utf-8") as fh:
        json.dump(summary, fh, ensure_ascii=False, indent=1)
    print(json.dumps(summary["arms"], ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
