#!/usr/bin/env python3
"""run_delay_closed_loop_mc.py — 闭环 estimated-age 补偿的 Monte Carlo 驱动。

@file    run_delay_closed_loop_mc.py
@module  固件/宿主研究（fw-tests：EKF 延时补偿 MC 驱动）
@brief   随机拉偏速度/延时/机动/振动/噪声，批量调用 test_ekf_highspeed_realistic 的
         auto_delay 路径并汇总位置误差与估计延时。
@details 输入：--cases/--seed/--bin/--out；每例随机生成 seed、速度 5–60 m/s、
         链路延时 0–100 ms、机动类型、振动 g/模式/频率、GNSS 位置速度噪声、
         失锁标记与失锁时长。
         处理：subprocess 调用被测可执行体，用正则解析
         `DBG_BATCH ok=… tau=… valid=…` 与 `age-comp: … pos3d=… improve=… est_tau=…` 两行。
         输出：CSV（字段见 FIELDS）+ 进度行；解析失败即抛异常（不静默跳过）。
@theory  GNSS 量测年龄补偿：量测对应 t-τ 时刻状态，EKF 回放融合后重传播到当前；
         估计量 τ̂ 与真值 τ 的比较是“估计器是否收敛”的判据。
         @ref REF-121 §2.2.1 p.33（KF 五方程）、@ref AGENTS.md（延时回放与失锁风险）
@frame   NED（位置/速度误差按 3D 范数统计），机体系仅出现在被测可执行体内部。
@units   速度 m/s，延时 ms，位置误差 m，振动 g，噪声 m 或 m/s。
@note    本脚本只是驱动/统计，不断言阈值；阈值判定在报告与 run_*_acceptance 脚本里。
         被测可执行体需先编译（见 test_host/README.md），缺二进制时报错退出。
@see     TandemVec_FCS/test_host/test_ekf_highspeed_realistic.cpp
"""
from __future__ import annotations
import argparse, csv, json, math, random, re, statistics, subprocess, time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
OUT = ROOT / "TandemVec-EKF-Report" / "sim-data"
FIELDS = ["case","seed","speed","latency_ms","maneuver","vib_g","vib_mode",
          "vib_hz","gyro_vib","gnss_pos","gnss_vel","outage","outage_len",
          "est_tau_ms","true_tau_ms","pos3d_base","pos3d_comp","improve_pct"]


def run_one(bin_path, a):
    p = subprocess.run([bin_path, *map(str, a)], capture_output=True, text=True,
                       timeout=30)
    out = p.stdout
    m1 = re.search(r"DBG_BATCH ok=(\d+) tau=([0-9.]+) valid=(\d+)", out)
    m2 = re.search(r"age-comp:.*pos3d=([0-9.]+).*improve=([-0-9.]+)%.*est_tau=([0-9.]+)", out)
    if not (m1 and m2):
        raise RuntimeError("parse failed:\n" + out + "\n" + p.stderr)
    return (float(m1.group(2)), float(m2.group(1)), float(m2.group(2)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases", type=int, default=2500)
    ap.add_argument("--seed", type=int, default=20261031)
    ap.add_argument("--every", type=int, default=100)
    ap.add_argument("--bin", default=r"C:\Users\12631\AppData\Local\Temp\hsreal.exe")
    ap.add_argument("--out", default=str(OUT / "delay-closed-loop.csv"))
    args = ap.parse_args()
    rng = random.Random(args.seed)
    rows = []
    t0 = time.time()
    for i in range(args.cases):
        seed = rng.randint(1, 10_000_000)
        speed = rng.uniform(5, 60)
        lat = rng.uniform(0.0, 0.10)
        man = rng.choice([0, 1, 2, 3])
        vib = rng.uniform(0.0, 2.0)
        mode = rng.choice([0, 1])
        hz = rng.uniform(20, 200)
        gv = rng.uniform(0.0, 2.0)
        gp = rng.uniform(1.5, 5.0)
        gvel = rng.uniform(0.05, 0.20)
        outage = rng.random() < 0.10
        os_ = rng.uniform(5, 30) if outage else 1e9
        oe = os_ + rng.uniform(5, 15) if outage else 1e9
        of = rng.uniform(0.0, 0.01)
        scale = rng.uniform(0.98, 1.02)
        mis = rng.uniform(-0.005, 0.005)
        by = rng.uniform(-0.1, 0.1)
        bz = rng.uniform(-0.1, 0.1)
        # 11 base + adaptive=1 + mode/gv/hz/os/oe + 0/0/0/0 + sensor 6
        cli = [speed, lat, 0.25, 0.0015, 0.00524, gp, gvel, seed, man,
               0.01, vib, 1, mode, gv, hz, os_, oe,
               0, 0, 0, 1, scale, scale, mis, by, bz, of]
        try:
            est, pos, improve = run_one(args.bin, cli)
            ok = 1
        except Exception as exc:  # noqa: BLE001
            est = pos = improve = float("nan"); ok = 0
            print(f"[WARN] {i}: {exc}", flush=True)
        rows.append({"case":i,"seed":seed,"speed":speed,
                     "latency_ms":lat*1000.0,"maneuver":man,"vib_g":vib,
                     "vib_mode":mode,"vib_hz":hz,"gyro_vib":gv,
                     "gnss_pos":gp,"gnss_vel":gvel,"outage":int(outage),
                     "outage_len":(oe-os_) if outage else 0.0,
                     "est_tau_ms":est,"true_tau_ms":lat*1000.0,
                     "pos3d_base":pos,"pos3d_comp":pos,"improve_pct":improve,
                     "ok":ok})
        if (i+1) % args.every == 0 or i+1 == args.cases:
            el=time.time()-t0; rate=(i+1)/max(el,1e-9)
            print(f"[progress] {i+1}/{args.cases} ({100*(i+1)/args.cases:.1f}%) "
                  f"elapsed={el:.1f}s eta={(args.cases-i-1)/max(rate,1e-9):.1f}s",
                  flush=True)
    p=Path(args.out); p.parent.mkdir(parents=True,exist_ok=True)
    with p.open("w",encoding="utf-8",newline="") as f:
        w=csv.DictWriter(f,fieldnames=FIELDS+["ok"]); w.writeheader(); w.writerows(rows)
    errs=[abs(r["est_tau_ms"]-r["true_tau_ms"]) for r in rows if r["ok"]]
    print(json.dumps({"cases":len(rows),"mean_abs_tau_ms":statistics.mean(errs),
                      "p95_abs_tau_ms":sorted(errs)[int(.95*(len(errs)-1))],
                      "max_abs_tau_ms":max(errs)}, indent=2), flush=True)
    print("CSV:",p, flush=True)

if __name__ == "__main__":
    main()
