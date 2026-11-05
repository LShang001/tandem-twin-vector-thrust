# -*- coding: utf-8 -*-
"""大过载/大机动卫惯组合导航矩阵（真实固件 Ekf15State 宿主台架）。
@file    run_high_dynamic_mc.py
@brief   大过载/大机动 EKF 宿主台架矩阵。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 驱动 test_ekf_high_dynamic.exe，按机动/负载/延迟/失联/杆臂组合采样。
@frame   N/A（只采集宿主台架 JSON 的 pos / att 误差标量，脚本不做坐标变换）
@units   m（位置）、°（姿态角）；NEES 为归一化比值

用法：
  HIGHDYN_EXE='C:/.../highdyn.exe' py -3.12 sim/run_high_dynamic_mc.py
"""
import concurrent.futures as cf
import json, os, re, subprocess
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TAG = os.environ.get("HIGHDYN_TAG", "2026-10-26")
OUT = os.path.join(HERE, "..", "sim-data", f"high-dyn-mc-{TAG}.jsonl")
SUM = os.path.join(HERE, "..", "sim-data", f"high-dyn-mc-{TAG}.json")
EXE = os.environ.get("HIGHDYN_EXE", "/tmp/highdyn.exe")
WORKERS = int(os.environ.get("HIGHDYN_WORKERS", "8"))
JSON_RE = re.compile(r"HIGH_DYN_JSON (\{.*\})")
SEEDS = list(range(1, int(os.environ.get("HIGHDYN_SEEDS", "11"))))


def case(profile, load, speed=80.0, rate=0.0, duration=40.0,
         latency=20.0, outage=None, a_scale=1.0, g_scale=1.0,
         misalign=0.0, lever=0.0, lever_comp=0, vib_g=0.0, extra=None):
    """构造一条台架用例的参数字典（未显式给出的维度取默认值）。

    参数：profile 机动剖面（turn/sturn/pull/roll/coning/negg）；load 过载 g；
    speed 速度 m/s；rate 角速率 °/s；latency GNSS 延迟 ms；outage 失联窗 (t0,t1) s；
    a_scale/g_scale 加计/陀螺尺度因子误差；misalign 失准角 deg；lever 杆臂 m 与
    lever_comp 是否补偿；vib_g 振动幅值 g；extra 追加的命令行参数。
    """
    return dict(profile=profile, load=load, speed=speed, rate=rate,
                duration=duration, latency=latency, outage=outage,
                a_scale=a_scale, g_scale=g_scale, misalign=misalign,
                lever=lever, lever_comp=lever_comp, vib_g=vib_g,
                extra=extra or [])


def build_cases():
    """按矩阵维度展开全部用例并给每条附上 seed，返回用例列表。

    覆盖：协调转弯/S 转弯/拉杆-推杆的 2–8 g × 80/150 m/s、连续滚转 180–1080 °/s、
    圆锥运动 120–900 °/s、负 g 专项、5/20 s 失联、IMU 尺度+失准 4 档、GNSS 延迟
    20–500 ms、杆臂 0.5/1.0 m（补偿/不补偿）、振动 1/2 g、±8 g 量程下的 9/10/12 g
    饱和边界。
    """
    cs = []
    # 基线：协调转弯/S 转弯/拉杆-推杆，2–8 g × 80/150 m/s
    for profile in ("turn", "sturn", "pull"):
        for load in (2.0, 4.0, 6.0, 8.0):
            for speed in (80.0, 150.0):
                for seed in SEEDS:
                    cs.append(dict(case(profile, load, speed), seed=seed))
    # 连续滚转：2/4 g × 180–1080 °/s
    for load in (2.0, 4.0):
        for rate in (180.0, 360.0, 720.0, 1080.0):
            for seed in SEEDS:
                cs.append(dict(case("roll", load, 80.0, rate), seed=seed))
    # 圆锥运动：120–900 °/s
    for rate in (120.0, 300.0, 600.0, 900.0):
        for seed in SEEDS:
            cs.append(dict(case("coning", 1.0, 80.0, rate), seed=seed))
    # 负 g 专项（姿态二义性边界）
    for load in (2.0, 4.0, 6.0):
        for seed in SEEDS:
            cs.append(dict(case("negg", load), seed=seed))
    # 失联：5/20 s，覆盖转弯/滚转/圆锥/纯正拉杆/负 g
    for prof, load, rate in (("turn", 4.0, 0.0), ("turn", 8.0, 0.0),
                             ("sturn", 4.0, 0.0), ("pull", 4.0, 0.0),
                             ("pull", 6.0, 0.0), ("negg", 4.0, 0.0),
                             ("roll", 2.0, 720.0), ("coning", 1.0, 600.0)):
        for outage in ((15.0, 20.0), (15.0, 35.0)):
            for seed in SEEDS:
                cs.append(dict(case(prof, load, 80.0, rate, duration=55.0,
                                    outage=outage), seed=seed))
    # IMU 尺度/失准：4 档，20 s 失联，6 个机动
    grades = ((1.0, 1.0, 0.0), (1.002, 1.001, 0.1),
              (1.005, 1.002, 0.2), (1.010, 1.005, 0.5))
    for prof, load, rate in (("turn", 6.0, 0.0), ("turn", 8.0, 0.0),
                             ("sturn", 6.0, 0.0), ("pull", 6.0, 0.0),
                             ("negg", 4.0, 0.0),
                             ("roll", 2.0, 720.0), ("coning", 1.0, 600.0)):
        for a_s, g_s, mis in grades:
            for seed in SEEDS:
                cs.append(dict(case(prof, load, 80.0, rate, duration=55.0,
                                    outage=(15.0, 35.0), a_scale=a_s,
                                    g_scale=g_s, misalign=mis), seed=seed))
    # GNSS 延迟 20–500 ms：机动 + 高速
    for prof, load in (("turn", 6.0), ("sturn", 6.0), ("pull", 6.0)):
        for speed in (80.0, 150.0):
            for lat in (20.0, 100.0, 200.0, 300.0, 330.0, 500.0):
                for seed in SEEDS:
                    cs.append(dict(case(prof, load, speed, latency=lat),
                                   seed=seed))
    # 杆臂：0.5/1.0 m，补偿/不补偿
    for prof, load, rate in (("roll", 2.0, 360.0), ("roll", 2.0, 720.0),
                             ("coning", 1.0, 600.0), ("turn", 6.0, 0.0)):
        for lever in (0.5, 1.0):
            for comp in (0, 1):
                for seed in SEEDS:
                    cs.append(dict(case(prof, load, 80.0, rate, lever=lever,
                                        lever_comp=comp), seed=seed))
    # 振动
    for prof, load, rate in (("turn", 8.0, 0.0), ("roll", 2.0, 720.0)):
        for vib in (1.0, 2.0):
            for seed in SEEDS:
                cs.append(dict(case(prof, load, 80.0, rate, vib_g=vib),
                               seed=seed))
    # 传感器饱和边界：±8 g 量程下的 9/10/12 g 指令
    for load in (9.0, 10.0, 12.0):
        for seed in SEEDS:
            cs.append(dict(case("turn", load, 80.0, duration=30.0), seed=seed))
    return cs


def one(c):
    """把一条用例展成 test_ekf_high_dynamic.exe 的命令行并执行（180 s 超时）。

    返回：解析出的 HIGH_DYN_JSON 结果字典，并回填该用例的全部参数便于分组；
    没抓到 JSON 时返回带 no_json=True 的字典——计入失败列，不静默丢弃。
    """
    cmd = [EXE, "--profile", c["profile"], "--load", str(c["load"]),
           "--speed", str(c["speed"]), "--rate-dps", str(c["rate"]),
           "--duration", str(c["duration"]), "--latency-ms", str(c["latency"]),
           "--accel-scale", str(c["a_scale"]), "--gyro-scale", str(c["g_scale"]),
           "--misalign-deg", str(c["misalign"]), "--lever-m", str(c["lever"]),
           "--lever-comp", str(c["lever_comp"]), "--vib-g", str(c["vib_g"]),
           "--seed", str(c["seed"]), "--json"]
    if c["outage"] is not None:
        cmd += ["--outage", f'{c["outage"][0]:g}:{c["outage"][1]:g}']
    cmd += c["extra"]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    mm = JSON_RE.findall(r.stdout + r.stderr)
    if not mm:
        d = dict(c)
        d.update(no_json=True)
        return d
    d = json.loads(mm[-1].replace("nan", "NaN").replace("inf", "Infinity"))
    d.update(c)
    return d


def stat(v):
    """对指标序列求 n / 中位 / p90 / p99 / 最大值，先剔除非数值与非有限值；
    全空时只返回 {"n": 0}（不给假的 0 值统计）。"""
    v = np.array([x for x in v if isinstance(x, (int, float)) and np.isfinite(x)], float)
    if not len(v):
        return dict(n=0)
    return dict(n=int(len(v)), med=float(np.median(v)), p90=float(np.percentile(v, 90)),
                p99=float(np.percentile(v, 99)), mx=float(v.max()))


def main():
    """命令行入口：展开矩阵 → 进程池并行跑（进程数取 HIGHDYN_WORKERS）→ 逐行写
    sim-data/high-dyn-mc-<TAG>.jsonl，再汇总全局与 (剖面×过载×角速率×速度) 分组
    统计，落盘同名 .json 并打印用例数、失败数与位置/姿态误差关键分位。

    EXE / TAG / SEEDS / WORKERS 均从环境变量取，便于换构建产物复跑。
    """
    cases = build_cases()
    print("cases", len(cases), flush=True)
    rows = []
    with cf.ProcessPoolExecutor(max_workers=WORKERS) as ex:
        for i, r in enumerate(ex.map(one, cases, chunksize=4)):
            rows.append(r)
            if (i + 1) % 100 == 0:
                print("done", i + 1, flush=True)
    with open(OUT, "w", encoding="utf-8") as f:
        for d in rows:
            f.write(json.dumps(d, ensure_ascii=False) + "\n")
    ok = [r for r in rows if not r.get("no_json")]
    summary = dict(
        n_cases=len(rows), n_ok=len(ok), no_json=len(rows) - len(ok),
        fail=int(sum(r.get("fail", 0) for r in ok)),
        pos_rmse=stat([r["pos_rmse"] for r in ok]),
        pos_max=stat([r["pos_max"] for r in ok]),
        att_rmse=stat([r["att_rmse_deg"] for r in ok]),
        att_max=stat([r["att_max_deg"] for r in ok]),
        nees_pos=stat([r["nees_pos_med"] for r in ok]),
        fused=stat([r["fused"] for r in ok]),
        rejected=stat([r["rejected"] for r in ok]))
    # 分组：profile × load/rate
    groups = {}
    for r in ok:
        key = f'{r["profile"]}_L{r["load"]:g}_R{r["rate"]:g}_V{r["speed"]:g}'
        groups.setdefault(key, []).append(r)
    gs = {}
    for key, rs in groups.items():
        gs[key] = dict(n=len(rs),
                       pos_rmse=stat([r["pos_rmse"] for r in rs]),
                       pos_max=stat([r["pos_max"] for r in rs]),
                       att_rmse=stat([r["att_rmse_deg"] for r in rs]),
                       att_max=stat([r["att_max_deg"] for r in rs]),
                       fail=int(sum(r.get("fail", 0) for r in rs)))
    summary["groups"] = gs
    json.dump(summary, open(SUM, "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)
    print(json.dumps({k: summary[k] for k in
                      ("n_cases", "n_ok", "no_json", "fail", "pos_rmse",
                       "pos_max", "att_rmse", "att_max", "nees_pos")},
                     ensure_ascii=False, indent=1))
    print("wrote", OUT)


if __name__ == "__main__":
    main()
