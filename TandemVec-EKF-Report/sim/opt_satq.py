# -*- coding: utf-8 -*-
"""饱和锁存参数（Q/enter/release）连续优化：CMA-ES、TPE、DE、Nelder-Mead。

@file    opt_satq.py
@brief   在真实固件 SIL 上对 `BFS_IMU_SATURATION_Q_INFLATE` 的三个连续参数做
          多优化器对比，目标以 10.1 g 位置为主，并对 8.4 g 姿态/航向退化设罚。
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 评估：直接调用 fw_real.exe --missile，解析 SIL_REAL_JSON；
          train seeds 1..N，另用独立 seeds 复核 top-5 防过拟合。
         输出 sim-data/satq_opt_2026-10-27.json。
@frame   NED
@units   m、m/s、°、s、m/s²
"""
import argparse
import concurrent.futures as cf
import json
import math
import os
import re
import subprocess
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "sim-data")
JSON_RE = re.compile(r"SIL_REAL_JSON (\{.*\})")
BASE = ["--real-nav", "--missile", "--seconds", "45", "--seed"]

CASES = [
    ("thrust10g_icm_full", ["--ms-thrust", "10000"]),
    ("thrust8g_icm_full", ["--ms-thrust", "8000"]),
    ("clean_icm_full", []),
]


def resolve_bin(path):
    if path and os.path.exists(path):
        return path
    if path and path.startswith("/tmp/"):
        alt = os.path.join(os.environ.get("TEMP", ""), path[5:])
        if os.path.exists(alt):
            return alt
    return path


def one(job):
    exe, seed, extra = job
    cmd = ([exe] + BASE + [str(seed), "--profile", "mag_clean",
                           "--imu-grade", "icm"] + extra)
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
    except subprocess.TimeoutExpired:
        return None
    mm = JSON_RE.findall(r.stdout + r.stderr)
    if not mm:
        return None
    try:
        return json.loads(mm[-1].replace("nan", "NaN").replace("inf", "Infinity"))
    except json.JSONDecodeError:
        return None


def med(vals):
    v = [x for x in vals if isinstance(x, (int, float)) and np.isfinite(x)]
    return float(np.median(v)) if v else float("nan")


def summarize(rows):
    return dict(
        pos=med([r["pos_rms"] for r in rows]),
        att=med([r["nav_att_rmse"] for r in rows]),
        yaw=med([r["nav_yaw_rmse"] for r in rows]),
        vel=med([r["nav_vel_rms"] for r in rows]),
    )


class Evaluator:
    def __init__(self, exe, off_exe, workers, seeds, cache_path):
        self.exe = resolve_bin(exe)
        self.off_exe = resolve_bin(off_exe)
        self.workers = workers
        self.seeds = list(range(1, seeds + 1))
        self.cache = {}
        self.cache_path = cache_path
        self.n_eval = 0
        self.history = []
        if os.path.exists(cache_path):
            d = json.load(open(cache_path, encoding="utf-8"))
            self.cache = d.get("cache", {})
            self.baseline = d.get("baseline")
        else:
            self.baseline = None
        if not self.baseline:
            self.baseline = self._run_cases(self.off_exe, [], self.seeds)
            self._save()

    def _run_cases(self, exe, extra_prefix, seeds):
        jobs = []
        for cfg, extra in CASES:
            for s in seeds:
                jobs.append((exe, s, extra_prefix + extra))
        out = {}
        with cf.ThreadPoolExecutor(max_workers=self.workers) as pool:
            rows = list(pool.map(one, jobs))
        i = 0
        for cfg, _ in CASES:
            seg = rows[i:i + len(seeds)]
            i += len(seeds)
            out[cfg] = summarize([r for r in seg if r])
        return out

    def _save(self):
        json.dump(dict(baseline=self.baseline, cache=self.cache,
                       history=self.history),
                  open(self.cache_path, "w", encoding="utf-8"),
                  ensure_ascii=False, indent=1)

    def key(self, params):
        q, enter, release = params
        return f"{q:.3f}|{enter:.3f}|{release:.3f}"

    def objective(self, params):
        k = self.key(params)
        if k in self.cache:
            return self.cache[k]["J"]
        q, enter, release = params
        extra = ["--sat-q-inflate", f"{q:.4f}",
                 "--sat-q-enter", f"{enter:.4f}",
                 "--sat-q-release", f"{release:.4f}"]
        m = self._run_cases(self.exe, extra, self.seeds)
        a, b, c = (m["thrust10g_icm_full"], m["thrust8g_icm_full"],
                   m["clean_icm_full"])
        ba, bb, bc = (self.baseline["thrust10g_icm_full"],
                      self.baseline["thrust8g_icm_full"],
                      self.baseline["clean_icm_full"])
        lp = math.log10(max(a["pos"], 1.0))
        J = (lp + 0.5 * a["att"] + 0.5 * a["vel"]
             + 2.0 * max(0.0, b["att"] - bb["att"])
             + 2.0 * max(0.0, b["yaw"] - bb["yaw"])
             + 1.0 * max(0.0, b["vel"] - bb["vel"])
             + 0.2 * max(0.0, b["pos"] - bb["pos"])
             + 1.0 * max(0.0, c["att"] - bc["att"])
             + 0.2 * max(0.0, c["pos"] - bc["pos"])
             + 3.0 * max(0.0, lp - 1.0))
        self.n_eval += 1
        rec = dict(params=[q, enter, release], J=J, at=time.time(), metrics=m)
        self.cache[k] = rec
        self.history.append(rec)
        self._save()
        print(f"[eval {self.n_eval:3d}] J={J:.3f} Q={q:.2f} "
              f"enter={enter:.3f} release={release:.2f} "
              f"pos10={a['pos']:.2f} att10={a['att']:.2f} "
              f"att8={b['att']:.3f} yaw8={b['yaw']:.3f}", flush=True)
        return J

    def score(self, params):
        return self.objective(params)


def decode(x):
    x = np.clip(np.asarray(x, float), 0.0, 1.0)
    q = 10.0 ** (x[0] * 1.60206)          # 1 .. 40
    enter = 0.02 + x[1] * 0.48            # 0.02 .. 0.50 s
    release = 0.2 + x[2] * 2.8            # 0.2 .. 3.0 s
    return [float(q), float(enter), float(release)]


def run_cma(ev, maxfevals):
    import cma
    es = cma.CMAEvolutionStrategy([0.55, 0.15, 0.2], 0.22,
                                  {"popsize": 6, "bounds": [0, 1],
                                   "maxfevals": maxfevals, "verbose": -9,
                                   "seed": 11})
    hist = []
    while not es.stop():
        xs = es.ask()
        fs = [ev.score(decode(x)) for x in xs]
        es.tell(xs, fs)
        hist.append(min(fs))
    best = decode(es.result.xbest)
    return best, float(es.result.fbest), hist


def run_tpe(ev, n_trials):
    import optuna
    optuna.logging.set_verbosity(optuna.logging.WARNING)
    hist = []

    def obj(trial):
        x = [trial.suggest_float(f"x{i}", 0.0, 1.0) for i in range(3)]
        return ev.score(decode(x))

    study = optuna.create_study(
        direction="minimize",
        sampler=optuna.samplers.TPESampler(seed=17, n_startup_trials=12))
    study.optimize(obj, n_trials=n_trials,
                   callbacks=[lambda st, tr: hist.append(st.best_value)])
    return decode([study.best_params[f"x{i}"] for i in range(3)]), \
        float(study.best_value), hist


def run_de(ev, maxiter):
    from scipy.optimize import differential_evolution
    counter = {"n": 0}

    def obj(x):
        counter["n"] += 1
        return ev.score(decode(x))

    r = differential_evolution(
        obj, [(0, 1)] * 3, popsize=5, maxiter=maxiter, seed=23,
        polish=False, tol=0.0, init="sobol")
    return decode(r.x), float(r.fun), []


def run_nm(ev, x0, maxiter):
    from scipy.optimize import minimize

    def obj(x):
        return ev.score(decode(x))

    r = minimize(obj, np.asarray(x0), method="Nelder-Mead",
                 options=dict(maxiter=maxiter, xatol=0.01, fatol=0.005))
    return decode(r.x), float(r.fun), []


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", default="/tmp/tv_fcs_missile_satq_on/fw_real.exe")
    ap.add_argument("--off-bin", default="/tmp/tv_fcs_missile_satq_off/fw_real.exe")
    ap.add_argument("--workers", type=int, default=10)
    ap.add_argument("--seeds", type=int, default=6)
    ap.add_argument("--cma-fevals", type=int, default=120)
    ap.add_argument("--tpe-trials", type=int, default=80)
    ap.add_argument("--de-iter", type=int, default=10)
    ap.add_argument("--nm-iter", type=int, default=40)
    ap.add_argument("--out", default="satq_opt_2026-10-27")
    args = ap.parse_args()

    cache_path = os.path.join(DATA, args.out + ".json")
    ev = Evaluator(args.bin, args.off_bin, args.workers, args.seeds,
                   cache_path)
    print("baseline", json.dumps(ev.baseline, ensure_ascii=False), flush=True)
    t0 = time.time()
    results = {}
    best = None
    for name, fn in [
        ("cma", lambda: run_cma(ev, args.cma_fevals)),
        ("tpe", lambda: run_tpe(ev, args.tpe_trials)),
        ("de", lambda: run_de(ev, args.de_iter)),
    ]:
        t = time.time()
        p, f, hist = fn()
        results[name] = dict(params=p, J=f, hist=hist, wall_s=time.time() - t)
        print(f"== {name}: J={f:.3f} params={p} wall={time.time()-t:.0f}s",
              flush=True)
        if best is None or f < best[1]:
            best = (p, f, name)
    x0 = [0.0, 0.0, 0.0]
    x0[0] = math.log10(best[0][0]) / 1.60206
    x0[1] = (best[0][1] - 0.02) / 0.48
    x0[2] = (best[0][2] - 0.2) / 2.8
    p, f, _ = run_nm(ev, x0, args.nm_iter)
    results["nm"] = dict(params=p, J=f, hist=[], wall_s=0.0)
    print(f"== nm: J={f:.3f} params={p}", flush=True)
    if f < best[1]:
        best = (p, f, "nm")

    # 独立 seeds 复核 top-5（train 之外），防过拟合。
    ranked = sorted(ev.cache.values(), key=lambda r: r["J"])[:5]
    val = []
    test_seeds = list(range(11, 11 + max(4, args.seeds)))
    for rec in ranked:
        q, enter, release = rec["params"]
        extra = ["--sat-q-inflate", f"{q:.4f}",
                 "--sat-q-enter", f"{enter:.4f}",
                 "--sat-q-release", f"{release:.4f}"]
        m = ev._run_cases(ev.exe, extra, test_seeds)
        val.append(dict(params=rec["params"], train_J=rec["J"],
                        train_metrics=rec["metrics"], test_metrics=m))
        a, b = m["thrust10g_icm_full"], m["thrust8g_icm_full"]
        print(f"   val Q={q:.2f} e={enter:.3f} r={release:.2f} "
              f"pos10={a['pos']:.2f} att10={a['att']:.2f} "
              f"att8={b['att']:.3f} yaw8={b['yaw']:.3f}", flush=True)

    out = dict(baseline=ev.baseline, cache=ev.cache, results=results,
               best=dict(params=best[0], J=best[1], optimizer=best[2]),
               validation=val, train_seeds=ev.seeds, test_seeds=test_seeds,
               n_eval=ev.n_eval, wall_s=time.time() - t0,
               bin=args.bin)
    json.dump(out, open(cache_path, "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)
    print("best", best, "evals", ev.n_eval, "wall", time.time() - t0)


if __name__ == "__main__":
    main()
