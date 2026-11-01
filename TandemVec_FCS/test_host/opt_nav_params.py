#!/usr/bin/env python3
"""EKF 连续参数全面优化（CMA-ES，8 维）。

项目规范（AGENTS.md）：连续参数最终调参默认用现代优化器，网格/手扫仅用于
初值与可视化。

优化参数（8 个）：
    1. LPF 陀螺带宽 [Hz]        [100, 500]
    2. LPF 加计带宽 [Hz]        [50, 300]
    3. hAcc 门限 [m]            [1.0, 10.0]（>9.5 视为关闭=1e9）
    4. q_accel 缩放             [0.5, 4.0]
    5. EKF accel_std            [0.10, 0.40] m/s²
    6. EKF gyro_std             [0.0005, 0.005] rad/s
    7. EKF accel_markov         [0.001, 0.05] m/s²
    8. EKF gyro_markov          [0.001, 0.02] rad/s

目标（test_imu_nav_perf 的 IMUPERF_OPT 模式）：
    static(M8N10) + maneuver(LowEnd5Hz) + vibration(0.5g/80Hz) + vib_outage
    cost = 加权姿态+位置 RMSE（详见 C++ 端）。

用法：
    IMUPERF_EXE=<exe> python test_host/opt_nav_params.py
输出：
    tmp/opt_cma_log.csv   每代最优/均值
    tmp/opt_cma_best.json 最优参数与 cost
"""
import json
import os
import subprocess
import sys
import tempfile

import cma

BOUNDS = [
    (100.0, 500.0),   # lpf_gyro
    (50.0, 300.0),    # lpf_accel
    (1.0, 10.0),      # hacc（>9.5 视为关闭）
    (0.5, 4.0),       # q_accel
    (0.10, 0.40),     # ekf_accel_std
    (0.0005, 0.005),  # ekf_gyro_std
    (0.001, 0.05),    # ekf_accel_markov
    (0.001, 0.02),    # ekf_gyro_markov
]
ENVS = [
    "IMUPERF_LPF_GYRO",
    "IMUPERF_LPF_ACCEL",
    "IMUPERF_HACC",
    "IMUPERF_Q_ACCEL",
    "IMUPERF_EKF_ACCEL_STD",
    "IMUPERF_EKF_GYRO_STD",
    "IMUPERF_EKF_ACCEL_MARKOV",
    "IMUPERF_EKF_GYRO_MARKOV",
]


def to_phys(x):
    out = []
    for (lo, hi), v in zip(BOUNDS, x):
        out.append(lo + (hi - lo) * min(max(v, 0.0), 1.0))
    if out[2] > 9.5:
        out[2] = 1.0e9
    return out


def make_cost(exe):
    def cost(x):
        p = to_phys(x)
        env = dict(os.environ)
        env["IMUPERF_OPT"] = "1"
        for name, v in zip(ENVS, p):
            env[name] = f"{v:.9g}"
        fd, out_path = tempfile.mkstemp(suffix=".csv")
        os.close(fd)
        try:
            with open(out_path, "w", encoding="utf-8") as fout:
                subprocess.run([exe], env=env, stdout=fout,
                               stderr=subprocess.DEVNULL, timeout=120,
                               check=False)
            with open(out_path, encoding="utf-8") as fin:
                for line in fin:
                    if line.startswith("OPT_RESULT,"):
                        return float(line.split(",")[1])
        except subprocess.TimeoutExpired:
            print("评估超时 120s", file=sys.stderr)
            return 1.0e6
        finally:
            try:
                os.unlink(out_path)
            except OSError:
                pass
        print("评估未产出 OPT_RESULT", file=sys.stderr)
        return 1.0e6
    return cost


def main():
    exe = os.environ.get("IMUPERF_EXE")
    if not exe:
        sys.exit("请设置 IMUPERF_EXE")
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out_dir = os.path.join(repo, "tmp")
    os.makedirs(out_dir, exist_ok=True)
    log_path = os.path.join(out_dir, "opt_cma_log.csv")
    best_path = os.path.join(out_dir, "opt_cma_best.json")

    dim = len(BOUNDS)
    x0 = [0.5] * dim
    cost = make_cost(exe)
    es = cma.CMAEvolutionStrategy(
        x0, 0.3,
        {"bounds": [[0.0] * dim, [1.0] * dim],
         "popsize": 8, "maxfevals": 320, "seed": 20261101,
         "verb_log": 0, "verbose": -9})

    best = {"cost": float("inf"), "params": None}
    n = 0
    with open(log_path, "w", encoding="utf-8") as log:
        log.write("eval,gen,best,mean\n")
        while not es.stop():
            xs = es.ask()
            fs = [cost(x) for x in xs]
            es.tell(xs, fs)
            n += len(xs)
            gen = es.countiter
            best_i = min(range(len(fs)), key=lambda i: fs[i])
            if fs[best_i] < best["cost"]:
                best = {"cost": fs[best_i], "params": to_phys(xs[best_i])}
            log.write(f"{n},{gen},{fs[best_i]:.6f},{sum(fs)/len(fs):.6f}\n")
            log.flush()
            print(f"gen {gen} best {fs[best_i]:.4f} (overall {best['cost']:.4f})",
                  flush=True)

    with open(best_path, "w", encoding="utf-8") as f:
        json.dump({"cost": best["cost"],
                   "params": dict(zip(ENVS, best["params"]))}, f,
                  ensure_ascii=False, indent=2)
    print("最优:", json.dumps(best, ensure_ascii=False, default=str))


if __name__ == "__main__":
    main()
