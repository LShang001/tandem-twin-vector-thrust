#!/usr/bin/env python3
"""trim_solve.py — 巡航四方程配平数值求解（VTOL 参数集 v0.2.0）。

@file    trim_solve.py
@module  analysis/配平求解
@brief   给定巡航速度 V，用坐标下降 + 逐变量二分求解 [aTrim, thrTrim, dtTrim, dfTrim]，
         并打印各速度点的配平值（**不写回**模型 JSON，回写须人工评审）。
@details 平衡条件（稳态水平飞行，ω=0，无陀螺/阻尼项）：
           Fx = 0, Fz = 0, My = 0, Mz = 0
         力/力矩表达式与 simulations/vector-thrust-lab/src/core/ 各模块逐项一致：
           T = kT·(thr·wMax)²（前后等转速）；cf=1、sf=0（前摆 0）
           Fx = Tf + Tt·cos(dt) + aX；Fz = −Tt·sin(dt) + aZ + m·g·cos(a0)
           My = −b·Tt·sin(dt) − Q·sin(df) + qb·Sw·cbar·(Cm0 + Cma·alpha)
           Mz =  a·Tt·sin(df) − Q·sin(dt)
           aX = L·sin(alpha) − D·cos(alpha)；aZ = −L·cos(alpha) − D·sin(alpha)
           L = qb·Sw·CLa·alpha；D = qb·Sw·(CD0 + CDk·CL²)；qb = 0.5·rho·V²
         求解：外层坐标下降（a0 → thr → dt → df），内层各变量单调故用二分；
         收敛判据为残差 max|r| < 1e-12。
         运行：仓库根目录 `py -3.12 scripts/trim_solve.py`（遍历 24–32 m/s）。
@theory  配平四方程与数值解 @ref MOD-003 §2.1–§2.2；
         推力矢量六维映射 @ref MOD-001 §2–§4；气动模型 @ref MOD-001 §6。
@frame  NED + FRD；alpha 为物理抬头角（显示 theta=−alpha，见 @ref MOD-002 §4.2）。
@units   SI：V m/s、alpha/dt/df rad、thr 无量纲、力 N、力矩 N·m。
@note    ★ 配平结果 dtTrim=−18.702° 已超出固件实机 15° 摆角限幅——28 m/s 巡航配平点
         在固件 15° 限幅下**不可达**（@ref MOD-003 §3.2、ASM-017）。写回参数前必须
         按 15° 重算或放宽限幅，不可直接用于实机。
         ★ 2026-10-23（A-04）新增 `--dmax-deg`/`--scan`：按固件限幅逐速度判定可行性；
         15° 下的可行巡航速度下限约 29.8 m/s（30 m/s 时 dt≈−14.8°），低于该速度
         尾摆配平需求反而更大（升力占比下降），不是“降速就安全”。
@see     @ref MOD-003、@ref models/aircraft-model.json、high-fidelity-analysis/trim_analysis.py
"""
import argparse
import json
import math
from pathlib import Path

MODEL_JSON = Path(__file__).resolve().parent.parent / "models" / "aircraft-model.json"

def load_params():
    """读取 models/aircraft-model.json 并展平为 name→value 字典。"""
    model = json.loads(MODEL_JSON.read_text(encoding="utf-8"))
    P = {}
    for s in model["sections"]:
        for p in s["parameters"]:
            P[p["name"]] = p["value"]
    return P

def residuals(x, P, V):
    """四方程配平残差 [Fx, Fz, My, Mz]（N / N·m）。

    @param[in] x (aTrim, thr, dtTrim, dfTrim)：迎角 rad、油门 0–1、两摆角 rad
    @param[in] P 参数字典；V 巡航速度 [m/s]
    @formula 见模块 docstring 的四式 @ref MOD-003 §2.1
    @return 4 维残差（SI）
    """
    a0, thr, dt, df = x  # 迎角、油门（转速比）、尾摆偏置、前摆偏置（rad）
    w = thr * P["wMax"]
    T = P["kT"] * w * w
    Q = P["kQ"] * w * w  # 稳态反扭（dWt=0）
    qb = 0.5 * P["rho"] * V * V
    alpha = a0
    CL = P["CLa"] * alpha
    L = qb * P["Sw"] * CL
    D = qb * P["Sw"] * (P["CD0"] + P["CDk"] * CL * CL)
    aX = L * math.sin(alpha) - D * math.cos(alpha)
    aZ = -L * math.cos(alpha) - D * math.sin(alpha)
    Fx = T * math.cos(df) + T * math.cos(dt) + aX
    Fy = T * math.sin(df)  # 侧向：前摆引入，配平后应≈0（小量，未计入四方程）
    Fz = -T * math.sin(dt) + aZ + P["m"] * P["g"] * math.cos(a0)
    My = -P["b"] * T * math.sin(dt) - Q * math.sin(df) + qb * P["Sw"] * P["cbar"] * (P["Cm0"] + P["Cma"] * alpha)
    Mz = P["a"] * T * math.sin(df) - Q * math.sin(dt)
    return [Fx, Fz, My, Mz]

def bisect(f, lo, hi, tol=1e-14, max_iter=100):
    """二分求根（要求 f(lo)、f(hi) 异号），用于逐变量坐标下降的一维子问题。"""
    flo = f(lo)
    if abs(flo) < tol:
        return lo
    fhi = f(hi)
    if flo * fhi > 0:
        raise RuntimeError(f"区间端点同号: f({lo})={flo} f({hi})={fhi}")
    for _ in range(max_iter):
        mid = (lo + hi) / 2
        fm = f(mid)
        if abs(fm) < tol:
            return mid
        if flo * fm < 0:
            hi, fhi = mid, fm
        else:
            lo, flo = mid, fm
    return (lo + hi) / 2


def solve(P, V, tol=1e-12, max_outer=300):
    """坐标下降：二分 a0（Fz=0）→ 二分 thr（Fx=0）→ 二分 dt（My=0）→ 二分 df（Mz=0），循环至收敛。"""
    qb = 0.5 * P["rho"] * V * V
    T_max = 2 * P["kT"] * P["wMax"] * P["wMax"]  # 双发满油门推力

    def fz(a0, thr, dt, df):
        T = P["kT"] * (thr * P["wMax"]) ** 2
        alpha = a0
        CL = P["CLa"] * alpha
        L = qb * P["Sw"] * CL
        D = qb * P["Sw"] * (P["CD0"] + P["CDk"] * CL * CL)
        aZ = -L * math.cos(alpha) - D * math.sin(alpha)
        return -T * math.sin(dt) + aZ + P["m"] * P["g"] * math.cos(a0)

    def fx(a0, thr, dt, df):
        T = P["kT"] * (thr * P["wMax"]) ** 2
        alpha = a0
        CL = P["CLa"] * alpha
        L = qb * P["Sw"] * CL
        D = qb * P["Sw"] * (P["CD0"] + P["CDk"] * CL * CL)
        aX = L * math.sin(alpha) - D * math.cos(alpha)
        return T * (math.cos(df) + math.cos(dt)) + aX

    def my(a0, thr, dt, df):
        T = P["kT"] * (thr * P["wMax"]) ** 2
        Q = P["kQ"] * (thr * P["wMax"]) ** 2
        alpha = a0
        My_aero = qb * P["Sw"] * P["cbar"] * (P["Cm0"] + P["Cma"] * alpha)
        return -P["b"] * T * math.sin(dt) - Q * math.sin(df) + My_aero

    def mz(a0, thr, dt, df):
        T = P["kT"] * (thr * P["wMax"]) ** 2
        Q = P["kQ"] * (thr * P["wMax"]) ** 2
        return P["a"] * T * math.sin(df) - Q * math.sin(dt)

    # 初始猜测：升力=重力（a0 小角度近似）
    a0 = P["m"] * P["g"] / (qb * P["Sw"] * P["CLa"])
    thr = 0.2
    dt = 0.0
    df = 0.0
    for _ in range(max_outer):
        # 1) 固定 (thr, dt, df)，二分 a0 解 Fz=0
        a0 = bisect(lambda a: fz(a, thr, dt, df), 0.0, 0.35)
        # 2) 固定 (a0, dt, df)，二分 thr 解 Fx=0（fx 随 thr 单调增）
        thr = bisect(lambda t: fx(a0, t, dt, df), 0.02, 0.98)
        # 3) 二分 dt 解 My=0（my 随 dt 单调减）
        dt = bisect(lambda d: my(a0, thr, d, df), -0.4363, 0.4363)
        # 4) 二分 df 解 Mz=0（mz 随 df 单调增）
        df = bisect(lambda f: mz(a0, thr, dt, f), -0.4363, 0.4363)
        r = residuals([a0, thr, dt, df], P, V)
        if max(abs(v) for v in r) < tol:
            return [a0, thr, dt, df]
    raise RuntimeError(f"V={V}: 坐标下降不收敛，残差={[f'{v:.2e}' for v in residuals([a0, thr, dt, df], P, V)]}")

def main():
    """对 24–32 m/s（或 --scan 的 24–40 m/s）逐速度求四方程配平并打印。

    `--dmax-deg` 给出固件摆角限幅时，逐速度标注 OK/超出，并打印可行区间；
    默认 25°（模型口径）不判定，保持旧输出。
    """
    ap = argparse.ArgumentParser(description="巡航四方程配平求解")
    ap.add_argument("--dmax-deg", type=float, default=None,
                    help="摆角限幅 [deg]；给出时判定配平是否在限幅内")
    ap.add_argument("--scan", action="store_true",
                    help="扫描 24–40 m/s（默认只打印 24–32 m/s 关键点）")
    args = ap.parse_args()

    P = load_params()
    limit = math.radians(args.dmax_deg) if args.dmax_deg is not None else None
    if args.scan:
        speeds = [24.0 + 0.25 * i for i in range(65)]  # 24–40 m/s
    else:
        speeds = (24.0, 26.0, 27.0, 28.0, 29.0, 30.0, 32.0)
    feasible = []
    for V in speeds:
        try:
            a0, thr, dt, df = solve(P, V)
            ok = (limit is None) or (abs(dt) <= limit and abs(df) <= limit)
            tag = ""
            if limit is not None:
                tag = "  [OK]" if ok else "  [超出限幅]"
                if ok:
                    feasible.append(V)
            print(f"V={V:5.2f}  aTrim={math.degrees(a0):8.4f}°  thrTrim={thr:.9f}  "
                  f"dtTrim={math.degrees(dt):8.4f}°  dfTrim={math.degrees(df):8.4f}°{tag}")
        except Exception as e:
            print(f"V={V:5.2f}  {e}")
    if limit is not None and feasible:
        print(f"\n[可行区间] {min(feasible):.2f}–{max(feasible):.2f} m/s "
              f"（|dt|,|df| ≤ {args.dmax_deg:.1f}°）")

if __name__ == "__main__":
    main()
