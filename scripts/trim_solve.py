#!/usr/bin/env python3
"""trim_solve.py — 纵向三方程配平数值求解（VTOL 参数集 v0.2.0）

给定巡航速度 V，求解 [aTrim, thrTrim, dtTrim] 满足：
  Fx = 0, Fz = 0, My = 0   （稳态水平飞行，ω=0，无陀螺/阻尼项）

力/力矩表达式与 simulations/vector-thrust-lab/src/core/ 各模块逐项一致：
  T = kT·(thr·wMax)²（前后等转速）；cf=1、sf=0（前摆 0）
  Fx = Tf + Tt·cos(dt) + aX
  Fz = -Tt·sin(dt) + aZ + m·g·cos(a0)
  My = -b·Tt·sin(dt) + qb·Sw·cbar·(Cm0 + Cma·alpha)
  aX = L·sin(alpha) - D·cos(alpha)；aZ = -L·cos(alpha) - D·sin(alpha)
  L = qb·Sw·CLa·alpha；D = qb·Sw·(CD0 + CDk·CL²)；CL = CLa·alpha
  alpha = atan2(V·sin(a0), V·cos(a0)) = a0；qb = 0.5·rho·V²
"""
import json
import math
from pathlib import Path

MODEL_JSON = Path(__file__).resolve().parent.parent / "models" / "aircraft-model.json"

def load_params():
    model = json.loads(MODEL_JSON.read_text(encoding="utf-8"))
    P = {}
    for s in model["sections"]:
        for p in s["parameters"]:
            P[p["name"]] = p["value"]
    return P

def residuals(x, P, V):
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
    P = load_params()
    for V in (24.0, 26.0, 27.0, 28.0, 29.0, 30.0, 32.0):
        try:
            a0, thr, dt, df = solve(P, V)
            print(f"V={V:5.1f}  aTrim={math.degrees(a0):8.4f}°  thrTrim={thr:.9f}  dtTrim={math.degrees(dt):8.4f}°  dfTrim={math.degrees(df):8.4f}°")
        except Exception as e:
            print(f"V={V:5.1f}  {e}")

if __name__ == "__main__":
    main()
