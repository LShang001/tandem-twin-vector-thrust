#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    check-propulsion-parity.py
@module  工具/三端同构核对（仿真 ↔ 固件 推进公式对拍）
@brief   对同一组工况，分别调用仿真 `propulsion.mjs` 与固件 `computeWrench()`
         算六维力/力矩，逐分量比对数值
@details 输入：内置工况表（推力占比/差速/上摆角/下摆角）；处理：生成两个临时
         运行器（Node 侧、C++ 侧）读同一份工况、用同一组机型参数各算一遍；
         输出：逐项误差表 + 最大误差/容差 + 退出码。
@theory  两侧公式（模型系，见 TandemVec_Propulsion.h 与 propulsion.mjs）：
             Fx = Tf·cf + Tt·ct      Fy = Tf·sf        Fz = -Tt·st
             Mx = -Qf·cf + Qt·ct     My = -b·Tt·st - Qf·sf
             Mz = a·Tf·sf - Qt·st
         稳态下 Qf=kQ·ωf²、Qt=kQ·ωt²；仿真另含 Jp·dω/dt 瞬态项，
         本脚本把两侧都预置到转速收敛点（dω/dt=0）以只比稳态部分。
@ref     TandemVec_FCS/include/TandemVec_Propulsion.h §稳态简化与口径差异说明；
         simulations/vector-thrust-lab/src/core/propulsion.mjs
@frame   推进模型系（x′=推力轴朝机头）；两侧同一套，不做置换
@units   Fx/Fy/Fz [N]、Mx/My/Mz [N·m]；角度用度输入
@note    ★ 容差用绝对值+相对值混合：`|Δ| <= 1e-6 + 1e-5·max(|a|,|b|)`。
         纯相对容差在小分量上会因 float32 乘加顺序差放大到 1.4e-5 而误报
         （2026-11-07 实测 Mx≈-3.5e-4 的分量，绝对差其实只有 4.9e-9）。
@see     tools/check-param-parity.py（参数层同源核对）
@test    本脚本即自检：退出码 0 表示两侧在 float32 精度内等价
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TMP = ROOT / "tmp" / "prop-parity"

# 工况：推力占比 thr、差速 dw、上摆角 df[deg]、下摆角 dt[deg]
CASES = [
    (0.50, 0.0, 0.0, 0.0),
    (0.50, 0.0, 10.0, 0.0),
    (0.50, 0.0, 0.0, 5.0),
    (0.50, 0.0, 10.0, 5.0),
    (0.50, 0.3, 8.0, -6.0),
    (0.75, 0.0, 0.0, 0.0),
    (0.30, -0.4, -12.0, 9.0),
    (1.00, 0.0, 20.0, -20.0),
    (0.10, 0.7, -5.0, 3.0),
]

NAMES = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"]
TOL_ABS = 1e-6
TOL_REL = 1e-5

NODE_RUNNER = r"""
import { stepPropulsion } from '../../simulations/vector-thrust-lab/src/core/propulsion.mjs';
import { P } from '../../simulations/vector-thrust-lab/src/core/parameters.mjs';
import { createInterface } from 'node:readline';
const DEG = Math.PI / 180;
const rl = createInterface({ input: process.stdin, terminal: false });
rl.on('line', (line) => {
  const t = line.trim();
  if (!t || t.startsWith('#')) return;
  const [thr, dw, dfDeg, dtDeg] = t.split(/\s+/).map(Number);
  if (![thr, dw, dfDeg, dtDeg].every(Number.isFinite)) return;
  const w0 = thr * P.wMax;
  const wfT = Math.min(w0 * Math.sqrt(Math.max(0, 1 + dw)), P.wMax);
  const wtT = Math.min(w0 * Math.sqrt(Math.max(0, 1 - dw)), P.wMax);
  const sim = {
    S: { thr, dfAct: dfDeg * DEG, dtAct: dtDeg * DEG, dwAct: dw, wf: wfT, wt: wtT },
    dyn: {}, prevWf: wfT, prevWt: wtT,
  };
  stepPropulsion(sim, P, 0.005);
  const d = sim.dyn;
  process.stdout.write([d.Fx, d.Fy, d.Fz, d.Mx, d.My, d.Mz].map((v) => v.toPrecision(9)).join(' ') + '\n');
});
"""

CPP_RUNNER = r"""
#include <cmath>
#include <cstdio>
#include "TandemVec_Propulsion.h"
int main() {
  const float DEG = 3.14159265358979323846f / 180.0f;
  float thr, dw, df_deg, dt_deg;
  while (std::scanf("%f %f %f %f", &thr, &dw, &df_deg, &dt_deg) == 4) {
    const TandemVecParams p = kDefaultTandemVecParams;
    const float w0 = thr * p.wMax;
    PropulsionState s;
    s.wf = std::fmin(w0 * std::sqrt(std::fmax(0.0f, 1.0f + dw)), p.wMax);
    s.wt = std::fmin(w0 * std::sqrt(std::fmax(0.0f, 1.0f - dw)), p.wMax);
    s.delta_f = df_deg * DEG;
    s.delta_t = dt_deg * DEG;
    const SixDOFWrench w = computeWrench(s, p);
    std::printf("%.9g %.9g %.9g %.9g %.9g %.9g\n", w.Fx, w.Fy, w.Fz, w.Mx, w.My, w.Mz);
  }
  return 0;
}
"""


def run(cmd: list[str], stdin_text: str, cwd: Path) -> str:
    proc = subprocess.run(
        cmd, input=stdin_text, text=True, capture_output=True, cwd=str(cwd)
    )
    if proc.returncode != 0:
        raise SystemExit(
            f"[FAIL] 命令失败：{' '.join(cmd)}\nstdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
        )
    return proc.stdout


def parse(text: str) -> list[list[float]]:
    return [list(map(float, line.split())) for line in text.splitlines() if line.strip()]


def main() -> int:
    TMP.mkdir(parents=True, exist_ok=True)
    node_file = TMP / "runner.mjs"
    cpp_file = TMP / "runner.cpp"
    exe_file = TMP / "runner.exe"
    node_file.write_text(NODE_RUNNER, encoding="utf-8")
    cpp_file.write_text(CPP_RUNNER, encoding="utf-8")

    cases_text = "\n".join(f"{t} {d} {df} {dt}" for t, d, df, dt in CASES) + "\n"

    # Node 侧（import 相对路径指向仓库内 simulations/）
    sim_out = run(["node", str(node_file)], cases_text, TMP)

    # C++ 侧：★ MinGW ld 写不了中文绝对路径（项目已知坑），编译时 cwd 设在仓库根、
    # 全部用相对路径；生成的可执行文件再由 Windows 用绝对路径启动（运行时没问题）。
    compile_cmd = [
        "g++", "-std=c++17",
        "-ITandemVec_FCS/include",
        str(cpp_file.relative_to(ROOT)),
        "-o", str(exe_file.relative_to(ROOT)),
    ]
    compiled = subprocess.run(compile_cmd, cwd=str(ROOT), capture_output=True, text=True)
    if compiled.returncode != 0:
        print("[FAIL] 固件侧编译失败：")
        print(compiled.stderr[-1500:])
        return 1
    fw_out = run([str(exe_file)], cases_text, TMP)

    sim_rows, fw_rows = parse(sim_out), parse(fw_out)
    if len(sim_rows) != len(fw_rows):
        print(f"[FAIL] 行数不一致：仿真 {len(sim_rows)} / 固件 {len(fw_rows)}")
        return 1

    print("=" * 68)
    print("仿真 propulsion.mjs ⇄ 固件 computeWrench 六维对拍（稳态）")
    print("=" * 68)
    print(f"工况 {len(CASES)} 个 × {len(NAMES)} 分量 = {len(CASES) * len(NAMES)} 项")
    print(f"容差：|Δ| <= {TOL_ABS:g} + {TOL_REL:g}·max(|a|,|b|)")
    print()
    print(f"{'工况':<6}{'分量':<6}{'仿真':>16}{'固件':>16}{'|Δ|':>12}  判定")
    print("-" * 68)

    bad = 0
    worst_ratio = 0.0
    worst_desc = ""
    for i, (a_row, b_row) in enumerate(zip(sim_rows, fw_rows)):
        for j, name in enumerate(NAMES):
            x, y = a_row[j], b_row[j]
            err = abs(x - y)
            lim = TOL_ABS + TOL_REL * max(abs(x), abs(y))
            ratio = err / lim
            ok = ratio <= 1.0
            if not ok:
                bad += 1
            if ratio > worst_ratio:
                worst_ratio = ratio
                worst_desc = f"工况{i + 1} {name}（仿真 {x:.9g} / 固件 {y:.9g}，|Δ|={err:.2e}）"
            print(f"{i + 1:<6}{name:<6}{x:>16.9g}{y:>16.9g}{err:>12.2e}  {'OK' if ok else '★超限'}")

    print()
    print(f"最大误差/容差 = {worst_ratio:.3f}")
    print(f"最大处：{worst_desc}")
    print()
    if bad:
        print(f"[FAIL] {bad} 项超限：两侧推进公式在稳态不再等价")
        return 1
    print("[PASS] 六维公式逐位等价（float32 精度内），三端同构的推进层成立")
    return 0


if __name__ == "__main__":
    sys.exit(main())
