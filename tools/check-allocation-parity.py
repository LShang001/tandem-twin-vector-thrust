#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    check-allocation-parity.py
@module  工具/三端同构核对（仿真 ↔ 固件 控制效能矩阵对拍）
@brief   对同一组工作点，分别调用仿真 `computeBTrue()` 与固件 `computeEffectMatrix()`
         算 3×3 控制效能 Jacobian B_true，逐项比对
@details 输入：内置工作点表（基准转速/差速/上摆角/下摆角）；处理：生成两个临时
         运行器（Node 侧、C++ 侧）读同一份输入、各自先按差速开方分配求 wf/wt
         再算 B；输出：逐项误差 + 最大误差/容差 + 退出码。
@theory  差速开方分配（行为红线）：wf = w0·√(1+Δω)、wt = w0·√(1−Δω)，
         两侧都必须用这一条公式推 wf/wt，才能保证 B 可比。
         B_true 的行是 [Mx,My,Mz]、列是 [Δω, δ_t, δ_f]（模型系轴名）。
         两边九项解析等价的前提：差速分配已收敛，即 wf²=w0²(1+Δω)、wt²=w0²(1−Δω)。
@ref     TandemVec_FCS/include/TandemVec_Propulsion.h §3 computeEffectMatrix；
         simulations/vector-thrust-lab/src/core/control-allocation.mjs computeBTrue
@frame   模型系（x′=推力轴朝机头）；两侧同一套，不做置换
@units   B 各项：[N·m]/[无量纲] 或 [N·m]/[rad]；角度用度输入、内部转弧度
@note    ★ 摆角工况表里有一条 25°，超出固件实机包线（15°）但落在仿真域概念窗口
         （dMax 双口径，见 DEC-002 A-04）。B 矩阵本身不含限幅，所以 25° 可以比；
         它比的是公式，不比限幅策略。
@see     tools/check-propulsion-parity.py（推进层对拍）、tools/check-param-parity.py
@test    本脚本即自检：退出码 0 表示两侧 B_true 在 float32 精度内等价
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TMP = ROOT / "tmp" / "alloc-parity"

# 工作点：基准转速占比 thr、差速 dw、上摆角 df[deg]、下摆角 dt[deg]
CASES = [
    (0.50, 0.0, 0.0, 0.0),
    (0.50, 0.0, 10.0, 0.0),
    (0.50, 0.0, 0.0, -8.0),
    (0.50, 0.3, 8.0, -6.0),
    (0.75, -0.5, -15.0, 12.0),
    (0.30, 0.7, 0.0, 0.0),
    (1.00, 0.0, 15.0, -15.0),
    (0.60, 0.2, 25.0, -25.0),  # 仿真域概念窗口（固件实机 15°，B 矩阵不含限幅故可比）
]

NAMES = [
    ["∂Mx/∂Δω", "∂Mx/∂δt", "∂Mx/∂δf"],
    ["∂My/∂Δω", "∂My/∂δt", "∂My/∂δf"],
    ["∂Mz/∂Δω", "∂Mz/∂δt", "∂Mz/∂δf"],
]
TOL_ABS = 1e-6
TOL_REL = 1e-5

NODE_RUNNER = r"""
import { computeBTrue } from '../../simulations/vector-thrust-lab/src/core/control-allocation.mjs';
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
  const B = computeBTrue(w0, dfDeg * DEG, dtDeg * DEG, dw, P);
  process.stdout.write(B.flat().map((v) => v.toPrecision(9)).join(' ') + '\n');
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
    const EffectMatrix E = computeEffectMatrix(s, p, w0);
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        std::printf("%.9g%c", E.M[i][j], (i == 2 && j == 2) ? '\n' : ' ');
  }
  return 0;
}
"""


def run(cmd: list[str], stdin_text: str, cwd: Path) -> str:
    proc = subprocess.run(cmd, input=stdin_text, text=True, capture_output=True, cwd=str(cwd))
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
    sim_out = run(["node", str(node_file)], cases_text, TMP)

    # MinGW ld 写不了中文绝对路径：编译 cwd 到仓库根、全用相对路径
    compile_cmd = [
        "g++", "-std=c++17", "-ITandemVec_FCS/include",
        str(cpp_file.relative_to(ROOT)), "-o", str(exe_file.relative_to(ROOT)),
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

    print("=" * 72)
    print("仿真 computeBTrue ⇄ 固件 computeEffectMatrix 对拍（3×3 B_true）")
    print("=" * 72)
    print(f"工作点 {len(CASES)} 个 × 9 项 = {len(CASES) * 9} 项")
    print(f"容差：|Δ| <= {TOL_ABS:g} + {TOL_REL:g}·max(|a|,|b|)")
    print()

    bad = 0
    worst_ratio = 0.0
    worst_desc = ""
    for i, (a_row, b_row) in enumerate(zip(sim_rows, fw_rows)):
        for k in range(9):
            x, y = a_row[k], b_row[k]
            err = abs(x - y)
            lim = TOL_ABS + TOL_REL * max(abs(x), abs(y))
            ratio = err / lim
            if ratio > 1.0:
                bad += 1
                r, c = divmod(k, 3)
                print(f"  ★超限 工况{i + 1} {NAMES[r][c]}: 仿真 {x:.9g} / 固件 {y:.9g} (|Δ|={err:.2e})")
            if ratio > worst_ratio:
                worst_ratio = ratio
                r, c = divmod(k, 3)
                worst_desc = f"工况{i + 1} {NAMES[r][c]}（仿真 {x:.9g} / 固件 {y:.9g}，|Δ|={err:.2e}）"

    print(f"最大误差/容差 = {worst_ratio:.3f}")
    print(f"最大处：{worst_desc}")
    print()
    if bad:
        print(f"[FAIL] {bad} 项超限：两侧 B_true 不再等价")
        return 1
    print("[PASS] 3×3 控制效能矩阵逐项等价（float32 精度内），三端同构的分配层成立")
    return 0


if __name__ == "__main__":
    sys.exit(main())
