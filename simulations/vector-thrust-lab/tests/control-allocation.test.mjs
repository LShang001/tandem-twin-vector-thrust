/**
 * @file    control-allocation.test.mjs
 * @module  仿真 tests/控制分配
 * @brief   锁定 B_true 解析 Jacobian、可逆性与增量分配精度
 * @details 覆盖：B 与中心差分逐元素比对；与 Python 参考式抽查 3 元素；B⁻¹·B=I；
 *          低油门奇异返回 null；Δu=B⁻¹·ΔM 的力矩复现。
 *          容差依据：差分 h=1e-7 的截断+相消误差量级 ~|M''|h²/6 + ε·|M|/h ≈ 1e-6
 *          → 元素比对取 1e-6；B⁻¹B 是纯代数取 1e-9；分配复现受一阶线性化限制取 1e-4。
 * @theory  摆角雅可比 @ref THY-003 §2.2；线性化分配/伪逆 @ref THY-003 §3.2；
 *          三端同构口径 @ref CTL-001 §8.5
 * @frame   机体系力矩，列序 [Δdw, Δδt, Δδf]
 * @units   N·m、rad、rad/s
 * @note 测试内置的 torqueAt 是 propulsion.mjs 六维映射的独立副本——若两处口径漂移，
 *       本文件会先失败（这是刻意的重复实现，不是冗余代码）
 * @see     src/core/control-allocation.mjs、src/core/propulsion.mjs
 */
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { P } from '../src/core/parameters.mjs';
import { computeBTrue, inv3 } from '../src/core/control-allocation.mjs';

// 推进力矩精确模型（复制 propulsion.mjs 的映射，用于数值差分）
function torqueAt(omega0, dw, df, dt, P) {
  const T0 = P.kT * omega0 * omega0, tau0 = P.kQ * omega0 * omega0;
  const wf = omega0 * Math.sqrt(1 + dw), wt = omega0 * Math.sqrt(1 - dw);
  const Tf = P.kT * wf * wf, Tt = P.kT * wt * wt;
  const Qf = P.kQ * wf * wf, Qt = P.kQ * wt * wt;
  const sf = Math.sin(df), cf = Math.cos(df), st = Math.sin(dt), ct = Math.cos(dt);
  return {
    Mx: -Qf * cf + Qt * ct,
    My: -P.b * Tt * st - Qf * sf,
    Mz: P.a * Tf * sf - Qt * st,
  };
}

// 数值差分 Jacobian（中心差分，h=1e-7）；torqueAt 参数序 (omega0, dw, df, dt)
// B 列序 [dw, δt, δf] → 列1 差分 δt（第 4 参）、列2 差分 δf（第 3 参）
function numericB(omega0, df, dt, dw, P) {
  const h = 1e-7;
  const m = (u, v, w) => {
    const t = torqueAt(omega0, u, v, w, P);
    return [t.Mx, t.My, t.Mz];
  };
  const base = m(dw, df, dt);
  const B = [[], [], []];
  const d0 = m(dw + h, df, dt).map((x, i) => (x - base[i]) / h);   // 列0: dw
  const d1 = m(dw, df, dt + h).map((x, i) => (x - base[i]) / h);   // 列1: δt
  const d2 = m(dw, df + h, dt).map((x, i) => (x - base[i]) / h);   // 列2: δf
  for (let i = 0; i < 3; i++) {
    B[i][0] = d0[i]; B[i][1] = d1[i]; B[i][2] = d2[i];
  }
  return B;
}

// 与 Python 相同的参考点（悬停配平附近）
const W0 = P.thrTrim * P.wMax;
const DF = P.dfTrim, DT = P.dtTrim, DW = 0.1;

test('B_true 元素与数值差分一致（悬停工作点）', () => {
  const B = computeBTrue(W0, DF, DT, DW, P);
  const Bn = numericB(W0, DF, DT, DW, P);
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      assert.ok(Math.abs(B[i][j] - Bn[i][j]) < 1e-6,
        `B[${i}][${j}]: 解析 ${B[i][j].toFixed(8)} vs 差分 ${Bn[i][j].toFixed(8)}`);
    }
  }
});

test('B_true 与 Python core.control_effectiveness 数值同构（抽查 3 元素）', () => {
  const B = computeBTrue(W0, DF, DT, DW, P);
  const T0 = P.kT * W0 * W0, tau0 = P.kQ * W0 * W0;
  const Tf = T0 * (1 + DW), taut = tau0 * (1 - DW), tauf = tau0 * (1 + DW);
  const sf = Math.sin(DF), cf = Math.cos(DF), st = Math.sin(DT), ct = Math.cos(DT);
  // Python: B[0,0]=-tau0(cf+ct)  B[1,1]=-b·Tt·ct（Tt=T0(1-dw)）  B[2,2]=a·Tf·cf
  assert.ok(Math.abs(B[0][0] + tau0 * (cf + ct)) < 1e-12, 'B[0][0] = −tau0(cf+ct)');
  assert.ok(Math.abs(B[1][1] + P.b * T0 * (1 - DW) * ct) < 1e-12, 'B[1][1] = −b·Tt·ct');
  assert.ok(Math.abs(B[2][2] - P.a * Tf * cf) < 1e-12, 'B[2][2] = a·Tf·cf');
});

test('B⁻¹·B = I（可逆，悬停工作点）', () => {
  const B = computeBTrue(W0, DF, DT, DW, P);
  const Binv = inv3(B);
  assert.ok(Binv, 'B 应可逆');
  const I = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      let s = 0;
      for (let k = 0; k < 3; k++) s += Binv[i][k] * B[k][j];
      I[i][j] = s;
    }
  }
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      assert.ok(Math.abs(I[i][j] - (i === j ? 1 : 0)) < 1e-9,
        `B⁻¹·B[${i}][${j}] = ${I[i][j].toFixed(6)} 应为 ${i === j ? 1 : 0}`);
    }
  }
});

test('低油门奇异防护：inv3 返回 null（det≈0）', () => {
  const B = computeBTrue(1, 0, 0, 0, P);   // ω0=1 → 推力/反扭 ≈ 0 → 奇异
  const Binv = inv3(B);
  assert.equal(Binv, null, '近奇异矩阵应返回 null（调用方回退直通）');
});

test('分配精度：Δu = B⁻¹·ΔM 精确复现目标力矩增量（悬停工作点）', () => {
  const B = computeBTrue(W0, DF, DT, DW, P);
  const Binv = inv3(B);
  assert.ok(Binv);
  const dM = [0.0002, -0.00015, 0.0001];  // 小增量（INDI 每步量级，一阶线性区内）
  const du = [
    Binv[0][0] * dM[0] + Binv[0][1] * dM[1] + Binv[0][2] * dM[2],
    Binv[1][0] * dM[0] + Binv[1][1] * dM[1] + Binv[1][2] * dM[2],
    Binv[2][0] * dM[0] + Binv[2][1] * dM[1] + Binv[2][2] * dM[2],
  ];
  // 执行增量后（数值差分验证）力矩变化 ≈ dM；du 列序 [dw, δt, δf]
  const before = torqueAt(W0, DW, DF, DT, P);
  const after = torqueAt(W0, DW + du[0], DF + du[2], DT + du[1], P);
  const dMx = after.Mx - before.Mx, dMy = after.My - before.My, dMz = after.Mz - before.Mz;
  assert.ok(Math.abs(dMx - dM[0]) < 1e-4, `ΔMx 复现误差 ${(dMx - dM[0]).toExponential(2)}`);
  assert.ok(Math.abs(dMy - dM[1]) < 1e-4, `ΔMy 复现误差 ${(dMy - dM[1]).toExponential(2)}`);
  assert.ok(Math.abs(dMz - dM[2]) < 1e-4, `ΔMz 复现误差 ${(dMz - dM[2]).toExponential(2)}`);
});
