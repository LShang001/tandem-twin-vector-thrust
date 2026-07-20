// propulsion.mjs 单元测试
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { P } from '../src/core/parameters.mjs';
import { createSimulationState } from '../src/core/state.mjs';
import { stepPropulsion } from '../src/core/propulsion.mjs';

const close = (a, b, eps = 1e-9) => assert.ok(Math.abs(a - b) < eps, `${a} ≉ ${b}`);

function primedSim(thr = 0.5) {
  const sim = createSimulationState(P);
  sim.S.thr = thr;
  sim.S.wf = sim.S.wt = thr * P.wMax;
  sim.prevWf = sim.S.wf; sim.prevWt = sim.S.wt;
  return sim;
}

test('推力与反扭矩稳态关系 T=kT·ω², τ=kQ·ω²', () => {
  const sim = primedSim();
  const dt = 0.004;
  stepPropulsion(sim, P, dt); // 指令=当前转速 → 稳态（dW≈0 以外项）
  const w = 0.5 * P.wMax;
  close(sim.dyn.Tf, P.kT * w * w, 1e-6);
  close(sim.dyn.Tt, P.kT * w * w, 1e-6);
});

test('差速分配保持平方转速和不变（一阶近似总推力不变）', () => {
  const dw = 0.3;
  const w0 = 0.5 * P.wMax;
  const wfT = w0 * Math.sqrt(1 + dw), wtT = w0 * Math.sqrt(1 - dw);
  close(wfT * wfT + wtT * wtT, 2 * w0 * w0, 1e-6);
});

test('前摆角符号：df>0 → Fy>0、Mz>0（偏航主控）', () => {
  const sim = primedSim();
  sim.S.dfAct = 0.2;
  stepPropulsion(sim, P, 0.004);
  assert.ok(sim.dyn.Fy > 0);
  assert.ok(sim.dyn.Mz > 0);
});

test('尾摆角符号：dt>0 → Fz<0、My<0（俯仰主控）', () => {
  const sim = primedSim();
  sim.S.dtAct = 0.2;
  stepPropulsion(sim, P, 0.004);
  assert.ok(sim.dyn.Fz < 0);
  assert.ok(sim.dyn.My < 0);
});

test('零摆角等转速 → 反扭对消（Mx≈0）', () => {
  const sim = primedSim();
  stepPropulsion(sim, P, 0.004);
  close(sim.dyn.Mx, 0, 1e-6);
});

test('电机一阶滞后：转速向目标指数趋近', () => {
  const sim = createSimulationState(P);
  sim.S.thr = 1.0; sim.S.wf = 0; sim.S.wt = 0; sim.prevWf = 0; sim.prevWt = 0;
  // 离散更新 ω += (ωt−ω)·min(dt/τM, 1)：步长 ≥ τM 时截断为 1（直达目标），
  // 故用 100 个小步长逼近连续一阶响应，一个时间常数后应达 63.2%。
  const n = 100;
  for (let i = 0; i < n; i++) stepPropulsion(sim, P, P.tauM / n);
  const expect = P.wMax * (1 - 1 / Math.E);
  assert.ok(Math.abs(sim.S.wf - expect) < P.wMax * 0.005,
    `wf=${sim.S.wf}, 期望≈${expect}`);
});
