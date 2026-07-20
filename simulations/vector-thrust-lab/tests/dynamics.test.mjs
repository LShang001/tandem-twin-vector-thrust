// dynamics.mjs 单元测试
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { P } from '../src/core/parameters.mjs';
import { createSimulationState } from '../src/core/state.mjs';
import { stepPhysics, physicsStep } from '../src/core/dynamics.mjs';

const cloneSim = (sim) => JSON.parse(JSON.stringify(sim));

test('子步调度：stepPhysics(dt) 等价于手动按 h=dt/n 逐步积分', () => {
  const simA = createSimulationState(P);
  const simB = createSimulationState(P);
  for (const sim of [simA, simB]) {
    sim.S.thr = 0.6; sim.S.dt = 0.05; sim.S.df = -0.03; sim.S.dw = 0.1;
  }
  const dt = 0.01;
  stepPhysics(simA, P, dt);
  const n = Math.max(1, Math.ceil(dt / P.maxStep));
  for (let i = 0; i < n; i++) physicsStep(simB, P, dt / n);
  assert.deepEqual(simA, simB, '调度结果应与手动子步完全一致');
});

test('长时间积分四元数保持归一化', () => {
  const sim = createSimulationState(P);
  sim.S.thr = 0.6;
  sim.S.dt = 0.1; sim.S.dw = 0.2; // 持续机动
  for (let i = 0; i < 600; i++) stepPhysics(sim, P, 1 / 60);
  const q = sim.S.quat;
  const norm = Math.hypot(q.x, q.y, q.z, q.w);
  assert.ok(Math.abs(norm - 1) < 1e-9, `|q|=${norm}`);
});

test('地面约束：下降触地清零下降速度并钳制位置', () => {
  const sim = createSimulationState(P);
  sim.S.sas = false; sim.S.thr = 0;
  sim.F.pos.z = P.groundZ + 0.01;
  sim.F.vel = { x: 0, y: 0, z: 5 }; // 机体系向下（单位姿态 ≈ 惯性系向下）
  sim.S.quat = { x: 0, y: 0, z: 0, w: 1 };
  physicsStep(sim, P, 0.004);
  assert.equal(sim.F.pos.z, P.groundZ);
  assert.equal(sim.F.vWorld.z, 0);
  // vel 由 vWorld 经四元数逆变换回机体系；步内气动力矩会微扰姿态，
  // 故 vel.z 存在 ~1e-6 量级残差，用容差断言而非精确相等。
  assert.ok(Math.abs(sim.F.vel.z) < 1e-5, `vel.z=${sim.F.vel.z}`);
});

test('地面约束不拦截上升运动', () => {
  const sim = createSimulationState(P);
  sim.S.sas = false; sim.S.thr = 0;
  sim.F.pos.z = P.groundZ + 0.01;
  sim.F.vel = { x: 0, y: 0, z: -5 }; // 上升（NED z 减小）
  sim.S.quat = { x: 0, y: 0, z: 0, w: 1 };
  physicsStep(sim, P, 0.004);
  assert.ok(sim.F.pos.z < P.groundZ + 0.01, '上升不应被地面约束拦截');
});

test('配平直飞有界（10 s 内速度与姿态不发散）', () => {
  const sim = createSimulationState(P);
  sim.S.wf = sim.S.wt = sim.S.thr * P.wMax; sim.prevWf = sim.S.wf; sim.prevWt = sim.S.wt;
  for (let i = 0; i < 600; i++) stepPhysics(sim, P, 1 / 60);
  assert.ok(sim.aero.V > 15 && sim.aero.V < 40, `V=${sim.aero.V}`);
  assert.ok(Math.abs(sim.F.euler.y) < 0.5, `theta=${sim.F.euler.y}`);
  assert.ok(Math.abs(sim.F.euler.x) < 0.5, `phi=${sim.F.euler.x}`);
});

test('气动力关闭时可复现角速度自由积分（无阻尼发散趋势）', () => {
  const sim = createSimulationState(P);
  sim.S.aero = false; sim.S.sas = false;
  sim.S.dt = 0.05; // 常值尾摆 → 持续俯仰力矩
  for (let i = 0; i < 120; i++) stepPhysics(sim, P, 1 / 60);
  assert.ok(Math.abs(sim.S.omega.y) > 0.5, '无阻尼时角速度应持续积累');
});
