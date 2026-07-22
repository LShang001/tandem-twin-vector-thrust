// aerodynamics.mjs 单元测试
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { P } from '../src/core/parameters.mjs';
import { createSimulationState } from '../src/core/state.mjs';
import { computeAero } from '../src/core/aerodynamics.mjs';

test('气动开关关闭时力与力矩为零', () => {
  const sim = createSimulationState(P);
  sim.S.aero = false;
  sim.F.vel = { x: 20, y: 1, z: 2 };
  const f = computeAero(sim, P);
  assert.deepEqual(f, { aX: 0, Y: 0, aZ: 0 });
  assert.equal(sim.aero.Mx + sim.aero.My + sim.aero.Mz, 0);
  // 空速遥测仍更新
  assert.ok(sim.aero.V > 0);
});

test('正迎角产生升力（机体系 -z 方向）', () => {
  const sim = createSimulationState(P);
  sim.F.vel = { x: 20, y: 0, z: 2 }; // α > 0
  const f = computeAero(sim, P);
  assert.ok(sim.aero.alpha > 0);
  assert.ok(f.aZ < 0, '升力沿机体系 -z');
  // 机体系 x 向合力 = L·sinα − D·cosα：升力前倾分量可能超过阻力，
  // 故不按符号断言，改与模型解析式精确比对。
  const qb = sim.aero.qbar;
  const al = sim.aero.alpha;
  const CL = P.CLa * al;
  const L = qb * P.Sw * CL;
  const D = qb * P.Sw * (P.CD0 + P.CDk * CL * CL);
  const expectedAx = L * Math.sin(al) - D * Math.cos(al);
  assert.ok(Math.abs(f.aX - expectedAx) < 1e-9, `aX=${f.aX} 期望 ${expectedAx}`);
});

test('正俯仰角速率产生负阻尼力矩（Cmq<0）', () => {
  const sim = createSimulationState(P);
  sim.F.vel = { x: 20, y: 0, z: 0 };
  sim.S.omega.y = 1.0;
  computeAero(sim, P);
  // Cm0 项单独为正，需扣除后检验阻尼符号
  const qb = sim.aero.qbar;
  const damping = sim.aero.My - qb * P.Sw * P.cbar * P.Cm0;
  assert.ok(damping < 0);
});

test('正侧滑角产生负侧力（CYb<0）与航向静稳定力矩（Cnb>0）', () => {
  const sim = createSimulationState(P);
  sim.F.vel = { x: 20, y: 2, z: 0 }; // β > 0
  const f = computeAero(sim, P);
  assert.ok(f.Y < 0);
  assert.ok(sim.aero.Mz > 0);
  assert.ok(sim.aero.Mx < 0, 'Clb<0 → 横滚静稳定');
});

test('零真实空速时动压与气动力为零，保护速度仅用于分母', () => {
  const sim = createSimulationState(P);
  sim.F.vel = { x: 0, y: 0, z: 0 };
  sim.S.omega = { x: 1, y: 1, z: 1 };
  const force = computeAero(sim, P);
  assert.equal(sim.aero.V, 0);
  assert.equal(sim.aero.qbar, 0);
  assert.ok(Math.abs(force.aX) === 0 && Math.abs(force.Y) === 0 && Math.abs(force.aZ) === 0);
  assert.ok(Math.abs(sim.aero.Mx) === 0);
  assert.ok(Math.abs(sim.aero.My) === 0);
  assert.ok(Math.abs(sim.aero.Mz) === 0);
  assert.ok(Number.isFinite(sim.aero.qbar));
});
