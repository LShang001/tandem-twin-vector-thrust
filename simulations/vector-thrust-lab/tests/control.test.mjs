// control.mjs（SAS）单元测试
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { P } from '../src/core/parameters.mjs';
import { createSimulationState } from '../src/core/state.mjs';
import { applySas } from '../src/core/control.mjs';

const DT = 0.004;

test('SAS 关闭时指令直通', () => {
  const sim = createSimulationState(P);
  sim.S.sas = false;
  sim.S.dt = 0.1; sim.S.df = -0.05; sim.S.dw = 0.2;
  sim.F.euler.x = 0.3; sim.F.euler.y = 0.3; sim.S.omega.z = 5; // 扰动不应生效
  applySas(sim, P, DT);
  assert.equal(sim.S.dtAct, 0.1);
  assert.equal(sim.S.dfAct, -0.05);
  assert.equal(sim.S.dwAct, 0.2);
});

test('俯仰反馈极性：正 theta 与正 q 增大尾摆指令（负效率通道取正号）', () => {
  const sim = createSimulationState(P);
  sim.F.euler.y = 0.1;
  sim.S.omega.y = 0.5;
  applySas(sim, P, DT);
  assert.ok(sim.S.dtAct > 0, '正姿态/角速率误差应产生正修正');
});

test('偏航反馈极性：正 r 减小幅前摆指令（正效率通道取负号）', () => {
  const sim = createSimulationState(P);
  sim.S.omega.z = 0.5;
  applySas(sim, P, DT);
  assert.ok(sim.S.dfAct < 0);
});

test('滚转反馈极性：正 phi 与正 p 增大差速（负效率通道取正号）', () => {
  const sim = createSimulationState(P);
  sim.F.euler.x = 0.1;
  sim.S.omega.x = 0.5;
  applySas(sim, P, DT);
  assert.ok(sim.S.dwAct > 0);
});

test('执行限幅：摆角 ±dMax，差速 ±dwMax', () => {
  const sim = createSimulationState(P);
  sim.F.euler.y = 100; sim.S.omega.y = 100;
  sim.F.euler.x = 100; sim.S.omega.x = 100;
  applySas(sim, P, DT);
  assert.ok(Math.abs(sim.S.dtAct) <= P.dMax + 1e-15);
  assert.ok(Math.abs(sim.S.dwAct) <= P.dwMax + 1e-15);
});

test('积分器限幅并随时间累积', () => {
  const sim = createSimulationState(P);
  sim.F.euler.y = 1.0; // 持续大误差
  for (let i = 0; i < 10000; i++) applySas(sim, P, DT);
  assert.ok(Math.abs(sim.S.intTh) <= P.intThMax + 1e-15, '俯仰积分限幅');
  assert.equal(sim.S.intTh, P.intThMax);
});
