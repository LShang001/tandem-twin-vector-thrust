// state.mjs 复位语义测试
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { P } from '../src/core/parameters.mjs';
import { createSimulationState, resetSimulationState } from '../src/core/state.mjs';

test('完整复位恢复可重复的配平动态状态并保留模式开关', () => {
  const sim = createSimulationState(P);
  sim.S.time = 12.3;
  sim.S.thr = 0.9; sim.S.dt = 0.2; sim.S.df = -0.1; sim.S.dw = 0.4;
  sim.S.dtAct = 0.3; sim.S.dfAct = -0.2; sim.S.dwAct = 0.5;
  sim.S.wf = 700; sim.S.wt = 300; sim.prevWf = 680; sim.prevWt = 320;
  sim.S.sasMode = 2; sim.S.aero = false; sim.S.lockXY = false;
  sim.dyn.Fx = 99; sim.aero.qbar = 88;

  resetSimulationState(sim, P);

  const wTrim = P.thrTrim * P.wMax;
  assert.equal(sim.S.time, 0);
  assert.equal(sim.S.thr, P.thrTrim);
  assert.deepEqual([sim.S.dt, sim.S.df, sim.S.dw], [0, 0, 0]);
  assert.deepEqual([sim.S.dtAct, sim.S.dfAct, sim.S.dwAct], [P.dtTrim, 0, 0]);
  assert.deepEqual([sim.S.wf, sim.S.wt, sim.prevWf, sim.prevWt], [wTrim, wTrim, wTrim, wTrim]);
  assert.equal(sim.dyn.Fx, 0);
  assert.equal(sim.aero.qbar, 0);
  assert.deepEqual([sim.S.sasMode, sim.S.aero, sim.S.lockXY], [2, false, false]);
});
