// state.mjs 复位语义测试
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { P } from '../src/core/parameters.mjs';
import {
  createSimulationState, resetSimulationState, resetPoseOnly, resetVtolHoverState, Q_HOVER,
} from '../src/core/state.mjs';

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
  assert.deepEqual([sim.S.dtAct, sim.S.dfAct, sim.S.dwAct], [P.dtTrim, P.dfTrim, 0]);
  assert.deepEqual([sim.S.wf, sim.S.wt, sim.prevWf, sim.prevWt], [wTrim, wTrim, wTrim, wTrim]);
  assert.equal(sim.dyn.Fx, 0);
  assert.equal(sim.aero.qbar, 0);
  assert.deepEqual([sim.S.sasMode, sim.S.aero, sim.S.lockXY], [2, false, false]);
});

test('轻量复位（巡航）：只复位飞行状态，保留模式/输入（thr/滑块/sasMode/aero）', () => {
  const sim = createSimulationState(P);
  resetSimulationState(sim, P);
  // 制造状态：模式/输入 + 飞行状态污染
  sim.S.sasMode = 2; sim.S.aero = false; sim.S.lockXY = true;
  sim.S.thr = 0.8; sim.S.dt = 0.15; sim.S.df = -0.1; sim.S.dw = 0.2;
  sim.S.time = 7.5;
  sim.S.omega.y = 3; sim.F.vel.x = 40; sim.F.vWorld.z = 25; sim.F.pos.x = 100;
  sim.S.intTh = 0.4; sim.S.intAlt = 0.9;
  sim.S.wf = 900; sim.prevWf = 890;

  resetPoseOnly(sim, P);

  // 保留：模式与输入
  assert.equal(sim.S.sasMode, 2, 'sasMode 保留');
  assert.equal(sim.S.aero, false, 'aero 保留');
  assert.equal(sim.S.lockXY, true, 'lockXY 保留');
  assert.equal(sim.S.thr, 0.8, '油门输入保留');
  assert.equal(sim.S.dt, 0.15, '滑块指令保留');
  assert.equal(sim.S.time, 7.5, '时间不归零');
  // 复位：飞行状态（lockXY=true → 水平速度按约束清零，垂直速度回配平）
  const a0 = P.aTrim;
  assert.equal(sim.F.pos.x, 0);
  assert.equal(sim.F.vel.x, 0, 'lockXY=true 时水平速度按约束清零');
  assert.equal(sim.F.vel.z, P.vTrim * Math.sin(a0), '垂直速度回配平');
  assert.equal(sim.S.omega.y, 0, '角速度清零');
  assert.ok(Math.abs(sim.S.quat.y - Math.sin(a0 / 2)) < 1e-12, '姿态回配平迎角');
  assert.equal(sim.S.intTh, 0, '积分器清零');
  assert.equal(sim.S.intAlt, 0);
  // 转速跟随保留的油门
  assert.ok(Math.abs(sim.S.wf - 0.8 * P.wMax) < 1e-9, '转速跟随当前油门');
  // 执行器基准
  assert.deepEqual([sim.S.dtAct, sim.S.dfAct, sim.S.dwAct], [P.dtTrim, P.dfTrim, 0]);
});

test('轻量复位（悬停）：保留 vtolMode/altHold/useBtrue/输入，复位到机头朝天', () => {
  const sim = createSimulationState(P);
  resetVtolHoverState(sim, P);
  sim.S.altHold = true; sim.S.useBtrue = true;
  sim.S.thr = 0.7; sim.S.dt = 0.2;
  sim.S.time = 3.2;
  sim.S.omega.x = 2; sim.F.vWorld.y = 15; sim.F.pos.z = -30;
  sim.S.quat.x = 0.5; sim.S.intAlt = 1.2; sim.S.dtAct = 0.3;

  resetPoseOnly(sim, P);

  assert.equal(sim.S.vtolMode, true, '悬停模式保留');
  assert.equal(sim.S.altHold, true, '定高保留');
  assert.equal(sim.S.useBtrue, true, 'B_true 保留');
  assert.equal(sim.S.thr, 0.7, '油门保留');
  assert.equal(sim.S.dt, 0.2, '倾斜指令保留');
  assert.equal(sim.S.time, 3.2, '时间不归零');
  assert.ok(Math.abs(sim.S.quat.x - Q_HOVER.x) < 1e-12 && Math.abs(sim.S.quat.w - Q_HOVER.w) < 1e-12,
    '姿态复位到机头朝天');
  assert.equal(sim.F.pos.z, 0, '高度复位');
  assert.equal(sim.S.omega.x, 0, '角速度清零');
  assert.equal(sim.S.intAlt, 0, '定高积分器清零（防旧积分瞬态）');
  assert.equal(sim.S.dtAct, 0, '悬停执行器回 0 摆角基准');
  assert.ok(Math.abs(sim.S.wf - 0.7 * P.wMax) < 1e-9, '转速跟随保留油门');
});
