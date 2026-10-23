/**
 * @file    telemetry.test.mjs
 * @module  仿真 tests/遥测快照
 * @brief   锁定遥测分组键集合、快照隔离性与 flags 派生
 * @details 覆盖：八组键名（actuators/aero/commands/flags/flight/forces/rotors/time）；
 *          修改快照不影响 sim；flags 反映 sasMode/aero/lockXY。
 *          容差依据：无浮点计算，全部 deepEqual/equal 精确断言。
 * @theory  N/A（数据搬运）；字段语义 @ref MOD-002 §8
 * @frame   继承 sim 约定
 * @units   继承 sim 约定
 * @note 键集合断言是 HUD 的接口契约（改名=破坏下游），刻意逐字比对
 * @see     src/core/telemetry.mjs、src/browser/hud.mjs
 */
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { P } from '../src/core/parameters.mjs';
import { createSimulationState } from '../src/core/state.mjs';
import { getTelemetry } from '../src/core/telemetry.mjs';

test('遥测快照包含全部遥测分组且为纯数据（无引用共享）', () => {
  const sim = createSimulationState(P);
  sim.S.thr = 0.6; sim.S.dt = 0.1; sim.S.df = -0.05; sim.S.dw = 0.2;
  sim.S.wf = 300; sim.S.wt = 250;
  sim.dyn.Tf = 1.23; sim.aero.V = 12.5;
  const t = getTelemetry(sim);
  assert.deepEqual(Object.keys(t).sort(),
    ['actuators', 'aero', 'commands', 'flags', 'flight', 'forces', 'rotors', 'time']);
  assert.equal(t.rotors.wf, 300);
  assert.equal(t.forces.Tf, 1.23);
  assert.equal(t.aero.V, 12.5);
  assert.equal(t.commands.thr, 0.6);
  assert.equal(t.actuators.dtAct, sim.S.dtAct);
  // 快照必须是拷贝：修改遥测对象不影响 sim 内部状态
  t.forces.Tf = 999; t.flight.pos.x = 999; t.rotors.wf = 999;
  assert.equal(sim.dyn.Tf, 1.23);
  assert.equal(sim.F.pos.x, 0);
  assert.equal(sim.S.wf, 300);
});

test('flags 反映 SAS 模式/气动/水平锁定开关', () => {
  const sim = createSimulationState(P);
  sim.S.sasMode = 0; sim.S.aero = false; sim.S.lockXY = true;
  let t = getTelemetry(sim);
  assert.deepEqual(t.flags, { sasMode: 0, sas: false, aero: false, lockXY: true });
  sim.S.sasMode = 3; sim.S.aero = true; sim.S.lockXY = false;
  t = getTelemetry(sim);
  assert.deepEqual(t.flags, { sasMode: 3, sas: true, aero: true, lockXY: false });
});

test('飞行状态分组携带位置/速度/姿态/角速度四元数', () => {
  const sim = createSimulationState(P);
  sim.F.pos = { x: 1, y: 2, z: 3 };
  sim.F.vel = { x: 10, y: 0, z: 0 };
  sim.S.quat = { x: 0.1, y: 0.2, z: 0.3, w: 0.9 };
  const f = getTelemetry(sim).flight;
  assert.deepEqual(f.pos, { x: 1, y: 2, z: 3 });
  assert.deepEqual(f.vel, { x: 10, y: 0, z: 0 });
  assert.deepEqual(f.quat, { x: 0.1, y: 0.2, z: 0.3, w: 0.9 });
  assert.deepEqual(Object.keys(f).sort(), ['euler', 'omega', 'pos', 'quat', 'vWorld', 'vel']);
});
