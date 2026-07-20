// control.mjs（SAS）单元测试
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { P } from '../src/core/parameters.mjs';
import { createSimulationState } from '../src/core/state.mjs';
import { applySas } from '../src/core/control.mjs';

const DT = 0.004;

test('SAS 关闭时指令直通', () => {
  const sim = createSimulationState(P);
  sim.S.sasMode = 0;
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

test('仅角速率模式（sasMode=2）：有阻尼反馈，无姿态比例/积分', () => {
  const sim = createSimulationState(P);
  sim.S.sasMode = 2;
  sim.S.omega.y = 0.5; sim.F.euler.y = 0.1;
  const prevIntTh = sim.S.intTh;
  applySas(sim, P, DT);
  // 角速率阻尼生效
  assert.ok(sim.S.dtAct > 0);
  // 积分器不应累加
  assert.equal(sim.S.intTh, prevIntTh);
  // 反馈仅来自阻尼项（无姿态比例贡献），修正量应小于全 SAS
  const dtRateOnly = sim.S.dtAct;
  const sim2 = createSimulationState(P);
  sim2.S.sasMode = 1;
  sim2.S.omega.y = 0.5; sim2.F.euler.y = 0.1;
  applySas(sim2, P, DT);
  assert.ok(Math.abs(dtRateOnly) < Math.abs(sim2.S.dtAct),
    '仅角速率模式的修正量应小于全 SAS 模式');
});

test('角速度闭环模式（sasMode=3）：滑块 = ω_ref，P 控制追零误差', () => {
  const sim = createSimulationState(P);
  sim.S.sasMode = 3;
  sim.S.dt = 1.0;   // q_ref = 1 rad/s（目标俯仰角速度）
  sim.S.omega.y = 0;  // 当前角速度 = 0, 需要加速
  applySas(sim, P, DT);
  // ω < ω_ref → (ω−ω_ref) < 0 → dtAct < 0 → M_y>0 → q̇>0 → 加速 ✓
  assert.ok(sim.S.dtAct < 0, '当前角速度小于目标时应产生加速力矩（dtAct<0）');
  assert.ok(Math.abs(sim.S.dtAct) <= P.dMax, '摆角不超限');

  // 当前角速度 = 目标 → 误差为零 → 指令为零
  const sim2 = createSimulationState(P);
  sim2.S.sasMode = 3;
  sim2.S.dt = 1.0; sim2.S.omega.y = 1.0;
  applySas(sim2, P, DT);
  assert.ok(Math.abs(sim2.S.dtAct) < 1e-9, '零误差时指令应为零');

  // 超调：当前 > 目标 → 需要减速 → dtAct > 0
  const sim3 = createSimulationState(P);
  sim3.S.sasMode = 3;
  sim3.S.dt = 1.0; sim3.S.omega.y = 2.0;
  applySas(sim3, P, DT);
  assert.ok(sim3.S.dtAct > 0, '当前角速度大于目标时应产生减速力矩（dtAct>0）');
});
