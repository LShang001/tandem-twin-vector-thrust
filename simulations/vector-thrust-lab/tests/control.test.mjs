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
  assert.equal(sim.S.dtAct, P.dtTrim + 0.1);
  assert.equal(sim.S.dfAct, P.dfTrim - 0.05); // 前摆指令含配平偏置 dfTrim
  assert.equal(sim.S.dwAct, 0.2);
});

test('俯仰姿态反馈与 theta_dot=-q 一致：正误差减小尾摆指令', () => {
  const sim = createSimulationState(P);
  sim.F.euler.y = -P.aTrim + 0.1;
  sim.S.omega.y = 0;
  applySas(sim, P, DT);
  assert.ok(sim.S.dtAct < P.dtTrim, 'theta 偏正时应产生使 q 增大、theta 回落的修正');
});

test('偏航反馈极性：正 r 减小幅前摆指令（正效率通道取负号）', () => {
  const sim = createSimulationState(P);
  sim.S.omega.z = 0.5;
  applySas(sim, P, DT);
  assert.ok(sim.S.dfAct < 0);
});

test('滚转姿态反馈与 phi_dot=-p 一致：正 phi 减小差速', () => {
  const sim = createSimulationState(P);
  sim.F.euler.x = 0.1;
  sim.S.omega.x = 0;
  applySas(sim, P, DT);
  assert.ok(sim.S.dwAct < 0);
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
  // 角速率阻尼生效（增量相对配平偏置为正）
  assert.ok(sim.S.dtAct > P.dtTrim, `dtAct=${sim.S.dtAct} 应大于配平偏置 ${P.dtTrim}`);
  // 积分器不应累加
  assert.equal(sim.S.intTh, prevIntTh);
  // 反馈仅来自阻尼项（无姿态比例贡献）
  const dtRateOnly = sim.S.dtAct;
  assert.ok(Math.abs(dtRateOnly - (P.dtTrim + P.sasQ * 0.5)) < 1e-12);
  const sim2 = createSimulationState(P);
  sim2.S.sasMode = 1;
  sim2.S.omega.y = 0.5; sim2.F.euler.y = 0.1;
  applySas(sim2, P, DT);
  assert.ok(sim2.S.dtAct < dtRateOnly, '全 SAS 应附加与显示角运动学一致的姿态恢复项');
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

  // 当前角速度 = 目标 → 仅保留配平偏置
  const sim2 = createSimulationState(P);
  sim2.S.sasMode = 3;
  sim2.S.dt = 1.0; sim2.S.omega.y = 1.0;
  applySas(sim2, P, DT);
  assert.ok(Math.abs(sim2.S.dtAct - P.dtTrim) < 1e-9, '零误差时应保留配平偏置');

  // 超调：当前 > 目标 → 需要减速 → dtAct > 0
  const sim3 = createSimulationState(P);
  sim3.S.sasMode = 3;
  sim3.S.dt = 1.0; sim3.S.omega.y = 2.0;
  applySas(sim3, P, DT);
  assert.ok(sim3.S.dtAct > 0, '当前角速度大于目标时应产生减速力矩（dtAct>0）');
});

test('sasMode 切换时积分器清零，防止旧累积值瞬态冲击', () => {
  const sim = createSimulationState(P);
  sim.S.sasMode = 1;
  sim.F.euler.y = 1.0; // 持续大误差 → 积分累积
  for (let i = 0; i < 1000; i++) applySas(sim, P, DT);
  assert.ok(Math.abs(sim.S.intTh) > 0.1, '前置：积分器应有累积');
  // 切到 mode 2 再切回 mode 1：积分器应清零
  sim.S.sasMode = 2;
  applySas(sim, P, DT);
  assert.equal(sim.S.intTh, 0, '切换瞬间积分器清零');
  assert.equal(sim.S.intPhi, 0);
  // 模式未变时积分器继续累积（不误清零）
  sim.S.sasMode = 1; sim.S._prevSasMode = 1;
  sim.F.euler.y = 0.5;
  for (let i = 0; i < 100; i++) applySas(sim, P, DT);
  assert.ok(sim.S.intTh > 0, '模式不变时积分器应继续累积');
  // 切回 mode 2 再次清零
  sim.S.sasMode = 2;
  applySas(sim, P, DT);
  assert.equal(sim.S.intTh, 0);
});

test('角速度闭环模式（sasMode=3）：偏航通道，前摆正效率通道取 (rRef−r)', () => {
  const sim = createSimulationState(P);
  sim.S.sasMode = 3;
  sim.S.df = 1.0;   // r_ref = 1 rad/s（目标偏航角速度）
  sim.S.omega.z = 0; // 当前偏航角速度 = 0
  applySas(sim, P, DT);
  // r < r_ref → (rRef−r) > 0 → dfAct > dfTrim → M_z 增大 → ṙ > 0 加速 ✓
  assert.ok(sim.S.dfAct > P.dfTrim, '当前偏航角速度小于目标时应产生加速修正（dfAct>dfTrim）');
  assert.ok(Math.abs(sim.S.dfAct) <= P.dMax, '前摆不超限');

  // 当前 = 目标 → 仅保留配平偏置
  const sim2 = createSimulationState(P);
  sim2.S.sasMode = 3;
  sim2.S.df = 1.0; sim2.S.omega.z = 1.0;
  applySas(sim2, P, DT);
  assert.ok(Math.abs(sim2.S.dfAct - P.dfTrim) < 1e-9, '零误差时应保留配平偏置');

  // 超调：当前 > 目标 → 需要减速 → dfAct < dfTrim
  const sim3 = createSimulationState(P);
  sim3.S.sasMode = 3;
  sim3.S.df = 1.0; sim3.S.omega.z = 2.0;
  applySas(sim3, P, DT);
  assert.ok(sim3.S.dfAct < P.dfTrim, '当前偏航角速度大于目标时应产生减速修正（dfAct<dfTrim）');
});

test('角速度闭环模式（sasMode=3）：滚转通道，差速效率为负取 (ω−pRef)', () => {
  const sim = createSimulationState(P);
  sim.S.sasMode = 3;
  sim.S.dw = 1.0;   // p_ref = 1 rad/s（目标滚转角速度）
  sim.S.omega.x = 0; // 当前滚转角速度 = 0
  applySas(sim, P, DT);
  // ω < p_ref → (ω−pRef) < 0 → dwAct < 0 → M_x > 0 → ṗ > 0 加速 ✓
  assert.ok(sim.S.dwAct < 0, '当前滚转角速度小于目标时应产生加速修正（dwAct<0）');
  assert.ok(Math.abs(sim.S.dwAct) <= P.dwMax, '差速不超限');

  // 当前 = 目标 → 差速修正归零（差速无配平偏置）
  const sim2 = createSimulationState(P);
  sim2.S.sasMode = 3;
  sim2.S.dw = 1.0; sim2.S.omega.x = 1.0;
  applySas(sim2, P, DT);
  assert.ok(Math.abs(sim2.S.dwAct) < 1e-9, '零误差时差速修正应为零');

  // 超调：当前 > 目标 → 需要减速 → dwAct > 0
  const sim3 = createSimulationState(P);
  sim3.S.sasMode = 3;
  sim3.S.dw = 1.0; sim3.S.omega.x = 2.0;
  applySas(sim3, P, DT);
  assert.ok(sim3.S.dwAct > 0, '当前滚转角速度大于目标时应产生减速修正（dwAct>0）');
});
