/**
 * @file    scenarios.mjs
 * @module  仿真 tests/场景运行器
 * @brief   用固定步长驱动 core 复现标准机动，供回归比对与基线生成共用
 * @details 输入：时长、每帧控制回调、初始化回调；处理：复位 → 按 1/60 s 帧步进
 *          （stepPhysics 内部再按 4 ms 子步细分）→ 每秒记录一次快照；
 *          输出：{final, samples}。
 *          不变量：帧步长固定 1/60 s、每秒 60 帧采样一次——与 UI 主循环同构，
 *          保证基线可由浏览器外复现。
 * @theory  场景对应 MOD-001 §7.3 的演示性数值检查（配平/俯仰/滚转/偏航阶跃）
 * @frame   机体系 FRD / NED
 * @units   s、m、m/s、rad、rad/s、N、N·m
 * @note 本文件不是测试文件（无 test()），被 regression.test.mjs 与 fixtures/generate.mjs 复用；
 *       改动采样口径会同时改基线 → 属行为红线流程
 * @see     tests/regression.test.mjs、tests/fixtures/generate.mjs
 */
import { P } from '../src/core/parameters.mjs';
import { createSimulationState, resetSimulationState } from '../src/core/state.mjs';
import { stepPhysics } from '../src/core/dynamics.mjs';

const FRAME_DT = 1 / 60;

function snapshot(sim) {
  const { S, F, dyn, aero } = sim;
  return {
    t: S.time,
    V: aero.V, alpha: aero.alpha,
    phi: F.euler.x, theta: F.euler.y, psi: F.euler.z,
    p: S.omega.x, q: S.omega.y, r: S.omega.z,
    x: F.pos.x, y: F.pos.y, z: F.pos.z,
    Tf: dyn.Tf, Tt: dyn.Tt, Mx: dyn.Mx, My: dyn.My, Mz: dyn.Mz,
  };
}

// 运行一个场景：先执行 setup，再按帧调用 control(sim, frameIndex)
// 每秒（每 60 帧）记录一次快照
export function runScenario(durationS, control = () => {}, setup = () => {}) {
  const sim = createSimulationState(P);
  resetSimulationState(sim, P);
  setup(sim);
  const steps = Math.round(durationS / FRAME_DT);
  const samples = [];
  for (let i = 0; i < steps; i++) {
    control(sim, i);
    sim.S.time += FRAME_DT;
    stepPhysics(sim, P, FRAME_DT);
    if ((i + 1) % 60 === 0) samples.push(snapshot(sim));
  }
  return { final: snapshot(sim), samples };
}

export const SCENARIOS = {
  // A: 配平直飞 10 s（VTOL 参数集 v0.2.0：12 m/s 巡航配平）
  trim: () => runScenario(10),
  // B: 离地自由飞行俯仰阶跃，δt=8° 保持 4 s 后撤除，共 8 s
  pitchStep: () => runScenario(
    8,
    (sim, i) => { sim.S.dt = (i < 240) ? 8 * Math.PI / 180 : 0; },
    (sim) => { sim.F.pos.z = -100; }, // 留出离地高度，避免响应混入地面钳位
  ),
  // C: 滚转阶跃 Δω=25%（UI 幅值内）保持 4 s 后撤除，共 8 s
  rollStep: () => runScenario(8, (sim, i) => {
    sim.S.dw = (i < 240) ? 0.25 * P.dwUiMax : 0;
  }),
  // D: 偏航阶跃 δ_f=8°（绝对前摆指令，配平偏置 dfTrim 之上）保持 4 s 后撤除，共 8 s
  yawStep: () => runScenario(8, (sim, i) => {
    sim.S.df = (i < 240) ? 8 * Math.PI / 180 : 0;
  }),
};
