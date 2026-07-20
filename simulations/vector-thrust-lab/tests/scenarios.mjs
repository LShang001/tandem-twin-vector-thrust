// ============================================================
//  测试场景运行器：固定步长驱动 core，供回归基线生成与比对
//  （本文件不是测试，由 regression 测试与 fixtures/generate.mjs 共用）
// ============================================================
import { P } from '../src/core/parameters.mjs';
import { createSimulationState } from '../src/core/state.mjs';
import { stepPhysics } from '../src/core/dynamics.mjs';

const FRAME_DT = 1 / 60;

function initRotors(sim) {
  sim.S.wf = sim.S.wt = sim.S.thr * P.wMax;
  sim.prevWf = sim.S.wf; sim.prevWt = sim.S.wt;
  sim.S.dtAct = sim.S.dt; sim.S.dfAct = sim.S.df; sim.S.dwAct = sim.S.dw;
}

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

// 运行一个场景：steps 帧，每帧前调用 control(sim, frameIndex)
// 每秒（每 60 帧）记录一次快照
export function runScenario(durationS, control = () => {}) {
  const sim = createSimulationState(P);
  initRotors(sim);
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
  // A: 配平直飞 10 s
  trim: () => runScenario(10),
  // B: 俯仰阶跃 δt=8° 保持 4 s 后撤除，共 8 s
  pitchStep: () => runScenario(8, (sim, i) => {
    sim.S.dt = (i < 240) ? 8 * Math.PI / 180 : 0;
  }),
  // C: 滚转阶跃 Δω=25%（UI 幅值内）保持 4 s 后撤除，共 8 s
  rollStep: () => runScenario(8, (sim, i) => {
    sim.S.dw = (i < 240) ? 0.25 * P.dwUiMax : 0;
  }),
};
