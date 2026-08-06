// 交叉验证（JS 侧）：vector-thrust-lab web core 跑同场景 → JSON 轨迹
// 与 run_py.py 同场景，由 compare.mjs 对比（门槛 <1e-6）
import { writeFileSync, mkdirSync } from 'node:fs';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { dirname, resolve } from 'node:path';

const HERE = dirname(fileURLToPath(import.meta.url));
const SIM = resolve(HERE, '../../../../simulations/vector-thrust-lab/src/core');
const OUT = resolve(HERE, 'cross');
const { P } = await import(pathToFileURL(resolve(SIM, 'parameters.mjs')).href);
const { createSimulationState, resetSimulationState, resetVtolHoverState } =
  await import(pathToFileURL(resolve(SIM, 'state.mjs')).href);
const { stepPhysics } = await import(pathToFileURL(resolve(SIM, 'dynamics.mjs')).href);

const DT = 0.004;
const D2R = Math.PI / 180;

function run(scene, secs) {
  const sim = createSimulationState(P);
  if (scene === 'vtol') {
    resetVtolHoverState(sim, P);
  } else if (scene === 'vtol_pert') {
    resetVtolHoverState(sim, P);
    sim.S.omega.y = 0.3;
  } else if (scene === 'vtol_alt') {
    resetVtolHoverState(sim, P);
    sim.S.altHold = true;
    sim.S.altRef = 5;
  } else if (scene === 'vtol_btrue') {
    resetVtolHoverState(sim, P);
    sim.S.useBtrue = true;
    sim.S.omega.y = 0.3; sim.S.omega.z = 0.2;
  } else if (scene === 'cruise_sas') {
    resetSimulationState(sim, P);
    sim.S.dt = 3 * D2R; sim.S.df = -2 * D2R; sim.S.dw = 0.1;
  } else if (scene === 'vtol_dw') {
    resetVtolHoverState(sim, P);
    sim.S.dw = 30 * D2R;
  } else {
    throw new Error(scene);
  }
  const trace = [];
  const steps = Math.round(secs / DT);
  for (let i = 0; i < steps; i++) {
    sim.S.time += DT;
    stepPhysics(sim, P, DT);
    if (i % 25 === 0) {
      const { S, F } = sim;
      trace.push({
        t: +(i * DT).toFixed(4),
        quat: [S.quat.x, S.quat.y, S.quat.z, S.quat.w],
        omega: [S.omega.x, S.omega.y, S.omega.z],
        vel: [F.vel.x, F.vel.y, F.vel.z],
        pos: [F.pos.x, F.pos.y, F.pos.z],
        thr: S.thr,
        dtAct: S.dtAct, dfAct: S.dfAct, dwAct: S.dwAct,
        intAlt: S.intAlt, intTh: S.intTh, intPhi: S.intPhi,
      });
    }
  }
  const { S, F } = sim;
  trace.push({
    t: +(steps * DT).toFixed(4),
    quat: [S.quat.x, S.quat.y, S.quat.z, S.quat.w],
    omega: [S.omega.x, S.omega.y, S.omega.z],
    vel: [F.vel.x, F.vel.y, F.vel.z],
    pos: [F.pos.x, F.pos.y, F.pos.z],
    thr: S.thr,
    dtAct: S.dtAct, dfAct: S.dfAct, dwAct: S.dwAct,
    intAlt: S.intAlt, intTh: S.intTh, intPhi: S.intPhi,
  });
  return trace;
}

const SCENES = {
  vtol: 8.0, vtol_pert: 8.0, vtol_alt: 12.0,
  vtol_btrue: 8.0, cruise_sas: 5.0, vtol_dw: 8.0,
};

mkdirSync(OUT, { recursive: true });
for (const [scene, secs] of Object.entries(SCENES)) {
  const trace = run(scene, secs);
  writeFileSync(resolve(OUT, `js-${scene}.json`), JSON.stringify(trace));
  const last = trace[trace.length - 1];
  console.log(`[OK] js-${scene}: ${trace.length} 点, 末点 thr=${last.thr.toFixed(4)} ` +
    `omega=(${last.omega.map(x => x.toFixed(4)).join(',')})`);
}
