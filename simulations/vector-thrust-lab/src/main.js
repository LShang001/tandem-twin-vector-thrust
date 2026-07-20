// ============================================================
//  纵列双发矢量推力布局 · 六维力/力矩动力学可视化
//  引导与主循环装配：core（纯计算） + browser（Three.js/DOM 适配）
//  机体系约定: x 前 / y 右 / z 下
// ============================================================
import * as THREE from 'three';
import { P } from './core/parameters.mjs';
import { createSimulationState } from './core/state.mjs';
import { stepPhysics } from './core/dynamics.mjs';
import { getTelemetry } from './core/telemetry.mjs';
import { createScene } from './browser/scene.mjs';
import { createEffects } from './browser/effects.mjs';
import { createAircraftView, updateAircraftView } from './browser/aircraft-view.mjs';
import { createHud } from './browser/hud.mjs';
import { createControlsUI } from './browser/controls-ui.mjs';
import { createScope } from './browser/scope.mjs';
import { createDemo } from './browser/demo.mjs';
import { createTheme } from './browser/theme.mjs';

// ---------- 装配 ----------
const sim = createSimulationState(P);
const sceneCtx = createScene(document.getElementById('scene'));
const effects = createEffects(sceneCtx.scene);
const view = createAircraftView(sceneCtx.scene);
const hud = createHud(document.getElementById('meters'));
const hooks = { stopDemo: () => {} };
const ui = createControlsUI({ sim, P, hooks });
const demo = createDemo({ sim, P, controls: sceneCtx.controls, ui });
hooks.stopDemo = demo.stopDemo;
createTheme({ scene: sceneCtx.scene, bloom: sceneCtx.bloom, lights: sceneCtx.lights, effects });

// ---------- 示波器（角速度波形，按钮开关） ----------
const scope = createScope(sim);
document.getElementById('b-scope').addEventListener('click', () => {
  scope.setVisible(!scope.visible);
  document.getElementById('b-scope').classList.toggle('active', scope.visible);
});

// ---------- 初始同步 ----------
ui.syncFromUI();
sim.S.wf = sim.S.wt = sim.S.thr * P.wMax; sim.prevWf = sim.S.wf; sim.prevWt = sim.S.wt;
sim.S.dtAct = sim.S.dt; sim.S.dfAct = sim.S.df; sim.S.dwAct = sim.S.dw;

// ---------- 主循环 ----------
const clock = new THREE.Clock();
let frame = 0;
function animate() {
  requestAnimationFrame(animate);
  const dt = Math.min(clock.getDelta(), P.frameCap);
  sim.S.time += dt;
  demo.demoStep(sim.S.time);
  stepPhysics(sim, P, dt);
  updateAircraftView(view, sim, P, dt);
  effects.update(dt, sim);
  if (++frame % 3 === 0) hud.sync(getTelemetry(sim));
  scope.update(performance.now());
  sceneCtx.controls.update();
  sceneCtx.composer.render();
}

animate();
setTimeout(() => { document.getElementById('loader').classList.add('done'); demo.startDemo('cine'); }, 900);
