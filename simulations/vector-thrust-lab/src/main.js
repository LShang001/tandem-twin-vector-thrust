/**
 * @file    main.js
 * @module  仿真 browser/入口装配
 * @brief   装配 core 状态机与 browser 渲染层，运行 requestAnimationFrame 主循环
 * @details 输入：DOM 容器与参数表；处理：构造 sim → 场景/效果/模型/HUD/UI/演示，
 *          每帧 demo → stepPhysics → 渲染同步 → 后处理；
 *          输出：持续更新的三维画面与 HUD。
 *          不变量：物理步长受 frameCap=0.05 s 截断；HUD 每 3 帧同步一次（降 DOM 压力）。
 * @theory  N/A（装配层）；物理与控制在 core/ 实现 @ref MOD-001 §5
 * @frame   机体系 FRD（动力学）/ NED（位置与地速）；渲染世界系与 NED 同向
 * @units   s（时间）、rad（姿态/摆角）、m（位置）
 * @redline 帧 delta ≤ 0.05 s 是行为红线——截断必须保留，否则掉帧时会一次性积掉大 dt
 * @note 初始化时把 dtAct/dfAct 置为配平偏置：否则首帧 SAS 会从 0 摆角开始把飞机"拽"一下
 * @see     core/dynamics.mjs（stepPhysics）、browser/scene.mjs（渲染上下文）
 */
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
sim.S.dtAct = P.dtTrim; sim.S.dfAct = P.dfTrim; sim.S.dwAct = sim.S.dw;

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
