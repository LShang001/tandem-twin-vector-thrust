// ============================================================
//  py-web-lab 前端装配：WebSocket 客户端 + Three.js 渲染
//  —— 所有物理/控制逻辑在 Python 服务端；本文件只做
//     指令发送 + 状态渲染（零计算）。
// ============================================================
import * as THREE from 'three';
import { createScene } from './browser/scene.mjs';
import { createEffects } from './browser/effects.mjs';
import { createAircraftView, updateAircraftView } from './browser/aircraft-view.mjs';
import { createHud } from './browser/hud.mjs';
import { createScope } from './browser/scope.mjs';
import { createTheme } from './browser/theme.mjs';

// ---------- 本地状态代理（形状 = 服务端 payload，供渲染模块消费） ----------
const sim = { S: null, F: null, dyn: null, aero: null, P: null };

// ---------- 渲染装配 ----------
const sceneCtx = createScene(document.getElementById('scene'));
const effects = createEffects(sceneCtx.scene);
const view = createAircraftView(sceneCtx.scene);
const hud = createHud(document.getElementById('meters'));
createTheme({ scene: sceneCtx.scene, bloom: sceneCtx.bloom, lights: sceneCtx.lights, effects });

const scope = createScope(sim);
document.getElementById('b-scope').addEventListener('click', () => {
  scope.setVisible(!scope.visible);
  document.getElementById('b-scope').classList.toggle('active', scope.visible);
});
document.getElementById('b-theme').addEventListener('click', () => {
  document.body.classList.toggle('light');
});

// ---------- WebSocket ----------
const D2R = Math.PI / 180;
let ws = null;
let pending = false;      // 有未返回的 step
let lastSent = 0;
let connected = false;

const connEl = document.getElementById('conn');
const loaderEl = document.getElementById('loader');
const engineTag = document.getElementById('engine-tag');

function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
}

function applyState(payload) {
  sim.S = payload.S;
  sim.F = payload.F;
  sim.dyn = payload.dyn;
  sim.aero = payload.aero;
  if (payload.params) sim.P = payload.params;
}

// ---------- 遥测组装（结构对齐 vector-thrust-lab getTelemetry） ----------
function getTelemetry() {
  const { S, F, dyn, aero } = sim;
  return {
    forces: { ...dyn },
    aero: { ...aero },
    rotors: { wf: S.wf, wt: S.wt },
    actuators: { dtAct: S.dtAct, dfAct: S.dfAct, dwAct: S.dwAct },
    commands: { thr: S.thr, dt: S.dt, df: S.df, dw: S.dw },
    flight: { pos: { ...F.pos }, vel: { ...F.vel }, vWorld: { ...F.vWorld },
              euler: { ...F.euler }, quat: { ...S.quat }, omega: { ...S.omega } },
    flags: { sasMode: S.sasMode, sas: S.sasMode > 0, aero: S.aero, lockXY: S.lockXY },
    time: S.time,
  };
}

// ---------- UI 状态刷新（标签/按钮跟随服务端模式） ----------
const SAS_LABELS = ['直通', '全 SAS', '阻尼', '角速度'];

function refreshModeUI() {
  if (!sim.S) return;
  const S = sim.S;
  const vtol = S.vtolMode;
  document.getElementById('b-vtol').textContent = `悬停模式：${vtol ? '开' : '关'}`;
  document.getElementById('b-vtol').classList.toggle('active', vtol);
  document.getElementById('b-sas').textContent =
    vtol ? `自稳：${S.sasMode === 0 ? '关（直通）' : '开（四元数）'}`
         : `增稳 SAS：${SAS_LABELS[S.sasMode] ?? S.sasMode}`;
  document.getElementById('b-sas').classList.toggle('active', S.sasMode !== 0);
  document.getElementById('b-aero').textContent = `气动力：${S.aero ? '开' : '忽略'}`;
  document.getElementById('b-aero').classList.toggle('active', S.aero);
  document.getElementById('b-btrue').style.display = vtol ? '' : 'none';
  document.getElementById('b-btrue').textContent = `B_true 分配：${S.useBtrue ? '开' : '关'}`;
  document.getElementById('b-btrue').classList.toggle('active', S.useBtrue);
  document.getElementById('b-alt').style.display = vtol ? '' : 'none';
  document.getElementById('b-alt').textContent = `定高：${S.altHold ? '开' : '关'}`;
  document.getElementById('b-alt').classList.toggle('active', S.altHold);
  document.getElementById('b-pause').textContent = S.paused ? '▶ 继续' : '⏸ 暂停';
  document.getElementById('b-pause').classList.toggle('active', S.paused);
  document.getElementById('row-alt').style.display = (vtol && S.altHold) ? '' : 'none';
  // 滑块语义标签
  document.getElementById('lbl-dt').innerHTML = vtol
    ? '俯仰倾斜 θ（绕 y_b）' : '俯仰摆角 δ<sub>t</sub>（尾电机·绕 y）';
  document.getElementById('lbl-df').innerHTML = vtol
    ? '侧倾 φ（绕 z_b）' : '偏航摆角 δ<sub>f</sub>（前电机·绕 z）';
  document.getElementById('lbl-dw').innerHTML = vtol
    ? '航向角速度 ψ̇（绕 x_b，差速）' : '差速 Δω（前 ⊕ / 尾 ⊖ → 滚转）';
  // 滑块范围随模式切换
  const dwEl = document.getElementById('s-dw');
  if (vtol) { dwEl.min = '-80'; dwEl.max = '80'; } else { dwEl.min = '-30'; dwEl.max = '30'; }
  syncSliders();
}

function syncSliders() {
  const S = sim.S;
  const set = (id, v) => { const el = document.getElementById(id); if (el !== document.activeElement) el.value = v; };
  set('s-thr', (S.thr * 100).toFixed(1));
  set('s-dt', (S.dt / D2R).toFixed(1));
  set('s-df', (S.df / D2R).toFixed(1));
  set('s-dw', (S.dw / D2R).toFixed(1));
  set('s-alt', S.altRef);
  // 数值显示
  const txt = (id, s) => { document.getElementById(id).textContent = s; };
  txt('v-thr', (S.thr * 100).toFixed(0) + '%');
  txt('v-dt', (S.dt / D2R).toFixed(1) + '°');
  txt('v-df', (S.df / D2R).toFixed(1) + '°');
  txt('v-dw', S.vtolMode ? (S.dw / D2R).toFixed(1) + '°/s' : (S.dw * 100).toFixed(1) + '%');
  txt('v-alt', S.altRef.toFixed(1) + 'm');
}

// ---------- 滑块/按钮事件 → 服务端 ----------
function bindControls() {
  const bindSlider = (id, fn) => {
    const el = document.getElementById(id);
    el.addEventListener('input', () => { fn(parseFloat(el.value)); });
  };
  bindSlider('s-thr', v => send({ cmd: 'set', S: { thr: v / 100 } }));
  bindSlider('s-dt', v => send({ cmd: 'set', S: { dt: v * D2R } }));
  bindSlider('s-df', v => send({ cmd: 'set', S: { df: v * D2R } }));
  bindSlider('s-dw', v => send({ cmd: 'set', S: { dw: v * D2R } }));
  bindSlider('s-alt', v => send({ cmd: 'set', S: { altRef: v } }));

  document.getElementById('b-sas').addEventListener('click', () => {
    const next = sim.S.vtolMode ? (sim.S.sasMode === 0 ? 1 : 0) : (sim.S.sasMode + 1) % 4;
    send({ cmd: 'set', S: { sasMode: next } });
  });
  document.getElementById('b-aero').addEventListener('click', () => {
    send({ cmd: 'set', S: { aero: !sim.S.aero } });
  });
  document.getElementById('b-vtol').addEventListener('click', () => {
    if (!sim.S.vtolMode) send({ cmd: 'reset', mode: 'vtol' });
    else send({ cmd: 'reset', mode: 'cruise' });   // 退出悬停 → 全复位回巡航配平
  });
  document.getElementById('b-btrue').addEventListener('click', () => {
    send({ cmd: 'set', S: { useBtrue: !sim.S.useBtrue } });
  });
  document.getElementById('b-alt').addEventListener('click', () => {
    send({ cmd: 'set', S: { altHold: !sim.S.altHold, altRef: parseFloat(document.getElementById('s-alt').value) } });
  });
  document.getElementById('b-reset').addEventListener('click', () => {
    send({ cmd: 'reset', mode: 'pose' });
  });
  document.getElementById('b-pause').addEventListener('click', () => {
    send({ cmd: 'set', S: { paused: !sim.S.paused } });
  });
}

// ---------- 主循环：帧步进（请求-响应自适应） ----------
let frame = 0;
const clock = new THREE.Clock();
let prevT = 0;

function animate(now) {
  requestAnimationFrame(animate);
  if (!connected || !sim.S) return;
  // 帧时间（首次 1/60）
  const dt = Math.min((now - prevT) / 1000 || 1 / 60, 0.1);
  prevT = now;
  if (!pending) {
    pending = true;
    lastSent = now;
    send({ cmd: 'step', dt });
  }
  // 渲染（用最新状态，即使 step 未返回）
  const dtR = Math.min((now - lastSent) / 1000, 0.05);
  updateAircraftView(view, sim, sim.P, dtR);
  effects.update(dtR, sim);
  if (++frame % 3 === 0) hud.sync(getTelemetry());
  scope.update(now);
  sceneCtx.controls.update();
  sceneCtx.composer.render();
}

function connect() {
  pending = false;   // 重置在途标志，防止重连后旧响应永不到达而卡死（review 复查）
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = () => {
    connected = true;
    connEl.textContent = '已连接 · Python 引擎';
    connEl.classList.add('ok');
    engineTag.textContent = 'SERVER: FastAPI + core.py';
    send({ cmd: 'init' });
  };
  ws.onmessage = ev => {
    let msg = null;
    try { msg = JSON.parse(ev.data); } catch (e) { console.error('[py-web-lab] bad JSON', e); }
    if (!msg) { pending = false; return; }
    if (msg.type === 'state') {
      applyState(msg);
      if (msg.cmd === 'step') pending = false;   // 仅 step 响应复位（review #4）
      refreshModeUI();
      loaderEl.classList.add('done');
    } else if (msg.type === 'error') {
      console.error('[py-web-lab]', msg.msg);
      if (msg.cmd === 'step') pending = false;
    }
  };
  ws.onclose = () => {
    connected = false;
    connEl.textContent = '连接断开 · 重连中…';
    connEl.classList.remove('ok');
    setTimeout(connect, 1500);
  };
  ws.onerror = () => { try { ws.close(); } catch { /* noop */ } };
}

// 启动
bindControls();
connect();
requestAnimationFrame(animate);
