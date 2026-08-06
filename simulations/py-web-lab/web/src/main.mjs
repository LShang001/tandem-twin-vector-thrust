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
  document.getElementById('b-hover').textContent = `水平约束：${S.lockXY ? '开' : '关'}`;
  document.getElementById('b-hover').classList.toggle('active', S.lockXY);
  const ctrlLabel = { sas: 'SAS', indi: 'INDI', lqr: 'LQR', adrc: 'ADRC' };
  document.getElementById('b-ctrl').textContent = `控制律：${ctrlLabel[S.ctrl] || S.ctrl}`;
  document.getElementById('b-btrue').style.display = (vtol && S.ctrl === 'sas') ? '' : 'none';  // B_true 仅级联模式
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

// ---------- 演示（对齐纯 web 版 demo.mjs，指令经 set 透传服务端） ----------
let demo = null;   // {name, t0}
const DEMO_BTNS = { pitch: 'b-pitch', yaw: 'b-yaw', roll: 'b-roll', cine: 'b-cine' };

function stopDemo() {
  demo = null;
  for (const k in DEMO_BTNS) document.getElementById(DEMO_BTNS[k]).classList.remove('active');
}

function startDemo(name) {
  stopDemo();
  for (const id of ['s-dt', 's-df', 's-dw']) cancelSpring(document.getElementById(id));
  send({ cmd: 'reset', mode: 'cruise' });   // 复位到巡航基线
  demo = { name, t0: null };                // t0 在 reset 生效后取（防竞态）
  document.getElementById(DEMO_BTNS[name]).classList.add('active');
}

function demoStep() {
  if (!demo || !sim.S) return;
  if (sim.S.paused) return;                 // 暂停时不发指令
  if (sim.S.vtolMode) { stopDemo(); return; }   // 悬停模式不演示
  if (demo.t0 === null || sim.S.time < demo.t0 - 1e-6) { demo.t0 = sim.S.time; return; }  // reset 生效后固定 t0（review blocking）
  const tau = sim.S.time - demo.t0, T = 3.2;
  const s = Math.sin(2 * Math.PI * tau / T);
  let set = {};
  switch (demo.name) {
    case 'pitch': set.dt = 18 * D2R * s; break;
    case 'yaw': set.df = 18 * D2R * s; break;
    case 'roll': set.dw = 0.28 * s; break;
    case 'cine': {
      set.thr = 0.55 + 0.25 * Math.min(tau / 6, 1) * (0.5 + 0.5 * Math.sin(tau * 0.5));
      const seg = tau % 12;
      set.dt = seg < 4 ? 16 * D2R * Math.sin(Math.PI * seg / 2) : 0;
      set.df = (seg >= 4 && seg < 8) ? 16 * D2R * Math.sin(Math.PI * (seg - 4) / 2) : 0;
      set.dw = seg >= 8 ? 0.26 * Math.sin(Math.PI * (seg - 8) / 2) : 0;
      break;
    }
  }
  send({ cmd: 'set', S: set });
}

// ---------- 滑块/按钮事件 → 服务端 ----------
// 回中开关（本地 UI 行为：开=弹簧摇杆松手回 0；关=松手停留）
let springBackOn = true;
const springAnims = new Map();

function cancelSpring(sl) {
  if (springAnims.has(sl)) { cancelAnimationFrame(springAnims.get(sl)); springAnims.delete(sl); }
}

function springBack(sl) {
  // 对齐纯 web 版 controls-ui.mjs：每帧 ×0.55 衰减，<0.6 归零
  // ⚠ 用闭包内部值衰减（不读 sl.value）：避免 syncSliders 把滑块写回服务端
  //   echo 值导致衰减速率受 RTT 干扰（review should-fix）
  cancelSpring(sl);
  let cur = parseFloat(sl.value);
  const stepFn = () => {
    cur = Math.abs(cur) < 0.6 ? 0 : cur * 0.55;
    sl.value = cur;
    const field = sl === document.getElementById('s-dt') ? 'dt'
      : sl === document.getElementById('s-df') ? 'df' : 'dw';
    send({ cmd: 'set', S: { [field]: cur * D2R } });
    if (cur !== 0) springAnims.set(sl, requestAnimationFrame(stepFn));
    else springAnims.delete(sl);
  };
  springAnims.set(sl, requestAnimationFrame(stepFn));
}

function bindControls() {
  const bindSlider = (id, fn) => {
    const el = document.getElementById(id);
    el.addEventListener('input', () => { stopDemo(); fn(parseFloat(el.value)); });
  };
  bindSlider('s-thr', v => send({ cmd: 'set', S: { thr: v / 100 } }));
  bindSlider('s-dt', v => send({ cmd: 'set', S: { dt: v * D2R } }));
  bindSlider('s-df', v => send({ cmd: 'set', S: { df: v * D2R } }));
  bindSlider('s-dw', v => send({ cmd: 'set', S: { dw: v * D2R } }));
  bindSlider('s-alt', v => send({ cmd: 'set', S: { altRef: v } }));

  for (const k in DEMO_BTNS) {
    document.getElementById(DEMO_BTNS[k]).addEventListener('click', () => {
      (demo && demo.name === k) ? stopDemo() : startDemo(k);
    });
  }

  // 弹簧回中：固定翼角速度闭环（sasMode=3）与悬停自稳模式（sasMode≠0）三滑块松手回中
  for (const id of ['s-dt', 's-df', 's-dw']) {
    const sl = document.getElementById(id);
    sl.addEventListener('pointerdown', () => cancelSpring(sl));
    const release = () => {
      if (!springBackOn) return;
      const S = sim.S;
      if (!S) return;
      if (S.sasMode === 3 || (S.vtolMode && S.sasMode !== 0)) springBack(sl);
    };
    sl.addEventListener('pointerup', release);
    sl.addEventListener('pointercancel', release);   // 拖动被打断也回中
    sl.addEventListener('touchend', release);
    sl.addEventListener('touchcancel', release);
    sl.addEventListener('keyup', release);   // 与纯 web 版一致：键盘调节松键回中（角速度闭环语义）
  }

  document.getElementById('b-sas').addEventListener('click', () => {
    const next = sim.S.vtolMode ? (sim.S.sasMode === 0 ? 1 : 0) : (sim.S.sasMode + 1) % 4;
    send({ cmd: 'set', S: { sasMode: next } });
  });
  document.getElementById('b-aero').addEventListener('click', () => {
    send({ cmd: 'set', S: { aero: !sim.S.aero } });
  });
  document.getElementById('b-vtol').addEventListener('click', () => {
    stopDemo();
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
    stopDemo();
    send({ cmd: 'reset', mode: 'pose' });
  });
  document.getElementById('b-pause').addEventListener('click', () => {
    send({ cmd: 'set', S: { paused: !sim.S.paused } });
  });
  document.getElementById('b-spring').addEventListener('click', () => {
    springBackOn = !springBackOn;
    document.getElementById('b-spring').textContent = `回中：${springBackOn ? '开' : '关'}`;
    document.getElementById('b-spring').classList.toggle('active', springBackOn);
  });
  document.getElementById('b-hover').addEventListener('click', () => {
    send({ cmd: 'set', S: { lockXY: !sim.S.lockXY } });
  });
  // 控制律切换：巡航 sas→indi；悬停 sas→lqr→adrc
  const CTRL_LABEL = { sas: 'SAS', indi: 'INDI', lqr: 'LQR', adrc: 'ADRC' };
  document.getElementById('b-ctrl').addEventListener('click', () => {
    const cur = sim.S.ctrl || 'sas';
    const seq = sim.S.vtolMode ? ['sas', 'lqr', 'adrc'] : ['sas', 'indi'];
    const next = seq[(seq.indexOf(cur) + 1) % seq.length];
    send({ cmd: 'set', S: { ctrl: next } });
  });
}

// ---------- 主循环：帧步进（请求-响应自适应） ----------
let frame = 0;
let prevT = 0;
let lastRender = 0;

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
  demoStep();
  // 渲染用独立时钟（⚠ 不能用 lastSent：服务端快时每帧都发 step，
  //   now−lastSent 恒为 0 → dt 驱动动画（螺旋桨等）全部冻结，2026-08-06 修复）
  let dtR = lastRender ? Math.min((now - lastRender) / 1000, 0.05) : 1 / 60;
  lastRender = now;
  if (sim.S.paused) dtR = 0;   // 暂停 = 时间冻结，渲染动画（螺旋桨/箭头）同步停
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

// 调试钩子（Playwright / 控制台可数值验证：螺旋桨旋转、姿态等）
window.__pwl = {
  sim, view,
  getProps: () => ({ fr: view.front.prop.rotation.x, tr: view.tail.prop.rotation.x }),
  getState: () => (sim.S ? { ...sim.S } : null),
};
