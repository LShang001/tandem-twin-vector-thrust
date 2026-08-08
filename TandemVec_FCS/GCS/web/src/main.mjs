// ============================================================
//  main.mjs — GCS 前端入口：WebSocket 客户端 + 页面路由 + 全局状态
// ============================================================
import { fmt, fmtInt } from './widget.mjs';

export const bus = new EventTarget();

const ws = { sock: null, retry: 0 };
export const state = {
  connected: false,
  mode: 'telemetry',        // telemetry | dbg
  snap: {},
  paramMeta: {},            // id → {name, type}
  paramValues: {},          // id → value
};

export function send(obj) {
  if (ws.sock && ws.sock.readyState === WebSocket.OPEN) {
    ws.sock.send(JSON.stringify(obj));
  }
}

export function emit(type, detail) {
  bus.dispatchEvent(new CustomEvent(type, { detail }));
}

function connectWs() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  ws.sock = new WebSocket(`${proto}://${location.host}/ws`);
  ws.sock.onopen = () => { ws.retry = 0; emit('ws-open'); };
  ws.sock.onmessage = (ev) => {
    let msg;
    try { msg = JSON.parse(ev.data); } catch { return; }
    handleMsg(msg);
  };
  ws.sock.onclose = () => {
    ws.sock = null;
    emit('ws-close');
    setTimeout(connectWs, Math.min(4000, 500 * (++ws.retry)));
  };
  ws.sock.onerror = () => { try { ws.sock.close(); } catch {} };
}

function handleMsg(msg) {
  switch (msg.type) {
    case 'conn':
      state.connected = msg.status === 'connected';
      emit('conn', msg);
      break;
    case 'ports':
      emit('ports', msg.ports || []);
      break;
    case 'telemetry':
      state.snap = msg.data || {};
      emit('telemetry', state.snap);
      break;
    case 'dbg_out':
      emit('dbg-out', msg.line);
      break;
    case 'dbg_mode':
      state.mode = msg.on ? 'dbg' : 'telemetry';
      emit('dbg-mode', msg);
      break;
    case 'device_info':
      emit('device-info', msg);
      break;
    case 'flash_segments':
      emit('flash-segments', msg.segments || []);
      break;
    case 'flash_done':
      emit('flash-done', msg);
      break;
    case 'flash_progress':
      emit('flash-progress', msg);
      break;
    case 'param_count':
    case 'param_value':
    case 'param_written':
    case 'param_written_pending':
      emit(msg.type, msg);
      break;
    case 'record_state':
      emit('record-state', msg);
      break;
    case 'replay_state':
      emit('replay-state', msg);
      break;
    case 'recordings':
      emit('recordings', msg.files || []);
      break;
    case 'log':
      emit('log', msg);
      break;
  }
}

// ---------------- 页面路由 ----------------
const pages = ['dashboard', 'view3d', 'scope', 'blackbox', 'params', 'console'];
let activePage = 'dashboard';
export function switchPage(name) {
  if (!pages.includes(name)) return;
  activePage = name;
  document.querySelectorAll('.nav-btn').forEach(b =>
    b.classList.toggle('active', b.dataset.page === name));
  document.querySelectorAll('.page').forEach(p =>
    p.classList.toggle('active', p.id === `page-${name}`));
  emit('page', name);
}

// ---------------- 顶部连接条 ----------------
const $ = (id) => document.getElementById(id);

function refreshPorts() {
  send({ cmd: 'list_ports' });
}
bus.addEventListener('ports', (e) => {
  const sel = $('portSelect');
  const prev = sel.value;
  sel.innerHTML = '';
  const ports = e.detail;
  if (!ports.length) {
    sel.innerHTML = '<option value="">（无串口）</option>';
  } else {
    for (const p of ports) sel.add(new Option(`${p.port} — ${p.desc || p.hwid}`, p.port));
    if (ports.some(p => p.port === prev)) sel.value = prev;
  }
});

$('btnConnect').addEventListener('click', () => {
  if (state.connected) {
    send({ cmd: 'disconnect' });
  } else {
    const port = $('portSelect').value;
    if (!port) { flashAlarm('warn', '请先选择串口'); return; }
    send({ cmd: 'connect', port, baud: parseInt($('baudSelect').value) });
  }
});

bus.addEventListener('conn', (e) => {
  const m = e.detail;
  const led = $('linkLed');
  if (m.status === 'connected') {
    led.className = 'link-led on';
    $('linkInfo').textContent = `${m.port} @ ${fmtInt(m.baud)}`;
    $('btnConnect').textContent = '断开';
    flashAlarm('ok', `已连接 ${m.port}`);
    // 自动拉取设备信息（0xE3：固件版本确认）
    send({ cmd: 'device_info' });
  } else {
    led.className = 'link-led off';
    $('linkInfo').textContent = m.status === 'error' ? `错误: ${m.reason || m.status}` : '未连接';
    $('btnConnect').textContent = '连接';
    if (m.status === 'error') flashAlarm('err', `连接失败: ${m.reason}`);
  }
});

// 设备信息（固件版本）→ 顶栏展示
bus.addEventListener('device-info', (e) => {
  const d = e.detail;
  if (d.dev_name) {
    const sw = d.sw_ver !== undefined ? ` · SW v${d.sw_ver}` : '';
    $('linkInfo').textContent = `${d.dev_name}${sw}`;
  }
});

// ---------------- 告警 ----------------
const alarmTimers = {};
export function flashAlarm(level, text) {
  const bar = $('alarmBar');
  const el = document.createElement('div');
  el.className = `alarm ${level === 'warn' ? 'warn' : ''}`;
  el.textContent = text;
  bar.appendChild(el);
  setTimeout(() => el.remove(), 6000);
}

// ---------------- 告警规则（遥测驱动） ----------------
const lastAlarm = {};
function checkAlarms(snap) {
  const rules = [
    ['unlock', '解锁状态已变化', () => snap.unlocked ? '⚠ 飞行器已解锁' : '🔒 飞行器已锁定', 4000],
  ];
  for (const [key, label, fn, cooldown] of rules) {
    if (snap[key] === undefined) continue;
    const sig = key + ':' + (snap[key] ? 1 : 0);
    if (lastAlarm[key] !== sig) {
      lastAlarm[key] = sig;
      if (snap[key] !== undefined && cooldown) flashAlarm('warn', fn());
    }
  }
  // 连续告警：低压/链路/GPS
  const now = Date.now();
  const checkCooldown = (key, ms) => { lastAlarm[key] = lastAlarm[key] || 0; if (now - lastAlarm[key] > ms) { lastAlarm[key] = now; return true; } return false; };
  if (snap.bat_voltage_v && snap.bat_voltage_v > 0 && snap.bat_voltage_v < 11.1 && checkCooldown('lowbat', 15000)) {
    flashAlarm('err', `电池电压低: ${fmt(snap.bat_voltage_v)} V`);
  }
  if (snap.unlocked === false && snap.gps_sats !== undefined && snap.gps_sats < 6 && checkCooldown('lowsats', 30000)) {
    flashAlarm('warn', `卫星数不足: ${snap.gps_sats}`);
  }
}
bus.addEventListener('telemetry', (e) => checkAlarms(e.detail));

// ---------------- 初始化 ----------------
// ★ 不依赖 window.load：es-module-shims 在 load 事件之后才执行 module-shim
//   脚本，load 监听会错过（白屏/loader 不隐藏/不连接）。模块执行时 DOM
//   已就绪（shim 在文档解析后处理），直接初始化 + setTimeout 兜底。
bus.addEventListener('ws-open', refreshPorts);
bus.addEventListener('ws-open', () => { if (state.connected) refreshPorts(); });

document.querySelectorAll('.nav-btn').forEach(b =>
  b.addEventListener('click', () => switchPage(b.dataset.page)));

// 立即初始化（module-shim 执行时 DOM 已就绪）
const loader = $('loader');
setTimeout(() => loader.classList.add('hide'), 3000);   // 兜底隐藏（不依赖 load）
connectWs();
switchPage('dashboard');
setInterval(refreshPorts, 15000);  // 周期刷新端口列表

// load 事件兜底（若在 load 前执行则提前隐藏 loader）
if (document.readyState === 'complete') {
  loader.classList.add('hide');
} else {
  window.addEventListener('load', () => loader.classList.add('hide'), { once: true });
}

// 各页面模块注册（懒加载）
async function loadPageModule(name) {
  const mods = {
    dashboard: () => import('./dashboard.mjs'),
    view3d: () => import('./view3d.mjs'),
    scope: () => import('./scope.mjs'),
    blackbox: () => import('./blackbox.mjs'),
    params: () => import('./params.mjs'),
    console: () => import('./console.mjs'),
  };
  const m = mods[name];
  if (m) { const mod = await m(); if (mod.activate) mod.activate(); }
}
bus.addEventListener('page', (e) => { loadPageModule(e.detail); });

export { fmt };
