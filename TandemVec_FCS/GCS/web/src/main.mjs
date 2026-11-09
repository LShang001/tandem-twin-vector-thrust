/**
 * @file    main.mjs
 * @module  GCS/前端入口（WS 客户端 + 页面路由 + 全局状态）
 * @brief   建立 WS 连接、分发消息到各页面模块，维护全局快照与参数表
 * @details 数据来源：WS /ws 消息（hello、conn、telemetry、param_*、vars_*、dbg_*、flash_* 等）；
 *          处理：JSON 解析 → 按 type 更新 state 并经 bus 派发事件；
 *          输出：连接/告警状态栏与页面切换。
 *          刷新频率：telemetry 由后端按 50Hz 节流推送（TELEMETRY_PERIOD=0.02）。
 *          不变量：BACKEND_VERSION 必须与后端 app.version 一致（hello 握手比对，
 *          版本不符说明前端页面配到了残留旧后端）。
 * @theory  N/A
 * @frame   N/A
 * @units   字段单位由后端快照后缀约定（_deg/_dps/_ms2/_ms/_m/_pct/_v），前端不换算
 * @ref     GCS-001 §上位机、COMM-001 §4 C 层（推送 50Hz）
 */
import { fmt, fmtInt } from './widget.mjs';

export const bus = new EventTarget();

// 供子模块主动发命令（如 Inspector 的轮询）。各模块私接 WebSocket 会造成
// 多条连接、后端 ws_clients 计数失真，统一走这个钩子。
window.__gcsSend = (obj) => {
  if (ws.sock && ws.sock.readyState === 1) ws.sock.send(JSON.stringify(obj));
};

const ws = { sock: null, retry: 0 };
const BACKEND_VERSION = '0.3.0';   // 前端期望的后端版本（hello 握手比对）
export const state = {
  connected: false,
  mode: 'telemetry',        // telemetry | dbg
  snap: {},
  paramMeta: {},            // id → {name, type}
  paramValues: {},          // id → value
};

/** @brief 通过 WS 发送一条命令对象
 *  @param {Object} obj 命令对象（{cmd, ...}）
 *  @note  未连接时静默丢弃，不排队重发
 *  @units  N/A
 *  @ref    GCS-001 §上位机
 */
export function send(obj) {
  if (ws.sock && ws.sock.readyState === WebSocket.OPEN) {
    ws.sock.send(JSON.stringify(obj));
  }
}

/** @brief 用 bus 派发自定义事件（跨模块解耦）
 *  @param {string} type 事件名；@param {*} detail 事件载荷
 *  @note  订阅方用 bus.addEventListener(type, ...) 接收
 *  @units  N/A
 *  @ref    GCS-001 §上位机
 */
export function emit(type, detail) {
  bus.dispatchEvent(new CustomEvent(type, { detail }));
}

function connectWs() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  ws.sock = new WebSocket(`${proto}://${location.host}/ws`);
  ws.sock.onopen = () => {
    ws.retry = 0;
    setSvcState(true);
    emit('ws-open');
  };
  ws.sock.onmessage = (ev) => {
    let msg;
    try { msg = JSON.parse(ev.data); } catch { return; }
    handleMsg(msg);
  };
  ws.sock.onclose = () => {
    ws.sock = null;
    setSvcState(false);
    emit('ws-close');
    setTimeout(connectWs, Math.min(4000, 500 * (++ws.retry)));
  };
  ws.sock.onerror = () => { try { ws.sock.close(); } catch {} };
}

// 后端服务状态指示（顶栏 svcLed/svcInfo）：WS 通 = 后端活
function setSvcState(up) {
  const led = $('svcLed'), info = $('svcInfo');
  if (!led || !info) return;
  led.className = 'link-led ' + (up ? 'on' : 'off');
  info.textContent = up ? '后端已连接' : '后端断开，重连中…';
}

function handleMsg(msg) {
  switch (msg.type) {
    case 'hello':
      // ★ 版本握手：旧后端进程残留时前端是新的、后端是旧的——醒目提示重启
      if (msg.version !== BACKEND_VERSION) {
        $('svcInfo').textContent = `后端版本过旧 v${msg.version || '?'}（需 v${BACKEND_VERSION}）——请重启 start.bat`;
        flashAlarm('err', `后端进程是旧版 v${msg.version || '?'}，界面是新版 v${BACKEND_VERSION}——请完全关闭并用 start.bat 重启`);
      }
      break;
    case 'conn':
      state.connected = msg.status === 'connected';
      if (msg.protocol) state.protocol = msg.protocol;
      if (state.connected) connectTs = Date.now();
      emit('conn', msg);
      break;
    case 'ports':
      emit('ports', msg.ports || []);
      break;
    case 'telemetry':
      state.snap = msg.data || {};
      emit('telemetry', state.snap);
      break;
    case 'inspector':
      emit('inspector', msg);
      break;
    case 'inspector_series':
      emit('inspector-series', msg);
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
      state.paramCount = msg.count;
      emit('param_count', msg);
      break;
    case 'param_info':
      // ★ 2026-08-10 全面审查修复：param_info/param_value 必须落 state
      //   （原实现只 emit——params.mjs 唯一数据源 state.paramValues 永空，
      //   参数页死锁为空）
      if (msg.name !== undefined) (state.paramMeta[msg.id] ??= {}).name = msg.name;
      if (msg.type !== undefined) (state.paramMeta[msg.id] ??= {}).type = msg.type;
      emit('param_info', msg);
      break;
    case 'param_value':
      state.paramValues[msg.id] = msg.value;
      if (msg.name !== undefined) (state.paramMeta[msg.id] ??= {}).name = msg.name;
      emit('param_value', msg);
      break;
    case 'param_written':
    case 'param_written_pending':
      emit(msg.type, msg);
      break;
    case 'vars_count':
    case 'vars_info':
      emit(msg.type, msg);
      break;
    case 'record_state':
      emit('record-state', msg);
      break;
    case 'replay_state':
      emit('replay-state', msg);
      break;
    case 'replay_progress':
      emit('replay-progress', msg);
      break;
    case 'mission_state':
      emit('mission-state', msg);
      break;
    case 'alarm':
      emit('alarm', msg);
      break;
    case 'alarm_state':
      emit('alarm-state', msg);
      break;
    case 'vehicles':
      emit('vehicles', msg);
      break;
    case 'vehicle_selected':
      emit('vehicle-selected', msg);
      break;
    case 'tuning_channels':
      emit('tuning-channels', msg);
      break;
    case 'tuning_result':
      emit('tuning-result', msg);
      break;
    case 'spectrum':
      emit('spectrum', msg);
      break;
    case 'spectrum_multi':
      emit('spectrum_multi', msg);
      break;
    case 'spectrogram':
      emit('spectrogram', msg);
      break;
    case 'calib_wizard':
      emit('calib-wizard', msg);
      break;
    case 'calib_state':
      emit('calib-state', msg);
      break;
    case 'calib_assess':
      emit('calib-assess', msg);
      break;
    case 'calib_progress':
      emit('calib-progress', msg);
      break;
    case 'recordings':
      emit('recordings', msg.files || []);
      break;
    case 'log':
      emit('log', msg);
      break;
    case 'link_stat':
      renderLinkStat(msg);
      // 同时广播给订阅方：地图页要用其中的 mission 字段判断任务上传结果
      emit('link-stat', msg);
      break;
    case 'rx_hex':
      emit('rx-hex', msg);
      break;
    case 'hex_state':
      emit('hex-state', msg);
      break;
  }
}

// ---------------- 链路健康（1Hz 后端统计） ----------------
let stallAlarmed = false;
let connectTs = 0;
function renderLinkStat(m) {
  const el = $('linkStat');
  if (!m.connected) { el.textContent = ''; el.className = 'link-stat'; stallAlarmed = false; return; }
  const kb = (m.bytes_s / 1024).toFixed(1);
  const bad = m.bad_s > 0 ? `<span class="bad"> · CRC✗${m.bad_s}/s</span>` : '';
  // 遥测停滞判定：连接且遥测模式下，超过 3s 没解出遥测字段（给 4s 连接宽限）
  const grace = Date.now() - connectTs < 4000;
  const stall = !grace && m.mode === 'telemetry' &&
    (m.tele_age_s === null ? true : m.tele_age_s > 3);
  if (stall) {
    // 四级诊断：无字节 / 有字节无帧 / 帧 CRC 灭 / 有响应帧但无遥测帧
    let txt;
    if (m.bytes_s === 0 && m.frames_s === 0) {
      txt = '⚠ 串口无字节到达 — 固件遥测未发送（检查固件运行状态/接线；控制台开「链路监听」确认）';
    } else if (m.frames_s === 0) {
      txt = `⚠ ${m.bytes_s} B/s 流入但切不出完整帧 — 波特率不匹配或乱码`;
    } else if (m.bad_s >= m.frames_s * 0.5) {
      txt = `⚠ ${m.frames_s} 帧/s 但 CRC 失败 ${m.bad_s}/s — 链路丢字节严重（VCP 带宽），换数传口或降波特率`;
    } else {
      txt = `⚠ 有协议帧（${m.frames_s}/s）但遥测帧为 0 — 固件遥测轮发未运行（或刚响应过命令）`;
    }
    el.textContent = txt;
    el.className = 'link-stat stall';
    if (!stallAlarmed) {
      stallAlarmed = true;
      flashAlarm('warn', txt.replace(/^⚠ /, ''));
    }
  } else {
    stallAlarmed = false;
    el.innerHTML = m.mode === 'dbg'
      ? `DBG 模式 · ${kb} kB/s`
      : `▼${kb} kB/s · ${m.tele_s} 遥测帧/s · ${m.fields} 字段${bad}`;
    el.className = 'link-stat';
  }
}

// ---------------- 页面路由 ----------------
const pages = ['dashboard', 'view3d', 'scope', 'gnss', 'map', 'video', 'alarms', 'tuning', 'calib', 'inspector', 'blackbox', 'params', 'console'];
let activePage = 'dashboard';
/** @brief 切换到指定页面（非法页面名忽略）
 *  @param {string} name 页面名（dashboard/scope/params/blackbox/console/view3d）
 *  @note  切换后向 bus 派发 'page' 事件，供页面懒激活与暂停渲染
 *  @units  N/A
 *  @ref    GCS-001 §上位机
 */
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
    send({ cmd: 'connect', port, baud: parseInt($('baudSelect').value), protocol: $('protocolSelect').value });
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
/** @brief 顶栏告警条显示
 *  @param {string} level info|warn|error；@param {string} text 提示文本
 *  @note  仅前端提示，不发送任何协议帧
 *  @units  N/A
 *  @ref    GCS-001 §上位机
 */
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

// 各页面模块激活（懒加载 + 失败重试）
// ★ 2026-08-08 双重修复：
//   1) 监听器必须先于 switchPage('dashboard') 注册——后注册会漏掉初始页
//      事件，首屏 activate 永不执行（曾致首屏全 "--"，点导航才偶然恢复）；
//   2) es-module-shims 初始化完成前触发动态 import() 会静默失败——
//      必须带重试，且连续失败要告警而不是无声死亡。
const PAGE_LOADERS = {
  dashboard: () => import('./dashboard.mjs'),
  view3d: () => import('./view3d.mjs'),
  scope: () => import('./scope.mjs'),
  gnss: () => import('./gnss.mjs'),
  map: () => import('./map.mjs'),
  video: () => import('./video.mjs'),
  alarms: () => import('./alarms.mjs'),
  tuning: () => import('./tuning.mjs'),
  spectrumview: () => import('./spectrumview.mjs'),
  calib: () => import('./calib.mjs'),
  layout: () => import('./layout.mjs'),
  inspector: () => import('./inspector.mjs'),
  blackbox: () => import('./blackbox.mjs'),
  params: () => import('./params.mjs'),
  console: () => import('./console.mjs'),
};
const _pageLoaded = new Set();
async function loadPageModule(name, attempt = 0) {
  if (_pageLoaded.has(name)) return;
  const loaderFn = PAGE_LOADERS[name];
  if (!loaderFn) return;
  try {
    const mod = await loaderFn();
    _pageLoaded.add(name);
    if (mod.activate) mod.activate();
  } catch (err) {
    if (attempt < 8) {   // shim 未就绪：退避重试（最长 ~4s）
      setTimeout(() => loadPageModule(name, attempt + 1), 400 + attempt * 100);
    } else {
      console.error(`页面模块 ${name} 加载失败`, err);
      flashAlarm('err', `页面模块 ${name} 加载失败：${(err && err.message) || err}`);
    }
  }
}
bus.addEventListener('page', (e) => { loadPageModule(e.detail); });

document.querySelectorAll('.nav-btn').forEach(b =>
  b.addEventListener('click', () => switchPage(b.dataset.page)));

// 立即初始化（module-shim 执行时 DOM 已就绪）
const loader = $('loader');
setTimeout(() => loader.classList.add('hide'), 3000);   // 兜底隐藏（不依赖 load）
connectWs();
switchPage('dashboard');
// 报警模块必须常驻加载，不能等用户切到报警页——顶部横幅要覆盖所有页面，
// 否则在仪表盘飞行时报警根本不显示（这是最危险的失效模式）。
loadPageModule('alarms');
// 布局要在仪表盘渲染前应用，否则会先按默认顺序画一遍再跳位（肉眼可见的闪动）
loadPageModule('layout');
// 频谱视图的 DOM 在调参页里，但它有自己的通道选择器，与调参页的语义无关
// （一个是阶跃响应、一个是频域）。放在这里常驻加载，不依赖调参页先被访问
// ——实测过「在 tuning.activate() 里动态 import」会静默失效，排查成本远高于收益。
loadPageModule('spectrumview');

// 多机：机号列表与切换。单串口只能连一台机，但一条 MAVLink 网络/链路
// 可以承载多台，按 sysid 区分；只有一台时选择器隐藏，不给界面添乱。
bus.addEventListener('vehicles', (e) => {
  const sel = document.getElementById('vehSelect');
  const wrap = document.getElementById('vehPicker');
  if (!sel || !wrap) return;
  const list = (e.detail || {}).vehicles || [];
  wrap.style.display = list.length > 1 ? '' : 'none';
  sel.innerHTML = list.map((v) =>
    `<option value="${v.sysid}">${v.sysid}${v.active ? ' ●' : ''}</option>`).join('');
  const active = list.find((v) => v.active);
  if (active) sel.value = String(active.sysid);
});
document.getElementById('vehSelect')?.addEventListener('change', (ev) => {
  send({ cmd: 'vehicle_select', sysid: Number(ev.target.value) });
});
// 定期刷新：新机上线/离线要能反映出来
setInterval(() => send({ cmd: 'vehicles' }), 3000);
setInterval(refreshPorts, 15000);  // 周期刷新端口列表

// load 事件兜底（若在 load 前执行则提前隐藏 loader）
if (document.readyState === 'complete') {
  loader.classList.add('hide');
} else {
  window.addEventListener('load', () => loader.classList.add('hide'), { once: true });
}

export { fmt };
