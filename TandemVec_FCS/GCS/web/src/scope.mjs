// ============================================================
//  scope.mjs — 多通道实时曲线（环形缓冲 canvas 滚动示波器）
// ============================================================
import { bus, state } from './main.mjs';

const $ = (id) => document.getElementById(id);

// 预设通道组（键 = 遥测快照字段）
const PRESETS = {
  '姿态 ATT': [
    ['roll_deg', '横滚 °', '#ff5d6c'],
    ['pitch_deg', '俯仰 °', '#3ea6ff'],
    ['heading_deg', '航向 °', '#29d3a2'],
  ],
  '角速率 GYRO': [
    ['gyro_x_dps', 'p °/s', '#ff5d6c'],
    ['gyro_y_dps', 'q °/s', '#3ea6ff'],
    ['gyro_z_dps', 'r °/s', '#29d3a2'],
  ],
  '加速度 ACC': [
    ['acc_x_ms2', 'ax', '#ff5d6c'],
    ['acc_y_ms2', 'ay', '#3ea6ff'],
    ['acc_z_ms2', 'az', '#29d3a2'],
  ],
  '速度 VEL': [
    ['vel_n_ms', '北', '#ff5d6c'],
    ['vel_e_ms', '东', '#3ea6ff'],
    ['vel_up_ms', '上', '#29d3a2'],
  ],
  '位置 POS': [
    ['rel_n_m', '北 m', '#ff5d6c'],
    ['rel_e_m', '东 m', '#3ea6ff'],
    ['rel_h_m', '高 m', '#29d3a2'],
  ],
  '控制量 CTRL': [
    ['ctrl_roll', 'αr', '#ff5d6c'],
    ['ctrl_pitch', 'αp', '#3ea6ff'],
    ['ctrl_thr_pct', '油门%', '#29d3a2'],
    ['ctrl_yaw', 'αy', '#ffb547'],
  ],
  'TVC 目标': [
    ['target_roll_deg', 'R目标', '#ff5d6c'],
    ['target_pitch_deg', 'P目标', '#3ea6ff'],
    ['target_yaw_rate_dps', 'Y速率', '#29d3a2'],
  ],
  '气压/压力': [
    ['alt_bar_m', '气压高 m', '#ff5d6c'],
    ['p1_mpa', 'P1', '#3ea6ff'],
    ['p2_mpa', 'P2', '#29d3a2'],
    ['bat_voltage_v', '电压 V', '#ffb547'],
  ],
};

const WIN_SEC = 20;          // 窗口时长
const HISTORY = 6000;        // 每通道历史点数（20s @ 帧率约 50Hz → 1000 点；留余量）
const MAX_CARDS = 4;

let cards = [];              // {key,label,color,hist:[...],el,canvas,ctx,lastV}
let paused = false;
let initialized = false;

export function activate() {
  if (initialized) return;
  initialized = true;

  const presetSel = $('scopePreset');
  for (const name of Object.keys(PRESETS)) presetSel.add(new Option(name, name));
  presetSel.value = '姿态 ATT';
  presetSel.addEventListener('change', () => buildCards(presetSel.value));
  $('scopePause').addEventListener('change', (e) => { paused = e.target.checked; });
  $('scopeClear').addEventListener('click', () => cards.forEach(c => c.hist.length = 0));
  $('scopeInfo').textContent = `窗口 ${WIN_SEC}s · ${Object.keys(PRESETS).length} 组预设`;

  buildCards(presetSel.value);
  bus.addEventListener('telemetry', onTelemetry);
  setInterval(() => cards.forEach(drawCard), 100);
}

function buildCards(presetName) {
  const wrap = $('scopeWrap');
  wrap.innerHTML = '';
  cards = [];
  const chans = PRESETS[presetName] || [];
  const groups = chunk(chans, 3);
  groups.slice(0, MAX_CARDS).forEach((ch, gi) => {
    const card = document.createElement('div');
    card.className = 'scope-card';
    card.innerHTML = `
      <div class="sc-title"><span>${presetName} #${gi + 1}</span><span class="sc-read"></span></div>
      <canvas></canvas>`;
    wrap.appendChild(card);
    const cv = card.querySelector('canvas');
    cards.push({ chans: ch, el: card, canvas: cv, hist: ch.map(() => []), last: ch.map(() => null) });
  });
  window.dispatchEvent(new Event('resize'));
}

function chunk(arr, n) {
  const out = [];
  for (let i = 0; i < arr.length; i += n) out.push(arr.slice(i, i + n));
  return out;
}

function onTelemetry(s) {
  if (paused || !cards.length) return;
  const t = (s.t_ms || 0) / 1000;
  for (const card of cards) {
    card.chans.forEach(([key], i) => {
      const v = s[key];
      if (v === undefined || v === null || Number.isNaN(v)) { card.hist[i].push(null); return; }
      card.hist[i].push([t, v]);
      if (card.hist[i].length > HISTORY) card.hist[i].shift();
    });
  }
}

function drawCard(card) {
  const dpr = devicePixelRatio || 1;
  const cv = card.canvas;
  const rect = cv.getBoundingClientRect();
  const W = Math.max(50, rect.width), H = Math.max(50, rect.height);
  if (cv.width !== W * dpr || cv.height !== H * dpr) { cv.width = W * dpr; cv.height = H * dpr; }
  const ctx = cv.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, W, H);

  // 网格
  ctx.strokeStyle = 'rgba(120,160,255,0.10)';
  ctx.lineWidth = 1;
  for (let i = 1; i < 5; i++) {
    ctx.beginPath(); ctx.moveTo(0, H * i / 5); ctx.lineTo(W, H * i / 5); ctx.stroke();
  }

  // 时间窗口（统一取最新值时间）
  let tEnd = 0;
  for (const h of card.hist) {
    const last = h[h.length - 1];
    if (Array.isArray(last) && last[0] > tEnd) tEnd = last[0];
  }
  if (!tEnd) return;
  const tStart = tEnd - WIN_SEC;

  // 量程
  let vmin = Infinity, vmax = -Infinity;
  for (const h of card.hist) for (const p of h) if (Array.isArray(p)) { if (p[1] < vmin) vmin = p[1]; if (p[1] > vmax) vmax = p[1]; }
  if (!isFinite(vmin) || vmin === vmax) { vmin = -1; vmax = 1; }
  const pad = (vmax - vmin) * 0.15 || 1;
  vmin -= pad; vmax += pad;

  const X = (t) => (t - tStart) / WIN_SEC * W;
  const Y = (v) => H - (v - vmin) / (vmax - vmin) * H;

  // 零线
  if (vmin < 0 && vmax > 0) {
    ctx.strokeStyle = 'rgba(255,255,255,0.15)';
    ctx.beginPath(); ctx.moveTo(0, Y(0)); ctx.lineTo(W, Y(0)); ctx.stroke();
  }

  // 读数标题
  const reads = card.chans.map(([, label, color], i) => {
    const p = card.hist[i][card.hist[i].length - 1];
    const v = Array.isArray(p) ? p[1] : NaN;
    return `<span style="color:${color}">${label} ${isFinite(v) ? v.toFixed(1) : '--'}</span>`;
  });
  card.el.querySelector('.sc-read').innerHTML = reads.join(' &nbsp;');

  // 曲线
  card.chans.forEach(([, , color], i) => {
    const h = card.hist[i];
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.6;
    ctx.beginPath();
    let pen = false;
    for (const p of h) {
      if (!Array.isArray(p)) { pen = false; continue; }
      if (p[0] < tStart) continue;
      const x = X(p[0]), y = Y(p[1]);
      if (!pen) { ctx.moveTo(x, y); pen = true; } else ctx.lineTo(x, y);
    }
    ctx.stroke();
  });
}
