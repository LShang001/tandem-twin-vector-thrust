/**
 * @file    scope.mjs
 * @module  GCS/前端示波器
 * @brief   多通道实时曲线（canvas 环形缓冲滚动显示）
 * @details 数据来源：bus 的 'telemetry' 事件（后端 50Hz 快照）；
 *          处理：按预设通道组取快照字段压入环形缓冲并重绘；
 *          输出：canvas 曲线。窗口 20s、每通道历史 6000 点；
 *          页面不可见时暂停重绘（省 CPU）。
 * @theory  N/A
 * @frame   仅显示，不参与计算
 * @units   通道单位见 PRESETS 标签（deg、deg/s、m/s²、m/s、m、%、V）
 * @ref     GCS-001 §上位机（实时曲线页）、COMM-001 §4 C 层（推送 50Hz）
 */
import { bus, send, state } from './main.mjs';

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
    ['ctrl_roll_dps2', 'αr °/s²', '#ff5d6c'],
    ['ctrl_pitch_dps2', 'αp °/s²', '#3ea6ff'],
    ['ctrl_thr_pct', '油门%', '#29d3a2'],
    ['ctrl_yaw_dps2', 'αy °/s²', '#ffb547'],
  ],
  'TVC 目标': [
    ['target_roll_deg', 'R目标', '#ff5d6c'],
    ['target_pitch_deg', 'P目标', '#3ea6ff'],
    ['target_yaw_rate_dps', 'Y速率', '#29d3a2'],
  ],
  '执行器 ACT': [
    ['tvc_upper_deg', 'δf °', '#ffb547'],
    ['tvc_lower_deg', 'δt °', '#3ea6ff'],
    ['motor_front_pct', '前推%', '#ff5d6c'],
    ['motor_rear_pct', '尾推%', '#29d3a2'],
  ],
  '高度/电压': [
    ['alt_bar_m', '气压高 m', '#ff5d6c'],
    ['bat_voltage_v', '电压 V', '#ffb547'],
  ],
};

const WIN_DEFAULT = 20;      // 默认窗口时长（可切 5/10/20/60 s）
const HISTORY = 6000;        // 每通道历史点数（20s @ 帧率约 50Hz → 1000 点；留余量）
const MAX_CARDS = 4;

let cards = [];              // {chans,el,canvas,hist,last,cursor}
let paused = false;
let winSec = WIN_DEFAULT;    // 当前窗口时长（秒）
let pageVisible = true;        // 页面不可见时停止重绘（scope 在后台白烧 CPU）
let initialized = false;

/** @brief 构建预设通道卡片并订阅 telemetry
 *  @details 初始化预设下拉/暂停/清空按钮，按 PRESETS 建卡片
 *  @note    幂等：initialized 后直接返回；页面不可见时暂停重绘
 *  @units   见 PRESETS 标签（deg、deg/s、m/s²、m/s、m、%、V）
 *  @ref     GCS-001 §上位机（实时曲线页）
 */
export function activate() {
  if (initialized) return;
  initialized = true;

  const presetSel = $('scopePreset');
  for (const name of Object.keys(PRESETS)) presetSel.add(new Option(name, name));
  presetSel.value = '姿态 ATT';
  presetSel.addEventListener('change', () => {
    buildCards(presetSel.value);
    // 换预设后通道变了，重新补一次历史，否则新卡片是空的
    requestHistory();
  });
  $('scopePause').addEventListener('change', (e) => { paused = e.target.checked; });
  $('scopeClear').addEventListener('click', () => cards.forEach(c => c.hist.length = 0));
  const winSel = $('scopeWin');
  if (winSel) {
    for (const w of [5, 10, 20, 60]) winSel.add(new Option(`${w}s`, String(w)));
    winSel.value = String(WIN_DEFAULT);
    winSel.addEventListener('change', () => { winSec = Number(winSel.value) || WIN_DEFAULT; });
  }
  const expBtn = $('scopeExport');
  if (expBtn) expBtn.addEventListener('click', exportCsv);
  $('scopeInfo').textContent = `${Object.keys(PRESETS).length} 组预设`;

  buildCards(presetSel.value);
  bus.addEventListener('scope-history', (e) => fillHistory((e.detail || {}).series || {}));
  bus.addEventListener('telemetry', (e) => onTelemetry(e.detail));   // ★ CustomEvent，快照在 detail
  // 首次进入就先拉一段后端历史：否则刚打开曲线页要等十几秒才看得见东西。
  requestHistory();
  bus.addEventListener('page', (e) => { pageVisible = (e.detail === 'scope'); });
  setInterval(() => { if (pageVisible) cards.forEach(drawCard); }, 100);
}

/**
 * 向后端要当前可见通道的历史，用来在刚打开页面时把画布铺满。
 * 数据源是后端的 g.hist（与频谱/调参页同一个），不是前端自己攒——
 * 前端攒的历史一切页或刷新就没了，"先看波形再切去分析"会断。
 */
function requestHistory() {
  const chans = [];
  for (const c of cards) {
    for (const n of c.chans) {
      if (!chans.includes(n)) chans.push(n);
    }
  }
  if (chans.length) send({ cmd: 'scope_history', channels: chans });
}

/**
 * 把后端历史并进各卡片。
 *
 * ★ 必须**合并**而不是「空才填」：切到这个页面时前端已经开始收实时数据了，
 *   如果只在历史为空时填充，就永远填不进去——画布上只有刚进来那几秒的数据，
 *   看起来像"历史功能没生效"（实测踩过）。
 *
 * 合并规则按时间排序，同一时刻（差值 < 1 ms）以后来的（实时）为准。
 */
function fillHistory(series) {
  for (const card of cards) {
    card.chans.forEach((n, i) => {
      const s = series[n];
      if (!Array.isArray(s) || s.length < 2) return;
      const live = card.hist[i] || [];
      if (!live.length) {
        card.hist[i] = s.slice(-HISTORY);
      } else {
        const merged = s.concat(live).sort((a, b) => a[0] - b[0]);
        const dedup = [];
        for (const p of merged) {
          const prev = dedup[dedup.length - 1];
          if (prev && Math.abs(prev[0] - p[0]) < 0.001) dedup[dedup.length - 1] = p;
          else dedup.push(p);
        }
        card.hist[i] = dedup.slice(-HISTORY);
      }
      const h = card.hist[i];
      const lastV = h.length ? h[h.length - 1][1] : null;
      if (Number.isFinite(lastV)) card.last[i] = lastV;
    });
  }
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
    const cardObj = { chans: ch, el: card, canvas: cv, hist: ch.map(() => []),
                      last: ch.map(() => null), cursor: null };
    // 游标：鼠标横向位置 → 该时刻各通道读数。
    // 用 mouseleave 清空而不是保存最后一次位置——鼠标移走后还留着竖线，
    // 会和"实时值"混淆，示波器上这是歧义。
    cv.addEventListener('mousemove', (e) => {
      const r = cv.getBoundingClientRect();
      cardObj.cursor = e.clientX - r.left;
    });
    cv.addEventListener('mouseleave', () => { cardObj.cursor = null; });
    cards.push(cardObj);
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
      // ★ 不用 shift()（O(n) 每点搬移，20Hz×12 通道会持续卡顿）——
      //   超 1.5×HISTORY 才截断一次，摊还 O(1)
      if (card.hist[i].length > HISTORY * 1.5) card.hist[i] = card.hist[i].slice(-HISTORY);
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
  const tStart = tEnd - winSec;

  // 量程：逐通道独立。
  // 共用一根 Y 轴时，航向(0~360°)会把横滚/俯仰(±6°)压成一条直线——实测
  // 三条线全部贴在底边分不出形状。专业示波器每个通道各自归一化，
  // 绝对值由图例与游标读数给出，形状才看得见。
  const ranges = card.chans.map((_, i) => {
    let lo = Infinity, hi = -Infinity;
    for (const p of card.hist[i]) {
      if (Array.isArray(p)) { if (p[1] < lo) lo = p[1]; if (p[1] > hi) hi = p[1]; }
    }
    if (!isFinite(lo)) return [-1, 1];
    if (hi - lo < 1e-9) { lo -= 1; hi += 1; }
    const pad = (hi - lo) * 0.15;
    return [lo - pad, hi + pad];
  });

  const X = (t) => (t - tStart) / winSec * W;
  const Y = (v, i) => {
    const [lo, hi] = ranges[i];
    return H - (v - lo) / (hi - lo) * H;
  };

  // 零线：逐通道画（各自量程下 0 的位置不同）
  ctx.strokeStyle = 'rgba(255,255,255,0.10)';
  ranges.forEach(([lo, hi], i) => {
    if (lo < 0 && hi > 0) {
      const y0 = Y(0, i);
      ctx.beginPath(); ctx.moveTo(0, y0); ctx.lineTo(W, y0); ctx.stroke();
    }
  });

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
      const x = X(p[0]), y = Y(p[1], i);
      if (!pen) { ctx.moveTo(x, y); pen = true; } else ctx.lineTo(x, y);
    }
    ctx.stroke();
  });

  drawCursor(card, ctx, W, H, tStart, tEnd, X, Y);
}

/** 在时间轴上二分查找最接近 tCur 的采样点 */
function nearestAt(hist, tCur) {
  if (!hist.length) return null;
  let lo = 0, hi = hist.length - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (hist[mid][0] < tCur) lo = mid + 1; else hi = mid;
  }
  // 命中的点与它前一个点里挑更近的
  let best = hist[lo];
  if (lo > 0) {
    const prev = hist[lo - 1];
    if (Math.abs(prev[0] - tCur) < Math.abs(best[0] - tCur)) best = prev;
  }
  // 距离超过半个采样周期就当没有数据（窗口边缘外/丢帧处不该硬画读数）
  if (Math.abs(best[0] - tCur) > 0.5) return null;
  return best;
}

/**
 * @brief 画鼠标游标：竖虚线 + 各通道交点 + 标题区游标读数
 * @note  游标存在时标题读数显示"游标时刻值"而不是"最新值"，并在前面标 ⌖，
 *        否则两个数在视觉上无法区分，容易把历史值当成实时值读。
 */
function drawCursor(card, ctx, W, H, tStart, tEnd, X, Y) {
  const cx = card.cursor;
  if (cx === null || cx < 0 || cx > W) return;
  const tCur = tStart + (cx / W) * winSec;

  ctx.save();
  ctx.strokeStyle = 'rgba(210,225,245,0.45)';
  ctx.lineWidth = 1;
  ctx.setLineDash([4, 3]);
  ctx.beginPath();
  ctx.moveTo(cx, 0);
  ctx.lineTo(cx, H);
  ctx.stroke();
  ctx.setLineDash([]);

  const reads = [];
  card.chans.forEach(([key, label, color], i) => {
    const p = nearestAt(card.hist[i], tCur);
    if (!p) { reads.push(`<span style="opacity:.4">${label} --</span>`); return; }
    const y = Y(p[1], i);
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(cx, y, 3, 0, Math.PI * 2);
    ctx.fill();
    reads.push(`<span style="color:${color}">⌖ ${label} ${p[1].toFixed(2)}</span>`);
  });

  // 时间标签贴底
  const rel = (tCur - tEnd).toFixed(1);
  ctx.fillStyle = 'rgba(200,215,235,0.75)';
  ctx.font = '10px ui-monospace, monospace';
  const txt = `${rel}s`;
  const tw = ctx.measureText(txt).width;
  const lx = Math.min(Math.max(2, cx - tw / 2), W - tw - 2);
  ctx.fillText(txt, lx, H - 4);
  ctx.restore();

  card.el.querySelector('.sc-read').innerHTML = reads.join(' &nbsp;');
}

/**
 * @brief 导出当前所有通道历史为 CSV
 * @note  合并导出：一张卡内的通道共用时间基准，跨卡之间不保证采样点对齐，
 *        所以按「时间戳,通道1,通道2…」逐卡分段写，而不是强行做时间轴对齐——
 *        伪对齐会伪造出并不存在的同步采样。
 */
function exportCsv() {
  if (!cards.length) return;
  const lines = [];
  cards.forEach((card, ci) => {
    lines.push(`# card${ci + 1}: ${card.chans.map(c => c[1]).join(' / ')}`);
    lines.push(['t_s', ...card.chans.map(c => c[0])].join(','));
    // 以第一通道为时间主轴，其余通道按时间就近取样
    const base = card.hist[0] || [];
    for (const p of base) {
      if (!Array.isArray(p)) continue;
      const row = [p[0].toFixed(3)];
      for (let i = 0; i < card.chans.length; i++) {
        if (i === 0) { row.push(p[1].toFixed(4)); continue; }
        const q = nearestAt(card.hist[i], p[0]);
        row.push(q ? q[1].toFixed(4) : '');
      }
      lines.push(row.join(','));
    }
    lines.push('');
  });
  const blob = new Blob([lines.join('\n')], { type: 'text/csv;charset=utf-8' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `scope_${new Date().toISOString().replace(/[:.]/g, '-')}.csv`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 3000);
}
