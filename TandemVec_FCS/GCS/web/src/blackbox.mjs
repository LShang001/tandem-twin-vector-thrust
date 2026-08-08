// ============================================================
//  blackbox.mjs — 黑匣子页：飞行段管理 / 表格预览 / 分组绘图 / CSV 导出
// ============================================================
import { bus, send, state } from './main.mjs';

const $ = (id) => document.getElementById(id);

let segments = [];          // 段列表 [{num,page,tms}]
let activeSeg = null;
let bbData = null;          // flash_done 载荷 {rows, row_count, sample_step, columns, segments}
let selectedCols = new Set();
let plotTab = 'attitude';
let initialized = false;

export function activate() {
  if (initialized) return;
  initialized = true;

  $('bbStat').addEventListener('click', () => send({ cmd: 'flash_stat' }));
  $('bbFindseg').addEventListener('click', () => {
    $('bbHint').textContent = '正在读取段信息…';
    send({ cmd: 'flash_findseg' });
  });
  $('bbExport').addEventListener('click', exportLatest);
  $('bbExit').addEventListener('click', () => {
    send({ cmd: 'dbg_exit' });
    $('bbHint').textContent = '已退出调试模式（遥测恢复）';
  });
  $('bbSaveCsv').addEventListener('click', saveCsv);

  bus.addEventListener('dbg-mode', (e) => {
    $('bbHint').textContent = e.detail.on ? '调试模式（遥测已暂停）' : '遥测模式';
  });
  bus.addEventListener('flash-segments', (e) => { segments = e.detail; renderSegments(); });
  bus.addEventListener('flash-progress', (e) => {
    $('bbHint').textContent = `正在下载数据… ${e.detail.done} 页`;
  });
  bus.addEventListener('flash-done', (e) => { bbData = e.detail; renderData(); });
  bus.addEventListener('log', (e) => { if (e.detail.level === 'error') $('bbHint').textContent = '⚠ ' + e.detail.msg; });
}

function exportLatest() {
  if (!segments.length) { $('bbHint').textContent = '请先「列出飞行段」'; return; }
  const seg = segments[segments.length - 1];
  activeSeg = seg.num;
  $('bbHint').textContent = `正在导出段 ${seg.num}（page ${seg.page}，2048 页）…`;
  send({ cmd: 'flash_export', start: seg.page, count: 2048 });
}

function renderSegments() {
  const wrap = $('bbSegments');
  wrap.innerHTML = '';
  if (!segments.length) { wrap.innerHTML = '<span class="hint">无飞行段记录</span>'; return; }
  for (const seg of segments) {
    const chip = document.createElement('div');
    chip.className = 'seg-chip' + (activeSeg === seg.num ? ' active' : '');
    chip.innerHTML = `段 ${seg.num} · 起始页 ${seg.page} · t=${(seg.tms / 1000).toFixed(1)}s`;
    chip.title = '点击导出本段';
    chip.addEventListener('click', () => {
      activeSeg = seg.num;
      $('bbHint').textContent = `正在导出段 ${seg.num}…`;
      send({ cmd: 'flash_export', start: seg.page, count: 2048 });
    });
    wrap.appendChild(chip);
  }
}

function renderData() {
  const { columns, segments: segs, row_count } = bbData;
  // 段 chips（含 S/E 元数据）
  renderSegments();
  $('bbInfo').textContent = `共 ${row_count} 帧 · ${columns.length} 列 · 列名来自 S 帧自描述`;

  // 列选择 chips
  selectedCols = new Set(columns.slice(0, 12));
  renderCols();

  // 表格
  renderTable();

  // 绘图
  renderPlots();
}

function renderCols() {
  const wrap = $('bbCols');
  wrap.innerHTML = '';
  for (const col of bbData.columns) {
    const chip = document.createElement('span');
    chip.className = 'col-chip' + (selectedCols.has(col) ? '' : ' off');
    chip.textContent = col;
    chip.addEventListener('click', () => {
      if (selectedCols.has(col)) selectedCols.delete(col); else selectedCols.add(col);
      renderCols();
      renderTable();
      renderPlots();
    });
    wrap.appendChild(chip);
  }
}

function renderTable() {
  const cols = ['seq', 't_ms', ...bbData.columns].filter(c => c === 'seq' || c === 't_ms' || selectedCols.has(c));
  const table = $('bbTable');
  let html = '<thead><tr>' + cols.map(c => `<th>${c}</th>`).join('') + '</tr></thead><tbody>';
  const rows = bbData.rows.slice(0, 200);
  // rows 结构：[seq, t_ms, vals[22], seg]
  for (const r of rows) {
    html += '<tr>';
    html += `<td>${r[0]}</td><td>${r[1]}</td>`;
    const vals = r[2];
    for (const c of bbData.columns) {
      if (!selectedCols.has(c)) continue;
      const i = bbData.columns.indexOf(c);
      html += `<td>${vals[i] !== undefined ? vals[i].toFixed(3) : ''}</td>`;
    }
    html += '</tr>';
  }
  table.innerHTML = html + '</tbody>';
}

function renderPlots() {
  const tabs = $('bbPlotTabs');
  const defs = [
    ['attitude', '姿态'],
    ['accel', '加速度'],
    ['gyro', '角速率'],
    ['velocity', '速度'],
    ['track', '水平轨迹'],
    ['tvc', 'TVC/压力'],
  ];
  tabs.innerHTML = '';
  for (const [key, label] of defs) {
    const b = document.createElement('button');
    b.className = 'btn small' + (plotTab === key ? ' primary' : '');
    b.textContent = label;
    b.addEventListener('click', () => { plotTab = key; renderPlots(); });
    tabs.appendChild(b);
  }
  const wrap = $('bbPlot');
  wrap.innerHTML = '';
  const cv = document.createElement('canvas');
  wrap.appendChild(cv);
  drawPlot(cv, plotTab);
}

function colIndex(name) {
  const i = bbData.columns.indexOf(name);
  return i >= 0 ? i : null;
}

function series(name) {
  const i = colIndex(name);
  if (i === null) return null;
  return bbData.rows.map(r => [r[1] / 1000, r[2][i]]);   // t_ms → s
}

function drawPlot(cv, tab) {
  const dpr = devicePixelRatio || 1;
  const W = Math.max(300, cv.clientWidth || 600);
  const H = Math.max(240, cv.clientHeight || 320);
  cv.width = W * dpr; cv.height = H * dpr;
  const ctx = cv.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, W, H);

  const PAD = { l: 56, r: 16, t: 16, b: 26 };
  const pw = W - PAD.l - PAD.r, ph = H - PAD.t - PAD.b;

  const groups = {
    attitude: [['roll_deg', '横滚', '#ff5d6c'], ['pitch_deg', '俯仰', '#3ea6ff'], ['heading_deg', '航向', '#29d3a2']],
    accel: [['accel_x_ms2', 'ax', '#ff5d6c'], ['accel_y_ms2', 'ay', '#3ea6ff'], ['accel_z_ms2', 'az', '#29d3a2']],
    gyro: [['gyro_x_dps', 'p', '#ff5d6c'], ['gyro_y_dps', 'q', '#3ea6ff'], ['gyro_z_dps', 'r', '#29d3a2']],
    velocity: [['vel_n_ms', '北', '#ff5d6c'], ['vel_e_ms', '东', '#3ea6ff'], ['vel_d_ms', '下', '#29d3a2']],
    tvc: [['tvc1_deg', '上摆', '#ff5d6c'], ['tvc2_deg', '下摆', '#3ea6ff'], ['p1', 'P1', '#29d3a2'], ['valve_ctrl', '阀门', '#ffb547']],
  };
  const chans = groups[tab] || [];

  if (tab === 'track') { drawTrack(ctx, PAD, pw, ph); return; }

  const all = [];
  for (const [name] of chans) { const s = series(name); if (s) all.push(...s); }
  if (!all.length) { ctx.fillStyle = '#7d8ca3'; ctx.font = '12px sans-serif'; ctx.textAlign = 'center'; ctx.fillText('（无数据——请导出段并选择列）', W / 2, H / 2); return; }

  let t0 = Infinity, t1 = -Infinity, vmin = Infinity, vmax = -Infinity;
  for (const [t, v] of all) { if (t < t0) t0 = t; if (t > t1) t1 = t; if (v < vmin) vmin = v; if (v > vmax) vmax = v; }
  if (t1 === t0) t1 = t0 + 1;
  const pad = (vmax - vmin) * 0.1 || 1; vmin -= pad; vmax += pad;

  // 网格 + 轴
  ctx.strokeStyle = 'rgba(120,160,255,0.12)';
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const y = PAD.t + ph * i / 4;
    ctx.beginPath(); ctx.moveTo(PAD.l, y); ctx.lineTo(W - PAD.r, y); ctx.stroke();
    const v = vmax - (vmax - vmin) * i / 4;
    ctx.fillStyle = '#7d8ca3'; ctx.font = '10px Consolas,monospace'; ctx.textAlign = 'right';
    ctx.fillText(v.toFixed(1), PAD.l - 6, y + 3);
  }
  for (let i = 0; i <= 5; i++) {
    const x = PAD.l + pw * i / 5;
    const t = t0 + (t1 - t0) * i / 5;
    ctx.fillStyle = '#7d8ca3'; ctx.textAlign = 'center';
    ctx.fillText(t.toFixed(1), x, H - 8);
  }
  ctx.fillStyle = '#7d8ca3'; ctx.textAlign = 'right';
  ctx.fillText('t/s', W - PAD.r, H - 8);

  // 零线
  if (vmin < 0 && vmax > 0) {
    const y0 = PAD.t + ph * (vmax - 0) / (vmax - vmin);
    ctx.strokeStyle = 'rgba(255,255,255,0.2)';
    ctx.beginPath(); ctx.moveTo(PAD.l, y0); ctx.lineTo(W - PAD.r, y0); ctx.stroke();
  }

  // 曲线
  chans.forEach(([name, label, color], ci) => {
    const s = series(name);
    if (!s) return;
    ctx.strokeStyle = color; ctx.lineWidth = 1.5;
    ctx.beginPath();
    let pen = false;
    for (const [t, v] of s) {
      const x = PAD.l + (t - t0) / (t1 - t0) * pw;
      const y = PAD.t + ph * (vmax - v) / (vmax - vmin);
      if (!pen) { ctx.moveTo(x, y); pen = true; } else ctx.lineTo(x, y);
    }
    ctx.stroke();
    // 图例
    ctx.fillStyle = color; ctx.font = '11px sans-serif'; ctx.textAlign = 'left';
    ctx.fillText(label, PAD.l + 8 + ci * 90, PAD.t + 12);
  });
}

function drawTrack(ctx, PAD, pw, ph) {
  const sN = series('rel_n_m'), sE = series('rel_e_m');
  if (!sN || !sE) { ctx.fillStyle = '#7d8ca3'; ctx.font = '12px sans-serif'; ctx.textAlign = 'center'; ctx.fillText('（无位置数据）', pw / 2, ph / 2); return; }
  let nmin = Infinity, nmax = -Infinity, emin = Infinity, emax = -Infinity;
  for (const [, v] of sN) { if (v < nmin) nmin = v; if (v > nmax) nmax = v; }
  for (const [, v] of sE) { if (v < emin) emin = v; if (v > emax) emax = v; }
  const span = Math.max(nmax - nmin, emax - emin, 1e-6);
  const X = (e) => PAD.l + (e - emin) / span * pw;
  const Y = (n) => PAD.t + (nmax - n) / span * ph;
  // 网格（以中点为原点）
  ctx.strokeStyle = 'rgba(120,160,255,0.12)';
  const mx = X((emin + emax) / 2), my = Y((nmin + nmax) / 2);
  ctx.beginPath(); ctx.moveTo(mx, PAD.t); ctx.lineTo(mx, PAD.t + ph); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(PAD.l, my); ctx.lineTo(PAD.l + pw, my); ctx.stroke();
  // 轨迹
  ctx.strokeStyle = '#29d3a2'; ctx.lineWidth = 1.8;
  ctx.beginPath();
  for (let i = 0; i < sN.length; i++) {
    const x = X(sE[i][1]), y = Y(sN[i][1]);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  }
  ctx.stroke();
  // 起点/终点
  ctx.fillStyle = '#ff5d6c'; ctx.beginPath(); ctx.arc(X(sE[0][1]), Y(sN[0][1]), 4, 0, 7); ctx.fill();
  ctx.fillStyle = '#3ea6ff'; ctx.beginPath(); ctx.arc(X(sE[sE.length - 1][1]), Y(sN[sN.length - 1][1]), 4, 0, 7); ctx.fill();
  ctx.fillStyle = '#7d8ca3'; ctx.font = '11px sans-serif';
  ctx.fillText('● 起点', X(sE[0][1]) + 8, Y(sN[0][1]) - 6);
  ctx.fillText('● 终点', X(sE[sE.length - 1][1]) + 8, Y(sN[sN.length - 1][1]) - 6);
  ctx.fillText('N ↑ / E →（相对起飞点 m）', PAD.l + 8, PAD.t + 12);
}

function saveCsv() {
  if (!bbData) return;
  let csv = 'seq,t_ms,' + bbData.columns.join(',') + '\n';
  for (const r of bbData.rows) {
    csv += r[0] + ',' + r[1] + ',' + r[2].map(v => Number(v).toFixed(4)).join(',') + '\n';
  }
  const blob = new Blob([csv], { type: 'text/csv' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `blackbox_seg${activeSeg ?? 'all'}.csv`;
  a.click();
  URL.revokeObjectURL(a.href);
}
