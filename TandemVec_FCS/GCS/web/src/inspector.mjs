/**
 * @file    inspector.mjs
 * @module  GCS/前端 MAVLink Inspector
 * @brief   接收消息清单 + 任意字段实时绘图 + 消息频率统计
 * @details 这是 QGC 里对飞控开发者最有价值的一块：它不预设"该看什么"，
 *          而是把链路上真实收到的每类消息、每个字段都摊开，由人自己勾选。
 *          排查"某条消息到底发没发""频率对不对""某字段是不是恒定"
 *          这类问题时，只有这一页能给出直接证据。
 *
 *          数据流：每 1 s 拉一次 inspector（清单 + 频率 + 字段最新值），
 *          再加一次 inspector_series（只取已勾选字段的历史）。
 *          为什么轮询而不是推送：Inspector 不是常开页，推送会让所有页面
 *          都替它承担带宽；1 s 的清单刷新对判读频率已足够。
 *
 *          series 里的 key 形如 "ATTITUDE.roll"，取自后端 hist 环形缓冲。
 * @theory  N/A（纯显示）
 * @frame   N/A
 * @units   频率 Hz、年龄 s、字段值随 MAVLink 定义
 * @ref     GCS-001 §上位机、MAVLink 协议消息定义
 */

import { bus, state } from './main.mjs';

const $ = (id) => document.getElementById(id);

const POLL_MS = 1000;        // 清单刷新周期
const MAX_CHOSEN = 8;        // 最多同时画 8 条，再多图就糊了
const SERIES_POINTS = 400;   // 每条曲线保留点数

const PALETTE = ['#ff5d6c', '#3ea6ff', '#29d3a2', '#ffb547',
                 '#b987ff', '#4dd0e1', '#ff8a65', '#c0ca33'];

const insp = {
  messages: [],        // [{type,count,hz,age_s,fields}]
  chosen: [],          // ["ATTITUDE.roll", ...]
  series: {},          // key → [[t,v],...]
  elapsed: 0,
  frames: 0,
  timer: null,
  activated: false,
  expanded: null,      // 当前展开查看字段的消息类型
};

function send(obj) {
  // main.mjs 的 ws 是模块私有，这里借助 bus 上的自定义事件不可行；
  // 统一走一个全局注册的发送钩子（main.mjs 启动时挂载）。
  if (typeof window.__gcsSend === 'function') window.__gcsSend(obj);
}

function poll() {
  send({ cmd: 'inspector' });
  if (insp.chosen.length) send({ cmd: 'inspector_series', keys: insp.chosen });
}

function onInspector(msg) {
  insp.messages = msg.messages || [];
  insp.elapsed = msg.elapsed_s || 0;
  insp.frames = msg.frames || 0;
  renderTable();
  renderStats();
}

function onSeries(msg) {
  insp.series = msg.series || {};
  drawSeries();
}

function renderStats() {
  const el = $('inspStats');
  if (!el) return;
  const total = insp.messages.reduce((a, m) => a + m.count, 0);
  el.textContent = `${insp.messages.length} 类消息 · ${total} 帧 · 运行 ${insp.elapsed.toFixed(0)} s`;
}

function renderTable() {
  const body = $('inspRows');
  if (!body) return;
  if (!insp.messages.length) {
    body.innerHTML = '<tr><td colspan="5" class="insp-empty">暂无消息（需 MAVLink 模式且设备在发）</td></tr>';
    return;
  }
  const rows = [];
  for (const m of insp.messages) {
    const expanded = insp.expanded === m.type;
    const color = m.age_s > 2 ? '#ff5d6c' : (m.age_s > 0.5 ? '#ffb547' : '#29d3a2');
    const fields = Object.keys(m.fields || {});
    rows.push(`<tr class="insp-row${expanded ? ' open' : ''}" data-type="${m.type}">
      <td class="insp-type">${expanded ? '▾' : '▸'} ${m.type}</td>
      <td class="num">${m.count}</td>
      <td class="num">${m.hz.toFixed(1)}</td>
      <td class="num" style="color:${color}">${m.age_s.toFixed(1)}</td>
      <td class="num">${fields.length}</td>
    </tr>`);
    if (expanded) {
      const cells = fields.map((f) => {
        const key = `${m.type}.${f}`;
        const on = insp.chosen.includes(key);
        const v = m.fields[f];
        const txt = typeof v === 'number' ? (Number.isInteger(v) ? v : v.toFixed(3)) : String(v);
        return `<label class="insp-field${on ? ' on' : ''}" data-key="${key}">
          <input type="checkbox" ${on ? 'checked' : ''}>
          <span class="fname">${f}</span>
          <span class="fval">${txt}</span>
        </label>`;
      }).join('');
      rows.push(`<tr class="insp-fields-row"><td colspan="5"><div class="insp-fields">${cells}</div></td></tr>`);
    }
  }
  body.innerHTML = rows.join('');
}

function drawSeries() {
  const cv = $('inspChart');
  if (!cv) return;
  const ctx = cv.getContext('2d');
  const w = cv.clientWidth || 640;
  const h = cv.clientHeight || 260;
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  ctx.clearRect(0, 0, w, h);

  const keys = insp.chosen.filter((k) => (insp.series[k] || []).length > 1);
  const legend = $('inspLegend');

  if (!keys.length) {
    ctx.fillStyle = 'rgba(150,170,195,0.6)';
    ctx.font = '12px ui-monospace, monospace';
    ctx.textAlign = 'center';
    ctx.fillText('在上方消息里展开并勾选字段，即可实时绘制', w / 2, h / 2);
    ctx.textAlign = 'left';
    if (legend) legend.innerHTML = '';
    return;
  }

  // 时间轴取所有选中字段的并集范围
  let tMin = Infinity;
  let tMax = -Infinity;
  for (const k of keys) {
    for (const p of insp.series[k]) {
      if (p[0] < tMin) tMin = p[0];
      if (p[0] > tMax) tMax = p[0];
    }
  }
  if (!isFinite(tMin) || tMax <= tMin) return;

  // 逐字段独立归一化——不同消息的字段量纲与量级都不同，
  // 共用一根 Y 轴只会让大部分曲线压成直线（scope 页踩过同一个坑）。
  keys.forEach((k, i) => {
    const pts = insp.series[k].slice(-SERIES_POINTS);
    let lo = Infinity;
    let hi = -Infinity;
    for (const p of pts) { if (p[1] < lo) lo = p[1]; if (p[1] > hi) hi = p[1]; }
    if (!isFinite(lo)) return;
    if (hi - lo < 1e-9) { lo -= 1; hi += 1; }
    const pad = (hi - lo) * 0.12;
    lo -= pad; hi += pad;

    ctx.strokeStyle = PALETTE[i % PALETTE.length];
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    let pen = false;
    for (const p of pts) {
      const x = (p[0] - tMin) / (tMax - tMin) * w;
      const y = h - (p[1] - lo) / (hi - lo) * h;
      if (!pen) { ctx.moveTo(x, y); pen = true; } else ctx.lineTo(x, y);
    }
    ctx.stroke();
  });

  if (legend) {
    legend.innerHTML = keys.map((k, i) => {
      const pts = insp.series[k];
      const last = pts[pts.length - 1];
      return `<span style="color:${PALETTE[i % PALETTE.length]}">■ ${k} = ${last[1].toFixed(2)}</span>`;
    }).join(' &nbsp; ');
  }
}

function toggleField(key) {
  const i = insp.chosen.indexOf(key);
  if (i >= 0) insp.chosen.splice(i, 1);
  else {
    if (insp.chosen.length >= MAX_CHOSEN) insp.chosen.shift();
    insp.chosen.push(key);
  }
  renderTable();
  poll();
}

export function activate() {
  if (insp.activated) return;
  insp.activated = true;

  bus.addEventListener('inspector', (e) => onInspector(e.detail));
  bus.addEventListener('inspector-series', (e) => onSeries(e.detail));

  const body = $('inspRows');
  if (body) {
    body.addEventListener('click', (e) => {
      const field = e.target.closest('.insp-field');
      if (field) { e.preventDefault(); toggleField(field.dataset.key); return; }
      const row = e.target.closest('.insp-row');
      if (row) {
        const t = row.dataset.type;
        insp.expanded = insp.expanded === t ? null : t;
        renderTable();
      }
    });
  }

  const clr = $('inspClear');
  if (clr) {
    clr.addEventListener('click', () => {
      insp.chosen = [];
      insp.series = {};
      renderTable();
      drawSeries();
    });
  }

  insp.timer = setInterval(poll, POLL_MS);
  window.addEventListener('resize', drawSeries);
  renderTable();
  drawSeries();
  poll();
}
