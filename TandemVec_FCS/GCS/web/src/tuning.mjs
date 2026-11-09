/**
 * @file    tuning.mjs
 * @module  GCS/前端 PID 调参可视化
 * @brief   阶跃响应分析：选指令/反馈通道 → 自动找阶跃 → 给出上升时间/超调/调节时间/稳态误差
 * @details 为什么需要这一页：调参时"改了 kp 之后响应变快还是变振荡"靠盯着实时
 *          波形用眼睛判断既不准也不可复现。这里把经典控制里的标准指标算出来，
 *          同一组参数前后对比就是数字对数字。
 *
 *          数据来源：后端 g.hist（双协议通用时序缓冲），不是前端自己攒——
 *          前端攒的数据一刷新就没，而且切页面会漏采样。
 *
 *          画图：指令与反馈画在同一坐标系（同一通道对），再标出阶跃时刻、
 *          目标值带、超调峰。看着图和数字能对上，才敢信数字。
 * @theory  经典二阶系统阶跃响应指标
 * @frame   N/A
 * @units   时间 s、角度 deg、超调 %
 * @ref     server/stepresponse.py、GCS-001 §上位机
 */

import { bus, send } from './main.mjs';

const $ = (id) => document.getElementById(id);

const tuning = {
  channels: [],
  result: null,
  activated: false,
  cmdCh: '',
  fbCh: '',
};

const PALETTE = { cmd: '#3ea6ff', fb: '#29d3a2', step: '#ffb547', peak: '#ff5d6c' };

function fillChannelSelects() {
  const selCmd = $('tuneCmdCh');
  const selFb = $('tuneFbCh');
  if (!selCmd || !selFb) return;
  const opts = tuning.channels
    .map((c) => `<option value="${c.name}">${c.name}（${c.n} 点）</option>`)
    .join('');
  selCmd.innerHTML = opts;
  selFb.innerHTML = opts;

  // 猜一对常用的：指令通道以 target_ 开头，反馈通道取同后缀
  const guess = (name) => name.replace(/^target_/, '');
  const firstCmd = tuning.channels.find((c) => c.name.startsWith('target_'));
  if (firstCmd) {
    selCmd.value = firstCmd.name;
    const want = guess(firstCmd.name);
    const fb = tuning.channels.find((c) => c.name === want
      || c.name === `${want}_deg` || c.name.endsWith(want));
    if (fb) selFb.value = fb.name;
  } else if (tuning.channels.length > 1) {
    selFb.value = tuning.channels[1].name;
  }
}

function drawWave() {
  const cv = $('tuneChart');
  if (!cv) return;
  const ctx = cv.getContext('2d');
  const w = cv.clientWidth || 640;
  const h = cv.clientHeight || 260;
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  ctx.clearRect(0, 0, w, h);

  const r = tuning.result;
  const waves = r && r.waves;
  if (!waves || !Object.keys(waves).length) {
    ctx.fillStyle = 'rgba(150,170,195,0.6)';
    ctx.font = '12px ui-monospace, monospace';
    ctx.textAlign = 'center';
    ctx.fillText('选择通道后点「分析」，或先采集一段飞控数据', w / 2, h / 2);
    ctx.textAlign = 'left';
    return;
  }

  const names = Object.keys(waves);
  // 时间与值范围取两条通道的并集，保证同一坐标系可比
  let tMin = Infinity, tMax = -Infinity, vMin = Infinity, vMax = -Infinity;
  for (const n of names) {
    for (const [t, v] of waves[n]) {
      if (t < tMin) tMin = t;
      if (t > tMax) tMax = t;
      if (v < vMin) vMin = v;
      if (v > vMax) vMax = v;
    }
  }
  if (!isFinite(tMin) || tMax <= tMin) return;
  if (!isFinite(vMin)) return;
  const padV = Math.max(1e-6, (vMax - vMin) * 0.12);
  vMin -= padV; vMax += padV;

  const X = (t) => (t - tMin) / (tMax - tMin) * (w - 8) + 4;
  const Y = (v) => h - (v - vMin) / (vMax - vMin) * (h - 12) - 6;

  // 网格
  ctx.strokeStyle = 'rgba(120,160,255,0.10)';
  ctx.lineWidth = 1;
  for (let i = 1; i < 4; i++) {
    const y = (h / 4) * i;
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
  }

  // 阶跃时刻竖线与超调峰标记
  const steps = (r.steps || []).filter((s) => s.ok);
  for (const s of steps) {
    if (s.t_step === undefined) continue;
    const x = X(s.t_step);
    ctx.strokeStyle = 'rgba(255,181,71,0.45)';
    ctx.setLineDash([4, 4]);
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
    ctx.setLineDash([]);
  }

  names.forEach((n, i) => {
    const isCmd = i === 0;
    ctx.strokeStyle = isCmd ? PALETTE.cmd : PALETTE.fb;
    ctx.lineWidth = isCmd ? 1.2 : 1.6;
    ctx.beginPath();
    let pen = false;
    for (const [t, v] of waves[n]) {
      const x = X(t); const y = Y(v);
      if (!pen) { ctx.moveTo(x, y); pen = true; } else ctx.lineTo(x, y);
    }
    ctx.stroke();
  });

  // 图例
  const lg = $('tuneLegend');
  if (lg) {
    lg.innerHTML = names.map((n, i) => {
      const last = waves[n][waves[n].length - 1];
      const color = i === 0 ? PALETTE.cmd : PALETTE.fb;
      return `<span style="color:${color}">■ ${n} = ${Number(last[1]).toFixed(3)}</span>`;
    }).join(' &nbsp; ');
  }
}

function renderResult() {
  const box = $('tuneMetrics');
  if (!box) return;
  const r = tuning.result;
  if (!r) {
    box.innerHTML = '<div class="hint">尚未分析</div>';
    return;
  }
  if (r.error) {
    box.innerHTML = `<div class="alarm-card level-warn"><span class="alarm-text">${r.error}</span></div>`;
    return;
  }
  const s = r.summary || {};
  const fmt = (v, unit = '', digits = 3) =>
    (v === null || v === undefined) ? '--' : `${Number(v).toFixed(digits)}${unit}`;
  const rows = [
    ['检测到阶跃', `${s.n_steps ?? 0} 次（有效 ${s.n_ok ?? 0} 次）`],
    ['平均上升时间', fmt(s.avg_rise_time_s, ' s')],
    ['平均超调', fmt(s.avg_overshoot_pct, ' %', 1)],
    ['最大超调', fmt(s.max_overshoot_pct, ' %', 1)],
    ['平均调节时间', fmt(s.avg_settling_time_s, ' s')],
    ['平均稳态误差', fmt(s.avg_steady_error_pct, ' %', 1)],
  ];
  const cards = rows.map(([k, v]) =>
    `<div class="tune-metric"><span class="k">${k}</span><span class="v">${v}</span></div>`).join('');

  // 逐次明细（最多列 8 次，太多没意义）
  const detail = (r.steps || []).filter((x) => x.ok).slice(0, 8).map((x, i) => `
    <tr>
      <td class="num">${i + 1}</td>
      <td class="num">${Number(x.t_step).toFixed(2)}</td>
      <td class="num">${fmt(x.rise_time_s)}</td>
      <td class="num">${fmt(x.overshoot_pct, '', 1)}</td>
      <td class="num">${fmt(x.settling_time_s)}</td>
      <td class="num">${fmt(x.steady_error_pct, '', 1)}</td>
      <td class="num">${x.ring_crossings ?? '--'}</td>
    </tr>`).join('');

  box.innerHTML = `<div class="tune-metrics-grid">${cards}</div>`
    + (detail
      ? `<table class="insp-table" style="margin-top:12px">
           <thead><tr><th>#</th><th>时刻 s</th><th>上升 s</th><th>超调 %</th>
             <th>调节 s</th><th>稳态 %</th><th>振铃</th></tr></thead>
           <tbody>${detail}</tbody></table>`
      : '<div class="hint" style="margin-top:10px">没有可评估的阶跃（可能需要更大的激励或降低最小幅值）</div>');
}

function analyze() {
  const cmdCh = $('tuneCmdCh')?.value;
  const fbCh = $('tuneFbCh')?.value;
  const minDelta = Number($('tuneMinDelta')?.value || 5);
  if (!cmdCh || !fbCh) return;
  tuning.cmdCh = cmdCh;
  tuning.fbCh = fbCh;
  send({
    cmd: 'tuning_analyze',
    cmd_ch: cmdCh,
    fb_ch: fbCh,
    min_delta: minDelta,
    window_s: 3.0,
  });
  const st = $('tuneState');
  if (st) st.textContent = `分析 ${cmdCh} → ${fbCh} …`;
}

export function activate() {
  if (tuning.activated) return;
  tuning.activated = true;


  bus.addEventListener('tuning-channels', (e) => {
    tuning.channels = (e.detail || {}).channels || [];
    fillChannelSelects();
    const st = $('tuneState');
    if (st) st.textContent = `${tuning.channels.length} 个通道可用`;
  });

  bus.addEventListener('tuning-result', (e) => {
    tuning.result = e.detail || {};
    renderResult();
    drawWave();
    const st = $('tuneState');
    if (st && !tuning.result.error) {
      const s = tuning.result.summary || {};
      st.textContent = `完成：${s.n_steps ?? 0} 次阶跃`;
    }
  });

  const btn = $('tuneRun');
  if (btn) btn.addEventListener('click', analyze);
  const refresh = $('tuneRefresh');
  if (refresh) {
    refresh.addEventListener('click', () => send({ cmd: 'tuning_channels' }));
  }

  window.addEventListener('resize', drawWave);
  send({ cmd: 'tuning_channels' });
  renderResult();
  drawWave();

}
