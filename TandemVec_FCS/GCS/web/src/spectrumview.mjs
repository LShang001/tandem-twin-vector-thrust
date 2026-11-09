/**
 * @file    spectrumview.mjs
 * @module  GCS/前端 频谱视图
 * @brief   对指定通道做 Welch 功率谱，画谱线 + 标主峰 + 三频带能量
 * @details 为什么波形页需要频谱：时域曲线看得出"在抖"，但看不出**抖在哪个频率**。
 *          姿态抖动 12 Hz 和 45 Hz 在时域上都是毛刺，但原因完全不同——
 *          前者多半是结构/桨叶一阶，后者更像传感器噪声或控制环振荡。
 *
 *          注意频率上限受采样率限制：遥测 50 Hz 的话，能看到的最高频率是 25 Hz
 *          （奈奎斯特）。这是采样定理的硬限制，页面上要写出来，否则会有人问
 *          "为什么 45 Hz 的振动看不到"——因为 50 Hz 采样根本采不到它。
 * @theory  Welch 功率谱；奈奎斯特上限 = fs/2
 * @frame   N/A
 * @units   频率 Hz、能量占比 %
 * @ref     server/spectrum.py
 */

import { bus, send } from './main.mjs';

const $ = (id) => document.getElementById(id);

const spec = {
  result: null,
  multi: null,
  spec2d: null,
  channel: '',
  selected: [],        // 多通道对比时选中的通道名（最多 4）
  activated: false,
};

const MULTI_COLORS = ['#3ea6ff', '#29d3a2', '#ffb547', '#b987ff'];

function draw() {
  const cv = $('specChart');
  if (!cv) return;
  const ctx = cv.getContext('2d');
  const w = cv.clientWidth || 640;
  const h = cv.clientHeight || 220;
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  ctx.clearRect(0, 0, w, h);

  const r = spec.result;
  if (!r || !r.ok) {
    ctx.fillStyle = 'rgba(150,170,195,0.6)';
    ctx.font = '12px ui-monospace, monospace';
    ctx.textAlign = 'center';
    ctx.fillText(r && r.reason ? r.reason : '选一个通道后点「频谱」',
      w / 2, h / 2);
    ctx.textAlign = 'left';
    return;
  }

  const freqs = r.freqs || [];
  const psd = r.psd_norm || [];
  if (freqs.length < 3) return;

  const fMax = freqs[freqs.length - 1];
  let pMax = 0;
  for (const p of psd) if (p > pMax) pMax = p;
  if (pMax <= 0) return;

  const X = (f) => (f / fMax) * (w - 8) + 4;
  const Y = (p) => h - (p / pMax) * (h - 16) - 8;

  // 网格 + 频率刻度
  ctx.strokeStyle = 'rgba(120,160,255,0.10)';
  ctx.lineWidth = 1;
  for (let i = 1; i < 5; i++) {
    const x = (w / 5) * i;
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h - 14); ctx.stroke();
  }
  ctx.fillStyle = 'rgba(140,170,200,0.75)';
  ctx.font = '10px ui-monospace, monospace';
  for (let i = 0; i <= 5; i++) {
    const f = (fMax / 5) * i;
    ctx.fillText(`${f.toFixed(1)}`, Math.min(w - 26, (w / 5) * i + 2), h - 3);
  }
  ctx.fillText('Hz', w - 18, h - 3);

  // 谱线（填充 + 描边）
  ctx.beginPath();
  ctx.moveTo(X(freqs[0]), h - 14);
  for (let i = 0; i < freqs.length; i++) ctx.lineTo(X(freqs[i]), Y(psd[i]));
  ctx.lineTo(X(freqs[freqs.length - 1]), h - 14);
  ctx.closePath();
  const grad = ctx.createLinearGradient(0, 0, 0, h);
  grad.addColorStop(0, 'rgba(62,166,255,0.42)');
  grad.addColorStop(1, 'rgba(62,166,255,0.02)');
  ctx.fillStyle = grad;
  ctx.fill();

  ctx.strokeStyle = '#3ea6ff';
  ctx.lineWidth = 1.4;
  ctx.beginPath();
  for (let i = 0; i < freqs.length; i++) {
    const x = X(freqs[i]); const y = Y(psd[i]);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  }
  ctx.stroke();

  // 主峰标记
  for (const p of (r.peaks || []).slice(0, 3)) {
    const x = X(p.freq_hz);
    ctx.strokeStyle = 'rgba(255,93,108,0.6)';
    ctx.setLineDash([3, 3]);
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h - 14); ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = '#ff5d6c';
    ctx.font = '10px ui-monospace, monospace';
    ctx.fillText(`${p.freq_hz.toFixed(1)}Hz`, Math.max(2, x - 18), 11);
  }
}

function renderInfo() {
  const box = $('specInfo');
  if (!box) return;
  const r = spec.result;
  if (!r || !r.ok) {
    box.innerHTML = `<span class="hint">${r && r.reason ? r.reason : '--'}</span>`;
    return;
  }
  const b = r.bands || {};
  const peaks = (r.peaks || []).slice(0, 3).map((p) =>
    `<span class="spec-peak">${p.freq_hz.toFixed(2)} Hz <i>${p.share_pct}%</i></span>`).join('');
  box.innerHTML = `
    <div class="spec-row"><span>采样率</span><b>${r.fs} Hz</b>
      <span>分辨率</span><b>${r.resolution_hz} Hz</b>
      <span>段数</span><b>${r.n_seg}</b></div>
    <div class="spec-row"><span>主峰</span>${peaks || '<b>--</b>'}</div>
    <div class="spec-row"><span>频带</span>
      <b>&lt;1 Hz ${b.low_lt1hz_pct}%</b>
      <b>1–10 Hz ${b.mid_1_10hz_pct}%</b>
      <b>&gt;10 Hz ${b.high_gt10hz_pct}%</b></div>
    <div class="hint" style="margin-top:4px">
      奈奎斯特上限 ${(r.fs / 2).toFixed(1)} Hz —— 高于此的频率采不到，不是没有
    </div>`;
}

function run() {
  const sel = $('specChannel');
  if (!sel || !sel.value) return;
  spec.channel = sel.value;
  send({ cmd: 'spectrum', channel: sel.value, nperseg: 256 });
  const st = $('specState');
  if (st) st.textContent = `分析 ${sel.value}…`;
}



/** 多通道叠加谱：同一频率网格上叠 2–4 条，便于看共模还是单轴 */
function drawMulti() {
  const cv = $('specChart');
  if (!cv) return;
  const ctx = cv.getContext('2d');
  const w = cv.clientWidth || 640;
  const h = cv.clientHeight || 220;
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  ctx.clearRect(0, 0, w, h);

  const r = spec.multi;
  if (!r || !r.ok) {
    ctx.fillStyle = 'rgba(150,170,195,0.6)';
    ctx.font = '12px ui-monospace, monospace';
    ctx.textAlign = 'center';
    ctx.fillText(r && r.reason ? r.reason : '勾选 2–4 个通道后点「多通道对比」', w / 2, h / 2);
    ctx.textAlign = 'left';
    return;
  }
  const chans = r.channels || [];
  if (!chans.length) return;

  let fMax = 0;
  for (const c of chans) {
    const f = (c.freqs || []);
    if (f.length) fMax = Math.max(fMax, f[f.length - 1]);
  }
  if (!(fMax > 0)) return;

  // 各通道归一化到自身最大值——不同通道幅值差一个量级时，共用一根纵轴会把
  // 小的压平；对比看的是**峰的位置**，不是绝对幅值。
  const X = (f) => (f / fMax) * (w - 8) + 4;

  ctx.strokeStyle = 'rgba(120,160,255,0.10)';
  for (let i = 1; i < 5; i++) {
    const x = (w / 5) * i;
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h - 14); ctx.stroke();
  }
  ctx.fillStyle = 'rgba(140,170,200,0.75)';
  ctx.font = '10px ui-monospace, monospace';
  for (let i = 0; i <= 5; i++) {
    ctx.fillText(`${((fMax / 5) * i).toFixed(1)}`,
      Math.min(w - 26, (w / 5) * i + 2), h - 3);
  }
  ctx.fillText('Hz', w - 18, h - 3);

  chans.forEach((c, i) => {
    const freqs = c.freqs || [];
    const psd = c.psd_norm || [];
    let pMax = 0;
    for (const p of psd) if (p > pMax) pMax = p;
    if (pMax <= 0) return;
    ctx.strokeStyle = MULTI_COLORS[i % MULTI_COLORS.length];
    ctx.lineWidth = 1.4;
    ctx.beginPath();
    for (let k = 0; k < freqs.length; k++) {
      const x = X(freqs[k]);
      const y = h - 14 - (psd[k] / pMax) * (h - 26);
      if (k === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
  });

  // 共模峰竖线
  for (const cp of (r.common || []).slice(0, 4)) {
    const x = X(cp.freq_hz);
    ctx.strokeStyle = `rgba(255,93,108,${0.25 + 0.15 * cp.n_channels})`;
    ctx.setLineDash([3, 3]);
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h - 14); ctx.stroke();
    ctx.setLineDash([]);
  }

  const box = $('specInfo');
  if (box) {
    const legend = chans.map((c, i) =>
      `<span style="color:${MULTI_COLORS[i % MULTI_COLORS.length]}">■ ${c.name}</span>`).join(' ');
    const common = (r.common || []).map((cp) =>
      `<span class="spec-peak">${cp.freq_hz.toFixed(2)} Hz <i>${cp.n_channels} 通道</i></span>`).join('');
    box.innerHTML = `
      <div class="spec-row">${legend}</div>
      <div class="spec-row"><span>共模峰（≥2 通道同频，±${r.tol_hz} Hz）</span>${common || '<b>未发现</b>'}</div>
      <div class="hint">共模峰多半是结构/桨叶/共模振动，调单轴增益无效；单轴独有的峰才优先查该轴的传感器与机械</div>`;
  }
}

/** 多选通道（最多 4 个） */
function toggleSelect(name) {
  const i = spec.selected.indexOf(name);
  if (i >= 0) spec.selected.splice(i, 1);
  else {
    if (spec.selected.length >= 4) spec.selected.shift();
    spec.selected.push(name);
  }
  renderMultiSelect();
}

function renderMultiSelect() {
  const box = $('specMultiList');
  if (!box) return;
  const sel = $('specChannel');
  const names = sel ? [...sel.options].map((o) => o.value) : [];
  box.innerHTML = names.map((n) => {
    const on = spec.selected.includes(n);
    return `<label class="spec-chk${on ? ' on' : ''}" data-name="${n}">
      <input type="checkbox" ${on ? 'checked' : ''}> ${n}</label>`;
  }).join('');
  box.querySelectorAll('.spec-chk').forEach((el) => {
    el.addEventListener('click', (ev) => {
      ev.preventDefault();
      toggleSelect(el.dataset.name);
    });
  });
  const st = $('specState');
  if (st && spec.selected.length) st.textContent = `已选 ${spec.selected.length} 个通道`;
}



/** 时频图：把频率（纵向）× 时间（横向）画成热力图 */
function drawSpectro() {
  const cv = $('specChart');
  if (!cv) return;
  const ctx = cv.getContext('2d');
  const w = cv.clientWidth || 640;
  const h = cv.clientHeight || 220;
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  ctx.clearRect(0, 0, w, h);

  const r = spec.spec2d;
  if (!r || !r.ok) {
    ctx.fillStyle = 'rgba(150,170,195,0.6)';
    ctx.font = '12px ui-monospace, monospace';
    ctx.textAlign = 'center';
    ctx.fillText(r && r.reason ? r.reason : '选通道后点「时频图」', w / 2, h / 2);
    ctx.textAlign = 'left';
    return;
  }
  const freqs = r.freqs || [];
  const times = r.times || [];
  const mag = r.mag || [];
  if (!freqs.length || !times.length) return;

  const left = 34;
  const bottom = 16;
  const pw = w - left - 4;
  const ph = h - bottom - 4;
  const nF = freqs.length;
  const nT = times.length;
  const cw = pw / nT;
  const chh = ph / nF;

  // 只画到奈奎斯特的一半以上会太挤：按实际 fMax 铺满纵向
  const fMax = freqs[nF - 1] || 1;
  for (let ti = 0; ti < nT; ti++) {
    for (let fi = 0; fi < nF; fi++) {
      const v = mag[fi] ? mag[fi][ti] : 0;
      if (v <= 0.02) continue;
      // 用蓝→青→黄→红 的工程色标；低值压暗，避免整片发亮看不出结构
      const a = Math.min(1, Math.pow(v, 0.6));
      const hue = 210 - 190 * a;      // 210(蓝) → 20(橙红)
      const light = 8 + 52 * a;
      ctx.fillStyle = `hsl(${hue}, 85%, ${light}%)`;
      ctx.fillRect(left + ti * cw, ph - (fi + 1) * chh, Math.ceil(cw) + 0.5,
        Math.ceil(chh) + 0.5);
    }
  }

  // 坐标轴
  ctx.strokeStyle = 'rgba(120,160,255,0.25)';
  ctx.beginPath();
  ctx.moveTo(left, h - bottom); ctx.lineTo(w - 4, h - bottom);
  ctx.moveTo(left, 4); ctx.lineTo(left, h - bottom);
  ctx.stroke();

  ctx.fillStyle = 'rgba(140,170,200,0.8)';
  ctx.font = '10px ui-monospace, monospace';
  // 频率刻度（0 / 中 / 顶）
  [0, 0.5, 1].forEach((f) => {
    const y = ph - f * ph + 3;
    ctx.fillText(`${(fMax * f).toFixed(0)}`, 2, Math.max(10, y));
  });
  ctx.fillText('Hz', 2, 8);
  // 时间刻度
  [0, 0.5, 1].forEach((f) => {
    const i = Math.min(nT - 1, Math.round(f * (nT - 1)));
    const tx = left + i * cw;
    ctx.fillText(`${times[i].toFixed(1)}s`, Math.min(w - 34, tx), h - 4);
  });

  const box = $('specInfo');
  if (box) {
    box.innerHTML = `
      <div class="spec-row"><span>通道</span><b>${r.channel || ''}</b>
        <span>采样率</span><b>${r.fs} Hz</b>
        <span>窗长</span><b>${r.nperseg}</b>
        <span>步进</span><b>${r.hop}</b>
        <span>列数</span><b>${r.n_cols}</b></div>
      <div class="hint">纵轴频率、横轴时间；亮点表示该时刻该频率能量强。
        用来回答「某个频率的振动是什么时候出现的」——单帧频谱看不出来</div>`;
  }
}

export function activate() {
  if (spec.activated) return;
  spec.activated = true;

  bus.addEventListener('tuning-channels', (e) => {
    const sel = $('specChannel');
    if (!sel) return;
    const list = (e.detail || {}).channels || [];
    const cur = sel.value;
    sel.innerHTML = list.map((c) =>
      `<option value="${c.name}">${c.name}（${c.n}）</option>`).join('');
    if (cur && list.some((c) => c.name === cur)) sel.value = cur;
    renderMultiSelect();
  });

  bus.addEventListener('spectrum', (e) => {
    spec.result = e.detail || {};
    draw();
    renderInfo();
    const st = $('specState');
    if (st) {
      st.textContent = spec.result.ok
        ? `${spec.channel} · ${spec.result.n_seg} 段平均`
        : (spec.result.reason || '失败');
    }
  });

  const btn = $('specRun');
  if (btn) btn.addEventListener('click', run);
  const mbtn = $('specMultiRun');
  if (mbtn) {
    mbtn.addEventListener('click', () => {
      if (spec.selected.length < 2) {
        const st = $('specState');
        if (st) st.textContent = '请先勾选 2–4 个通道';
        return;
      }
      send({ cmd: 'spectrum_multi', channels: spec.selected, nperseg: 256 });
      const st = $('specState');
      if (st) st.textContent = `多通道对比 ${spec.selected.length} 个…`;
    });
  }
  const sbtn = $('spec2dRun');
  if (sbtn) {
    sbtn.addEventListener('click', () => {
      const sel = $('specChannel');
      if (!sel || !sel.value) return;
      spec.channel = sel.value;
      send({ cmd: 'spectrogram', channel: sel.value, nperseg: 128, hop: 64 });
      const st = $('specState');
      if (st) st.textContent = `时频图 ${sel.value}…`;
    });
  }
  bus.addEventListener('spectrogram', (e) => {
    spec.spec2d = e.detail || {};
    drawSpectro();
    const st = $('specState');
    if (st) {
      st.textContent = spec.spec2d.ok
        ? `${spec.channel} 时频图 · ${spec.spec2d.n_cols} 列`
        : (spec.spec2d.reason || '失败');
    }
  });

  bus.addEventListener('spectrum_multi', (e) => {
    spec.multi = e.detail || {};
    drawMulti();
    const st = $('specState');
    if (st) {
      st.textContent = spec.multi.ok
        ? `共模峰 ${(spec.multi.common || []).length} 个 · ${spec.multi.n_compared} 通道`
        : (spec.multi.reason || '失败');
    }
  });
  const refresh = $('specRefresh');
  if (refresh) {
    refresh.addEventListener('click', () => send({ cmd: 'tuning_channels' }));
  }
  window.addEventListener('resize', draw);

  send({ cmd: 'tuning_channels' });
  draw();
  renderInfo();
}
