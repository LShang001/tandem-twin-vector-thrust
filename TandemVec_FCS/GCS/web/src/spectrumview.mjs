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
  channel: '',
  activated: false,
};

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
  const refresh = $('specRefresh');
  if (refresh) {
    refresh.addEventListener('click', () => send({ cmd: 'tuning_channels' }));
  }
  window.addEventListener('resize', draw);

  send({ cmd: 'tuning_channels' });
  draw();
  renderInfo();
}
