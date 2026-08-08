// ============================================================
//  widget.mjs — 通用小部件：格式化 / 人工地平线 / 条形指示
// ============================================================

export function fmt(v, digits = 1) {
  if (v === undefined || v === null || Number.isNaN(v)) return '--';
  return Number(v).toFixed(digits);
}
export function fmtInt(v) {
  if (v === undefined || v === null || Number.isNaN(v)) return '--';
  return Math.round(Number(v)).toString();
}

/** 从遥测快照安全取值 */
export function val(snap, key, def = undefined) {
  const v = snap[key];
  return (v === undefined || v === null) ? def : v;
}

// ============================================================
//  人工地平线（canvas）— 横滚/俯仰/航向 + 刻度
// ============================================================
export function drawAttitude(cv, rollDeg, pitchDeg, headingDeg) {
  const dpr = window.devicePixelRatio || 1;
  const W = cv.width, H = cv.height;
  const ctx = cv.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const w = W / dpr, h = H / dpr;
  const cx = w / 2, cy = h / 2;
  const R = Math.min(w, h) * 0.42;

  // 背景
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = '#0b1019';
  ctx.fillRect(0, 0, w, h);

  // 地平线：先转 roll，再按 pitch 平移
  const p = Math.max(-45, Math.min(45, pitchDeg || 0));
  const dy = p / 45 * R * 0.9;
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(-(rollDeg || 0) * Math.PI / 180);

  // 天地
  ctx.fillStyle = '#123a5e';
  ctx.beginPath();
  ctx.rect(-w, -h, 2 * w, dy);
  ctx.fill();
  ctx.fillStyle = '#3d2f1f';
  ctx.beginPath();
  ctx.rect(-w, dy, 2 * w, 2 * h);
  ctx.fill();

  // 俯仰刻度（+10° 刻度线）
  ctx.strokeStyle = 'rgba(255,255,255,0.55)';
  ctx.lineWidth = 1.5;
  for (let deg = -40; deg <= 40; deg += 10) {
    if (deg === 0) continue;
    const y = dy + deg / 45 * R * 0.9;
    const lw = (deg % 20 === 0) ? 44 : 26;
    ctx.beginPath();
    ctx.moveTo(-lw, y); ctx.lineTo(lw, y);
    ctx.stroke();
    ctx.fillStyle = 'rgba(255,255,255,0.75)';
    ctx.font = '10px Consolas, monospace';
    ctx.textAlign = 'left';
    ctx.fillText(`${deg > 0 ? '+' : ''}${deg}`, lw + 6, y + 3);
  }
  // 地平线主横线
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 2.5;
  ctx.beginPath(); ctx.moveTo(-w, dy); ctx.lineTo(w, dy); ctx.stroke();

  // 机体参考：中央小飞机标记 + 基准线
  ctx.restore();
  ctx.strokeStyle = '#ffd34d';
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(cx - 26, cy); ctx.lineTo(cx - 10, cy);
  ctx.moveTo(cx + 10, cy); ctx.lineTo(cx + 26, cy);
  ctx.stroke();
  ctx.fillStyle = '#ffd34d';
  ctx.beginPath();
  ctx.moveTo(cx, cy - 5); ctx.lineTo(cx + 8, cy); ctx.lineTo(cx, cy + 5); ctx.lineTo(cx - 8, cy);
  ctx.closePath(); ctx.fill();

  // 外圈刻度 + 横滚指针（顶）
  ctx.strokeStyle = 'rgba(120,160,255,0.35)';
  ctx.lineWidth = 1;
  ctx.beginPath(); ctx.arc(cx, cy, R, 0, Math.PI * 2); ctx.stroke();
  for (let deg = 0; deg < 360; deg += 15) {
    const a = (deg - 90) * Math.PI / 180;
    const r1 = R - (deg % 90 === 0 ? 14 : 7);
    ctx.strokeStyle = (deg % 90 === 0) ? 'rgba(255,255,255,0.7)' : 'rgba(255,255,255,0.3)';
    ctx.beginPath();
    ctx.moveTo(cx + r1 * Math.cos(a), cy + r1 * Math.sin(a));
    ctx.lineTo(cx + R * Math.cos(a), cy + R * Math.sin(a));
    ctx.stroke();
  }
  // 顶部横滚指针
  const rp = -(rollDeg || 0) * Math.PI / 180;
  ctx.fillStyle = '#ffd34d';
  ctx.beginPath();
  ctx.moveTo(cx + R * Math.cos(rp) * 0.95, cy + R * Math.sin(rp) * 0.95);
  ctx.lineTo(cx + R * Math.cos(rp + 2.9) * 0.8, cy + R * Math.sin(rp + 2.9) * 0.8);
  ctx.lineTo(cx + R * Math.cos(rp - 2.9) * 0.8, cy + R * Math.sin(rp - 2.9) * 0.8);
  ctx.closePath(); ctx.fill();

  // 航向显示（底部）
  ctx.fillStyle = '#9cc8f5';
  ctx.font = '12px Consolas, monospace';
  ctx.textAlign = 'center';
  ctx.fillText(`HDG ${fmt(headingDeg, 1)}°`, cx, h - 14);
}

// ============================================================
//  条形指示（RC 通道 / 控制输出）
// ============================================================
export function barRow(label, el, { center = false, color = '' } = {}) {
  el.classList.add('bar-row');
  if (center) el.classList.add('center');
  el.innerHTML = `
    <div class="bar-label">${label}</div>
    <div class="bar-track"><div class="bar-fill" style="${color ? `background:${color};` : ''}"></div></div>
    <div class="bar-val">--</div>`;
  const fill = el.querySelector('.bar-fill');
  const vtext = el.querySelector('.bar-val');
  return {
    set(v, { min = 0, max = 100, text = null, invert = false } = {}) {
      if (v === undefined || v === null || Number.isNaN(v)) { fill.style.width = '0%'; vtext.textContent = '--'; return; }
      let pct = (v - min) / (max - min) * 100;
      pct = Math.max(0, Math.min(100, pct));
      if (invert) pct = 100 - pct;
      fill.style.width = pct + '%';
      vtext.textContent = text ?? v.toFixed(1);
    },
  };
}
