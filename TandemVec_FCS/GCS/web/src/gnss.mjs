/**
 * @file    gnss.mjs
 * @module  GCS/前端 GNSS 视图
 * @brief   GNSS 专项诊断页：定位状态卡 + 北东平面散点 + 高度/地速/星数/PDOP 曲线
 * @details 为什么单独做一页：GNSS 问题几乎都不是"看一个数字"能判断的。
 *          定位是否真的收敛，要看北东平面散点有没有收成一点（还是绕圈漂移）；
 *          是否在掉星，要看星数和 PDOP 随时间的形状；多路径/遮挡会表现为
 *          高度突跳而水平尚可。这些都需要"时间序列 + 平面散布"两类视图，
 *          仪表盘上那 6 个瞬时数值给不出判断依据。
 *          数据来源：bus 的 'telemetry' 事件（后端 50Hz 快照），字段在
 *          AnoCom 的 0x30 帧与 MAVLink 的 GPS_RAW_INT 两路里已被统一命名
 *          （gps_fix/gps_sats/gps_lat/gps_lon/gps_alt_m/gps_pdop/gps_sacc/gps_vacc）。
 *          处理：按快照压入环形缓冲并重绘；页面不可见时跳过重绘省 CPU。
 * @theory  N/A（纯显示，不做坐标变换与滤波）
 * @frame   散点图的横轴 = 相对首个定位点的东向偏移，纵轴 = 北向偏移；
 *          不做磁偏/投影换算，仅用于看漂移形状，不作导航依据
 * @units   经度/纬度 deg、高度 m、速度 m/s、PDOP 无量纲、精度 m
 * @ref     GCS-001 §上位机、COMM-001 §4 C 层、REF-175（NEO-F10N 性能）
 */

import { bus, state } from './main.mjs';

const $ = (id) => document.getElementById(id);

// 缓冲上限：50Hz 快照下降采样到 10Hz 存，600 点 ≈ 60 s 历史
const MAX_POINTS = 600;
const SAMPLE_DIV = 5;

const FIX_TEXT = {
  0: '无定位', 1: '无定位', 2: '2D', 3: '3D', 4: 'DGPS', 5: 'RTK 浮点', 6: 'RTK 固定',
  7: '静态', 8: '外推',
};

// fix → 语义色（无定位红 / 2D 橙 / 3D 绿 / RTK 青）
function fixColor(fix) {
  if (fix >= 5) return '#29d3a2';
  if (fix >= 3) return '#3ea6ff';
  if (fix === 2) return '#ffb547';
  return '#ff5d6c';
}

const gnss = {
  pts: [],        // [{n, e, alt}] 北东平面散点（米）
  hist: [],       // [{t, alt, gs, sats, pdop}] 时序
  origin: null,   // {lat, lon} 首个 3D 定位点
  last: null,     // 最近一帧快照
  tick: 0,
  paused: false,
  activated: false,
};

function num(v) {
  const x = Number(v);
  return Number.isFinite(x) ? x : null;
}

/** 经纬度差 → 北东米（小范围平面近似，够看漂移，不作导航） */
function latLonToNe(lat, lon, lat0, lon0) {
  const mPerDegLat = 111132.92 - 559.82 * Math.cos(2 * lat0 * Math.PI / 180);
  const mPerDegLon = 111412.84 * Math.cos(lat0 * Math.PI / 180) - 93.5 * Math.cos(3 * lat0 * Math.PI / 180);
  return { n: (lat - lat0) * mPerDegLat, e: (lon - lon0) * mPerDegLon };
}

function onTelemetry(s) {
  if (gnss.paused) return;
  gnss.last = s;

  const fix = num(s.gps_fix) ?? num(s.fix_type);
  const sats = num(s.gps_sats) ?? num(s.sats);
  const lat = num(s.gps_lat);
  const lon = num(s.gps_lon);
  const alt = num(s.gps_alt_m) ?? num(s.alt_msl_m);
  const pdop = num(s.gps_pdop);
  const vn = num(s.gps_vel_n) ?? num(s.vel_n_ms);
  const ve = num(s.gps_vel_e) ?? num(s.vel_e_ms);

  // 3D 及以上才认为位置可信，才进散点
  if (fix !== null && fix >= 3 && lat !== null && lon !== null
      && !(lat === 0 && lon === 0)) {
    if (!gnss.origin) {
      gnss.origin = { lat, lon };
    } else {
      gnss.tick += 1;
      if (gnss.tick % SAMPLE_DIV === 0) {
        const { n, e } = latLonToNe(lat, lon, gnss.origin.lat, gnss.origin.lon);
        gnss.pts.push({ n, e, alt: alt ?? 0 });
        if (gnss.pts.length > MAX_POINTS) gnss.pts.shift();
      }
    }
  }

  if (fix !== null || sats !== null) {
    gnss.tick += 1;
    if (gnss.tick % SAMPLE_DIV === 0) {
      const gs = (vn !== null && ve !== null) ? Math.hypot(vn, ve) : null;
      gnss.hist.push({
        t: performance.now() / 1000,
        alt, gs, sats, pdop,
      });
      if (gnss.hist.length > MAX_POINTS) gnss.hist.shift();
    }
  }

  updateCards();
  drawAll();
}

function updateCards() {
  const s = gnss.last || {};
  const fix = num(s.gps_fix) ?? num(s.fix_type);
  const sats = num(s.gps_sats) ?? num(s.sats);
  const pdop = num(s.gps_pdop);
  const sacc = num(s.gps_sacc) ?? num(s.gps_eph);
  const vacc = num(s.gps_vacc);
  const lat = num(s.gps_lat);
  const lon = num(s.gps_lon);
  const alt = num(s.gps_alt_m);

  const badge = $('gnssFixBadge');
  if (badge) {
    const txt = fix === null ? '无数据' : (FIX_TEXT[fix] || `类型 ${fix}`);
    badge.textContent = txt;
    badge.style.color = fixColor(fix ?? 0);
    badge.style.borderColor = fixColor(fix ?? 0);
  }
  const set = (id, v) => { const el = $(id); if (el) el.textContent = v; };
  set('gnssSats', sats === null ? '--' : String(sats));
  set('gnssPdop', pdop === null ? '--' : pdop.toFixed(1));
  set('gnssSacc', sacc === null ? '--' : `${sacc.toFixed(1)} m`);
  set('gnssVacc', vacc === null ? '--' : `${vacc.toFixed(1)} m`);
  set('gnssLat', lat === null ? '--' : lat.toFixed(7));
  set('gnssLon', lon === null ? '--' : lon.toFixed(7));
  set('gnssAlt', alt === null ? '--' : `${alt.toFixed(1)} m`);
  set('gnssPts', String(gnss.pts.length));
  const rate = $('gnssRate');
  if (rate) {
    rate.textContent = gnss.origin ? '已锁定原点' : '等待 3D 定位';
  }
}

function drawAll() {
  drawScatter();
  drawSeries();
}

/** 北东平面散点：看定位漂移形状与收敛程度 */
function drawScatter() {
  const cv = $('gnssScatter');
  if (!cv) return;
  const ctx = cv.getContext('2d');
  const w = cv.clientWidth || 320;
  const h = cv.clientHeight || 260;
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  ctx.clearRect(0, 0, w, h);

  // 量程：取散点范围的 1.2 倍，最小 2 m，避免极小漂移被放大成噪声
  let span = 2;
  for (const p of gnss.pts) {
    span = Math.max(span, Math.abs(p.n), Math.abs(p.e));
  }
  span *= 1.2;

  const cx = w / 2;
  const cy = h / 2;
  const scale = Math.min(w, h) / 2 / span;

  // 网格 + 刻度
  ctx.strokeStyle = 'rgba(120,150,190,0.18)';
  ctx.lineWidth = 1;
  for (let i = -2; i <= 2; i++) {
    const gx = cx + (i / 2) * (w / 2);
    const gy = cy + (i / 2) * (h / 2);
    ctx.beginPath(); ctx.moveTo(gx, 0); ctx.lineTo(gx, h); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, gy); ctx.lineTo(w, gy); ctx.stroke();
  }
  // 十字轴
  ctx.strokeStyle = 'rgba(160,190,230,0.45)';
  ctx.beginPath(); ctx.moveTo(cx, 0); ctx.lineTo(cx, h); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(0, cy); ctx.lineTo(w, cy); ctx.stroke();

  // 量程标注
  ctx.fillStyle = 'rgba(170,200,235,0.75)';
  ctx.font = '11px ui-monospace, monospace';
  ctx.fillText(`±${span.toFixed(1)} m`, 8, 16);
  ctx.fillText('N', cx + 4, 14);
  ctx.fillText('E', w - 16, cy - 6);

  if (!gnss.pts.length) {
    ctx.fillStyle = 'rgba(150,170,195,0.6)';
    ctx.textAlign = 'center';
    ctx.fillText('等待 3D 定位…', cx, cy + 40);
    ctx.textAlign = 'left';
    return;
  }

  // 轨迹连线（淡）+ 散点（亮），越新越亮
  ctx.strokeStyle = 'rgba(62,166,255,0.35)';
  ctx.beginPath();
  gnss.pts.forEach((p, i) => {
    const x = cx + p.e * scale;
    const y = cy - p.n * scale;
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();

  const n = gnss.pts.length;
  gnss.pts.forEach((p, i) => {
    const a = 0.15 + 0.75 * ((i + 1) / n);
    ctx.fillStyle = `rgba(41,211,162,${a.toFixed(3)})`;
    const x = cx + p.e * scale;
    const y = cy - p.n * scale;
    ctx.beginPath();
    ctx.arc(x, y, i === n - 1 ? 3.2 : 1.6, 0, Math.PI * 2);
    ctx.fill();
  });
}

/** 高度 / 地速 / 星数 / PDOP 四条时序 */
function drawSeries() {
  const cv = $('gnssSeries');
  if (!cv) return;
  const ctx = cv.getContext('2d');
  const w = cv.clientWidth || 640;
  const h = cv.clientHeight || 200;
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  ctx.clearRect(0, 0, w, h);

  // 没数据时也要画一句提示：整块空白会让人以为页面坏了，
  // 而不是"还没连上设备"——空状态必须自己说明自己。
  if (!gnss.hist.length) {
    ctx.fillStyle = 'rgba(150,170,195,0.6)';
    ctx.font = '12px ui-monospace, monospace';
    ctx.textAlign = 'center';
    ctx.fillText('等待 GNSS 数据…（连接设备后自动开始）', w / 2, h / 2);
    ctx.textAlign = 'left';
    return;
  }

  const series = [
    ['alt', '高度 m', '#3ea6ff'],
    ['gs', '地速 m/s', '#29d3a2'],
    ['sats', '星数', '#ffb547'],
    ['pdop', 'PDOP', '#ff5d6c'],
  ];

  const padL = 8, padR = 8, padT = 10, padB = 16;
  const innerW = w - padL - padR;
  const innerH = h - padT - padB;

  // 每条独立归一化（量纲不同，共用轴没有意义）
  series.forEach(([key, , color]) => {
    const vals = gnss.hist.map(p => num(p[key])).filter(v => v !== null);
    if (vals.length < 2) return;
    let lo = Math.min(...vals);
    let hi = Math.max(...vals);
    if (hi - lo < 1e-6) { lo -= 1; hi += 1; }
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.4;
    ctx.beginPath();
    let started = false;
    gnss.hist.forEach((p, i) => {
      const v = num(p[key]);
      if (v === null) return;
      const x = padL + (i / (gnss.hist.length - 1)) * innerW;
      const y = padT + innerH - ((v - lo) / (hi - lo)) * innerH;
      if (!started) { ctx.moveTo(x, y); started = true; } else { ctx.lineTo(x, y); }
    });
    ctx.stroke();
  });

  // 图例
  ctx.font = '11px ui-monospace, monospace';
  let lx = padL;
  series.forEach(([key, label, color]) => {
    const vals = gnss.hist.map(p => num(p[key])).filter(v => v !== null);
    if (!vals.length) return;
    ctx.fillStyle = color;
    ctx.fillRect(lx, h - 11, 8, 3);
    ctx.fillStyle = 'rgba(200,215,235,0.85)';
    const last = vals[vals.length - 1];
    ctx.fillText(`${label} ${last.toFixed(1)}`, lx + 12, h - 6);
    lx += ctx.measureText(`${label} ${last.toFixed(1)}`).width + 30;
  });
}

export function activate() {
  if (gnss.activated) return;
  gnss.activated = true;
  bus.addEventListener('telemetry', (e) => onTelemetry(e.detail || {}));
  const btn = $('gnssPause');
  if (btn) {
    btn.addEventListener('click', () => {
      gnss.paused = !gnss.paused;
      btn.textContent = gnss.paused ? '继续' : '暂停';
    });
  }
  const clr = $('gnssClear');
  if (clr) {
    clr.addEventListener('click', () => {
      gnss.pts.length = 0;
      gnss.hist.length = 0;
      gnss.origin = null;
      drawAll();
      updateCards();
    });
  }
  window.addEventListener('resize', () => drawAll());
  updateCards();
  drawAll();
}
