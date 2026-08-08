// ============================================================
//  dashboard.mjs — 仪表盘：人工地平线 + HUD + RC/控制条 + GNSS
// ============================================================
import { bus, state } from './main.mjs';
import { drawAttitude, drawTvcDial, barRow, fmt, fmtInt, val } from './widget.mjs';

const $ = (id) => document.getElementById(id);
const MODE_NAMES = ['MANUAL', 'AUTO_POS', 'AUTO_ALT', 'GUIDED'];
const CHIP_ORDER = [
  ['chipMode', 'mode', (s) => `模式 ${MODE_NAMES[s.flight_mode] ?? '?'}`],
  ['chipUnlock', 'unlock', (s) => s.unlocked ? '已解锁' : '锁定'],
  ['chipGps', 'gps', (s) => `GPS ${s.gps_fix ?? '--'}/${s.gps_sats ?? '--'}星`],
  ['chipFusion', 'fusion', (s) => `融合 ${s.fusion_sta !== undefined ? (s.fusion_sta & 0x3f) ? 'EKF' : '未初始' : '--'}`],
  ['chipDeta', 'deta', (s) => `DETA100 ${s.deta100_online ? '在线' : '离线'}`],
];

let rcBars = [];
let ctrlBars = [];
let actBars = [];
let initializing = false;

export function activate() {
  if (initializing) return;
  initializing = true;
  const rcNames = ['CH1', 'CH2', 'CH3 油门', 'CH4 解锁', 'CH5 点火', 'CH6 模式', 'CH7 TVC', 'CH8'];
  const rcWrap = $('rcBars');
  rcWrap.innerHTML = '';
  rcBars = rcNames.map((n) => {
    const el = document.createElement('div');
    rcWrap.appendChild(el);
    return barRow(n, el, { center: true });
  });

  const ctrlWrap = $('ctrlBars');
  ctrlWrap.innerHTML = '';
  const ctrlDefs = [
    ['α_roll (rad/s²)', 'ctrl_roll', { min: -10, max: 10 }],
    ['α_pitch', 'ctrl_pitch', { min: -10, max: 10 }],
    ['油门 %', 'ctrl_thr_pct', { min: 0, max: 100 }],
    ['α_yaw (差速)', 'ctrl_yaw', { min: -10, max: 10 }],
    ['目标 R', 'target_roll_deg', { min: -45, max: 45 }],
    ['目标 P', 'target_pitch_deg', { min: -45, max: 45 }],
  ];
  ctrlBars = ctrlDefs.map(([label, key, opts]) => {
    const el = document.createElement('div');
    ctrlWrap.appendChild(el);
    const bar = barRow(label, el, { center: true, color: key.includes('roll') ? 'linear-gradient(90deg,#ff5d6c,#ffb547)' : undefined });
    return { bar, key, opts };
  });

  // 执行器条（0x40 帧：TVC 摆角 / 电机推力 / 差速）
  const actWrap = $('actBars');
  actWrap.innerHTML = '';
  const actDefs = [
    // [标签, 快照key, 量程, 居中(±量程), 配色]
    ['前摆 δf °', 'tvc_front_deg', { min: -15, max: 15 }, true, 'linear-gradient(90deg,#ff5d6c,#ffb547)'],
    ['尾摆 δt °', 'tvc_rear_deg', { min: -15, max: 15 }, true, 'linear-gradient(90deg,#3ea6ff,#29d3a2)'],
    ['前电机 %', 'motor_front_pct', { min: 0, max: 100 }, false, 'linear-gradient(90deg,#ffb547,#ff5d6c)'],
    ['尾电机 %', 'motor_rear_pct', { min: 0, max: 100 }, false, 'linear-gradient(90deg,#29d3a2,#3ea6ff)'],
    ['差速 Δω', 'dw', { min: -1, max: 1 }, true, 'linear-gradient(90deg,#b48cff,#ff5d6c)'],
  ];
  actBars = actDefs.map(([label, key, opts, center, color]) => {
    const el = document.createElement('div');
    actWrap.appendChild(el);
    const bar = barRow(label, el, { center, color });
    return { bar, key, opts };
  });

  bus.addEventListener('telemetry', (e) => onTelemetry(e.detail));   // ★ CustomEvent，快照在 detail
  bus.addEventListener('page', onPage);
  onTelemetry(state.snap);
}

function onPage(e) {
  if (e.detail === 'dashboard') onTelemetry(state.snap);
}

function onTelemetry(s) {
  // 姿态
  drawAttitude($('cvAttitude'), s.roll_deg, s.pitch_deg, s.heading_deg);
  $('attReadout').textContent =
    `R ${fmt(s.roll_deg, 1)}°   P ${fmt(s.pitch_deg, 1)}°   H ${fmt(s.heading_deg, 1)}°`;

  // HUD
  const velN = val(s, 'vel_n_ms'), velE = val(s, 'vel_e_ms'), velUp = val(s, 'vel_up_ms');
  $('hudSpeed').textContent = (velN !== undefined && velE !== undefined)
    ? fmt(Math.hypot(velN, velE), 1) : '--';
  $('hudVspeed').textContent = fmt(velUp, 1);
  $('hudAlt').textContent = fmt(val(s, 'alt_fu_m', val(s, 'rel_h_m')), 1);
  const rn = val(s, 'rel_n_m'), re = val(s, 'rel_e_m');
  $('hudPos').textContent = (rn !== undefined && re !== undefined) ? `${fmt(rn, 1)} / ${fmt(re, 1)}` : '--';
  $('hudThr').textContent = fmt(val(s, 'ctrl_thr_pct'));
  $('hudBat').textContent = fmt(val(s, 'bat_voltage_v', 0), 2);
  const tr = val(s, 'target_roll_deg'), tp = val(s, 'target_pitch_deg');
  $('hudTar').textContent = (tr !== undefined && tp !== undefined) ? `${fmt(tr, 1)} / ${fmt(tp, 1)}` : '--';

  // 状态灯
  const chips = [
    ['chipMode', 'mode', s.flight_mode !== undefined && s.flight_mode !== 0, s.flight_mode !== undefined],
    ['chipUnlock', 'unlock', !!s.unlocked, s.unlocked !== undefined],
    ['chipGps', 'gps', (s.gps_fix !== undefined && s.gps_fix >= 3 && s.gps_sats >= 6), s.gps_sats !== undefined],
    ['chipFusion', 'fusion', !!(s.fusion_sta & 0x3f), s.fusion_sta !== undefined],
    ['chipDeta', 'deta', !!s.deta100_online, s.deta100_online !== undefined],
  ];
  for (const [id, , ok, known] of chips) {
    const el = $(id);
    el.className = 'status-chip ' + (known ? (ok ? 'ok' : 'bad') : '');
  }
  $('chipMode').querySelector('span').textContent = `模式 ${MODE_NAMES[s.flight_mode] ?? '--'}`;
  $('chipUnlock').querySelector('span').textContent = s.unlocked ? '已解锁' : '锁定';
  $('chipGps').querySelector('span').textContent = `GPS ${s.gps_fix ?? '--'}/${s.gps_sats ?? '--'}星`;
  $('chipFusion').querySelector('span').textContent = (s.fusion_sta & 0x3f) ? '融合 EKF' : '融合 --';
  $('chipDeta').querySelector('span').textContent = s.deta100_online ? 'DETA100 在线' : 'DETA100 离线';

  // RC 条（raw_rc_values，us 量纲）
  rcBars.forEach((bar, i) => {
    const v = s[`rc${i + 1}`];
    bar.set(v, { min: 1000, max: 2000, text: v !== undefined ? fmtInt(v) : '--' });
  });

  // 控制条
  for (const { bar, key, opts } of ctrlBars) {
    bar.set(val(s, key), opts);
  }

  // 执行器（0x40 帧）：TVC 矢量仪表 + 条 + 饱和标记
  const df = val(s, 'tvc_front_deg'), dt = val(s, 'tvc_rear_deg');
  const mf = val(s, 'motor_front_pct'), mt = val(s, 'motor_rear_pct');
  drawTvcDial($('cvTvcFront'), df, mf, { color: '#ffb547' });
  drawTvcDial($('cvTvcRear'), dt, mt, { color: '#3ea6ff' });
  $('tvcFrontText').textContent = (df !== undefined && mf !== undefined)
    ? `δf ${fmt(df, 1)}° · ${fmtInt(mf)}%` : 'δf --° · --%';
  $('tvcRearText').textContent = (dt !== undefined && mt !== undefined)
    ? `δt ${fmt(dt, 1)}° · ${fmtInt(mt)}%` : 'δt --° · --%';
  for (const { bar, key, opts } of actBars) {
    bar.set(val(s, key), opts);
  }
  const sats = [
    ['satDf', 'sat_df'], ['satDt', 'sat_dt'], ['satDw', 'sat_dw'],
  ];
  for (const [id, key] of sats) {
    const el = $(id);
    if (s[key] === undefined) el.className = 'status-chip';
    else el.className = 'status-chip ' + (s[key] ? 'bad' : 'ok');
  }

  // GNSS
  $('gpsFix').textContent = s.gps_fix !== undefined ? `Fix ${s.gps_fix}` : '--';
  $('gpsSats').textContent = fmtInt(s.gps_sats);
  $('gpsLon').textContent = fmt(s.gps_lon, 6);
  $('gpsLat').textContent = fmt(s.gps_lat, 6);
  $('gpsAlt').textContent = fmt(s.gps_alt_m, 1);
  $('gpsPdop').textContent = fmt(s.gps_pdop, 1);
}

// 立即渲染一次（页面加载时）
bus.addEventListener('ws-open', () => onTelemetry(state.snap));
