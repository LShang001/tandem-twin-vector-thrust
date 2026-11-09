/**
 * @file    dashboard.mjs
 * @module  GCS/前端仪表盘
 * @brief   飞行仪表总览：状态带、人工地平线、KPI、姿态跟踪、控制输出、RC、GNSS、执行器
 * @details 数据来源：bus 的 'telemetry' 事件（后端 50 Hz 快照，AnoCom 与 MAVLink 两协议
 *          在 server 侧已归一成同一套字段名，本模块不做协议分支解析）；
 *          处理：按字段存在性自适应——链路报什么就显示什么，报不了的行整行隐藏，
 *          不显示"永远 --"的假行（这是本次重做的核心判据）；
 *          输出：DOM 文本 + Canvas 人工地平线/摆角仪表。
 * @theory  N/A
 * @frame   NED 下 3-2-1 欧拉角（roll/pitch/heading，deg），仅显示不参与计算
 * @units   deg、deg/s、deg/s²、%、us、m、V（字段名自带后缀，不做二次缩放）
 * @ref     GCS-001 §上位机（仪表盘页）、PROTOCOL-001 §2（0x03/0x21/0x40 字段语义）
 */
import { bus, state } from './main.mjs';
import { drawAttitude, drawTvcDial, barRow, fmt, fmtInt, val } from './widget.mjs';

const $ = (id) => document.getElementById(id);
// 飞行模式名：两协议编号语义不同（AnoCom 自定义 0–3；ArduPilot custom_mode 0–25），
// 用同一张表会把 GUIDED 显示成 AUTO_ALT，所以按协议分开查
const MODE_NAMES_ANO = ['MANUAL', 'AUTO_POS', 'AUTO_ALT', 'GUIDED'];
const MODE_NAMES_MAV = ['STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'LOITER', 'RTL', 'CIRCLE',
  'LAND', 'DRIFT', 'SPORT', 'FLIP', 'AUTOTUNE', 'POSHOLD', 'BRAKE', 'THROW', 'AVOID_ADSB',
  'GUIDED_NOGPS', 'SMART_RTL', 'FLOWHOLD', 'FOLLOW', 'ZIGZAG', 'SYSTEMID', 'AUTOROTATE', 'AUTO_RTL'];

/**
 * @brief   链路协议判定（纯前端特征推断，不改协议层）
 * @details AnoCom 有 0xF1/0xF2 与 0x03 融合字这些专有字段；MAVLink 有 rate/armed/lat_deg。
 *          两边字段名已归一，靠"只有某协议才有的字段"区分，不做耗时统计。
 * @param   {object} s 遥测快照
 * @returns {string|null} 'AnoCom' / 'MAVLink v2' / null（未知）
 */
function detectProto(s) {
  if (!s || !Object.keys(s).length) return null;
  if (s.tvc_upper_deg !== undefined || s.deta100_online !== undefined) return 'AnoCom';
  if (s.roll_rate_dps !== undefined || s.armed !== undefined || s.lat_deg !== undefined) return 'MAVLink v2';
  if (s.ctrl_roll_dps2 !== undefined || s.fusion_sta !== undefined) return 'AnoCom';
  return null;
}

// 控制输出候选行：[标签, 快照键, 量程, 配色, 是否自动扩量程]
// AnoCom 走 α 域（目标角加速度），MAVLink 走角速率；同一物理位置只留一条，由协议决定建哪套。
const CTRL_A = [
  ['α_roll', 'ctrl_roll_dps2', -600, 600, 'linear-gradient(90deg,#ff5d6c,#ffb547)', true, 'deg/s²'],
  ['α_pitch', 'ctrl_pitch_dps2', -600, 600, 'linear-gradient(90deg,#3ea6ff,#29d3a2)', true, 'deg/s²'],
  ['α_yaw', 'ctrl_yaw_dps2', -600, 600, 'linear-gradient(90deg,#b48cff,#ff5d6c)', true, 'deg/s²'],
];
const CTRL_M = [
  ['roll 角速率', 'roll_rate_dps', -120, 120, 'linear-gradient(90deg,#ff5d6c,#ffb547)', true, 'deg/s'],
  ['pitch 角速率', 'pitch_rate_dps', -120, 120, 'linear-gradient(90deg,#3ea6ff,#29d3a2)', true, 'deg/s'],
  ['yaw 角速率', 'yaw_rate_dps', -120, 120, 'linear-gradient(90deg,#b48cff,#ff5d6c)', true, 'deg/s'],
];
const CTRL_COMMON = [
  ['油门', ['ctrl_thr_pct', 'throttle_pct'], 0, 100, undefined, false, '%'],
  ['目标 R', 'target_roll_deg', -45, 45, undefined, false, '°'],
  ['目标 P', 'target_pitch_deg', -45, 45, undefined, false, '°'],
];

// 执行器（AnoCom 0xF1 帧）：机型专有输出，中性命名不绑某一机型术语
const ACT_DEFS = [
  ['摆角 A', 'tvc_upper_deg', -15, 15, 'linear-gradient(90deg,#ff5d6c,#ffb547)', true, '°'],
  ['摆角 B', 'tvc_lower_deg', -15, 15, 'linear-gradient(90deg,#3ea6ff,#29d3a2)', true, '°'],
  ['电机 A', 'motor_front_pct', 0, 100, 'linear-gradient(90deg,#ffb547,#ff5d6c)', false, '%'],
  ['电机 B', 'motor_rear_pct', 0, 100, 'linear-gradient(90deg,#29d3a2,#3ea6ff)', false, '%'],
  ['差速 Δω', 'dw', -1, 1, 'linear-gradient(90deg,#b48cff,#ff5d6c)', false, ''],
];

/** @brief 按别名数组取第一个有值的字段（两协议字段名不同，界面只认逻辑量）
 *  @param {object} s 快照  @param {string|string[]} key 单键或别名数组
 *  @returns {number|undefined} 首个有值的数 */
function pick(s, key) {
  const keys = Array.isArray(key) ? key : [key];
  for (const k of keys) { const v = s[k]; if (v !== undefined && v !== null) return v; }
  return undefined;
}

let rcBars = [];
let ctrlRows = [];      // {wrap, bar, key, opts, span0, auto, unit, maxSeen}
let actRows = [];
let varsRows = {};      // 变量名 → {valEl}
let initializing = false;
let builtProto = null;  // 已按哪个协议建过控制输出行（协议变化时重建）

/**
 * @brief   构建仪表盘 DOM 并订阅 telemetry（后端 50 Hz 快照）
 * @details 建 RC 通道条、控制输出行（先按未知协议建，首个快照到达后按协议重建）、
 *          执行器条与人工地平线、GNSS 状态；订阅 bus 的 'telemetry' 与 'page'
 * @note    幂等：initializing 后直接返回
 * @units   deg、deg/s、deg/s²、%、us、m、V
 * @ref     GCS-001 §上位机（仪表盘页）
 */
export function activate() {
  if (initializing) return;
  initializing = true;

  // RC 通道（us 量纲，8 通道）
  const rcNames = ['CH1', 'CH2', 'CH3 油门', 'CH4 解锁', 'CH5 点火', 'CH6 模式', 'CH7 TVC', 'CH8'];
  const rcWrap = $('rcBars');
  rcWrap.innerHTML = '';
  rcBars = rcNames.map((n) => {
    const el = document.createElement('div');
    rcWrap.appendChild(el);
    return barRow(n, el, { center: true });
  });

  buildCtrlRows('AnoCom');   // 先按 AnoCom 建，首个快照到了再按真实协议校正

  // 执行器条（AnoCom 0xF1 帧）
  const actWrap = $('actBars');
  actWrap.innerHTML = '';
  actRows = ACT_DEFS.map(([label, key, mn, mx, color, auto, unit]) => {
    const el = document.createElement('div');
    actWrap.appendChild(el);
    return { wrap: el, bar: barRow(label, el, { center: mn < 0, color }), key, mn, mx, auto, unit };
  });

  bus.addEventListener('telemetry', (e) => onTelemetry(e.detail));   // ★ CustomEvent，快照在 detail
  bus.addEventListener('page', onPage);
  onTelemetry(state.snap);
}

/**
 * @brief   按协议重建控制输出行（同一物理位置只留一条：α 域或角速率）
 * @details 先清空容器再建；COMMON 段（油门/目标姿态）两协议都有，放后面
 * @param   {string} proto 'AnoCom' / 'MAVLink v2'
 */
function buildCtrlRows(proto) {
  if (builtProto === proto) return;
  builtProto = proto;
  const wrap = $('ctrlBars');
  wrap.innerHTML = '';
  const defs = (proto === 'MAVLink v2' ? CTRL_M : CTRL_A).concat(CTRL_COMMON);
  ctrlRows = defs.map(([label, key, mn, mx, color, auto, unit]) => {
    const el = document.createElement('div');
    wrap.appendChild(el);
    const bar = barRow(label, el, { center: mn < 0, color });
    return { wrap: el, bar, key, mn, mx, auto, unit, maxSeen: Math.abs(mn) };
  });
  const hint = $('ctrlHint');
  if (hint) hint.textContent = proto === 'MAVLink v2' ? '角速率 · 机载上报' : '目标角加速度 α · 机型分配';
}

// 变量面板：telemetry 快照里 vars_ 前缀键动态渲染（上限 16 行，超出滚动）
function renderVars(s) {
  const body = $('varsBody');
  const empty = $('varsEmpty');
  let found = 0;
  for (const [k, v] of Object.entries(s)) {
    if (!k.startsWith('vars_') || v === undefined) continue;
    const name = k.slice(5);
    found++;
    let row = varsRows[name];
    if (!row) {
      const div = document.createElement('div');
      div.className = 'vars-row';
      const nm = document.createElement('span');
      nm.className = 'vars-name';
      nm.textContent = name;
      const vl = document.createElement('span');
      vl.className = 'vars-val';
      div.append(nm, vl);
      body.appendChild(div);
      row = { valEl: vl };
      varsRows[name] = row;
    }
    row.valEl.textContent = (typeof v === 'number') ? v.toFixed(4) : String(v);
  }
  if (empty) empty.style.display = found ? 'none' : '';
}

function onPage(e) {
  if (e.detail === 'dashboard') onTelemetry(state.snap);
}

/**
 * @brief   更新状态带单项
 * @param   {string} id 元素 id
 * @param   {string} text 主文本
 * @param   {string} cls '' | 'ok' | 'warn' | 'bad' | 'off'
 */
function setStat(id, text, cls) {
  const el = $(id);
  if (!el) return;
  el.className = 'dstat ' + (cls || '');
  const b = el.querySelector('b');
  if (b) b.textContent = text;
}

// 遥测到达频率（Hz）——状态带「遥测」项，反映链路是否还在喘气
let hzCount = 0;
let hzT0 = performance.now();
let hzVal = null;

function onTelemetry(s) {
  if (!s) return;
  hzCount++;
  const nowMs = performance.now();
  if (nowMs - hzT0 >= 1000) {
    hzVal = hzCount * 1000 / (nowMs - hzT0);
    hzCount = 0; hzT0 = nowMs;
  }

  // 协议自适应：首个带特征的快照决定控制输出建哪套行
  const proto = detectProto(s);
  if (proto && proto !== builtProto) buildCtrlRows(proto);

  // ---- 状态带 ----
  const arm = s.unlocked !== undefined ? s.unlocked : s.armed;
  const fix = s.gps_fix, sats = s.gps_sats;
  setStat('stProto', proto || '--', proto ? 'ok' : 'off');
  const modeNames = proto === 'MAVLink v2' ? MODE_NAMES_MAV : MODE_NAMES_ANO;
  setStat('stMode', s.flight_mode === undefined ? '--' : (modeNames[s.flight_mode] ?? `#${s.flight_mode}`),
    s.flight_mode === undefined ? 'off' : (s.flight_mode === 0 ? '' : 'ok'));
  setStat('stArm', arm === undefined ? '--' : (arm ? '已解锁' : '锁定'),
    arm === undefined ? 'off' : (arm ? 'warn' : 'ok'));
  setStat('stGps', (fix === undefined && sats === undefined) ? '--'
    : `Fix ${fix ?? '--'} · ${sats ?? '--'} 星`,
  (fix === undefined) ? 'off' : (fix >= 3 && (sats ?? 0) >= 6 ? 'ok' : 'bad'));
  // 这一格显示"机载健康"：AnoCom 报融合状态位（0x03 帧），MAVLink 报 CPU 负载（SYS_STATUS.load）
  if (s.fusion_sta !== undefined) {
    setStat('stFusion', (s.fusion_sta & 0x3f) ? 'EKF 收敛' : '未初始化', (s.fusion_sta & 0x3f) ? 'ok' : 'warn');
  } else if (s.cpu_load_pct !== undefined) {
    setStat('stFusion', `CPU ${s.cpu_load_pct.toFixed(0)}%`, s.cpu_load_pct > 85 ? 'bad' : (s.cpu_load_pct > 70 ? 'warn' : 'ok'));
  } else {
    setStat('stFusion', '未上报', 'off');
  }
  setStat('stLink', hzVal === null ? '--' : `${hzVal.toFixed(1)} Hz`,
    hzVal === null ? 'off' : (hzVal >= 5 ? 'ok' : (hzVal > 0 ? 'warn' : 'bad')));

  // ---- 姿态 ----
  drawAttitude($('cvAttitude'), s.roll_deg, s.pitch_deg, s.heading_deg ?? s.yaw_deg);
  $('attRoll').textContent = fmt(s.roll_deg, 1) + '°';
  $('attPitch').textContent = fmt(s.pitch_deg, 1) + '°';
  $('attHead').textContent = fmt(s.heading_deg ?? s.yaw_deg, 1) + '°';
  const rr = val(s, 'roll_rate_dps'), pr = val(s, 'pitch_rate_dps'), yr = val(s, 'yaw_rate_dps');
  $('attRate').textContent = (rr !== undefined)
    ? `角速率 ${fmt(rr, 1)} / ${fmt(pr, 1)} / ${fmt(yr, 1)} °/s`
    : '角速率 — 本链路未上报';

  // ---- KPI ----
  const velN = val(s, 'vel_n_ms'), velE = val(s, 'vel_e_ms'), velUp = val(s, 'vel_up_ms');
  $('kSpeed').textContent = (velN !== undefined && velE !== undefined) ? fmt(Math.hypot(velN, velE), 1) : '--';
  $('kVspeed').textContent = fmt(velUp, 1);
  $('kAlt').textContent = fmt(val(s, 'alt_fu_m', val(s, 'rel_h_m')), 1);
  $('kThr').textContent = fmt(pick(s, ['ctrl_thr_pct', 'throttle_pct']), 0);
  $('kBat').textContent = fmt(val(s, 'bat_voltage_v'), 2);
  const rn = val(s, 'rel_n_m'), re = val(s, 'rel_e_m');
  $('kPos').textContent = (rn !== undefined && re !== undefined) ? `${fmt(rn, 1)}/${fmt(re, 1)}` : '--';

  // ---- 姿态跟踪（指令 vs 实际）----
  const tr = val(s, 'target_roll_deg'), tp = val(s, 'target_pitch_deg');
  const hasTar = tr !== undefined || tp !== undefined;
  $('trackHint').textContent = hasTar ? '' : '本链路无姿态指令帧';
  $('tkRollCmd').textContent = fmt(tr, 1) + '°';
  $('tkPitchCmd').textContent = fmt(tp, 1) + '°';
  $('tkRollAct').textContent = fmt(s.roll_deg, 1) + '°';
  $('tkPitchAct').textContent = fmt(s.pitch_deg, 1) + '°';
  const eR = (tr !== undefined && s.roll_deg !== undefined) ? tr - s.roll_deg : undefined;
  const eP = (tp !== undefined && s.pitch_deg !== undefined) ? tp - s.pitch_deg : undefined;
  $('tkRollErr').textContent = fmt(eR, 2) + '°';
  $('tkPitchErr').textContent = fmt(eP, 2) + '°';
  // 误差着色：>5° 红，>2° 橙，其余绿
  for (const [id, e] of [['tkRollErr', eR], ['tkPitchErr', eP]]) {
    const el = $(id);
    el.className = 'err ' + (e === undefined ? '' : (Math.abs(e) > 5 ? 'bad' : (Math.abs(e) > 2 ? 'warn' : 'ok')));
  }

  // ---- RC 条（raw_rc_values，us 量纲）----
  const hasRc = s.rc1 !== undefined || (s.rc_channels !== undefined && s.rc_channels.length > 0);
  $('rcBars').style.display = hasRc ? '' : 'none';
  // ★ 不能写 style.display = ''：那是"清除内联样式"，会回落到 CSS 的 display:none
  $('rcNoData').style.display = hasRc ? 'none' : 'block';
  if (hasRc) {
    rcBars.forEach((bar, i) => {
      const v = s[`rc${i + 1}`];
      bar.set(v, { min: 1000, max: 2000, text: v !== undefined ? fmtInt(v) : '--' });
    });
  }

  // ---- 控制输出（按字段存在性显隐；不存在整行隐藏，不留 "--" 假行）----
  for (const r of ctrlRows) {
    const v = pick(s, r.key);
    if (v === undefined) {
      r.wrap.style.display = 'none';
      continue;
    }
    r.wrap.style.display = '';
    let mn = r.mn, mx = r.mx;
    if (r.auto) {                     // 量程自适应：α 域随机型差异可达 10 倍
      r.maxSeen = Math.max(r.maxSeen * 0.999, Math.abs(v));
      const span = Math.max(Math.abs(r.mn), Math.ceil(r.maxSeen / 100) * 100);
      mn = -span; mx = span;
    }
    r.bar.set(v, { min: mn, max: mx, text: fmt(v, v > 100 || v < -100 ? 0 : 1) });
  }

  // ---- 执行器（AnoCom 0xF1）：有字段才显示仪表区 ----
  const hasAct = s.tvc_upper_deg !== undefined || s.motor_front_pct !== undefined;
  $('actBody').classList.toggle('no-data', !hasAct);
  $('actHint').textContent = hasAct ? '机型输出帧 · 实测量' : '本链路不上报';
  if (hasAct) {
    const df = val(s, 'tvc_upper_deg'), dt = val(s, 'tvc_lower_deg');
    const mf = val(s, 'motor_front_pct'), mt = val(s, 'motor_rear_pct');
    drawTvcDial($('cvTvcFront'), df, mf, { color: '#ffb547' });
    drawTvcDial($('cvTvcRear'), dt, mt, { color: '#3ea6ff' });
    $('tvcFrontText').textContent = `摆角A ${fmt(df, 1)}° · 电机A ${fmtInt(mf)}%`;
    $('tvcRearText').textContent = `摆角B ${fmt(dt, 1)}° · 电机B ${fmtInt(mt)}%`;
    for (const r of actRows) {
      r.bar.set(val(s, r.key), { min: r.mn, max: r.mx, text: fmt(val(s, r.key), 1) });
    }
    for (const [id, key] of [['satDf', 'sat_df'], ['satDt', 'sat_dt'], ['satDw', 'sat_dw']]) {
      const el = $(id);
      el.className = 'status-chip' + (s[key] === undefined ? '' : (s[key] ? ' bad' : ' ok'));
    }
  }

  // ---- GNSS（两协议字段名不同：anocom=gps_lat/gps_lon，mavlink=lat_deg/lon_deg）----
  const gFix = s.gps_fix ?? s.fix_type;
  $('gpsFix').textContent = gFix !== undefined ? `Fix ${gFix}` : '--';
  $('gpsSats').textContent = fmtInt(s.gps_sats ?? s.satellites_visible);
  $('gpsLon').textContent = fmt(s.gps_lon ?? s.lon_deg, 6);
  $('gpsLat').textContent = fmt(s.gps_lat ?? s.lat_deg, 6);
  $('gpsAlt').textContent = fmt(s.gps_alt_m ?? (s.alt_mm !== undefined ? s.alt_mm / 1000 : undefined), 1);
  // 精度项：AnoCom 直报 PDOP（无量纲），MAVLink 只有水平精度 gps_eph（m）。
  // 两者单位不同，所以标签跟着换——共用 "PDOP" 标签会把 0.9 m 说成 PDOP 0.9，是错的。
  if (s.gps_pdop !== undefined) {
    $('gpsAccLabel').textContent = 'PDOP';
    $('gpsPdop').textContent = fmt(s.gps_pdop, 1);
  } else if (s.gps_eph !== undefined) {
    $('gpsAccLabel').textContent = '水平精度';
    $('gpsPdop').textContent = fmt(s.gps_eph, 2) + ' m';
  } else {
    $('gpsAccLabel').textContent = '精度';
    $('gpsPdop').textContent = '--';
  }

  // ---- 变量面板 ----
  renderVars(s);
}
