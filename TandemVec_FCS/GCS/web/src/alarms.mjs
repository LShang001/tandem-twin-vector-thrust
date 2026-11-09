/**
 * @file    alarms.mjs
 * @module  GCS/前端 报警与事件
 * @brief   活动告警、事件历史、规则说明；并维护全局顶部告警横幅
 * @details 两个关注点：
 *          ① **顶部横幅**常驻在所有页面之上——告警如果只在某个页面里可见，
 *             切页就会漏看，这是最危险的失效模式；
 *          ② **事件历史**回答"什么时候坏的、什么时候好的"，比"当前有没有问题"
 *             更有用于事后复盘。
 *
 *          数据来源：WS `alarm` 消息（状态跳变时推送）+ `alarm_state`
 *          （主动拉取全量）。跳变推送保证及时，全量拉取保证页面首次打开时
 *          不会漏掉已经存在的活动告警。
 *
 *          声音：只在 `crit` 级且用户已交互过（浏览器自动播放策略）时响一声，
 *          用短促方波而不是长音——现场吵，长音听不出是警告还是噪音。
 * @theory  N/A
 * @frame   N/A
 * @units   时间戳为 Unix 秒
 * @ref     server/alarms.py、GCS-001 §上位机
 */

import { bus, send } from './main.mjs';

const $ = (id) => document.getElementById(id);

const LEVEL_LABEL = { crit: '严重', warn: '警告', info: '提示' };
const LEVEL_COLOR = { crit: '#ff5d6c', warn: '#ffb547', info: '#7d92ad' };

const alarmState = {
  active: [],
  events: [],
  rules: [],
  activated: false,
  bannerEl: null,
  soundOn: false,
  audioCtx: null,
};

/** 响一声短促提示音（仅严重级） */
function beep() {
  if (!alarmState.soundOn) return;
  try {
    if (!alarmState.audioCtx) {
      const AC = window.AudioContext || window.webkitAudioContext;
      if (!AC) return;
      alarmState.audioCtx = new AC();
    }
    const ctx = alarmState.audioCtx;
    const osc = ctx.createOscillator();
    const gain = ctx.createGain();
    osc.type = 'square';
    osc.frequency.value = 880;
    gain.gain.setValueAtTime(0.06, ctx.currentTime);
    gain.gain.exponentialRampToValueAtTime(0.0001, ctx.currentTime + 0.22);
    osc.connect(gain).connect(ctx.destination);
    osc.start();
    osc.stop(ctx.currentTime + 0.24);
  } catch (e) { /* 音频不可用时静默，不影响告警显示 */ }
}

function fmtTime(ts) {
  if (!ts) return '--';
  const d = new Date(ts * 1000);
  const p = (n) => String(n).padStart(2, '0');
  return `${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}`;
}

/** 顶部横幅：任何页面都能看到活动告警 */
function renderBanner() {
  const el = alarmState.bannerEl;
  if (!el) return;
  if (!alarmState.active.length) {
    el.classList.remove('show');
    el.innerHTML = '';
    // 横幅消失时把让出的高度还回去，否则页面顶部会留一条空白
    document.body.classList.remove('has-alarm');
    return;
  }
  document.body.classList.add('has-alarm');
  // 严重优先、其次警告、最后提示
  const order = { crit: 0, warn: 1, info: 2 };
  const sorted = [...alarmState.active].sort(
    (a, b) => (order[a.level] ?? 9) - (order[b.level] ?? 9));
  const worst = sorted[0];
  el.className = `alarm-banner show level-${worst.level}`;
  el.innerHTML = sorted.map((a) =>
    `<span class="alarm-item"><b>${LEVEL_LABEL[a.level] || a.level}</b> ${a.text}</span>`,
  ).join('<span class="alarm-sep">·</span>');
}

function renderActive() {
  const box = $('alarmActive');
  if (!box) return;
  if (!alarmState.active.length) {
    box.innerHTML = '<div class="alarm-ok">✓ 当前无告警</div>';
    return;
  }
  box.innerHTML = alarmState.active.map((a) => `
    <div class="alarm-card level-${a.level}">
      <span class="alarm-badge">${LEVEL_LABEL[a.level] || a.level}</span>
      <span class="alarm-text">${a.text}</span>
    </div>`).join('');
}

function renderEvents() {
  const body = $('alarmEventRows');
  if (!body) return;
  if (!alarmState.events.length) {
    body.innerHTML = '<tr><td colspan="4" class="insp-empty">暂无事件</td></tr>';
    return;
  }
  // 新的在上
  const rows = [...alarmState.events].reverse();
  body.innerHTML = rows.map((e) => {
    const cls = e.kind === 'clear' ? 'ev-clear' : `ev-${e.level}`;
    const kind = e.kind === 'clear' ? '恢复' : '触发';
    return `<tr class="${cls}">
      <td class="num">${fmtTime(e.ts)}</td>
      <td>${kind}</td>
      <td>${e.title}</td>
      <td>${e.text}</td>
    </tr>`;
  }).join('');
}

function renderRules() {
  const body = $('alarmRuleRows');
  if (!body || !alarmState.rules.length) return;
  body.innerHTML = alarmState.rules.map((r) => `
    <tr>
      <td>${r.title}</td>
      <td><span class="alarm-badge small level-${r.level}">${LEVEL_LABEL[r.level] || r.level}</span></td>
      <td class="num">${r.cooldown_s ? r.cooldown_s + ' s' : '—'}</td>
      <td class="hint">${r.note || ''}</td>
    </tr>`).join('');
}

function applyState(d) {
  alarmState.active = d.active || [];
  alarmState.events = d.events || [];
  if (d.rules) alarmState.rules = d.rules;
  renderBanner();
  renderActive();
  renderEvents();
  renderRules();
  const stat = $('alarmStat');
  if (stat) {
    stat.textContent =
      `${alarmState.active.length} 条活动 · ${alarmState.events.length} 条事件`;
  }
}

function onAlarmEvent(d) {
  if (!d || !d.key) return;
  if (d.kind === 'raise') {
    alarmState.events.push(d);
    const i = alarmState.active.findIndex((a) => a.key === d.key);
    if (i >= 0) alarmState.active[i] = d; else alarmState.active.push(d);
    if (d.level === 'crit') beep();
  } else {
    alarmState.events.push(d);
    alarmState.active = alarmState.active.filter((a) => a.key !== d.key);
  }
  renderBanner();
  renderActive();
  renderEvents();
  const stat = $('alarmStat');
  if (stat) {
    stat.textContent =
      `${alarmState.active.length} 条活动 · ${alarmState.events.length} 条事件`;
  }
}

export function activate() {
  if (alarmState.activated) return;
  alarmState.activated = true;

  alarmState.bannerEl = $('alarmBanner');

  bus.addEventListener('alarm', (e) => onAlarmEvent(e.detail));
  bus.addEventListener('alarm-state', (e) => applyState(e.detail || {}));

  const clr = $('alarmClearEv');
  if (clr) clr.addEventListener('click', () => send({ cmd: 'alarm_clear_events' }));

  const snd = $('alarmSound');
  if (snd) {
    snd.addEventListener('change', () => {
      alarmState.soundOn = snd.checked;
      if (snd.checked) beep();   // 顺便解锁音频上下文
    });
  }

  // 首次拉全量：跳变推送只覆盖"之后"的变化，已存在的活动告警得主动问一次
  send({ cmd: 'alarm_state' });
  renderBanner();
}
