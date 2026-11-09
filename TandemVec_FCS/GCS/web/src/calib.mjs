/**
 * @file    calib.mjs
 * @module  GCS/前端 传感器校准向导
 * @brief   磁罗盘标定：分步执行 → 实时解析固件回传 → 质量判读
 * @details 固件的 `magcal` 命令本身是完整的，但使用者要记住一串前置条件：
 *          必须未解锁、自由采集要绕三轴且样本≥30、采完要 fit、应用后必须 save、
 *          判好坏要看四个数字。这些不该要求人背下来——向导把它变成点击 + 提示。
 *
 *          关键设计：**判读在前、保存在后**。如果先 save 再判读，用户看到
 *          "残差太大"时坏参数已经写进 Flash 了。所以向导把 assess 放在 save
 *          之前，并在结果不合格时把 save 按钮标红劝阻。
 * @theory  N/A
 * @frame   N/A
 * @units   见后端 calibration.py
 * @ref     server/calibration.py
 */

import { bus, send } from './main.mjs';

const $ = (id) => document.getElementById(id);

const calib = {
  steps: [],
  cur: 0,
  data: {},
  assess: null,
  activated: false,
  log: [],
};

function renderSteps() {
  const box = $('calibSteps');
  if (!box) return;
  if (!calib.steps.length) {
    box.innerHTML = '<div class="hint">正在读取向导步骤…</div>';
    return;
  }
  box.innerHTML = calib.steps.map((s, i) => {
    const cls = i < calib.cur ? 'done' : (i === calib.cur ? 'cur' : 'todo');
    return `<div class="calib-step ${cls}">
      <div class="calib-step-head">
        <span class="calib-num">${i + 1}</span>
        <span class="calib-title">${s.title}</span>
        <button class="btn small" data-step="${s.id}">执行</button>
      </div>
      <div class="calib-hint">${s.hint}</div>
    </div>`;
  }).join('');

  box.querySelectorAll('button[data-step]').forEach((b) => {
    b.addEventListener('click', () => {
      const id = b.dataset.step;
      calib.cur = Math.max(calib.cur, calib.steps.findIndex((s) => s.id === id));
      send({ cmd: 'calib_step', id });
      renderSteps();
    });
  });
}

function renderData() {
  const box = $('calibData');
  if (!box) return;
  const d = calib.data || {};
  const rows = [
    ['采样数', d.samples],
    ['拟合残差', d.residual_rms],
    ['特征值比', d.eig_ratio],
    ['姿态覆盖', d.coverage],
    ['总场强度', d.field_mag_ut],
  ];
  const any = rows.some(([, v]) => v !== undefined);
  if (!any) {
    box.innerHTML = '<div class="hint">尚未读到标定数据（执行「查看当前标定状态」开始）</div>';
    return;
  }
  box.innerHTML = `<div class="calib-kv">${rows.map(([k, v]) => `
    <div class="calib-kv-item">
      <span class="k">${k}</span>
      <span class="v">${v === undefined ? '--' :
        (typeof v === 'number' ? (Number.isInteger(v) ? v : v.toFixed(4)) : v)}</span>
    </div>`).join('')}</div>`;
}

function renderAssess() {
  const box = $('calibAssess');
  if (!box) return;
  const a = calib.assess;
  if (!a) {
    box.innerHTML = '';
    return;
  }
  if (!a.checks || !a.checks.length) {
    box.innerHTML = '<div class="hint">还没有可判读的数据</div>';
    return;
  }
  const head = a.ok
    ? `<div class="calib-verdict ok">✓ 本次标定可用（得分 ${a.score}）</div>`
    : `<div class="calib-verdict bad">✗ 本次标定有问题（得分 ${a.score}）</div>`;
  const items = a.checks.map((c) => `
    <div class="calib-check ${c.ok ? 'ok' : 'bad'}">
      <span class="ck">${c.ok ? '✓' : '✗'}</span>
      <span class="cn">${c.name}</span>
      <span class="cv">${c.value}</span>
      <span class="cm">${c.msg}</span>
    </div>`).join('');
  box.innerHTML = head + items;
}

function renderLog() {
  const box = $('calibLog');
  if (!box) return;
  if (!calib.log.length) {
    box.innerHTML = '<div class="hint">尚无固件回传</div>';
    return;
  }
  box.innerHTML = calib.log.slice(-80).map((l) =>
    `<div class="calib-log-line">${l}</div>`).join('');
  box.scrollTop = box.scrollHeight;
}

export function activate() {
  if (calib.activated) return;
  calib.activated = true;

  bus.addEventListener('calib-wizard', (e) => {
    calib.steps = (e.detail || {}).steps || [];
    renderSteps();
  });

  bus.addEventListener('calib-state', (e) => {
    const d = e.detail || {};
    calib.data = d.data || {};
    calib.assess = d.assess || null;
    renderData();
    renderAssess();
  });

  bus.addEventListener('calib-assess', (e) => {
    const d = e.detail || {};
    calib.data = d.data || calib.data;
    calib.assess = d.assess || null;
    renderData();
    renderAssess();
  });

  bus.addEventListener('calib-progress', (e) => {
    const d = e.detail || {};
    if (d.phase === 'sent') {
      calib.log.push(`» 发送 ${d.cmd}`);
      renderLog();
    }
  });

  bus.addEventListener('dbg-out', (e) => {
    const line = (e.detail || {}).line;
    if (!line) return;
    // 只把与磁标定相关的行收进向导日志，避免刷屏
    if (/magcal|samples|residualRms|eigRatio|coverageRatio|fieldMag|DBG\]\s+\[mag/i.test(line)) {
      calib.log.push(line);
      renderLog();
    }
  });

  const reset = $('calibReset');
  if (reset) {
    reset.addEventListener('click', () => {
      calib.data = {};
      calib.assess = null;
      calib.log = [];
      send({ cmd: 'calib_reset' });
      renderData(); renderAssess(); renderLog();
    });
  }
  const re = $('calibReassess');
  if (re) re.addEventListener('click', () => send({ cmd: 'calib_assess' }));

  send({ cmd: 'calib_wizard' });
  renderData();
  renderAssess();
  renderLog();
}
