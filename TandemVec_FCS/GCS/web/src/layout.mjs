/**
 * @file    layout.mjs
 * @module  GCS/前端 仪表盘布局管理
 * @brief   面板显隐 + 顺序调整 + 预设 + 本地持久化
 * @details 为什么用「按钮上移/下移 + 勾选显隐」而不是拖拽：
 *          ① 现场常用平板/触屏，拖拽在触屏上容易误触，而且拖拽到滚动容器
 *             边缘时会和页面滚动打架；
 *          ② 只有六七个面板时，上移/下移的点击次数比精确拖拽更少也更准；
 *          ③ 拖拽需要处理 pointer 事件、吸附、占位，代码量是按钮的十倍，
 *             收益不匹配。
 *
 *          持久化用 localStorage 而不是服务端：布局是**显示偏好**，
 *          换一台电脑本该是另一套；放服务端反而会让"我这台机器喜欢的样子"
 *          在别人的机器上生效。也不进 PWA 的 SW 缓存，避免旧布局卡住新面板。
 *
 *          未知面板处理：如果 localStorage 里的 id 在当前面板表里找不到
 *          （旧版本存的、或面板改名了），直接忽略而不是报错——否则升级后
 *          整个仪表盘会空白。
 * @theory  N/A
 * @frame   N/A
 * @units   N/A
 * @note    顺序通过 CSS `order` 实现，不改 DOM 结构：这样各面板模块的
 *          querySelector 与事件绑定都不受影响。
 * @ref     GCS-001 §上位机、web/src/dashboard.mjs
 */

import { bus } from './main.mjs';

const $ = (id) => document.getElementById(id);
const STORAGE_KEY = 'unignc.gcs.layout.v1';

// 面板表：id 与 index.html 里的 class 对应，改 index.html 时要同步这里
export const PANELS = [
  { id: 'ai', sel: '.ai-panel', name: '人工地平线' },
  { id: 'hud', sel: '.hud-panel', name: '飞行数据 HUD' },
  { id: 'gps', sel: '.gps-panel', name: 'GNSS' },
  { id: 'rc', sel: '.rc-panel', name: '遥控通道' },
  { id: 'tvc', sel: '.tvc-panel', name: '控制输出' },
  { id: 'act', sel: '.act-panel', name: '执行器' },
  { id: 'vars', sel: '.vars-panel', name: '变量监视' },
];

// 预设：飞行看姿态与数据，调试看执行器与控制量
export const PRESETS = {
  flight: { order: ['ai', 'hud', 'gps', 'rc', 'tvc', 'act', 'vars'], hidden: ['rc', 'tvc', 'act'] },
  debug: { order: ['tvc', 'act', 'rc', 'hud', 'ai', 'gps', 'vars'], hidden: [] },
  full: { order: ['ai', 'hud', 'gps', 'rc', 'tvc', 'act', 'vars'], hidden: [] },
  minimal: { order: ['ai', 'hud', 'gps', 'rc', 'tvc', 'act', 'vars'], hidden: ['rc', 'tvc', 'act', 'gps'] },
};

const layout = {
  order: PRESETS.full.order.slice(),
  hidden: [],
  activated: false,
  grid: null,
};

function sanitize(state) {
  // 只保留当前存在的面板；缺失的补到末尾，多余的丢弃
  const known = new Set(PANELS.map((p) => p.id));
  const order = (state.order || []).filter((id) => known.has(id));
  for (const p of PANELS) {
    if (!order.includes(p.id)) order.push(p.id);
  }
  const hidden = (state.hidden || []).filter((id) => known.has(id));
  return { order, hidden };
}

export function load() {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return;
    const parsed = JSON.parse(raw);
    const clean = sanitize(parsed);
    layout.order = clean.order;
    layout.hidden = clean.hidden;
  } catch (e) {
    // 存的东西坏了就当没存过——不能因为一条坏数据让仪表盘起不来
    layout.order = PRESETS.full.order.slice();
    layout.hidden = [];
  }
}

export function save() {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify({
      order: layout.order, hidden: layout.hidden,
    }));
  } catch (e) { /* 隐私模式/配额满时静默 */ }
}

/** 把当前 layout 应用到 DOM */
export function apply() {
  for (const p of PANELS) {
    const el = document.querySelector(p.sel);
    if (!el) continue;
    const idx = layout.order.indexOf(p.id);
    el.style.order = idx >= 0 ? String(idx) : '99';
    el.style.display = layout.hidden.includes(p.id) ? 'none' : '';
  }
}

function move(id, dir) {
  const i = layout.order.indexOf(id);
  if (i < 0) return;
  const j = i + dir;
  if (j < 0 || j >= layout.order.length) return;
  [layout.order[i], layout.order[j]] = [layout.order[j], layout.order[i]];
  save();
  apply();
  renderEditor();
}

function toggle(id) {
  const i = layout.hidden.indexOf(id);
  if (i >= 0) layout.hidden.splice(i, 1);
  else layout.hidden.push(id);
  save();
  apply();
  renderEditor();
}

function renderEditor() {
  const box = $('layoutList');
  if (!box) return;
  box.innerHTML = layout.order.map((id, i) => {
    const p = PANELS.find((x) => x.id === id);
    if (!p) return '';
    const hidden = layout.hidden.includes(id);
    return `<div class="layout-item${hidden ? ' off' : ''}" data-id="${id}">
      <label class="layout-chk">
        <input type="checkbox" data-show="${id}" ${hidden ? '' : 'checked'}>
        <span>${p.name}</span>
      </label>
      <span class="layout-btns">
        <button class="btn small" data-move="${id}" data-dir="-1" ${i === 0 ? 'disabled' : ''}>↑</button>
        <button class="btn small" data-move="${id}" data-dir="1"
          ${i === layout.order.length - 1 ? 'disabled' : ''}>↓</button>
      </span>
    </div>`;
  }).join('');

  box.querySelectorAll('input[data-show]').forEach((cb) => {
    cb.addEventListener('change', () => toggle(cb.dataset.show));
  });
  box.querySelectorAll('button[data-move]').forEach((b) => {
    b.addEventListener('click', () => move(b.dataset.move, Number(b.dataset.dir)));
  });
}

function applyPreset(name) {
  const p = PRESETS[name];
  if (!p) return;
  const clean = sanitize({ order: p.order, hidden: p.hidden });
  layout.order = clean.order;
  layout.hidden = clean.hidden;
  save();
  apply();
  renderEditor();
}

export function activate() {
  if (layout.activated) return;
  layout.activated = true;

  load();
  apply();

  const btn = $('layoutBtn');
  const ed = $('layoutEditor');
  if (btn && ed) {
    btn.addEventListener('click', () => {
      const open = ed.classList.toggle('show');
      btn.classList.toggle('active', open);
      if (open) renderEditor();
    });
  }

  document.querySelectorAll('[data-preset]').forEach((b) => {
    b.addEventListener('click', () => applyPreset(b.dataset.preset));
  });

  const reset = $('layoutReset');
  if (reset) {
    reset.addEventListener('click', () => applyPreset('full'));
  }

  renderEditor();
}

/** 供测试/调试用：导出当前布局 */
export function currentLayout() {
  return { order: layout.order.slice(), hidden: layout.hidden.slice() };
}
