/**
 * @file    params.mjs
 * @module  GCS/前端参数页
 * @brief   参数分组表格展示与主动 0xE0/0xE1 在线读写（写后读回校验）
 * @details 数据来源：WS 消息 param_count/param_info/param_value/param_written；
 *          名称/类型来自固件 E2 信息帧（自描述），分组/单位/范围来自本地元数据
 *          （LOCAL_GROUPS/FIELD_RANGE）；处理：用户编辑 → send({cmd:'param_write'})
 *          → 写后读回比对，超差告警；输出：表格与写入状态提示。
 * @theory  N/A
 * @frame   N/A
 * @units   参数量纲随环：姿态外环 deg/s、角速率内环 deg/s²、垂直/水平串级 m/s 与 m/s²
 * @ref     PROTOCOL-001 §4（0xE0-0xE3 参数链路）、GCS-001 §2
 */
import { bus, send, state, flashAlarm } from './main.mjs';

const $ = (id) => document.getElementById(id);

// 本地显示元数据（与固件注册表同名；未命中走"其他"组）
const LOCAL_GROUPS = [
  ['姿态外环（deg 域）', ['att_roll', 'att_pitch', 'att_yaw']],
  ['角速率内环', ['rate_roll', 'rate_pitch', 'rate_yaw']],
  ['垂直串级', ['alt_pos', 'alt_vel']],
  ['水平位置环', ['pos_n', 'pos_e']],
  ['水平速度环', ['vel_n', 'vel_e']],
  ['角速率滤波', ['speed_filter_alpha']],
  ['外环输出滤波', ['angle_out_filter_alpha']],
  ['内环输出滤波', ['output_filter_alpha']],
];
const FIELD_LABEL = {
  kp: 'Kp', ki: 'Ki', kd: 'Kd', out_min: '输出下限', out_max: '输出上限',
  int_limit: '积分钳位', threshold: '分离阈值', filter_alpha: '滤波α', enabled: '启用',
};
const FIELD_RANGE = {
  kp: [0, 50, 0.01], ki: [0, 1, 0.0001], kd: [0, 50, 0.01],
  out_min: [-1000, 0, 0.1], out_max: [0, 1000, 0.1],
  int_limit: [0, 1000, 1], threshold: [0, 1000, 0.1], filter_alpha: [0, 1, 0.01],
  enabled: [0, 1, 1],
};

let params = [];      // [{id, name, type, value, pending}]
let initialized = false;
let pendingWrites = new Set();

/** @brief 挂载参数页事件并触发首次读取
 *  @details 绑定读全部/恢复默认/写入按钮，订阅 param_* 消息
 *  @note    幂等：initialized 后直接返回
 *  @units   参数量纲随环（deg/s、deg/s²、m/s、m/s²）
 *  @ref     PROTOCOL-001 §4
 */
export function activate() {
  if (initialized) return;
  initialized = true;

  $('pmReadAll').addEventListener('click', readAll);
  $('pmRestore').addEventListener('click', () => {
    if (state.protocol === 'mavlink') {
      flashAlarm('warn', 'MAVLink 模式未提供恢复出厂默认；请用 AnoCom 或参数快照');
      return;
    }
    if (!confirm('确认将所有参数恢复为出厂默认值？')) return;
    send({ cmd: 'param_restore' });
    setTimeout(readAll, 800);
    flashAlarm('warn', '已发送恢复默认命令（RAM 生效，重启回默认）');
  });

  bus.addEventListener('param_value', (e) => {
    const m = e.detail;
    const p = params.find(x => x.id === m.id);
    if (p) { p.value = m.value; render(); }
  });
  bus.addEventListener('param_written', (e) => {
    const p = params.find(x => x.id === e.detail.id);
    if (p) { p.pending = false; }
    pendingWrites.delete(e.detail.id);
    $('pmInfo').textContent = `参数 ${e.detail.id} 写入成功（校验帧确认）`;
    render();
  });
  bus.addEventListener('param_written_pending', (e) => {
    pendingWrites.add(e.detail.id);
    const p = params.find(x => x.id === e.detail.id);
    if (p) p.pending = true;
    render();
  });
  bus.addEventListener('log', (e) => {
    if (e.detail.level === 'error') { flashAlarm('err', e.detail.msg); $('pmInfo').textContent = '⚠ ' + e.detail.msg; }
  });
}

async function readAll() {
  if (!state.connected) { flashAlarm('warn', '请先连接飞控'); return; }
  $('pmInfo').textContent = '正在读取全部参数…';
  // MAVLink 模式走 MAV_PARAM_REQUEST_LIST；AnoCom 走 0xE0 流程
  send({ cmd: state.protocol === 'mavlink' ? 'mav_param_list' : 'param_read_all' });
  // 后端全量读取：前端等待 3.5s 后渲染（值逐帧到达）。
  // MAVLink 偶发丢一帧时，若少于 param_count 自动补读一次。
  let retried = false;
  const settle = () => {
    params = Object.entries(state.paramValues || {}).map(([id, value]) => ({
      id: Number(id), name: (state.paramMeta || {})[id]?.name || `id${id}`,
      type: (state.paramMeta || {})[id]?.type || 'float', value,
    }));
    render();
    const total = Number(state.paramCount || 0);
    if (state.protocol === 'mavlink' && total > 0 && params.length < total && !retried) {
      retried = true;
      send({ cmd: 'mav_param_list' });
      setTimeout(settle, 1800);
      $('pmInfo').textContent = `已收 ${params.length}/${total}，补读一次…`;
      return;
    }
    $('pmInfo').textContent = state.protocol === 'mavlink'
      ? `已显示 ${params.length}/${total || '?'} 个参数（MAVLink PARAM_VALUE）`
      : `已显示 ${params.length} 个参数（名称自 E2 信息帧）`;
  };
  setTimeout(settle, 3500);
}

function groupOf(name) {
  if (!name) return '其他';
  const key = name.split('.')[0] || name.replace(/\[\d+\]$/, '');
  for (const [g, keys] of LOCAL_GROUPS) {
    if (keys.includes(key)) return g;
  }
  return '其他';
}

function rangeOf(name) {
  const field = name.includes('.') ? name.split('.')[1] : 'filter_alpha';
  if (field === 'enabled') return [0, 1, 1];
  const r = FIELD_RANGE[field];
  return r || [0, 100, 0.1];
}

function labelOf(name) {
  const field = name.includes('.') ? name.split('.')[1] : null;
  return field ? FIELD_LABEL[field] || field : name;
}

function render() {
  const wrap = $('pmGroups');
  // 按组聚合
  const groups = new Map();
  for (const p of params) {
    const g = groupOf(p.name);
    if (!groups.has(g)) groups.set(g, []);
    groups.get(g).push(p);
  }
  let html = '';
  for (const [gname, list] of groups) {
    html += `<div class="pm-group"><div class="pm-group-title">${gname} · ${list.length}</div>`;
    html += `<table class="pm-table"><thead><tr><th>参数</th><th>名称</th><th>值</th><th>状态</th></tr></thead><tbody>`;
    for (const p of list) {
      const [min, max, step] = rangeOf(p.name);
      const isEnabled = p.name.endsWith('.enabled');
      const readonly = isEnabled;
      html += `<tr>
        <td class="p-name" data-name="${p.name}">${p.id}</td>
        <td>${p.name}<div class="p-desc">${labelOf(p.name)}</div></td>
        <td><input type="number" data-id="${p.id}" value="${p.value ?? ''}"
          ${readonly ? 'disabled' : ''} min="${min}" max="${max}" step="${step}"></td>
        <td class="p-write${p.pending ? ' pending' : ''}">${p.pending ? '写入中…' : (isEnabled ? '只读' : '')}</td>
      </tr>`;
    }
    html += '</tbody></table></div>';
  }
  wrap.innerHTML = html || '<div class="hint" style="padding:16px">尚无参数——点击「读取全部」</div>';

  // 输入变化 → 写入
  wrap.querySelectorAll('input[type=number]').forEach((inp) => {
    inp.addEventListener('change', () => {
      const id = Number(inp.dataset.id);
      const val = parseFloat(inp.value);
      if (Number.isNaN(val)) return;
      // ★ 2026-11-06 审计修复：.p-name 单元格装的是参数 ID，取它会让 rangeOf 落到
      //   filter_alpha 分支（[0,1]），导致所有 |值|>1 的合法写入被误判越界。改取 data-name。
      const [min, max] = rangeOf(inp.closest('tr').querySelector('.p-name').dataset.name || '');
      if (val < min || val > max) {
        flashAlarm('warn', `参数 ${id} 超出范围 [${min}, ${max}]，未发送`);
        return;
      }
      const p = params.find(x => x.id === id);
      const writeValue = (p && p.type === 'uint8') ? (val !== 0 ? 1 : 0) : val;
      if (state.protocol === 'mavlink') {
        send({ cmd: 'mav_param_set', name: p?.name || '', value: writeValue });
      } else if (p && p.type === 'uint8') {
        send({ cmd: 'param_write', id, value: writeValue });
      } else {
        send({ cmd: 'param_write', id, value: writeValue });
      }
      inp.classList.add('p-write-pending');
    });
  });
}
