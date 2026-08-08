// ============================================================
//  params.mjs — 参数页：AnoCom 0xE0/0xE1 在线读写（分组表格）
//  名称/类型来自固件 E2 信息帧（自描述）；分组/单位/范围来自本地元数据
// ============================================================
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

export function activate() {
  if (initialized) return;
  initialized = true;

  $('pmReadAll').addEventListener('click', readAll);
  $('pmRestore').addEventListener('click', () => {
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
  send({ cmd: 'param_read_all' });
  // 后端全量读取：前端等待 3s 后渲染（值逐帧到达）
  setTimeout(() => {
    params = Object.entries(state.paramValues || {}).map(([id, value]) => ({
      id: Number(id), name: (state.paramMeta || {})[id]?.name || `id${id}`,
      type: (state.paramMeta || {})[id]?.type || 'float', value,
    }));
    // 后端 param_value 事件也会填充；未收到 E2 名称时用本地名称补全
    render();
    $('pmInfo').textContent = `已显示 ${params.length} 个参数（名称自 E2 信息帧）`;
  }, 3500);
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
        <td class="p-name">${p.id}</td>
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
      const [min, max] = rangeOf(inp.closest('tr').querySelector('.p-name').textContent || '');
      if (val < min || val > max) {
        flashAlarm('warn', `参数 ${id} 超出范围 [${min}, ${max}]，未发送`);
        return;
      }
      const p = params.find(x => x.id === id);
      if (p && p.type === 'uint8') {
        send({ cmd: 'param_write', id, value: val !== 0 ? 1 : 0 });
      } else {
        send({ cmd: 'param_write', id, value: val });
      }
      inp.classList.add('p-write-pending');
    });
  });
}
