// ============================================================
//  console.mjs — DBG 控制台 + 数据记录 / 回放
// ============================================================
import { bus, send, state, flashAlarm } from './main.mjs';

const $ = (id) => document.getElementById(id);

const HISTORY_MAX = 2000;
let initialized = false;

export function activate() {
  if (initialized) return;
  initialized = true;

  $('conEnter').addEventListener('click', () => send({ cmd: 'dbg_enter' }));
  $('conExit').addEventListener('click', () => send({ cmd: 'dbg_exit' }));
  $('conSend').addEventListener('click', sendLine);
  $('conLine').addEventListener('keydown', (e) => { if (e.key === 'Enter') sendLine(); });

  $('recStart').addEventListener('click', () => send({ cmd: 'record_start' }));
  $('recStop').addEventListener('click', () => send({ cmd: 'record_stop' }));
  $('recReplay').addEventListener('click', () => {
    const fn = $('recList').value;
    if (!fn) { flashAlarm('warn', '请先选择要回放的文件'); return; }
    send({ cmd: 'replay_start', file: fn });
  });
  $('recStopReplay').addEventListener('click', () => send({ cmd: 'replay_stop' }));

  bus.addEventListener('dbg-out', (e) => pushLine(e.detail));
  bus.addEventListener('dbg-mode', (e) => {
    pushLine(e.detail.on ? '>>> 已进入调试模式（遥测暂停）' : '>>> 已退出调试模式（遥测恢复）', 'in');
    $('conEnter').disabled = e.detail.on;
    $('conExit').disabled = !e.detail.on;
  });
  bus.addEventListener('log', (e) => pushLine(`[${e.detail.level || 'log'}] ${e.detail.msg || ''}`, e.detail.level === 'error' ? 'warn' : ''));
  bus.addEventListener('record-state', (e) => {
    $('recInfo').textContent = e.detail.on ? `● 记录中：${e.detail.file}` : '记录已停止';
    $('recStart').disabled = e.detail.on;
  });
  bus.addEventListener('replay-state', (e) => {
    if (!e.detail.on) { $('recInfo').textContent = '回放结束'; $('recStopReplay').disabled = true; return; }
    $('recInfo').textContent = `▶ 回放中：${e.detail.file}`;
    $('recStopReplay').disabled = false;
  });
  bus.addEventListener('recordings', (e) => {
    const sel = $('recList');
    sel.innerHTML = '';
    for (const f of e.detail) sel.add(new Option(`${f.name}（${(f.size / 1024).toFixed(0)}KB）`, f.name));
  });
  bus.addEventListener('conn', (e) => { if (e.detail.status === 'connected') refreshRecordings(); });

  refreshRecordings();
}

function sendLine() {
  const inp = $('conLine');
  const line = inp.value.trim();
  if (!line) return;
  pushLine(`> ${line}`, 'in');
  inp.value = '';
  if (state.mode === 'dbg') {
    send({ cmd: 'dbg_cmd', line });
  } else {
    // 未在调试模式：常见命令自动进入
    if (line.startsWith('flash') || line.startsWith('datalog') || line === 'help' || line.startsWith('ws')) {
      send({ cmd: 'dbg_enter' });
      setTimeout(() => send({ cmd: 'dbg_cmd', line }), 500);
      pushLine('（自动进入调试模式…）', 'in');
    } else {
      pushLine('⚠ 未连接或未进入调试模式（点「进入调试模式」）', 'warn');
    }
  }
}

function pushLine(text, cls = '') {
  const out = $('conOut');
  const div = document.createElement('div');
  div.className = cls;
  div.textContent = text;
  out.appendChild(div);
  // 限制历史长度
  while (out.childElementCount > HISTORY_MAX) out.removeChild(out.firstChild);
  if ($('conAutoscroll').checked) out.scrollTop = out.scrollHeight;
}

function refreshRecordings() {
  send({ cmd: 'list_recordings' });
}
