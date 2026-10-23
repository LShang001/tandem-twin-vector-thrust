/**
 * @file    console.mjs
 * @module  GCS/前端控制台
 * @brief   DBG 命令行终端与遥测记录/回放控制
 * @details 数据来源：WS dbg_out/dbg_mode/record_state/replay_state/recordings；
 *          处理：输入行 → send({cmd:'dbg_cmd'})，输出终端历史（上限 2000 行）；
 *          记录/回放按钮驱动后端 CSV 录制与回放任务。
 *          说明：DBG 与遥测互斥，进入 DBG 后后端停发 telemetry。
 * @theory  N/A
 * @frame   N/A
 * @units   N/A
 * @ref     AGENTS.md 踩坑记录（DBG 调试模式）、GCS-001 §上位机（记录/回放）
 */
import { bus, send, state, flashAlarm } from './main.mjs';

const $ = (id) => document.getElementById(id);

const HISTORY_MAX = 2000;
let initialized = false;

/** @brief 挂载控制台/记录回放页事件
 *  @details 绑定 DBG 输入、记录与回放按钮，订阅 dbg_out/dbg_mode/record_state
 *  @note    幂等：initialized 后直接返回；DBG 与遥测互斥
 *  @units   N/A
 *  @ref     GCS-001 §上位机
 */
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

  // ---- 链路监听（hex）----
  $('hexToggle').addEventListener('click', () => {
    send({ cmd: hexOn ? 'hex_off' : 'hex_on' });
  });
  $('hexClear').addEventListener('click', () => { $('hexOut').textContent = ''; });
  bus.addEventListener('hex-state', (e) => {
    hexOn = !!e.detail.on;
    $('hexToggle').textContent = hexOn ? '停止监听' : '开始监听';
    $('hexToggle').classList.toggle('primary', hexOn);
    if (hexOn) pushHex('--- 监听开始：等待串口字节…（无输出 = 链路无数据） ---');
  });
  bus.addEventListener('rx-hex', (e) => {
    const m = e.detail;
    pushHex(`${m.hex}\n  |${m.ascii}|`);
  });

  refreshRecordings();
}

let hexOn = false;
const HEX_MAX_CHARS = 20000;
function pushHex(text) {
  const out = $('hexOut');
  out.textContent += (out.textContent ? '\n' : '') + text;
  if (out.textContent.length > HEX_MAX_CHARS) {
    out.textContent = out.textContent.slice(-HEX_MAX_CHARS / 2);
  }
  out.scrollTop = out.scrollHeight;
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
