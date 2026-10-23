/**
 * @file    scope.mjs
 * @module  仿真 browser/示波器
 * @brief   p/q/r 角速度波形示波器（环形缓冲 + Canvas 绘制 + 面板拖拽）
 * @details 输入：sim.S.omega 与 sasMode=3 时的滑块目标角速度；
 *          处理：~60 Hz 采样进 480 点环形缓冲 → ~30 fps 绘制实线（实测）与虚线（目标）；
 *          输出：Canvas 像素与面板位置。
 *          不变量：缓冲窗口 8 s 与 ±180°/s 量程固定，超量程波形被裁边而不缩放。
 * @theory  N/A（显示层）；目标角速度语义见 @ref CTL-001 §4.1（mode 3）
 * @frame   机体角速度 p/q/r（绕 x/y/z_b）
 * @units   内部 rad/s 与 rad，绘制时转 °/s
 * @note 采样与绘制各自节流，是为了让 480 点缓冲恰好铺满 8 s 窗口，不受显示器刷新率影响
 * @see     main.js（b-scope 按钮与 update）、control.mjs（目标角速度来源）
 */

const DEG = Math.PI / 180;
const COLORS = { p: '#fb7185', q: '#34d399', r: '#818cf8' };   // 与图例一致
const WINDOW_S = 8;        // 时间窗口（秒）
const N = 480;             // 环形缓冲点数（≈60Hz 采样）
const SCALE = Math.PI;     // 纵轴量程 ±π rad/s（±180°/s）

// @note 目标角速度只在 sasMode=3 有定义，其余模式写 NaN → 虚线断点（不画假目标）
// @note 面板拖拽把 left/top 固定为像素值，避免容器缩放时定位漂移
/**
 * @brief 创建示波器实例并绑定拖拽/可见性
 * @param {Object} sim 仿真状态（只读 S.omega/S.sasMode/S.dw/dt/df）
 * @returns {{setVisible: Function, update: Function, visible: boolean}}
 */
export function createScope(sim) {
  const { S } = sim;
  const panel = document.getElementById('scope');
  const canvas = document.getElementById('scopeCanvas');
  const ctx = canvas.getContext('2d');

  // 高分屏适配
  const dpr = Math.min(devicePixelRatio || 1, 2);
  const W = 360, H = 150;
  canvas.width = W * dpr; canvas.height = H * dpr;
  canvas.style.width = W + 'px'; canvas.style.height = H + 'px';
  ctx.scale(dpr, dpr);

  const buf = {
    p: new Float32Array(N), q: new Float32Array(N), r: new Float32Array(N),
    pr: new Float32Array(N), qr: new Float32Array(N), rr: new Float32Array(N),
  };
  let head = 0;            // 下一个写入位置
  let lastSample = 0;
  let lastDraw = 0;
  let visible = false;

  // ---- 标题栏拖拽：抓住 #scopeHead 可将面板拖到任意位置 ----
  const headBar = document.getElementById('scopeHead');
  let drag = null;
  headBar.addEventListener('pointerdown', (e) => {
    const rect = panel.getBoundingClientRect();
    drag = { dx: e.clientX - rect.left, dy: e.clientY - rect.top };
    panel.style.left = rect.left + 'px';
    panel.style.top = rect.top + 'px';
    panel.style.bottom = 'auto';
    headBar.setPointerCapture(e.pointerId);
  });
  headBar.addEventListener('pointermove', (e) => {
    if (!drag) return;
    const x = Math.max(0, Math.min(e.clientX - drag.dx, innerWidth - panel.offsetWidth));
    const y = Math.max(0, Math.min(e.clientY - drag.dy, innerHeight - panel.offsetHeight));
    panel.style.left = x + 'px';
    panel.style.top = y + 'px';
  });
  const endDrag = () => { drag = null; };
  headBar.addEventListener('pointerup', endDrag);
  headBar.addEventListener('pointercancel', endDrag);

  function setVisible(v) {
    visible = v;
    panel.classList.toggle('show', v);
  }

  function pushSample() {
    buf.p[head] = S.omega.x; buf.q[head] = S.omega.y; buf.r[head] = S.omega.z;
    buf.pr[head] = S.sasMode === 3 ? S.dw : NaN;   // rate 模式下滑块指令即 ω_ref
    buf.qr[head] = S.sasMode === 3 ? S.dt : NaN;
    buf.rr[head] = S.sasMode === 3 ? S.df : NaN;
    head = (head + 1) % N;
  }

  function trace(arr, color, dashed) {
    ctx.strokeStyle = color;
    ctx.globalAlpha = dashed ? 0.55 : 1;
    ctx.setLineDash(dashed ? [5, 4] : []);
    ctx.lineWidth = dashed ? 1 : 1.6;
    ctx.beginPath();
    let started = false;
    for (let i = 0; i < N; i++) {
      const v = arr[(head + i) % N];
      if (Number.isNaN(v)) { started = false; continue; }
      const x = (i / (N - 1)) * W;
      const y = H / 2 - (v / SCALE) * (H / 2 - 8);
      if (!started) { ctx.moveTo(x, y); started = true; }
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.globalAlpha = 1;
  }

  function draw() {
    ctx.clearRect(0, 0, W, H);
    // 网格：±90°/s 虚线 + 零线
    ctx.strokeStyle = 'rgba(148,163,184,.25)';
    ctx.lineWidth = 1;
    for (const frac of [0.25, 0.75]) {
      ctx.beginPath(); ctx.moveTo(0, H * frac); ctx.lineTo(W, H * frac); ctx.stroke();
    }
    ctx.strokeStyle = 'rgba(203,213,235,.5)';
    ctx.beginPath(); ctx.moveTo(0, H / 2); ctx.lineTo(W, H / 2); ctx.stroke();
    // 量程标注
    ctx.fillStyle = 'rgba(157,176,204,.9)';
    ctx.font = '9px Consolas, monospace';
    ctx.fillText('+180°/s', 4, 11);
    ctx.fillText('-180°/s', 4, H - 4);
    // 时间标注
    ctx.fillText(`${WINDOW_S}s`, W - 20, H - 4);
    // 目标值虚线（仅角速度闭环模式有数据）
    trace(buf.pr, COLORS.p, true); trace(buf.qr, COLORS.q, true); trace(buf.rr, COLORS.r, true);
    // 实际值实线
    trace(buf.p, COLORS.p, false); trace(buf.q, COLORS.q, false); trace(buf.r, COLORS.r, false);
    // 当前值读数
    ctx.font = '10px Consolas, monospace';
    const cur = [S.omega.x, S.omega.y, S.omega.z];
    const keys = ['p', 'q', 'r'];
    for (let i = 0; i < 3; i++) {
      ctx.fillStyle = COLORS[keys[i]];
      ctx.fillText(`${keys[i]} ${(cur[i] / DEG).toFixed(0).padStart(4)}`, 44 + i * 62, 11);
    }
  }

  function update(now) {
    if (!visible) return;
    // 采样节流 ~60Hz：保证 480 点缓冲在 8s 时间窗口内均匀填满，不受屏幕刷新率影响
    if (now - lastSample > 16) { pushSample(); lastSample = now; }
    if (now - lastDraw > 33) { draw(); lastDraw = now; }   // 绘制节流 ~30fps
  }

  return { setVisible, update, get visible() { return visible; } };
}
