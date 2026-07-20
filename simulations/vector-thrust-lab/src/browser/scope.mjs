// ============================================================
//  角速度示波器：p/q/r 波形曲线（可开关）
//  实线 = 实际机体角速度；角速度闭环模式下虚线 = 目标角速度 ω_ref
// ============================================================

const DEG = Math.PI / 180;
const COLORS = { p: '#fb7185', q: '#34d399', r: '#818cf8' };   // 与图例一致
const WINDOW_S = 8;        // 时间窗口（秒）
const N = 480;             // 环形缓冲点数（≈60Hz 采样）
const SCALE = Math.PI;     // 纵轴量程 ±π rad/s（±180°/s）

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
    pushSample();
    if (now - lastDraw > 33) { draw(); lastDraw = now; }   // 绘制节流 ~30fps
  }

  return { setVisible, update, get visible() { return visible; } };
}
