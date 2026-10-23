/**
 * @file    hud.mjs
 * @module  analysis/py-web-lab 渲染（HUD 与遥测）
 * @brief   生成通道曲线与数值仪表，显示力/力矩/姿态/转速等遥测。
 * @details 输入：dyn/aero/S 遥测字段；处理：按 CH 表映射量程与双极性；
 *          输出：DOM 更新（力 N、力矩 N·m）。
 * @theory  N/A（显示层；量的物理含义见 @ref MOD-001 §4）
 * @frame   显示为机体系分量（Fx/Fy/Fz、Mx/My/Mz）
 * @units   N、N·m、rad/s、rad；CH.max 为绘图量程上限
 * @note    Mx 量程 1.6 N·m，远小于 My/Mz 的 6 N·m——滚转权限由反扭矩差驱动，
 *          量级天然小一个数量级（@ref MOD-001 §3.2、§11）。
 * @see     main.mjs、scope.mjs、@ref MOD-001
 */

const CH = [
  { id: 'Fx', name: 'F_x 纵向力',  color: '#22d3ee', max: 40,  unit: 'N',   bipolar: false },
  { id: 'Fy', name: 'F_y 侧向力',  color: '#22d3ee', max: 10,  unit: 'N',   bipolar: true  },
  { id: 'Fz', name: 'F_z 垂向力',  color: '#22d3ee', max: 10,  unit: 'N',   bipolar: true  },
  { id: 'Mx', name: 'M_x 推进滚转矩', color: '#fb7185', max: 1.6, unit: 'N·m', bipolar: true  },
  { id: 'My', name: 'M_y 推进俯仰矩', color: '#34d399', max: 6,   unit: 'N·m', bipolar: true  },
  { id: 'Mz', name: 'M_z 推进偏航矩', color: '#818cf8', max: 6,   unit: 'N·m', bipolar: true  },
];

/**
 * 在给定容器内构建 HUD（通道数值 + 力/力矩曲线），返回逐帧更新接口。
 * @param {HTMLElement} box 承载 HUD 的 DOM 容器
 * @returns {Object} { update(sim) }：按遥测刷新数值与曲线（N / N·m / rad）
 */
export function createHud(box) {
  const $ = id => document.getElementById(id);
  const meterEls = {};
  for (const c of CH) {
    const div = document.createElement('div');
    div.className = 'meter';
    div.innerHTML = `<div class="meter-head"><span class="name" style="color:${c.color}">${c.name}</span>
      <span class="num" id="num-${c.id}">0.00</span></div>
      <div class="bar">${c.bipolar ? '<div class="zero"></div>' : ''}<div class="fill" id="fill-${c.id}" style="background:${c.color};box-shadow:0 0 8px ${c.color}"></div></div>`;
    box.appendChild(div);
    meterEls[c.id] = { fill: div.querySelector(`#fill-${c.id}`), num: div.querySelector(`#num-${c.id}`), cfg: c };
  }

  function setMeter(id, v) {
    const { fill, num, cfg } = meterEls[id];
    const r = Math.min(Math.abs(v) / cfg.max, 1);
    if (cfg.bipolar) {
      fill.style.left = v >= 0 ? '50%' : `${50 - r * 50}%`;
      fill.style.width = `${r * 50}%`;
    } else {
      fill.style.left = '0'; fill.style.width = `${r * 100}%`;
    }
    num.textContent = v.toFixed(2);
  }

  function setText(id, text, optStyle) {
    const el = $(id);
    if (!el) return;
    el.textContent = text;
    if (optStyle) Object.assign(el.style, optStyle);
  }

  // t = core/telemetry.getTelemetry(sim) 快照
  function sync(t) {
    const dyn = t.forces, aero = t.aero, F = t.flight;
    setMeter('Fx', dyn.Fx); setMeter('Fy', dyn.Fy); setMeter('Fz', dyn.Fz);
    setMeter('Mx', dyn.Mx); setMeter('My', dyn.My); setMeter('Mz', dyn.Mz);
    setText('d-tf', dyn.Tf.toFixed(1) + ' N');
    setText('d-tt', dyn.Tt.toFixed(1) + ' N');
    setText('d-qf', dyn.Qf.toFixed(2) + ' N·m');
    setText('d-qt', dyn.Qt.toFixed(2) + ' N·m');
    const dq = dyn.Qt - dyn.Qf;
    setText('d-dq', Math.abs(dq) < 0.02 ? '对消 ✓' : dq.toFixed(2) + ' N·m',
      { color: Math.abs(dq) < 0.02 ? '#34d399' : '#fb7185' });
    setText('d-wf', Math.round(t.rotors.wf) + ' rad/s');
    setText('d-wt', Math.round(t.rotors.wt) + ' rad/s');
    setText('d-v', aero.V.toFixed(1) + ' m/s');
    setText('d-aoa', (aero.alpha * 57.2958).toFixed(1) + '°');
    setText('d-h', (-F.pos.z).toFixed(1) + ' m');
    setText('d-att', `${(F.euler.x*57.3).toFixed(0)} / ${(F.euler.y*57.3).toFixed(0)} / ${(F.euler.z*57.3).toFixed(0)}°`);
    setText('d-amy', t.flags.aero ? aero.My.toFixed(2) + ' N·m' : '已忽略',
      t.flags.aero ? null : { color: '#f59e0b' });
    setText('e-mx', dyn.Mx.toFixed(2));
    setText('e-my', dyn.My.toFixed(2));
    setText('e-mz', dyn.Mz.toFixed(2));
  }

  return { sync };
}
