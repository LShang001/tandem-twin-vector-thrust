/**
 * @file    hud.mjs
 * @module  仿真 browser/HUD
 * @brief   六维力/力矩条形表与数字遥测的 DOM 渲染
 * @details 输入：core/telemetry 的遥测快照；处理：按通道满量程归一化条宽、
 *          更新数字文本与告警颜色；输出：DOM 内容。
 *          不变量：HUD 只读快照，不写 sim；量程 CH[].max 是显示契约而非物理上限。
 * @theory  N/A（显示层）；欧拉角直接显示 telemetry 的显示角（theta 取负约定）
 *          @ref MOD-002 §4（HUD 已把负号纳入操作者习惯）
 * @frame   显示机体系力/力矩；姿态角为项目显示约定（非标准 3-2-1）
 * @units   力 N、力矩 N·m、转速 rad/s、空速 m/s、高度 m、姿态 °、时间 s
 * @note 姿态显示直接用 57.3 系数把 rad 转 °——不要在这里"修正"符号，
 *       否则与 SAS 的显示角语义分叉（红线：theta=−asin(R13) 全链一致）
 * @see     core/telemetry.mjs（数据来源）、main.js（每 3 帧同步一次）
 */

const CH = [
  { id: 'Fx', name: 'F_x 纵向力',  color: '#22d3ee', max: 40,  unit: 'N',   bipolar: false },
  { id: 'Fy', name: 'F_y 侧向力',  color: '#22d3ee', max: 10,  unit: 'N',   bipolar: true  },
  { id: 'Fz', name: 'F_z 垂向力',  color: '#22d3ee', max: 10,  unit: 'N',   bipolar: true  },
  { id: 'Mx', name: 'M_x 推进滚转矩', color: '#fb7185', max: 1.6, unit: 'N·m', bipolar: true  },
  { id: 'My', name: 'M_y 推进俯仰矩', color: '#34d399', max: 6,   unit: 'N·m', bipolar: true  },
  { id: 'Mz', name: 'M_z 推进偏航矩', color: '#818cf8', max: 6,   unit: 'N·m', bipolar: true  },
];

// @note 双向条（bipolar）从中线向两侧生长：正负力矩方向是操作者判读的主要信息
// @note 差速对消判定阈值 0.02 N·m 是显示级阈值（零摆角附近的反扭残差），不用于控制
/**
 * @brief 在容器内生成六条力/力矩仪表并返回同步函数
 * @param {HTMLElement} box 仪表容器（#meters）
 * @returns {{sync: Function}} sync(t) 接收 core/telemetry 的遥测快照
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
