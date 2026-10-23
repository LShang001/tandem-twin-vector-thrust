/**
 * @file    demo.mjs
 * @module  仿真 browser/演示序列
 * @brief   自动演示脚本：正弦俯仰/偏航/滚转与综合动作，驱动 UI 滑块
 * @details 输入：仿真时间 t 与演示名（pitch/yaw/roll/cine）；
 *          处理：按单周期 3.2 s 的正弦（或 12 s 分段综合序列）写 S.dt/df/dw/thr，
 *          再调 ui.pushToUI 让滑块跟随；输出：状态指令变化（不直接算力）。
 *          不变量：演示只改指令、不改模式；开始演示前先完整复位（resetSimulationState）。
 * @theory  激励幅值 18°（摆角）/0.28（Δω）在 SAS 线性工作区内 @ref CTL-001 §4
 * @frame   N/A（指令层；dt/df 绕机体 y/z 轴）
 * @units   角度 °（内部转 rad）、差速无量纲
 * @note 演示是"给人看的"，幅值/周期不是识别数据；任何定量结论都不能取自演示
 * @see     main.js（demoStep 每帧调用）、controls-ui.mjs（pushToUI）
 */
import { resetSimulationState } from '../core/state.mjs';

const D2R = Math.PI / 180;

// @note 再次点击同一按钮 = 停止（toggle 语义）；开始前完整复位以保证可重复
// @note demoStep 每帧调用，τ 用 S.time−t0 计算，所以暂停/复位后相位自动归零
/**
 * @brief 创建演示控制器并绑定按钮
 * @param {Object} opts {sim, P, controls, ui}；controls 用 autoRotate 做环绕运镜
 * @returns {{startDemo: Function, stopDemo: Function, demoStep: Function}}
 */
export function createDemo({ sim, P, controls, ui }) {
  const S = sim.S;
  const $ = id => document.getElementById(id);
  const demoBtns = { pitch: $('b-pitch'), yaw: $('b-yaw'), roll: $('b-roll'), cine: $('b-cine') };
  let demo = null;   // 当前演示 {name, t0}

  function stopDemo() {
    demo = null; controls.autoRotate = false;
    for (const k in demoBtns) demoBtns[k].classList.remove('active');
  }

  function startDemo(name) {
    stopDemo();
    resetSimulationState(sim, P);
    demo = { name, t0: S.time };
    demoBtns[name].classList.add('active');
    if (name === 'cine') controls.autoRotate = true;
  }

  for (const k in demoBtns) demoBtns[k].addEventListener('click', () => {
    (demo && demo.name === k) ? stopDemo() : startDemo(k);
  });

  function demoStep(t) {
    if (!demo) return;
    const τ = t - demo.t0, T = 3.2;                  // 单周期 3.2s
    const s = Math.sin(2 * Math.PI * τ / T);
    switch (demo.name) {
      case 'pitch': S.dt = 18 * D2R * s; break;
      case 'yaw':   S.df = 18 * D2R * s; break;
      case 'roll':  S.dw = 0.28 * s; break;
      case 'cine': {                                     // 综合: 油门爬升 → 俯仰 → 偏航 → 滚转
        S.thr = 0.55 + 0.25 * Math.min(τ / 6, 1) * (0.5 + 0.5 * Math.sin(τ * 0.5));
        const seg = τ % 12;
        S.dt = seg < 4 ? 16 * D2R * Math.sin(Math.PI * seg / 2) : 0;
        S.df = (seg >= 4 && seg < 8) ? 16 * D2R * Math.sin(Math.PI * (seg - 4) / 2) : 0;
        S.dw = seg >= 8 ? 0.26 * Math.sin(Math.PI * (seg - 8) / 2) : 0;
        break;
      }
    }
    ui.pushToUI();
  }

  return { startDemo, stopDemo, demoStep };
}
