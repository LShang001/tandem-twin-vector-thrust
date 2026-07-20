// ============================================================
//  演示序列：俯仰 / 偏航 / 滚转 / 综合
// ============================================================
import { resetFlightState } from '../core/state.mjs';

const D2R = Math.PI / 180;

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
    demo = { name, t0: S.time };
    resetFlightState(sim, P);
    S.dt = 0; S.df = 0; S.dw = 0;
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
