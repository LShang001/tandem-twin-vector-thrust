// ============================================================
//  控制面板 UI：滑块 / 开关 / 复位 / 模态框
// ============================================================
import { resetFlightState } from '../core/state.mjs';

export function createControlsUI({ sim, P, hooks }) {
  const S = sim.S;
  const $ = id => document.getElementById(id);
  const sliders = { thr: $('s-thr'), dt: $('s-dt'), df: $('s-df'), dw: $('s-dw') };

  function syncFromUI() {
    S.thr = sliders.thr.value / 100;
    S.dt  = sliders.dt.value * Math.PI / 180;
    S.df  = sliders.df.value * Math.PI / 180;
    S.dw  = sliders.dw.value / 100 * P.dwUiMax;      // 差速幅值 ±0.55
    $('v-thr').textContent = `${sliders.thr.value}%`;
    $('v-dt').textContent  = `${(+sliders.dt.value).toFixed(1)}°`;
    $('v-df').textContent  = `${(+sliders.df.value).toFixed(1)}°`;
    $('v-dw').textContent  = `${sliders.dw.value}%`;
  }

  function pushToUI() {
    sliders.thr.value = Math.round(S.thr * 100);
    sliders.dt.value  = (S.dt * 180 / Math.PI).toFixed(1);
    sliders.df.value  = (S.df * 180 / Math.PI).toFixed(1);
    sliders.dw.value  = Math.round(S.dw / P.dwUiMax * 100);
    syncFromUI();
  }

  for (const k in sliders) sliders[k].addEventListener('input', () => { hooks.stopDemo(); syncFromUI(); });

  function resetAll() {
    hooks.stopDemo();
    S.thr = P.thrTrim; S.dt = 0; S.df = 0; S.dw = 0;
    resetFlightState(sim, P);
    pushToUI();
  }
  $('b-reset').addEventListener('click', resetAll);

  $('b-sas').addEventListener('click', () => {
    S.sas = !S.sas;
    $('b-sas').classList.toggle('active', S.sas);
    $('b-sas').textContent = S.sas ? '增稳 SAS：开' : '增稳 SAS：关';
  });
  $('b-aero').addEventListener('click', () => {
    S.aero = !S.aero;
    $('b-aero').classList.toggle('active', S.aero);
    $('b-aero').textContent = S.aero ? '气动力：开' : '气动力：忽略';
  });
  $('b-info').addEventListener('click', () => $('modal').classList.add('open'));
  $('modalClose').addEventListener('click', () => $('modal').classList.remove('open'));
  $('modal').addEventListener('click', e => { if (e.target.id === 'modal') $('modal').classList.remove('open'); });

  return { syncFromUI, pushToUI };
}
