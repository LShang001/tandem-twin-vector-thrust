// ============================================================
//  控制面板 UI：滑块 / 开关 / 复位 / 模态框
// ============================================================
import { resetSimulationState } from '../core/state.mjs';

export function createControlsUI({ sim, P, hooks }) {
  const S = sim.S;
  const $ = id => document.getElementById(id);
  const sliders = { thr: $('s-thr'), dt: $('s-dt'), df: $('s-df'), dw: $('s-dw') };

  function syncFromUI() {
    const rateMode = S.sasMode === 3;
    S.thr = sliders.thr.value / 100;
    if (rateMode) {
      // 角速度闭环模式：滑块 → 目标角速度 (rad/s)
      S.dt = (sliders.dt.value / 25) * P.rateQMax;           // ±π/2 rad/s
      S.df = (sliders.df.value / 25) * P.rateQMax;
      S.dw = (sliders.dw.value / 30) * P.ratePMax;           // ±π rad/s
    } else {
      // 阻尼/全SAS/关闭模式：滑块 → 摆角指令 / 差速指令
      S.dt = sliders.dt.value * Math.PI / 180;               // °→rad
      S.df = sliders.df.value * Math.PI / 180;
      S.dw = sliders.dw.value / 100 * P.dwUiMax;             // 差速幅值
    }
    $('v-thr').textContent = `${sliders.thr.value}%`;
    if (rateMode) {
      $('v-dt').textContent = `${(S.dt * 180 / Math.PI).toFixed(1)}°/s`;
      $('v-df').textContent = `${(S.df * 180 / Math.PI).toFixed(1)}°/s`;
      $('v-dw').textContent = `${(S.dw * 180 / Math.PI).toFixed(0)}°/s`;
    } else {
      $('v-dt').textContent = `${(+sliders.dt.value).toFixed(1)}°`;
      $('v-df').textContent = `${(+sliders.df.value).toFixed(1)}°`;
      $('v-dw').textContent = `${sliders.dw.value}%`;
    }
  }

  function pushToUI() {
    const rateMode = S.sasMode === 3;
    sliders.thr.value = Math.round(S.thr * 100);
    if (rateMode) {
      sliders.dt.value = (S.dt / P.rateQMax * 25).toFixed(1);
      sliders.df.value = (S.df / P.rateQMax * 25).toFixed(1);
      sliders.dw.value = Math.round(S.dw / P.ratePMax * 30);
    } else {
      sliders.dt.value = (S.dt * 180 / Math.PI).toFixed(1);
      sliders.df.value = (S.df * 180 / Math.PI).toFixed(1);
      sliders.dw.value = Math.round(S.dw / P.dwUiMax * 100);
    }
    syncFromUI();
  }

  for (const k in sliders) sliders[k].addEventListener('input', () => { hooks.stopDemo(); syncFromUI(); });

  // ---- 角速度闭环模式：指令滑块松开自动回中（弹簧摇杆式，目标角速度归零） ----
  const springAnims = new Map();
  function cancelSpring(sl) {
    if (springAnims.has(sl)) { cancelAnimationFrame(springAnims.get(sl)); springAnims.delete(sl); }
  }
  function springBack(sl) {
    cancelSpring(sl);
    const stepFn = () => {
      const next = Math.abs(+sl.value) < 0.6 ? 0 : +sl.value * 0.55;
      sl.value = next;
      syncFromUI();
      if (next !== 0) springAnims.set(sl, requestAnimationFrame(stepFn));
      else springAnims.delete(sl);
    };
    springAnims.set(sl, requestAnimationFrame(stepFn));
  }
  for (const sl of [sliders.dt, sliders.df, sliders.dw]) {   // 油门不回中
    sl.addEventListener('pointerdown', () => cancelSpring(sl));
    const release = () => { if (S.sasMode === 3) springBack(sl); };
    sl.addEventListener('pointerup', release);
    sl.addEventListener('touchend', release);
    sl.addEventListener('keyup', release);
  }

  function resetAll() {
    hooks.stopDemo();
    resetSimulationState(sim, P);
    pushToUI();
  }
  $('b-reset').addEventListener('click', resetAll);

  const sasLabels = ['增稳 SAS：关', '增稳 SAS：全部', '增稳 SAS：仅角速率', '增稳 SAS：角速度闭环'];
  $('b-sas').addEventListener('click', () => {
    S.sasMode = (S.sasMode + 1) % 4;
    $('b-sas').classList.toggle('active', S.sasMode > 0);
    $('b-sas').textContent = sasLabels[S.sasMode];
    pushToUI();   // 切换模式时同步滑块映射和显示格式
  });
  $('b-aero').addEventListener('click', () => {
    S.aero = !S.aero;
    $('b-aero').classList.toggle('active', S.aero);
    $('b-aero').textContent = S.aero ? '气动力：开' : '气动力：忽略';
  });
  $('b-hover').addEventListener('click', () => {
    S.lockXY = !S.lockXY;
    $('b-hover').classList.toggle('active', S.lockXY);
    $('b-hover').textContent = S.lockXY ? '水平约束：开' : '水平约束：关';
  });
  $('b-info').addEventListener('click', () => $('modal').classList.add('open'));
  $('modalClose').addEventListener('click', () => $('modal').classList.remove('open'));
  $('modal').addEventListener('click', e => { if (e.target.id === 'modal') $('modal').classList.remove('open'); });

  return { syncFromUI, pushToUI };
}
