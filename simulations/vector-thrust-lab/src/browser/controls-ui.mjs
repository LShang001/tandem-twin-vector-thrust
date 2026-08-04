// ============================================================
//  控制面板 UI：滑块 / 开关 / 复位 / 模态框
// ============================================================
import { resetSimulationState, resetVtolHoverState } from '../core/state.mjs';

export function createControlsUI({ sim, P, hooks }) {
  const S = sim.S;
  const $ = id => document.getElementById(id);
  const sliders = { thr: $('s-thr'), dt: $('s-dt'), df: $('s-df'), dw: $('s-dw') };

  // 模式相关按钮文案统一刷新（模式可能被复位/演示间接切换）
  function refreshModeUI() {
    $('b-vtol').textContent = S.vtolMode ? '悬停模式：开' : '悬停模式：关';
    $('b-vtol').classList.toggle('active', S.vtolMode);
    $('b-aero').textContent = S.aero ? '气动力：开' : '气动力：忽略';
    $('b-aero').classList.toggle('active', S.aero);
    if (S.vtolMode) {
      $('b-sas').textContent = S.sasMode === 0 ? '自稳：关（直通）' : '自稳：开（四元数）';
      $('b-sas').classList.toggle('active', S.sasMode > 0);
      // 滑块语义切换为姿态指令（四轴式），标签同步更新
      $('lbl-dt').innerHTML = '俯仰倾斜指令 θ（绕 y<sub>b</sub>）';
      $('lbl-df').innerHTML = '侧倾指令 φ（绕 z<sub>b</sub>）';
      $('lbl-dw').innerHTML = '航向指令 ψ（绕 x<sub>b</sub>，差速）';
    } else {
      $('b-sas').textContent = sasLabels[S.sasMode];
      $('b-sas').classList.toggle('active', S.sasMode > 0);
      $('lbl-dt').innerHTML = '俯仰摆角 δ<sub>t</sub>（尾电机·绕 y）';
      $('lbl-df').innerHTML = '偏航摆角 δ<sub>f</sub>（前电机·绕 z）';
      $('lbl-dw').innerHTML = '差速 Δω（前 ⊕ / 尾 ⊖ → 滚转）';
    }
  }

  function syncFromUI() {
    const rateMode = S.sasMode === 3;
    S.thr = sliders.thr.value / 100;
    if (S.vtolMode) {
      // 悬停模式：滑块 = 目标姿态角指令（四轴式语义）
      //   dw → 航向 ψ（绕 x_b=世界竖直轴）、dt → 俯仰倾斜（绕 y_b）、df → 侧倾（绕 z_b）
      S.dt = sliders.dt.value * Math.PI / 180;
      S.df = sliders.df.value * Math.PI / 180;
      S.dw = sliders.dw.value * Math.PI / 180;
      $('v-thr').textContent = `${sliders.thr.value}%`;
      $('v-dt').textContent = `${(+sliders.dt.value).toFixed(1)}°`;
      $('v-df').textContent = `${(+sliders.df.value).toFixed(1)}°`;
      $('v-dw').textContent = `${(+sliders.dw.value).toFixed(1)}°`;
      return;
    }
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
    if (S.vtolMode) {
      sliders.dt.value = (S.dt * 180 / Math.PI).toFixed(1);
      sliders.df.value = (S.df * 180 / Math.PI).toFixed(1);
      sliders.dw.value = (S.dw * 180 / Math.PI).toFixed(1);
    } else if (rateMode) {
      sliders.dt.value = (S.dt / P.rateQMax * 25).toFixed(1);
      sliders.df.value = (S.df / P.rateQMax * 25).toFixed(1);
      sliders.dw.value = Math.round(S.dw / P.ratePMax * 30);
    } else {
      sliders.dt.value = (S.dt * 180 / Math.PI).toFixed(1);
      sliders.df.value = (S.df * 180 / Math.PI).toFixed(1);
      sliders.dw.value = Math.round(S.dw / P.dwUiMax * 100);
    }
    syncFromUI();
    refreshModeUI();
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
    if (S.vtolMode) {
      // 悬停模式：自稳仅区分 开/关（四元数自稳 vs 直通）
      S.sasMode = S.sasMode === 0 ? 1 : 0;
    } else {
      S.sasMode = (S.sasMode + 1) % 4;
    }
    refreshModeUI();
    pushToUI();   // 切换模式时同步滑块映射和显示格式
  });
  $('b-aero').addEventListener('click', () => {
    S.aero = !S.aero;
    refreshModeUI();
  });
  $('b-vtol').addEventListener('click', () => {
    hooks.stopDemo();
    if (S.vtolMode) {
      resetSimulationState(sim, P);  // 切回固定翼巡航（内部 vtolMode=false）
      S.aero = true;                 // 巡航恢复机翼（气动力开）
    } else {
      resetVtolHoverState(sim, P);   // ★ 进入悬停：机头朝天 + 无翼（aero=false）
    }
    pushToUI();
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
