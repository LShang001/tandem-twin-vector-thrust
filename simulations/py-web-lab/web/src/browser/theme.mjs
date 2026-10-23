/**
 * @file    theme.mjs
 * @module  analysis/py-web-lab 渲染（主题）
 * @brief   定义深/浅两套配色与光照参数，供场景与效果模块切换。
 * @details 输入：主题名 dark/light；处理：返回颜色、雾、辉光、光强参数集；
 *          输出：供 scene/effects/aircraft-view 使用的常量表。
 * @theory  N/A（渲染层）
 * @frame   N/A
 * @units   颜色为十六进制、强度为无量纲倍数
 * @note    主题只影响视觉，不改变任何仿真状态。
 * @see     scene.mjs、effects.mjs
 */
import * as THREE from 'three';

const THEMES = {
  dark:  { bg: 0x272b33, fog: [30, 120], grid: [0x6b7a92, 0x3d4450], gridOp: 0.55,
           stars: true,  air: 0x7dd3fc, bloom: 0.30, amb: 1.3, key: 1.7, fill: 1.0,
           sky: { top: 0x171b23, horizonCol: 0x3d4450, bottom: 0x1f232b, ring: 0x8a97a8, ringOp: 0.75 } },
  light: { bg: 0xe9eff8, fog: [24, 80], grid: [0x7aa2d8, 0xb9c8dc], gridOp: 0.9,
           stars: false, air: 0x3b82f6, bloom: 0.30, amb: 0.9,  key: 1.75, fill: 0.85,
           sky: { top: 0x6f9fd6, horizonCol: 0xe4edf7, bottom: 0xc9d5e2, ring: 0x7186a0, ringOp: 0.9 } },
};

/**
 * 主题切换器：把深/浅两套配色应用到场景、辉光、灯光与环境效果。
 * @param {Object} ctx { scene, bloom, lights, effects } 需要换肤的对象集合
 * @returns {Object} { toggle(), apply(name) }：切换并返回当前主题名
 */
export function createTheme({ scene, bloom, lights, effects }) {
  const $ = id => document.getElementById(id);

  function setTheme(name) {
    const t = THEMES[name] || THEMES.dark;
    scene.background.setHex(t.bg);
    scene.fog.color.setHex(t.bg);
    scene.fog.near = t.fog[0]; scene.fog.far = t.fog[1];
    effects.rebuildGrid(t.grid[0], t.grid[1], t.gridOp);
    effects.rebuildSky(t.sky);
    effects.stars.visible = t.stars;
    effects.air.material.color.setHex(t.air);
    effects.air.material.blending = name === 'light' ? THREE.NormalBlending : THREE.AdditiveBlending;
    effects.air.material.needsUpdate = true;
    bloom.strength = t.bloom;
    lights.amb.intensity = t.amb; lights.key.intensity = t.key; lights.fill.intensity = t.fill;
    document.body.classList.toggle('light', name === 'light');
    $('b-theme').textContent = name === 'light' ? '☾ 深色' : '☀ 浅色';
    try { localStorage.setItem('vt-theme', name); } catch (e) {}
  }

  $('b-theme').addEventListener('click', () => {
    setTheme(document.body.classList.contains('light') ? 'dark' : 'light');
  });
  try { if (localStorage.getItem('vt-theme') === 'light') setTheme('light'); } catch (e) {}

  return { setTheme };
}
