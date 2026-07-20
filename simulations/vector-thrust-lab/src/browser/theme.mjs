// ============================================================
//  深色 / 浅色主题
// ============================================================
import * as THREE from 'three';

const THEMES = {
  dark:  { bg: 0x04060c, fog: [26, 85], grid: [0x1d4ed8, 0x12233f], gridOp: 0.5,
           stars: true,  air: 0x67e8f9, bloom: 0.55, amb: 0.55, key: 1.55, fill: 0.65 },
  light: { bg: 0xe9eff8, fog: [24, 80], grid: [0x7aa2d8, 0xb9c8dc], gridOp: 0.9,
           stars: false, air: 0x3b82f6, bloom: 0.30, amb: 0.9,  key: 1.75, fill: 0.85 },
};

export function createTheme({ scene, bloom, lights, effects }) {
  const $ = id => document.getElementById(id);

  function setTheme(name) {
    const t = THEMES[name] || THEMES.dark;
    scene.background.setHex(t.bg);
    scene.fog.color.setHex(t.bg);
    scene.fog.near = t.fog[0]; scene.fog.far = t.fog[1];
    effects.rebuildGrid(t.grid[0], t.grid[1], t.gridOp);
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
