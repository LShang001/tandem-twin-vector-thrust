// ============================================================
//  环境效果：地面网格 / 星空 / 气流粒子 / 网格跟随
// ============================================================
import * as THREE from 'three';

const AIR_N = 1300;

export function createEffects(scene) {
  // ---------- 地面网格 ----------
  let grid = null;
  function rebuildGrid(c1, c2, op) {
    if (grid) { scene.remove(grid); grid.geometry.dispose(); grid.material.dispose(); }
    grid = new THREE.GridHelper(80, 64, c1, c2);
    grid.rotation.x = Math.PI / 2;           // 转到 x-y 平面
    grid.position.z = 7.5;
    grid.material.transparent = true; grid.material.opacity = op;
    scene.add(grid);
  }
  rebuildGrid(0x1d4ed8, 0x12233f, 0.5);

  // ---------- 星空 ----------
  const stars = (() => {
    const g = new THREE.BufferGeometry(); const n = 900; const p = new Float32Array(n * 3);
    for (let i = 0; i < n; i++) {
      const r = 90 + Math.random() * 60, th = Math.random() * Math.PI * 2, ph = Math.acos(2 * Math.random() - 1);
      p[i*3] = r * Math.sin(ph) * Math.cos(th); p[i*3+1] = r * Math.sin(ph) * Math.sin(th); p[i*3+2] = r * Math.cos(ph);
    }
    g.setAttribute('position', new THREE.BufferAttribute(p, 3));
    const s = new THREE.Points(g, new THREE.PointsMaterial({ color: 0x9cc8ff, size: 0.55, sizeAttenuation: true, transparent: true, opacity: 0.65, fog: false }));
    scene.add(s); return s;
  })();

  // ---------- 气流粒子（速度 ∝ 油门，可视化来流） ----------
  const airGeo = new THREE.BufferGeometry();
  const airPos = new Float32Array(AIR_N * 3);
  const airSeed = new Float32Array(AIR_N);
  for (let i = 0; i < AIR_N; i++) {
    airPos[i*3]   = (Math.random() * 2 - 1) * 34;
    airPos[i*3+1] = (Math.random() * 2 - 1) * 15;
    airPos[i*3+2] = -6 + Math.random() * 11;
    airSeed[i] = 0.4 + Math.random();
  }
  airGeo.setAttribute('position', new THREE.BufferAttribute(airPos, 3));
  const air = new THREE.Points(airGeo, new THREE.PointsMaterial({
    color: 0x67e8f9, size: 0.055, transparent: true, opacity: 0.5,
    blending: THREE.AdditiveBlending, depthWrite: false }));
  scene.add(air);

  // ---------- 每帧更新：气流粒子 + 地面网格跟随 ----------
  function update(dt, sim) {
    const { F, aero } = sim;
    // 气流粒子 = 相对风（真实空速驱动; 悬停时近乎静止）
    const vw = F.vWorld;
    const pos = airGeo.attributes.position.array;
    for (let i = 0; i < AIR_N; i++) {
      pos[i*3]   -= vw.x * airSeed[i] * dt;
      pos[i*3+1] -= vw.y * airSeed[i] * dt;
      pos[i*3+2] -= vw.z * airSeed[i] * dt;
      if (pos[i*3] < -34) pos[i*3] += 68; else if (pos[i*3] > 34) pos[i*3] -= 68;
      if (pos[i*3+1] < -15) pos[i*3+1] += 30; else if (pos[i*3+1] > 15) pos[i*3+1] -= 30;
      if (pos[i*3+2] < -6)  pos[i*3+2] += 11; else if (pos[i*3+2] > 5) pos[i*3+2] -= 11;
    }
    airGeo.attributes.position.needsUpdate = true;
    air.material.opacity = 0.08 + 0.45 * Math.min(aero.V / 25, 1);

    // 地面网格跟随（载机跟随系: 网格反衬平移 + 真实高度）
    grid.position.x = -(F.pos.x % 1.25);
    grid.position.y = -(F.pos.y % 1.25);
    grid.position.z = Math.max(1.6, 7.5 - F.pos.z);
  }

  return { rebuildGrid, stars, air, update };
}
