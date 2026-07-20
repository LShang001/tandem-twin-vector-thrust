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
  rebuildGrid(0x6b7a92, 0x3d4450, 0.55);

  // ---------- 星空 ----------
  const stars = (() => {
    const g = new THREE.BufferGeometry(); const n = 600; const p = new Float32Array(n * 3);
    for (let i = 0; i < n; i++) {
      const r = 90 + Math.random() * 60, th = Math.random() * Math.PI * 2, ph = Math.acos(2 * Math.random() - 1);
      p[i*3] = r * Math.sin(ph) * Math.cos(th); p[i*3+1] = r * Math.sin(ph) * Math.sin(th); p[i*3+2] = r * Math.cos(ph);
    }
    g.setAttribute('position', new THREE.BufferAttribute(p, 3));
    const s = new THREE.Points(g, new THREE.PointsMaterial({ color: 0xb9d6ff, size: 0.5, sizeAttenuation: true, transparent: true, opacity: 0.5, fog: false }));
    scene.add(s); return s;
  })();

  // ---------- 天空穹顶（渐变） + 地平线环（世界系固定，反衬姿态） ----------
  const SKY_R = 150;
  const sky = (() => {
    const g = new THREE.SphereGeometry(SKY_R, 40, 28);
    const m = new THREE.MeshBasicMaterial({ vertexColors: true, side: THREE.BackSide, fog: false, depthWrite: false });
    const s = new THREE.Mesh(g, m);
    s.renderOrder = -1;                    // 最先绘制，不遮挡任何物体
    scene.add(s); return s;
  })();

  function paintSky(top, horizonCol, bottom) {
    const pos = sky.geometry.attributes.position;
    const colors = new Float32Array(pos.count * 3);
    const cT = new THREE.Color(top), cH = new THREE.Color(horizonCol), cB = new THREE.Color(bottom);
    const c = new THREE.Color();
    for (let i = 0; i < pos.count; i++) {
      const t = pos.getZ(i) / SKY_R;       // z 向下：t<0 天顶，t>0 地下
      if (t < 0) c.lerpColors(cH, cT, Math.min(-t * 1.7, 1));
      else       c.lerpColors(cH, cB, Math.min( t * 1.7, 1));
      colors[i*3] = c.r; colors[i*3+1] = c.g; colors[i*3+2] = c.b;
    }
    sky.geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  }

  // 地平线环 + 方位刻度（每 30°，每 90° 加长）——辅助判读姿态与航向
  const ringMat = new THREE.MeshBasicMaterial({ color: 0x8a97a8, transparent: true, opacity: 0.75, fog: false });
  const horizonRing = new THREE.Group();
  horizonRing.add(new THREE.Mesh(new THREE.TorusGeometry(120, 0.25, 8, 160), ringMat));
  for (let i = 0; i < 12; i++) {
    const major = i % 3 === 0;
    const tick = new THREE.Mesh(new THREE.BoxGeometry(0.6, 0.6, major ? 5 : 2.4), ringMat);
    const a = i * Math.PI / 6;
    tick.position.set(120 * Math.cos(a), 120 * Math.sin(a), 0);
    horizonRing.add(tick);
  }
  scene.add(horizonRing);

  function rebuildSky({ top, horizonCol, bottom, ring, ringOp }) {
    paintSky(top, horizonCol, bottom);
    ringMat.color.setHex(ring); ringMat.opacity = ringOp;
  }
  rebuildSky({ top: 0x171b23, horizonCol: 0x3d4450, bottom: 0x1f232b, ring: 0x8a97a8, ringOp: 0.75 });

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
    color: 0x7dd3fc, size: 0.055, transparent: true, opacity: 0.5,
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

  return { rebuildGrid, rebuildSky, stars, air, update };
}
