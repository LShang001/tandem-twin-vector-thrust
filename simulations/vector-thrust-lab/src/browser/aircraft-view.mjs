// ============================================================
//  飞行器程序化几何与三维可视化同步
//  注意：渲染几何为示意性非尺度模型，与动力学力臂（a=b=0.62m）无关
// ============================================================
import * as THREE from 'three';

// ---------- 材质 ----------
const M = {
  hull:  new THREE.MeshStandardMaterial({ color: 0xd9dee7, metalness: 0.75, roughness: 0.38 }),
  dark:  new THREE.MeshStandardMaterial({ color: 0x646e7d, metalness: 0.8,  roughness: 0.40 }),
  wing:  new THREE.MeshStandardMaterial({ color: 0xb4bdcc, metalness: 0.65, roughness: 0.35 }),
  pod:   new THREE.MeshStandardMaterial({ color: 0x646e7d, metalness: 0.9,  roughness: 0.28 }),
  ring:  new THREE.MeshStandardMaterial({ color: 0xd9dee7, metalness: 0.95, roughness: 0.25, emissive: 0x0e7490, emissiveIntensity: 0.50 }),
  neonC: new THREE.MeshBasicMaterial({ color: 0x22d3ee }),
  neonA: new THREE.MeshBasicMaterial({ color: 0xfbbf24 }),
  blade: new THREE.MeshStandardMaterial({ color: 0x4d5665, metalness: 0.7, roughness: 0.30 }),
};

function mesh(geo, mat, x=0, y=0, z=0) {
  const m = new THREE.Mesh(geo, mat); m.position.set(x, y, z); return m;
}

function makeBlurDisc(r) {
  const cv = document.createElement('canvas'); cv.width = cv.height = 128;
  const ctx = cv.getContext('2d');
  const grd = ctx.createRadialGradient(64, 64, 8, 64, 64, 64);
  grd.addColorStop(0, 'rgba(150,220,255,0.0)');
  grd.addColorStop(0.55, 'rgba(150,220,255,0.22)');
  grd.addColorStop(0.85, 'rgba(150,220,255,0.10)');
  grd.addColorStop(1, 'rgba(150,220,255,0)');
  ctx.fillStyle = grd; ctx.fillRect(0, 0, 128, 128);
  const tex = new THREE.CanvasTexture(cv);
  const m = new THREE.Mesh(new THREE.CircleGeometry(r, 40),
    new THREE.MeshBasicMaterial({ map: tex, transparent: true, side: THREE.DoubleSide, depthWrite: false }));
  return m;
}

function makeProp() {
  const prop = new THREE.Group();
  for (let i = 0; i < 3; i++) {
    const bl = mesh(new THREE.BoxGeometry(0.035, 0.42, 0.07), M.blade, 0, 0.21, 0);
    bl.rotation.x = 0.42;                       // 桨距角
    const holder = new THREE.Group(); holder.add(bl);
    holder.rotation.x = i * Math.PI * 2 / 3;
    prop.add(holder);
  }
  prop.add(mesh(new THREE.ConeGeometry(0.075, 0.16, 16), M.pod, 0.06, 0, 0)).children;
  const spinner = prop.children[prop.children.length - 1];
  spinner.rotation.z = -Math.PI / 2;
  return prop;
}

function makeMotorPod(propAtPlusX) {
  // 返回 { gimbal, prop, disc } —— 摆座组原点在转轴处
  const gimbal = new THREE.Group();
  const nac = mesh(new THREE.CylinderGeometry(0.13, 0.13, 0.52, 20), M.pod);
  nac.rotation.z = Math.PI / 2;                 // 轴线 → x
  gimbal.add(nac);
  const band = mesh(new THREE.TorusGeometry(0.135, 0.018, 8, 28), M.ring, 0.1, 0, 0);
  band.rotation.y = Math.PI / 2;
  gimbal.add(band);
  const prop = makeProp();
  prop.position.x = propAtPlusX ? 0.34 : -0.34;
  if (!propAtPlusX) prop.rotation.y = Math.PI;  // 尾桨为推进式
  gimbal.add(prop);
  const disc = makeBlurDisc(0.46);
  disc.rotation.y = Math.PI / 2;
  disc.position.x = propAtPlusX ? 0.36 : -0.36;
  gimbal.add(disc);
  return { gimbal, prop, disc };
}

function makeArrow(color, headScale=1) {
  const g = new THREE.Group();
  const mat = new THREE.MeshBasicMaterial({ color });
  const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 1, 10), mat);
  shaft.rotation.z = -Math.PI / 2; shaft.position.x = 0.5;   // 沿 +x, 单位长度
  const head = new THREE.Mesh(new THREE.ConeGeometry(0.06 * headScale, 0.16 * headScale, 14), mat);
  head.rotation.z = -Math.PI / 2; head.position.x = 1.08;
  g.add(shaft, head);
  g.userData.setLen = (L) => {
    const len = Math.max(Math.abs(L), 0.001);
    shaft.scale.y = len; shaft.position.x = len / 2;
    head.position.x = len + 0.08 * headScale;
    g.visible = Math.abs(L) > 0.02;
  };
  return g;
}

function makeTorqueArc(color, sign) {
  const g = new THREE.Group();
  const arc = 4.4; // rad
  const tor = new THREE.Mesh(new THREE.TorusGeometry(0.3, 0.018, 8, 44, arc), new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.9 }));
  tor.rotation.y = Math.PI / 2;                 // 环轴 → x
  const cone = new THREE.Mesh(new THREE.ConeGeometry(0.05, 0.13, 12), tor.material);
  // 箭头放在弧末端切向
  const ex = 0, ey = 0.3 * Math.cos(arc), ez = 0.3 * Math.sin(arc);
  cone.position.set(ex, ey, ez);
  cone.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),
    new THREE.Vector3(0, -Math.sin(arc) * sign, Math.cos(arc) * sign).normalize());
  const holder = new THREE.Group(); holder.add(tor, cone);
  holder.rotation.x = sign > 0 ? 0 : Math.PI;   // 方向 = 反扭矩符号
  g.add(holder);
  g.userData.holder = holder;
  return g;
}

function makeLabel(text, color) {
  const cv = document.createElement('canvas'); cv.width = 256; cv.height = 96;
  const ctx = cv.getContext('2d');
  ctx.font = '600 46px "SF Mono", Consolas, monospace';
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  ctx.shadowColor = color; ctx.shadowBlur = 18;
  ctx.fillStyle = color; ctx.fillText(text, 128, 48);
  const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(cv), transparent: true, depthWrite: false }));
  sp.scale.set(0.62, 0.23, 1);
  return sp;
}

export function createAircraftView(scene) {
  const aircraft = new THREE.Group();
  scene.add(aircraft);

  // 机身（旋成体: 机头 → 尾椎一条光滑型线）
  {
    const prof = [
      [0.012, 1.80], [0.09, 1.68], [0.17, 1.50], [0.235, 1.20],
      [0.265, 0.70], [0.28, 0.10], [0.27, -0.45], [0.235, -0.95],
      [0.18, -1.35], [0.11, -1.62], [0.045, -1.78], [0.012, -1.84],
    ].map(p => new THREE.Vector2(p[0], p[1]));
    const fuse = new THREE.Mesh(new THREE.LatheGeometry(prof, 44), M.hull);
    fuse.rotation.z = -Math.PI / 2;                 // 型线 +y → 机头 +x
    aircraft.add(fuse);
  }

  // 座舱盖
  const canopy = mesh(new THREE.SphereGeometry(0.155, 24, 16), undefined, 0.42, 0, -0.17);
  canopy.scale.set(2.3, 1.05, 0.62);
  canopy.material = new THREE.MeshStandardMaterial({ color: 0x39424e, metalness: 1, roughness: 0.06 });
  aircraft.add(canopy);

  // 主翼（平面形状画在 x-y 平面, 沿 z 拉伸厚度 —— 水平机翼）
  M.wing.side = THREE.DoubleSide;
  function makeWingGeo() {
    const s = new THREE.Shape();
    s.moveTo(0.5, 0.16); s.lineTo(-0.6, 0.16); s.lineTo(-0.98, 2.05); s.lineTo(-0.42, 2.05); s.closePath();
    const g = new THREE.ExtrudeGeometry(s, { depth: 0.06, bevelEnabled: true, bevelThickness: 0.015, bevelSize: 0.02, bevelSegments: 2 });
    g.translate(0, 0, -0.03);
    return g;
  }
  const wingGeo = makeWingGeo();
  const wingR = new THREE.Mesh(wingGeo, M.wing);          // +y 右翼
  const wingL = new THREE.Mesh(wingGeo, M.wing);
  wingL.scale.y = -1;                                      // 镜像左翼
  aircraft.add(wingR, wingL);

  // 翼尖小翼 + 航行灯
  for (const side of [1, -1]) {
    const fin = mesh(new THREE.BoxGeometry(0.42, 0.035, 0.3), M.wing, -0.7, 2.02 * side, -0.17);
    fin.rotation.x = side * 0.28;
    aircraft.add(fin);
    const tip = mesh(new THREE.SphereGeometry(0.045, 10, 8), side > 0 ? M.neonC : M.neonA, -0.88, 2.03 * side, -0.02);
    aircraft.add(tip);
  }

  // 机身霓虹饰条
  const stripe = mesh(new THREE.BoxGeometry(2.1, 0.012, 0.012), M.neonC, 0.1, 0, -0.24);
  aircraft.add(stripe);
  const stripe2 = mesh(new THREE.BoxGeometry(1.4, 0.012, 0.012), M.neonA, -0.4, 0, 0.245);
  aircraft.add(stripe2);

  // ---------- 质心标记 ----------
  const cg = new THREE.Group();
  cg.add(mesh(new THREE.SphereGeometry(0.05, 14, 10), new THREE.MeshBasicMaterial({ color: 0xffffff })));
  const cgRing = new THREE.Mesh(new THREE.TorusGeometry(0.1, 0.008, 8, 32), new THREE.MeshBasicMaterial({ color: 0x94a3b8 }));
  cg.add(cgRing);
  aircraft.add(cg);

  // ---------- 矢量电机座（前：绕 z 偏航摆 / 尾：绕 y 俯仰摆） ----------
  // 前电机（拉力式, 绕 z 摆）
  const front = makeMotorPod(true);
  front.gimbal.position.set(1.78, 0, 0);
  aircraft.add(front.gimbal);
  const ringF = mesh(new THREE.TorusGeometry(0.2, 0.02, 8, 40), M.ring, 1.66, 0, 0);
  aircraft.add(ringF);

  // 尾电机（推进式, 绕 y 摆）
  const tail = makeMotorPod(false);
  tail.gimbal.position.set(-1.82, 0, 0);
  aircraft.add(tail.gimbal);
  const ringT = mesh(new THREE.TorusGeometry(0.2, 0.02, 8, 40), M.ring, -1.7, 0, 0);
  ringT.rotation.x = Math.PI / 2;                 // 绕 y ⇒ 环在 x-z 平面
  aircraft.add(ringT);

  // ---------- 推力箭头（挂在各自摆座内, 随摆角自动偏转） ----------
  const thrustF = makeArrow(0x22d3ee); thrustF.position.set(0.5, 0, 0); front.gimbal.add(thrustF);
  const thrustT = makeArrow(0x22d3ee); thrustT.position.set(0.4, 0, 0); tail.gimbal.add(thrustT);

  // ---------- 质心合力矩箭头 Mx/My/Mz ----------
  function makeAxisArrow(color, dir) {
    const a = makeArrow(color, 1.25);
    const base = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(1,0,0), dir.clone().normalize());
    a.userData.dir = dir.clone().normalize();
    a.userData.baseQ = base;
    aircraft.add(a);
    return a;
  }
  const mArrowX = makeAxisArrow(0xfb7185, new THREE.Vector3(1, 0, 0));
  const mArrowY = makeAxisArrow(0x34d399, new THREE.Vector3(0, 1, 0));
  const mArrowZ = makeAxisArrow(0x818cf8, new THREE.Vector3(0, 0, 1));

  // ---------- 反扭矩弧 ----------
  const arcF = makeTorqueArc(0xfbbf24, -1); arcF.position.set(-0.15, 0, 0); front.gimbal.add(arcF); // 前反扭 −x
  const arcT = makeTorqueArc(0xfbbf24, +1); arcT.position.set(0.15, 0, 0);  tail.gimbal.add(arcT);  // 尾反扭 +x

  // ---------- 3D 文字标签 ----------
  const labX = makeLabel('Mx', '#fb7185'); aircraft.add(labX);
  const labY = makeLabel('My', '#34d399'); aircraft.add(labY);
  const labZ = makeLabel('Mz', '#818cf8'); aircraft.add(labZ);
  const labTF = makeLabel('T', '#22d3ee'); front.gimbal.add(labTF);
  const labTT = makeLabel('T', '#22d3ee'); tail.gimbal.add(labTT);

  // ---------- 体轴参考线（细虚线） ----------
  {
    const mk = (dir, len, col) => {
      const g = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), dir.clone().multiplyScalar(len)]);
      const l = new THREE.Line(g, new THREE.LineDashedMaterial({ color: col, dashSize: 0.12, gapSize: 0.1, transparent: true, opacity: 0.35 }));
      l.computeLineDistances(); aircraft.add(l);
    };
    mk(new THREE.Vector3(1,0,0), 3.4, 0xfb7185);
    mk(new THREE.Vector3(0,1,0), 3.4, 0x34d399);
    mk(new THREE.Vector3(0,0,1), 2.2, 0x818cf8);
  }

  return { aircraft, front, tail, cgRing, thrustF, thrustT, labTF, labTT, arcF, arcT, mArrowX, mArrowY, mArrowZ, labX, labY, labZ };
}

// ---------- 可视化同步（每帧） ----------
export function updateAircraftView(view, sim, P, dt) {
  const { S, dyn } = sim;
  // 姿态 → 机体
  view.aircraft.quaternion.copy(S.quat);

  // 摆角（显示 SAS 修正后的实际执行角）
  view.front.gimbal.rotation.z = S.dfAct;        // 前: 绕 z（偏航）
  view.tail.gimbal.rotation.y  = S.dtAct;        // 尾: 绕 y（俯仰）

  // 螺旋桨对转（视觉转速 ≠ 物理转速, 仅示意）
  view.front.prop.rotation.x += dt * (6 + S.wf * 0.05);
  view.tail.prop.rotation.x  -= dt * (6 + S.wt * 0.05);
  view.front.disc.material.opacity = 0.15 + 0.5 * (S.wf / P.wMax);
  view.tail.disc.material.opacity  = 0.15 + 0.5 * (S.wt / P.wMax);

  // 推力箭头（长度 ∝ T）
  const kF = 0.062;
  view.thrustF.userData.setLen(dyn.Tf * kF);
  view.thrustT.userData.setLen(dyn.Tt * kF);
  view.labTF.position.set(0.5 + dyn.Tf * kF + 0.35, 0, -0.18);
  view.labTT.position.set(0.4 + dyn.Tt * kF + 0.35, 0, -0.18);

  // 反扭矩弧（缩放 ∝ τ）
  view.arcF.userData.holder.scale.setScalar(0.25 + 1.5 * Math.min(Math.abs(dyn.Qf) / 1.5, 1.4));
  view.arcT.userData.holder.scale.setScalar(0.25 + 1.5 * Math.min(Math.abs(dyn.Qt) / 1.5, 1.4));
  view.arcF.userData.holder.rotation.x += dt * 2.2;
  view.arcT.userData.holder.rotation.x += dt * 2.2;

  // 质心合力矩箭头（含方向翻转）
  const kM = 0.4;
  const setMArrow = (arr, lab, v) => {
    const s = v >= 0 ? 1 : -1;
    arr.quaternion.copy(arr.userData.baseQ);
    if (s < 0) arr.quaternion.multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,0,1), Math.PI));
    arr.userData.setLen(Math.abs(v) * kM);
    lab.visible = arr.visible;
    const d = arr.userData.dir.clone().multiplyScalar(s * (Math.abs(v) * kM + 0.5));
    lab.position.copy(d);
  };
  setMArrow(view.mArrowX, view.labX, dyn.Mx);
  setMArrow(view.mArrowY, view.labY, dyn.My);
  setMArrow(view.mArrowZ, view.labZ, dyn.Mz);

  // 质心标记旋转
  view.cgRing.rotation.x += dt * 0.8; view.cgRing.rotation.y += dt * 0.6;
}
