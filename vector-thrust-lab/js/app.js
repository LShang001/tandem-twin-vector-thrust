// ============================================================
//  纵列双发矢量推力布局 · 六维力/力矩动力学可视化
//  机体系约定: x 前 / y 右 / z 下（与推导的旋转矩阵完全一致）
// ============================================================
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { UnrealBloomPass } from 'three/addons/postprocessing/UnrealBloomPass.js';

// ---------- 物理参数（小型 UAV 量级） ----------
const P = {
  kT: 2.2e-5,        // 推力系数  T = kT·ω²        [N·s²]
  kQ: 1.6e-6,        // 反扭系数  τ = kQ·ω²        [N·m·s²]
  Jp: 2.0e-4,        // 桨+转子转动惯量             [kg·m²]
  wMax: 900,         // 最大转速                    [rad/s]
  a: 0.62,           // 前电机到质心距离            [m]
  b: 0.62,           // 尾电机到质心距离            [m]
  Ix: 0.09, Iy: 0.34, Iz: 0.36,   // 机体惯量      [kg·m²]
  dMax: 25 * Math.PI / 180,
  // ---- 质量 / 气动（高保真 6-DOF）----
  m: 2.6, g: 9.81,                // 质量 / 重力加速度
  rho: 1.225, Sw: 0.55,           // 空气密度 / 机翼面积
  cbar: 0.35, bspan: 2.2,         // 平均气动弦长 / 展长
  CLa: 4.6,  CD0: 0.035, CDk: 0.045,   // 升力线斜率 / 零升阻力 / 诱导阻力因子
  CYb: -0.35,                            // 侧力-侧滑导数
  Cm0: 0.011,                            // 机翼配平弯度矩（α_trim≈1.6° 处气动矩自平衡）
  Cma: -0.4, Cmq: -9.0,                  // 俯仰静稳定 / 俯仰阻尼导数
  Clb: -0.09, Clp: -0.5,                 // 横滚静稳定 / 滚转阻尼导数
  Cnb: 0.12, Cnr: -0.2,                  // 航向静稳定 / 偏航阻尼导数
  // ---- SAS 增稳（角速率 + 姿态角比例/积分反馈 → 摆角/差速修正）----
  sasQ: 0.14, sasR: 0.14, sasP: 0.18, sasTh: 0.3, sasPhi: 0.4, sasI: 0.1, sasIPhi: 0.15,
  aTrim: 1.6 * Math.PI / 180,     // 初始配平迎角
  vTrim: 24, thrTrim: 0.5,
};

// ---------- 运行状态 ----------
const S = {
  thr: 0.5,           // 总油门 0..1
  df: 0,              // 前摆角指令（偏航, 绕 z）
  dt: 0,              // 尾摆角指令（俯仰, 绕 y）
  dw: 0,              // 差速指令 -1..1
  dtAct: 0, dfAct: 0, dwAct: 0,   // SAS 修正后的实际执行量
  sas: true,          // 增稳开关
  aero: true,         // 气动力开关（false = 仅电机推力）
  wf: 0, wt: 0,       // 实际转速（一阶滞后）
  intTh: 0, intPhi: 0,  // SAS 积分器（俯仰/滚转）
  omega: new THREE.Vector3(),   // 机体角速度 [p q r] (rad/s)
  quat: new THREE.Quaternion(), // 姿态四元数
  time: 0,
  demo: null,         // 当前演示 {name, t0}
};

// ---------- 飞行状态（完整 6-DOF） ----------
const F = {
  pos: new THREE.Vector3(0, 0, 0),      // 惯性系位置（z 向下, 渲染用载机跟随系）
  vel: new THREE.Vector3(),             // 机体系速度 [u v w]
  vWorld: new THREE.Vector3(),          // 惯性系速度（驱动气流粒子/地面）
  euler: new THREE.Vector3(),           // φ θ ψ（显示/SAS）
};
function resetFlightState() {           // 以配平状态初始化（α0 = θ0 = α_trim, 平飞航迹）
  const a0 = P.aTrim;
  F.vel.set(P.vTrim * Math.cos(a0), 0, P.vTrim * Math.sin(a0));
  F.vWorld.set(P.vTrim, 0, 0);
  F.pos.set(0, 0, 0);
  S.quat.set(0, Math.sin(a0 / 2), 0, Math.cos(a0 / 2));
  S.omega.set(0, 0, 0);
  S.intTh = 0; S.intPhi = 0;
}
resetFlightState();

// ---------- 渲染器 / 场景 ----------
const container = document.getElementById('scene');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(innerWidth, innerHeight);
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.0;
container.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x04060c);
scene.fog = new THREE.Fog(0x04060c, 26, 85);

const camera = new THREE.PerspectiveCamera(46, innerWidth / innerHeight, 0.1, 300);
camera.up.set(0, 0, -1);                 // z 向下 ⇒ “上”是 -z
camera.position.set(4.9, -5.9, -3.4);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.06;
controls.minDistance = 3.2;
controls.maxDistance = 40;
controls.autoRotateSpeed = 0.9;

// ---------- 灯光 ----------
const amb = new THREE.AmbientLight(0x8fb4dd, 0.55); scene.add(amb);
const key = new THREE.DirectionalLight(0xdfeaff, 1.55);
key.position.set(6, -8, -12); scene.add(key);
const fill = new THREE.DirectionalLight(0x7aa2d8, 0.65);
fill.position.set(-6, 8, -4); scene.add(fill);
const rimCyan = new THREE.PointLight(0x38bdf8, 55, 40); rimCyan.position.set(-9, 7, 5); scene.add(rimCyan);
const rimWarm = new THREE.PointLight(0xf59e0b, 32, 40); rimWarm.position.set(10, 6, 3); scene.add(rimWarm);

// ---------- 环境: 地面网格 / 星空 / 气流粒子 ----------
let grid = null;
function makeGrid(c1, c2, op) {
  if (grid) { scene.remove(grid); grid.geometry.dispose(); grid.material.dispose(); }
  grid = new THREE.GridHelper(80, 64, c1, c2);
  grid.rotation.x = Math.PI / 2;           // 转到 x-y 平面
  grid.position.z = 7.5;
  grid.material.transparent = true; grid.material.opacity = op;
  scene.add(grid);
}
makeGrid(0x1d4ed8, 0x12233f, 0.5);

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

// 气流粒子（速度 ∝ 油门，可视化来流）
const AIR_N = 1300;
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

// ---------- 辉光后处理 ----------
const composer = new EffectComposer(renderer);
composer.addPass(new RenderPass(scene, camera));
const bloom = new UnrealBloomPass(new THREE.Vector2(innerWidth, innerHeight), 0.55, 0.45, 0.85);
composer.addPass(bloom);

addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
  composer.setSize(innerWidth, innerHeight);
});

// ============================================================
//  材质
// ============================================================
const M = {
  hull:  new THREE.MeshStandardMaterial({ color: 0x7d8ea8, metalness: 0.82, roughness: 0.42 }),
  dark:  new THREE.MeshStandardMaterial({ color: 0x27334a, metalness: 0.8,  roughness: 0.45 }),
  wing:  new THREE.MeshStandardMaterial({ color: 0x4a5f85, metalness: 0.72, roughness: 0.4 }),
  pod:   new THREE.MeshStandardMaterial({ color: 0x1c2740, metalness: 0.9,  roughness: 0.3 }),
  ring:  new THREE.MeshStandardMaterial({ color: 0x8ea2c0, metalness: 0.95, roughness: 0.25, emissive: 0x0e7490, emissiveIntensity: 0.35 }),
  neonC: new THREE.MeshBasicMaterial({ color: 0x22d3ee }),
  neonA: new THREE.MeshBasicMaterial({ color: 0xfbbf24 }),
  blade: new THREE.MeshStandardMaterial({ color: 0x111a2e, metalness: 0.7, roughness: 0.35 }),
};

// ============================================================
//  飞行器机体
// ============================================================
const aircraft = new THREE.Group();
scene.add(aircraft);

function mesh(geo, mat, x=0, y=0, z=0) {
  const m = new THREE.Mesh(geo, mat); m.position.set(x, y, z); return m;
}

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

// 座舱盖（光滑玻璃隆起, 半埋入机身上表面）
const canopy = mesh(new THREE.SphereGeometry(0.155, 24, 16), undefined, 0.42, 0, -0.17);
canopy.scale.set(2.3, 1.05, 0.62);
canopy.material = new THREE.MeshStandardMaterial({ color: 0x0a1526, metalness: 1, roughness: 0.06 });
aircraft.add(canopy);

// 主翼（平面形状直接画在 x-y 平面, 沿 z 拉伸厚度 —— 水平机翼）
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

// 翼尖小翼（贴在翼尖上, 向上 -z 立起）+ 航行灯
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

// ============================================================
//  矢量电机座（前：绕 z 偏航摆 / 尾：绕 y 俯仰摆）
// ============================================================
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

// 前电机（拉力式, 绕 z 摆）
const front = makeMotorPod(true);
front.gimbal.position.set(1.78, 0, 0);
aircraft.add(front.gimbal);
// 前摆座转环（绕 z ⇒ 环在 x-y 平面）
const ringF = mesh(new THREE.TorusGeometry(0.2, 0.02, 8, 40), M.ring, 1.66, 0, 0);
aircraft.add(ringF);

// 尾电机（推进式, 绕 y 摆）
const tail = makeMotorPod(false);
tail.gimbal.position.set(-1.82, 0, 0);
aircraft.add(tail.gimbal);
const ringT = mesh(new THREE.TorusGeometry(0.2, 0.02, 8, 40), M.ring, -1.7, 0, 0);
ringT.rotation.x = Math.PI / 2;                 // 绕 y ⇒ 环在 x-z 平面
aircraft.add(ringT);

// ============================================================
//  力 / 力矩可视化
// ============================================================
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

// 推力箭头（挂在各自摆座内, 随摆角自动偏转）
const thrustF = makeArrow(0x22d3ee); thrustF.position.set(0.5, 0, 0); front.gimbal.add(thrustF);
const thrustT = makeArrow(0x22d3ee); thrustT.position.set(0.4, 0, 0); tail.gimbal.add(thrustT);

// 质心合力矩箭头 Mx/My/Mz（体轴 RGB 配色: 滚转红 / 俯仰绿 / 偏航蓝紫）
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

// 反扭矩弧（琥珀色弧 + 箭头, 绕电机轴）
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
const arcF = makeTorqueArc(0xfbbf24, -1); arcF.position.set(-0.15, 0, 0); front.gimbal.add(arcF); // 前反扭 −x
const arcT = makeTorqueArc(0xfbbf24, +1); arcT.position.set(0.15, 0, 0);  tail.gimbal.add(arcT);  // 尾反扭 +x

// ---------- 3D 文字标签 ----------
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

// ============================================================
//  HUD 仪表
// ============================================================
const CH = [
  { id: 'Fx', name: 'F_x 纵向力',  color: '#22d3ee', max: 40,  unit: 'N',   bipolar: false },
  { id: 'Fy', name: 'F_y 侧向力',  color: '#22d3ee', max: 10,  unit: 'N',   bipolar: true  },
  { id: 'Fz', name: 'F_z 垂向力',  color: '#22d3ee', max: 10,  unit: 'N',   bipolar: true  },
  { id: 'Mx', name: 'M_x 滚转力矩', color: '#fb7185', max: 1.6, unit: 'N·m', bipolar: true  },
  { id: 'My', name: 'M_y 俯仰力矩', color: '#34d399', max: 6,   unit: 'N·m', bipolar: true  },
  { id: 'Mz', name: 'M_z 偏航力矩', color: '#818cf8', max: 6,   unit: 'N·m', bipolar: true  },
];
const meterEls = {};
{
  const box = document.getElementById('meters');
  for (const c of CH) {
    const div = document.createElement('div');
    div.className = 'meter';
    div.innerHTML = `<div class="meter-head"><span class="name" style="color:${c.color}">${c.name}</span>
      <span class="num" id="num-${c.id}">0.00</span></div>
      <div class="bar">${c.bipolar ? '<div class="zero"></div>' : ''}<div class="fill" id="fill-${c.id}" style="background:${c.color};box-shadow:0 0 8px ${c.color}"></div></div>`;
    box.appendChild(div);
    meterEls[c.id] = { fill: div.querySelector(`#fill-${c.id}`), num: div.querySelector(`#num-${c.id}`), cfg: c };
  }
}
function setMeter(id, v) {
  const { fill, num, cfg } = meterEls[id];
  const r = Math.min(Math.abs(v) / cfg.max, 1);
  if (cfg.bipolar) {
    fill.style.left = v >= 0 ? '50%' : `${50 - r * 50}%`;
    fill.style.width = `${r * 50}%`;
  } else {
    fill.style.left = '0'; fill.style.width = `${r * 100}%`;
  }
  num.textContent = v.toFixed(2);
}

// ============================================================
//  UI 绑定
// ============================================================
const $ = id => document.getElementById(id);
const sliders = { thr: $('s-thr'), dt: $('s-dt'), df: $('s-df'), dw: $('s-dw') };
function syncFromUI() {
  S.thr = sliders.thr.value / 100;
  S.dt  = sliders.dt.value * Math.PI / 180;
  S.df  = sliders.df.value * Math.PI / 180;
  S.dw  = sliders.dw.value / 100 * 0.55;         // 差速幅值 ±0.55
  $('v-thr').textContent = `${sliders.thr.value}%`;
  $('v-dt').textContent  = `${(+sliders.dt.value).toFixed(1)}°`;
  $('v-df').textContent  = `${(+sliders.df.value).toFixed(1)}°`;
  $('v-dw').textContent  = `${sliders.dw.value}%`;
}
function pushToUI() {
  sliders.thr.value = Math.round(S.thr * 100);
  sliders.dt.value  = (S.dt * 180 / Math.PI).toFixed(1);
  sliders.df.value  = (S.df * 180 / Math.PI).toFixed(1);
  sliders.dw.value  = Math.round(S.dw / 0.55 * 100);
  syncFromUI();
}
for (const k in sliders) sliders[k].addEventListener('input', () => { stopDemo(); syncFromUI(); });

function resetAll() {
  stopDemo();
  S.thr = P.thrTrim; S.dt = 0; S.df = 0; S.dw = 0;
  resetFlightState();
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

// ============================================================
//  演示序列
// ============================================================
const demoBtns = { pitch: $('b-pitch'), yaw: $('b-yaw'), roll: $('b-roll'), cine: $('b-cine') };
function stopDemo() {
  S.demo = null; controls.autoRotate = false;
  for (const k in demoBtns) demoBtns[k].classList.remove('active');
}
function startDemo(name) {
  stopDemo();
  S.demo = { name, t0: S.time };
  resetFlightState();
  S.dt = 0; S.df = 0; S.dw = 0;
  demoBtns[name].classList.add('active');
  if (name === 'cine') controls.autoRotate = true;
}
for (const k in demoBtns) demoBtns[k].addEventListener('click', () => {
  (S.demo && S.demo.name === k) ? stopDemo() : startDemo(k);
});

const D2R = Math.PI / 180;
function demoStep(t) {
  if (!S.demo) return;
  const τ = t - S.demo.t0, T = 3.2;                  // 单周期 3.2s
  const s = Math.sin(2 * Math.PI * τ / T);
  switch (S.demo.name) {
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
  pushToUI();
}

// ============================================================
//  高保真 6-DOF 动力学解算
//  平动: m·v̇ = F推力 + F气动 + m·g − m·ω×v        （机体系）
//  转动: I·ω̇ = M推力 + M气动 − ω×(I·ω) − ω×h转子
// ============================================================
const dyn  = { Fx:0, Fy:0, Fz:0, Mx:0, My:0, Mz:0, Tf:0, Tt:0, Qf:0, Qt:0 };
const aero = { V:0, qbar:0, alpha:0, beta:0, Mx:0, My:0, Mz:0 };
let prevWf = 0, prevWt = 0;
const _qi = new THREE.Quaternion(), _hv = new THREE.Vector3(), _gb = new THREE.Vector3();
const _m4 = new THREE.Matrix4();

// 数值积分子步长（滚转通道气动阻尼时间常数小, 显式积分需 ≤4ms）
function stepPhysics(dt) {
  const n = Math.max(1, Math.ceil(dt / 0.004));
  const h = dt / n;
  for (let i = 0; i < n; i++) physicsStep(h);
}

function physicsStep(dt) {
  // ---------- 姿态角（供 SAS 与 HUD） ----------
  const e = _m4.makeRotationFromQuaternion(S.quat).elements;
  const theta = -Math.asin(THREE.MathUtils.clamp(e[8], -1, 1));   // −asin(R13)
  const phi   = Math.atan2(e[9], e[10]);                          // atan2(R23, R33)
  const psi   = Math.atan2(e[4], e[0]);                           // atan2(R12, R11)
  F.euler.set(phi, theta, psi);

  // ---------- SAS 增稳: 角速率阻尼 + 姿态角保持（摆角/差速修正） ----------
  let dtC = S.dt, dfC = S.df, dwC = S.dw;
  if (S.sas) {
    // 反馈极性按各通道控制效率符号整定（∂My/∂δ_t<0, ∂Mx/∂Δω<0 → 正号; ∂Mz/∂δ_f>0 → 负号）
    // 比例 + 积分（积分消除常值配平误差, 带抗饱和限幅）
    S.intTh  = THREE.MathUtils.clamp(S.intTh + theta * dt, -0.5, 0.5);
    S.intPhi = THREE.MathUtils.clamp(S.intPhi + phi * dt,  -0.3, 0.3);
    dtC = THREE.MathUtils.clamp(dtC + P.sasQ * S.omega.y + P.sasTh * theta + P.sasI * S.intTh,   -P.dMax, P.dMax);
    dfC = THREE.MathUtils.clamp(dfC - P.sasR * S.omega.z,                                    -P.dMax, P.dMax);
    dwC = THREE.MathUtils.clamp(dwC + P.sasP * S.omega.x + P.sasPhi * phi + P.sasIPhi * S.intPhi, -0.7, 0.7);
  }
  S.dtAct = dtC; S.dfAct = dfC; S.dwAct = dwC;

  // ---------- 动力装置（电机一阶惯性 + 电磁反扭） ----------
  const w0 = S.thr * P.wMax;
  const wfT = w0 * Math.sqrt(Math.max(0, 1 + dwC));   // 差速分配: ω_f²+ω_t² ≈ 常数
  const wtT = w0 * Math.sqrt(Math.max(0, 1 - dwC));
  const tauM = 0.28;
  S.wf += (wfT - S.wf) * Math.min(dt / tauM, 1);
  S.wt += (wtT - S.wt) * Math.min(dt / tauM, 1);
  const dWf = (S.wf - prevWf) / Math.max(dt, 1e-4), dWt = (S.wt - prevWt) / Math.max(dt, 1e-4);
  prevWf = S.wf; prevWt = S.wt;

  const Tf = P.kT * S.wf * S.wf, Tt = P.kT * S.wt * S.wt;
  const Qf = P.kQ * S.wf * S.wf + P.Jp * dWf;         // 传给机体的反扭矩 = 电磁扭矩
  const Qt = P.kQ * S.wt * S.wt + P.Jp * dWt;
  const cf = Math.cos(dfC), sf = Math.sin(dfC);
  const ct = Math.cos(dtC), st = Math.sin(dtC);

  dyn.Tf = Tf; dyn.Tt = Tt; dyn.Qf = Qf; dyn.Qt = Qt;
  dyn.Fx = Tf * cf + Tt * ct;
  dyn.Fy = Tf * sf;
  dyn.Fz = -Tt * st;
  dyn.Mx = -Qf * cf + Qt * ct;                        // 滚转: 反扭差
  dyn.My = -P.b * Tt * st - Qf * sf;                  // 俯仰: 尾摆 + 耦合
  dyn.Mz =  P.a * Tf * sf - Qt * st;                  // 偏航: 前摆 + 耦合

  // ---------- 空气动力（风轴系 → 机体系） ----------
  const u = F.vel.x, v = F.vel.y, wv = F.vel.z;
  const V  = Math.max(Math.hypot(u, v, wv), 0.5);
  const al = Math.atan2(wv, u);                       // 迎角 α
  const be = Math.asin(THREE.MathUtils.clamp(v / V, -1, 1));  // 侧滑角 β
  const qb = 0.5 * P.rho * V * V;                     // 动压
  aero.V = V; aero.qbar = qb; aero.alpha = al; aero.beta = be;
  let aX = 0, aZ = 0, Y = 0;                          // 气动力（可被开关忽略）
  aero.Mx = 0; aero.My = 0; aero.Mz = 0;
  if (S.aero) {
    const CL = P.CLa * al;
    const L  = qb * P.Sw * CL;                        // 升力
    const D  = qb * P.Sw * (P.CD0 + P.CDk * CL * CL); // 阻力（零升+诱导）
    Y  = qb * P.Sw * P.CYb * be;                      // 侧力
    aX =  L * Math.sin(al) - D * Math.cos(al);
    aZ = -L * Math.cos(al) - D * Math.sin(al);
    // 气动矩: 静稳定项（α/β 恢复） + 阻尼导数项（p̂ q̂ r̂ 无量纲角速率）
    const pH = S.omega.x * P.bspan / (2 * V), qH = S.omega.y * P.cbar / (2 * V), rH = S.omega.z * P.bspan / (2 * V);
    aero.Mx = qb * P.Sw * P.bspan * (P.Clb * be + P.Clp * pH);
    aero.My = qb * P.Sw * P.cbar  * (P.Cm0 + P.Cma * al + P.Cmq * qH);
    aero.Mz = qb * P.Sw * P.bspan * (P.Cnb * be + P.Cnr * rH);
  }

  // ---------- 重力在机体系中的分量 ----------
  _gb.set(0, 0, P.g).applyQuaternion(_qi.copy(S.quat).invert());

  // ---------- 平动方程: v̇ = F/m − ω×v ----------
  const Fx = dyn.Fx + aX + P.m * _gb.x;
  const Fy = dyn.Fy + Y  + P.m * _gb.y;
  const Fz = dyn.Fz + aZ + P.m * _gb.z;
  const om = S.omega;
  F.vel.x += (Fx / P.m - (om.y * wv - om.z * v))  * dt;
  F.vel.y += (Fy / P.m - (om.z * u  - om.x * wv)) * dt;
  F.vel.z += (Fz / P.m - (om.x * v  - om.y * u))  * dt;

  // ---------- 转动方程: I·ω̇ = M − ω×(I·ω) − ω×h转子 ----------
  _hv.set(P.Jp * (S.wf * cf - S.wt * ct),              // 双转子角动量（前正后反, 沿摆座轴）
          P.Jp *  S.wf * sf,
          P.Jp *  S.wt * st);
  const gx = (P.Iz - P.Iy) * om.y * om.z + (om.y * _hv.z - om.z * _hv.y);
  const gy = (P.Ix - P.Iz) * om.z * om.x + (om.z * _hv.x - om.x * _hv.z);
  const gz = (P.Iy - P.Ix) * om.x * om.y + (om.x * _hv.y - om.y * _hv.x);
  om.x += ((dyn.Mx + aero.Mx - gx) / P.Ix) * dt;
  om.y += ((dyn.My + aero.My - gy) / P.Iy) * dt;
  om.z += ((dyn.Mz + aero.Mz - gz) / P.Iz) * dt;

  // ---------- 姿态积分: q̇ = ½·q⊗ω_body（机体系角速度右乘） ----------
  const half = dt / 2;
  const q = S.quat;
  const wq = new THREE.Quaternion(half * om.x, half * om.y, half * om.z, 0);
  const dq = q.clone().multiply(wq);
  q.set(q.x + dq.x, q.y + dq.y, q.z + dq.z, q.w + dq.w).normalize();
  aircraft.quaternion.copy(q);

  // ---------- 位置积分（惯性系; 渲染采用载机跟随系） ----------
  F.vWorld.copy(F.vel).applyQuaternion(q);
  F.pos.addScaledVector(F.vWorld, dt);
  if (F.pos.z > 6.2 && F.vWorld.z > 0) {              // 地面约束
    F.pos.z = 6.2; F.vWorld.z = 0;
    F.vel.copy(F.vWorld.applyQuaternion(_qi.copy(q).invert()));
  }
}

// ============================================================
//  可视化同步
// ============================================================
function syncVisuals(dt) {
  // 摆角（显示 SAS 修正后的实际执行角）
  front.gimbal.rotation.z = S.dfAct;           // 前: 绕 z（偏航）
  tail.gimbal.rotation.y  = S.dtAct;           // 尾: 绕 y（俯仰）
  // 螺旋桨对转（视觉转速 ≠ 物理转速, 仅示意）
  front.prop.rotation.x += dt * (6 + S.wf * 0.05);
  tail.prop.rotation.x  -= dt * (6 + S.wt * 0.05);
  front.disc.material.opacity = 0.15 + 0.5 * (S.wf / P.wMax);
  tail.disc.material.opacity  = 0.15 + 0.5 * (S.wt / P.wMax);

  // 推力箭头（长度 ∝ T）
  const kF = 0.062;
  thrustF.userData.setLen(dyn.Tf * kF);
  thrustT.userData.setLen(dyn.Tt * kF);
  labTF.position.set(0.5 + dyn.Tf * kF + 0.35, 0, -0.18);
  labTT.position.set(0.4 + dyn.Tt * kF + 0.35, 0, -0.18);

  // 反扭矩弧（缩放 ∝ τ）
  arcF.userData.holder.scale.setScalar(0.25 + 1.5 * Math.min(Math.abs(dyn.Qf) / 1.5, 1.4));
  arcT.userData.holder.scale.setScalar(0.25 + 1.5 * Math.min(Math.abs(dyn.Qt) / 1.5, 1.4));
  arcF.userData.holder.rotation.x += dt * 2.2;
  arcT.userData.holder.rotation.x += dt * 2.2;

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
  setMArrow(mArrowX, labX, dyn.Mx);
  setMArrow(mArrowY, labY, dyn.My);
  setMArrow(mArrowZ, labZ, dyn.Mz);

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

  cgRing.rotation.x += dt * 0.8; cgRing.rotation.y += dt * 0.6;
}

// ---------- HUD 刷新（每 3 帧） ----------
let frame = 0;
function syncHUD() {
  setMeter('Fx', dyn.Fx); setMeter('Fy', dyn.Fy); setMeter('Fz', dyn.Fz);
  setMeter('Mx', dyn.Mx); setMeter('My', dyn.My); setMeter('Mz', dyn.Mz);
  $('d-tf').textContent = dyn.Tf.toFixed(1) + ' N';
  $('d-tt').textContent = dyn.Tt.toFixed(1) + ' N';
  $('d-qf').textContent = dyn.Qf.toFixed(2) + ' N·m';
  $('d-qt').textContent = dyn.Qt.toFixed(2) + ' N·m';
  const dq = dyn.Qt - dyn.Qf;
  $('d-dq').textContent = (Math.abs(dq) < 0.02 ? '对消 ✓' : dq.toFixed(2) + ' N·m');
  $('d-dq').style.color = Math.abs(dq) < 0.02 ? '#34d399' : '#fb7185';
  $('d-wf').textContent = Math.round(S.wf) + ' rad/s';
  $('d-wt').textContent = Math.round(S.wt) + ' rad/s';
  $('d-v').textContent = aero.V.toFixed(1) + ' m/s';
  $('d-aoa').textContent = (aero.alpha * 57.2958).toFixed(1) + '°';
  $('d-h').textContent = (-F.pos.z).toFixed(1) + ' m';
  $('d-att').textContent = `${(F.euler.x*57.3).toFixed(0)} / ${(F.euler.y*57.3).toFixed(0)} / ${(F.euler.z*57.3).toFixed(0)}°`;
  $('d-amy').textContent = S.aero ? aero.My.toFixed(2) + ' N·m' : '已忽略';
  $('d-amy').style.color = S.aero ? '' : '#f59e0b';
  $('e-mx').textContent = dyn.Mx.toFixed(2);
  $('e-my').textContent = dyn.My.toFixed(2);
  $('e-mz').textContent = dyn.Mz.toFixed(2);
}

// ============================================================
//  主循环
// ============================================================
const clock = new THREE.Clock();
function animate() {
  requestAnimationFrame(animate);
  const dt = Math.min(clock.getDelta(), 0.05);
  S.time += dt;
  demoStep(S.time);
  stepPhysics(dt);
  syncVisuals(dt);
  if (++frame % 3 === 0) syncHUD();
  controls.update();
  composer.render();
}

syncFromUI();
S.wf = S.wt = S.thr * P.wMax; prevWf = S.wf; prevWt = S.wt;
S.dtAct = S.dt; S.dfAct = S.df; S.dwAct = S.dw;
animate();
setTimeout(() => { $('loader').classList.add('done'); startDemo('cine'); }, 900);

// ============================================================
//  深色 / 浅色主题
// ============================================================
const THEMES = {
  dark:  { bg: 0x04060c, fog: [26, 85], grid: [0x1d4ed8, 0x12233f], gridOp: 0.5,
           stars: true,  air: 0x67e8f9, bloom: 0.55, amb: 0.55, key: 1.55, fill: 0.65 },
  light: { bg: 0xe9eff8, fog: [24, 80], grid: [0x7aa2d8, 0xb9c8dc], gridOp: 0.9,
           stars: false, air: 0x3b82f6, bloom: 0.30, amb: 0.9,  key: 1.75, fill: 0.85 },
};
function setTheme(name) {
  const t = THEMES[name] || THEMES.dark;
  scene.background.setHex(t.bg);
  scene.fog.color.setHex(t.bg);
  scene.fog.near = t.fog[0]; scene.fog.far = t.fog[1];
  makeGrid(t.grid[0], t.grid[1], t.gridOp);
  stars.visible = t.stars;
  air.material.color.setHex(t.air);
  air.material.blending = name === 'light' ? THREE.NormalBlending : THREE.AdditiveBlending;
  air.material.needsUpdate = true;
  bloom.strength = t.bloom;
  amb.intensity = t.amb; key.intensity = t.key; fill.intensity = t.fill;
  document.body.classList.toggle('light', name === 'light');
  $('b-theme').textContent = name === 'light' ? '☾ 深色' : '☀ 浅色';
  try { localStorage.setItem('vt-theme', name); } catch (e) {}
}
$('b-theme').addEventListener('click', () => {
  setTheme(document.body.classList.contains('light') ? 'dark' : 'light');
});
try { if (localStorage.getItem('vt-theme') === 'light') setTheme('light'); } catch (e) {}
