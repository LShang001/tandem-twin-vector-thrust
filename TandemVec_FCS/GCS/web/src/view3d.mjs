// ============================================================
//  view3d.mjs — Three.js 3D 姿态视图
//  纵列双发构型简模：机身 + 前后电机短舱 + 前摆座（绕 z）/尾摆座（绕 y）
// ============================================================
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { bus, state } from './main.mjs';

let scene, camera, renderer, controls;
let groupAirframe, gimbalFront, gimbalRear, propFront, propRear;
let running = false;
let tvc1 = 0, tvc2 = 0, thr = 0;
let animFrame = 0;

export function activate() {
  if (running) return;
  running = true;
  const container = document.getElementById('v3dContainer');
  const W = container.clientWidth || 800;
  const H = container.clientHeight || 600;

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0b1019);
  scene.fog = new THREE.Fog(0x0b1019, 30, 70);

  camera = new THREE.PerspectiveCamera(45, W / H, 0.1, 200);
  camera.position.set(6.5, 4.5, 7.5);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(W, H);
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  container.appendChild(renderer.domElement);

  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.target.set(0, 1.2, 0);

  lights();
  buildAircraft();
  buildGround();
  window.addEventListener('resize', onResize);
  bus.addEventListener('telemetry', onTelemetry);
  bus.addEventListener('page', onPage);
  animate();
}

function onPage(e) { if (e.detail === 'view3d') syncPose(); }

function onResize() {
  const c = document.getElementById('v3dContainer');
  if (!c) return;
  camera.aspect = c.clientWidth / c.clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(c.clientWidth, c.clientHeight);
}

function lights() {
  scene.add(new THREE.HemisphereLight(0x9cc8ff, 0x2a2018, 0.9));
  const dir = new THREE.DirectionalLight(0xffffff, 1.6);
  dir.position.set(8, 12, 6);
  scene.add(dir);
}

function mat(color, rough = 0.6, metal = 0.2) {
  return new THREE.MeshStandardMaterial({ color, roughness: rough, metalness: metal });
}

function buildAircraft() {
  // 整体组（姿态四元数驱动）
  groupAirframe = new THREE.Group();

  // 机身（纵列：x 前）
  const fuse = new THREE.Mesh(new THREE.BoxGeometry(1.1, 0.5, 2.6), mat(0xd8dee8, 0.5));
  fuse.position.y = 0.45;
  const nose = new THREE.Mesh(new THREE.ConeGeometry(0.28, 0.5, 4), mat(0x9cc8f5, 0.4));
  nose.rotation.z = Math.PI / 4;
  nose.position.set(0, 0.4, 1.5);
  const tail = new THREE.Mesh(new THREE.BoxGeometry(0.9, 0.28, 0.5), mat(0xd8dee8, 0.5));
  tail.position.set(0, 0.75, -1.4);
  groupAirframe.add(fuse, nose, tail);

  // 前摆座（绕 z_b 摆动 → 偏航主控）：短舱 + 前电机
  gimbalFront = new THREE.Group();
  gimbalFront.position.set(0, 0.5, 1.05);
  const podF = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.44, 0.5), mat(0x3ea6ff, 0.35));
  gimbalFront.add(podF);
  propFront = propeller(0x3ea6ff);
  gimbalFront.add(propFront);
  groupAirframe.add(gimbalFront);

  // 尾摆座（绕 y_b 摆动 → 俯仰主控）：短舱 + 尾电机
  gimbalRear = new THREE.Group();
  gimbalRear.position.set(0, 0.5, -1.05);
  const podR = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.44, 0.5), mat(0x29d3a2, 0.35));
  gimbalRear.add(podR);
  propRear = propeller(0x29d3a2);
  gimbalRear.add(propRear);
  groupAirframe.add(gimbalRear);

  scene.add(groupAirframe);
}

function propeller(color) {
  const g = new THREE.Group();
  const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.09, 0.14, 12), mat(color, 0.3));
  const blade = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.5, 0.16), mat(0xcccccc, 0.4));
  blade.position.y = 0.34;
  const blade2 = blade.clone();
  blade2.rotation.y = Math.PI / 2;
  g.add(hub, blade, blade2);
  g.position.y = 0.42;
  return g;
}

function buildGround() {
  const grid = new THREE.GridHelper(20, 20, 0x3ea6ff, 0x24405e);
  grid.position.y = 0.001;
  scene.add(grid);
  const plane = new THREE.Mesh(new THREE.PlaneGeometry(40, 40),
    new THREE.MeshStandardMaterial({ color: 0x10161f, roughness: 1 }));
  plane.rotation.x = -Math.PI / 2;
  scene.add(plane);
}

// NED → Three 坐标（NED: x前 y右 z下；Three: x右 y上 z前）
function nedToThree(q) {
  // 机体 NED 姿态四元数 → 世界系。映射：NED(x,y,z) → Three(x,y,z) = (y, -z, x)
  const M = new THREE.Matrix4();
  const P = new THREE.Matrix4().makeBasis(
    new THREE.Vector3(0, 0, 1),   // NED x → Three z
    new THREE.Vector3(1, 0, 0),   // NED y → Three x
    new THREE.Vector3(0, -1, 0),  // NED z → Three -y
  );
  const Q = new THREE.Quaternion(q.x, q.y, q.z, q.w);   // THREE 与 NED 同为右手系，直接套用
  return M.multiplyMatrices(P, new THREE.Matrix4().makeRotationFromQuaternion(Q)).decompose();
}

function onTelemetry(s) {
  if (s.qx !== undefined && s.qw !== undefined && s.qx !== null) {
    // 固件未发四元数帧（0x04 未启用），用欧拉角合成 NED 四元数（3-2-1 约定下）
    const r = THREE.MathUtils.degToRad(s.roll_deg || 0);
    const p = THREE.MathUtils.degToRad(s.pitch_deg || 0);
    const h = THREE.MathUtils.degToRad(s.heading_deg || 0);
    const q = new THREE.Quaternion().setFromEuler(new THREE.Euler(p, h, r, 'YXZ'));
    q.normalize();
    applyPose(q);
  }
  if (s.ctrl_thr_pct !== undefined) thr = s.ctrl_thr_pct / 100;
}

function applyPose(q) {
  if (!groupAirframe) return;
  const M = new THREE.Matrix4();
  const P = new THREE.Matrix4().makeBasis(
    new THREE.Vector3(0, 0, 1),
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(0, -1, 0),
  );
  M.multiplyMatrices(P, new THREE.Matrix4().makeRotationFromQuaternion(q));
  groupAirframe.quaternion.setFromRotationMatrix(M);
  groupAirframe.position.set(0, 0.6, 0);
}

function syncPose() {
  const s = state.snap;
  if (s.roll_deg !== undefined) {
    const r = THREE.MathUtils.degToRad(s.roll_deg || 0);
    const p = THREE.MathUtils.degToRad(s.pitch_deg || 0);
    const h = THREE.MathUtils.degToRad(s.heading_deg || 0);
    applyPose(new THREE.Quaternion().setFromEuler(new THREE.Euler(p, h, r, 'YXZ')).normalize());
  }
}

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  if (gimbalFront) {
    // 前摆座绕局部 z 摆动（简化：以 ctrl_roll 示意）
    gimbalFront.rotation.z = THREE.MathUtils.degToRad(tvc1);
    gimbalRear.rotation.y = THREE.MathUtils.degToRad(tvc2);
  }
  if (propFront) {
    const w = 6 + thr * 40;
    propFront.rotation.y += 0.12 * w;
    propRear.rotation.y -= 0.12 * w;   // 反转
  }
  renderer.render(scene, camera);
}
