// ============================================================
//  view3d.mjs — 基础 3D 姿态视图（Three.js）
//  设计原则：只画"姿态"本身——六面异色长方体 + 前向箭头 + 机背立柱
//            + 地平参考，无机型细节，方向判读无歧义。
//
//  坐标映射（★ 2026-08-08 重写，旧实现缺 Pᵀ 相似变换导致轴向错乱）：
//    模型在机体系建造：+x 前 / +y 右 / +z 下（NED 机体轴）。
//    嵌套两个 Group：
//      groupBasis  —— 固定旋转 P（NED→Three 基变换，det=1 真旋转）
//                     Three_x=NED_y(右)  Three_y=-NED_z(上)  Three_z=-NED_x(前)
//      groupBody   —— 姿态四元数 q_ned（机体相对 NED 世界，标准 3-2-1）
//    顶点最终变换 = P · R_ned · v —— 数学上是严格的相似变换。
//    q_ned 由欧拉角合成：q = qz(ψ) ⊗ qy(θ) ⊗ qx(φ) ⇔ THREE Euler 'ZYX'。
// ============================================================
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { bus, state } from './main.mjs';

let scene, camera, renderer, controls;
let groupBasis, groupBody;
let running = false;
let pageVisible = true;        // 页面不可见时停止 WebGL 渲染（省 GPU/CPU）

// NED→Three 基变换 P（列 = NED 基向量在 Three 系中的像）
//   NED_x(前) → (0,0,-1)   NED_y(右) → (1,0,0)   NED_z(下) → (0,-1,0)
// 行列式 +1（真旋转，非镜像）。
const P_BASIS = new THREE.Matrix4().makeBasis(
  new THREE.Vector3(0, 0, -1),
  new THREE.Vector3(1, 0, 0),
  new THREE.Vector3(0, -1, 0),
);

export function activate() {
  if (running) return;
  running = true;
  const container = document.getElementById('v3dContainer');
  const W = container.clientWidth || 800;
  const H = container.clientHeight || 600;

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0b1019);

  camera = new THREE.PerspectiveCamera(45, W / H, 0.1, 200);
  camera.position.set(4.5, 3.0, 5.5);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(W, H);
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  container.appendChild(renderer.domElement);

  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.target.set(0, 0, 0);

  lights();
  buildAircraft();
  buildReference();
  window.addEventListener('resize', onResize);
  bus.addEventListener('telemetry', (e) => onTelemetry(e.detail));   // ★ CustomEvent，快照在 detail
  bus.addEventListener('page', onPage);
  animate();
}

function onPage(e) {
  pageVisible = (e.detail === 'view3d');
  if (pageVisible) syncPose();
}

function onResize() {
  const c = document.getElementById('v3dContainer');
  if (!c || !renderer) return;
  camera.aspect = c.clientWidth / c.clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(c.clientWidth, c.clientHeight);
}

function lights() {
  scene.add(new THREE.HemisphereLight(0xbcd8ff, 0x2a2018, 1.0));
  const dir = new THREE.DirectionalLight(0xffffff, 1.8);
  dir.position.set(6, 10, 4);
  scene.add(dir);
}

// 基础几何体姿态块（机体系：+x 前 / +y 右 / +z 下）
// 判读设计：长方体六面异色 + 棱线描边——
//   红=前(+x)  亮蓝=机背(-z,朝天面)  深灰=机腹(+z,朝地面)  青=右(+y)  暗青=左(-y)
// 任何姿态下"哪个面朝天、红面朝哪"一眼可读，无机型歧义。
function buildAircraft() {
  groupBasis = new THREE.Group();
  groupBasis.quaternion.setFromRotationMatrix(P_BASIS);   // 固定基变换
  groupBody = new THREE.Group();                          // 姿态驱动
  groupBasis.add(groupBody);
  scene.add(groupBasis);

  const face = (color, rough = 0.55) =>
    new THREE.MeshStandardMaterial({ color, roughness: rough, metalness: 0.1 });
  const mats = [
    face(0xff5d6c),   // +x 前（红）
    face(0x5a6472),   // -x 后（灰）
    face(0x29d3a2),   // +y 右（青）
    face(0x1d6b54),   // -y 左（暗青）
    face(0x3a3f47),   // +z 机腹（深灰，朝地）
    face(0x9cc8f5),   // -z 机背（亮蓝，朝天）
  ];
  const body = new THREE.Mesh(new THREE.BoxGeometry(1.8, 1.4, 0.5), mats);
  groupBody.add(body);

  // 棱线描边（任何光照下轮廓清晰）
  const edges = new THREE.LineSegments(
    new THREE.EdgesGeometry(body.geometry),
    new THREE.LineBasicMaterial({ color: 0x0b1019 }),
  );
  groupBody.add(edges);

  // 前向箭头（+x 伸出，偏航判读参照）
  const spike = new THREE.Mesh(new THREE.ConeGeometry(0.14, 0.6, 12), face(0xff5d6c, 0.4));
  spike.rotation.z = -Math.PI / 2;      // 锥尖默认 +y → +x
  spike.position.x = 1.2;
  groupBody.add(spike);

  // 机背立柱（-z 伸出，滚转/俯仰判读参照："柱子永远该朝天"）
  const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 0.8, 10), face(0xf2f6fc, 0.4));
  mast.rotation.x = Math.PI / 2;        // 圆柱默认 +y → -z（机背方向）
  mast.position.z = -0.6;
  const mastTip = new THREE.Mesh(new THREE.SphereGeometry(0.1, 12, 10), face(0xffffff, 0.3));
  mastTip.position.z = -1.0;
  groupBody.add(mast, mastTip);
}

// 地平参考：地面网格 + 天/地色球 + 指北箭头
function buildReference() {
  const grid = new THREE.GridHelper(24, 24, 0x3ea6ff, 0x22354c);
  grid.position.y = -2.0;
  scene.add(grid);

  // 地面（暗）与天空（亮雾球）——翻滚时不丢失上下参照
  const ground = new THREE.Mesh(new THREE.PlaneGeometry(60, 60),
    new THREE.MeshStandardMaterial({ color: 0x141a24, roughness: 1 }));
  ground.rotation.x = -Math.PI / 2;
  ground.position.y = -2.01;
  scene.add(ground);

  // 指北箭头（Three -z = NED +x = 北/前）
  const dir = new THREE.Vector3(0, 0, -1);
  const arrow = new THREE.ArrowHelper(dir, new THREE.Vector3(0, -1.9, 0), 3.5, 0xff5d6c, 0.5, 0.3);
  scene.add(arrow);
}

// 欧拉角（deg，固件 0x03 帧）→ NED 姿态四元数（标准 3-2-1：R = Rz(ψ)Ry(θ)Rx(φ)）
function eulerToQuatNed(rollDeg, pitchDeg, headingDeg) {
  const e = new THREE.Euler(
    THREE.MathUtils.degToRad(rollDeg),
    THREE.MathUtils.degToRad(pitchDeg),
    THREE.MathUtils.degToRad(headingDeg),
    'ZYX',   // q = qz ⊗ qy ⊗ qx —— 与标准 3-2-1 矩阵乘序一致
  );
  return new THREE.Quaternion().setFromEuler(e).normalize();
}

function onTelemetry(s) {
  if (s.roll_deg === undefined || s.roll_deg === null) return;
  groupBody.quaternion.copy(eulerToQuatNed(s.roll_deg, s.pitch_deg || 0, s.heading_deg || 0));
}

function syncPose() {
  const s = state.snap;
  if (s.roll_deg !== undefined && groupBody) {
    groupBody.quaternion.copy(eulerToQuatNed(s.roll_deg || 0, s.pitch_deg || 0, s.heading_deg || 0));
  }
}

function animate() {
  requestAnimationFrame(animate);
  if (!pageVisible) return;    // 切走页面后空转 rAF，不做 WebGL 渲染
  controls.update();
  renderer.render(scene, camera);
}
