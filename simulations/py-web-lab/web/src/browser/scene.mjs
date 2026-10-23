/**
 * @file    scene.mjs
 * @module  analysis/py-web-lab 渲染（场景/相机/后处理）
 * @brief   建立 WebGL 渲染器、场景、相机、灯光与辉光后处理管线。
 * @details 输入：DOM 容器；处理：初始化 Three.js 渲染链路与 resize；
 *          输出：{renderer, scene, camera, composer} 供 main.mjs 使用。
 * @theory  N/A（渲染层）
 * @frame   世界系 NED；相机为轨道控制（视觉观察，不参与姿态解算）
 * @units   渲染单位、像素
 * @note    ACES 色调映射 + UnrealBloom 为视觉增强，不改变任何物理量。
 * @see     main.mjs、theme.mjs
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { UnrealBloomPass } from 'three/addons/postprocessing/UnrealBloomPass.js';

/**
 * 初始化 WebGL 渲染链路（渲染器/场景/相机/灯光/辉光后处理）并处理窗口 resize。
 * @param {HTMLElement} container 渲染画布容器
 * @returns {Object} { renderer, scene, camera, composer }
 */
export function createScene(container) {
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.setSize(innerWidth, innerHeight);
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.22;
  container.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x272b33);
  scene.fog = new THREE.Fog(0x272b33, 30, 120);

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
  const amb = new THREE.AmbientLight(0xe8edf4, 1.3); scene.add(amb);
  const key = new THREE.DirectionalLight(0xffffff, 1.7);
  key.position.set(6, -8, -12); scene.add(key);
  const fill = new THREE.DirectionalLight(0xc9d4e4, 1.0);
  fill.position.set(-6, 8, -4); scene.add(fill);
  const rimCyan = new THREE.PointLight(0x38bdf8, 30, 40); rimCyan.position.set(-9, 7, 5); scene.add(rimCyan);
  const rimWarm = new THREE.PointLight(0xf59e0b, 26, 40); rimWarm.position.set(10, 6, 3); scene.add(rimWarm);

  // ---------- 辉光后处理 ----------
  const composer = new EffectComposer(renderer);
  composer.addPass(new RenderPass(scene, camera));
  const bloom = new UnrealBloomPass(new THREE.Vector2(innerWidth, innerHeight), 0.30, 0.45, 0.85);
  composer.addPass(bloom);

  addEventListener('resize', () => {
    camera.aspect = innerWidth / innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(innerWidth, innerHeight);
    composer.setSize(innerWidth, innerHeight);
  });

  return { renderer, scene, camera, controls, composer, bloom, lights: { amb, key, fill } };
}
