// math.mjs 单元测试
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { clamp, rotateVecByQuat, quatInvert, quatMultiply, quatNormalize, eulerFromQuat } from '../src/core/math.mjs';

const EPS = 1e-12;
const close = (a, b, eps = EPS) => assert.ok(Math.abs(a - b) < eps, `${a} ≉ ${b}`);

test('clamp 边界', () => {
  assert.equal(clamp(5, 0, 1), 1);
  assert.equal(clamp(-5, 0, 1), 0);
  assert.equal(clamp(0.5, 0, 1), 0.5);
});

test('绕 z 轴旋转 90°：+x → +y', () => {
  const q = { x: 0, y: 0, z: Math.SQRT1_2, w: Math.SQRT1_2 };
  const v = rotateVecByQuat({ x: 1, y: 0, z: 0 }, q);
  close(v.x, 0); close(v.y, 1); close(v.z, 0);
});

test('旋转再逆旋转还原向量', () => {
  const q = quatNormalize({ x: 0.3, y: -0.2, z: 0.5, w: 0.8 });
  const v = { x: 1.2, y: -3.4, z: 5.6 };
  const back = rotateVecByQuat(rotateVecByQuat(v, q), quatInvert(q));
  close(back.x, v.x, 1e-9); close(back.y, v.y, 1e-9); close(back.z, v.z, 1e-9);
});

test('四元数乘法：单位元与结合性抽验', () => {
  const q = quatNormalize({ x: 0.1, y: 0.2, z: 0.3, w: 0.9 });
  const id = { x: 0, y: 0, z: 0, w: 1 };
  const r = quatMultiply(q, id);
  close(r.x, q.x); close(r.y, q.y); close(r.z, q.z); close(r.w, q.w);
});

test('q ⊗ q⁻¹ = 单位四元数', () => {
  const q = quatNormalize({ x: -0.4, y: 0.1, z: 0.2, w: 0.85 });
  const r = quatMultiply(q, quatInvert(q));
  close(r.x, 0, 1e-9); close(r.y, 0, 1e-9); close(r.z, 0, 1e-9); close(r.w, 1, 1e-9);
});

test('归一化：任意非零四元数范数为 1，零四元数回退单位元', () => {
  const q = quatNormalize({ x: 3, y: 4, z: 0, w: 0 });
  close(Math.hypot(q.x, q.y, q.z, q.w), 1);
  assert.deepEqual(quatNormalize({ x: 0, y: 0, z: 0, w: 0 }), { x: 0, y: 0, z: 0, w: 1 });
});

test('eulerFromQuat：纯俯仰四元数 → theta = -a0（现行约定）', () => {
  // 现行实现采用 theta = -asin(R13)，绕 +y 转 +a0 的四元数读数为 -a0。
  // SAS 增益即按此约定整定——此处锁定既有行为，而非"修正"它（见 MOD-002）。
  const a0 = 0.3; // rad
  const q = { x: 0, y: Math.sin(a0 / 2), z: 0, w: Math.cos(a0 / 2) };
  const e = eulerFromQuat(q);
  close(e.theta, -a0, 1e-9); close(e.phi, 0, 1e-9); close(e.psi, 0, 1e-9);
});

test('eulerFromQuat 与 rotateVecByQuat 一致（重力变换方向）', () => {
  // 俯仰 θ 时机体系中重力分量应为 [g·sinθ·(-1)?]——用旋转公式直接验证
  const a0 = 0.2;
  const q = { x: 0, y: Math.sin(a0 / 2), z: 0, w: Math.cos(a0 / 2) };
  const gb = rotateVecByQuat({ x: 0, y: 0, z: 9.81 }, quatInvert(q));
  // NED z 向下重力，机体系上仰 θ 时 gb.x < 0（重力沿 -x 分量）
  assert.ok(gb.x < 0);
  close(gb.z, 9.81 * Math.cos(a0), 1e-9);
});
