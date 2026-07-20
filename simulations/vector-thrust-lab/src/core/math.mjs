// ============================================================
//  纯 JavaScript 数学工具 —— core 层禁止依赖 Three.js
//  运算语义与 THREE.Vector3 / Quaternion / Matrix4 严格一致
// ============================================================

export const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));

export const vec3 = (x = 0, y = 0, z = 0) => ({ x, y, z });
export const quat = (x = 0, y = 0, z = 0, w = 1) => ({ x, y, z, w });

// v' = q ⊗ v ⊗ q*（THREE.Vector3.applyQuaternion 同款公式）
export function rotateVecByQuat(v, q) {
  const { x: vx, y: vy, z: vz } = v;
  const { x: qx, y: qy, z: qz, w: qw } = q;
  // t = 2 × cross(q.xyz, v)
  const tx = 2 * (qy * vz - qz * vy);
  const ty = 2 * (qz * vx - qx * vz);
  const tz = 2 * (qx * vy - qy * vx);
  return {
    x: vx + qw * tx + qy * tz - qz * ty,
    y: vy + qw * ty + qz * tx - qx * tz,
    z: vz + qw * tz + qx * ty - qy * tx,
  };
}

// 单位四元数的逆 = 共轭（与 THREE.Quaternion.invert 一致）
export const quatInvert = (q) => ({ x: -q.x, y: -q.y, z: -q.z, w: q.w });

// a ⊗ b（THREE.Quaternion.multiply 的分量顺序）
export function quatMultiply(a, b) {
  return {
    x: a.x * b.w + a.w * b.x + a.y * b.z - a.z * b.y,
    y: a.y * b.w + a.w * b.y + a.z * b.x - a.x * b.z,
    z: a.z * b.w + a.w * b.z + a.x * b.y - a.y * b.x,
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  };
}

export function quatNormalize(q) {
  const l = Math.hypot(q.x, q.y, q.z, q.w);
  if (l === 0) return { x: 0, y: 0, z: 0, w: 1 };
  return { x: q.x / l, y: q.y / l, z: q.z / l, w: q.w / l };
}

// 由单位四元数提取 3-2-1 欧拉角 {phi, theta, psi}
// 与 THREE.Matrix4.makeRotationFromQuaternion 列主序元素
// e[8]=m13, e[9]=m23, e[10]=m33, e[4]=m12, e[0]=m11 的用法一致
export function eulerFromQuat(q) {
  const { x, y, z, w } = q;
  const x2 = x + x, y2 = y + y, z2 = z + z;
  const xx = x * x2, xy = x * y2, xz = x * z2;
  const yy = y * y2, yz = y * z2, zz = z * z2;
  const wx = w * x2, wy = w * y2, wz = w * z2;
  const m11 = 1 - (yy + zz);
  const m12 = xy - wz;
  const m13 = xz + wy;
  const m23 = yz - wx;
  const m33 = 1 - (xx + yy);
  return {
    phi: Math.atan2(m23, m33),
    theta: -Math.asin(clamp(m13, -1, 1)),
    psi: Math.atan2(m12, m11),
  };
}
