// ============================================================
//  六自由度刚体动力学解算
//  平动: m·v̇ = F推力 + F气动 + m·g − m·ω×v        （机体系）
//  转动: I·ω̇ = M推力 + M气动 − ω×(I·ω) − ω×h转子
// ============================================================
import { eulerFromQuat, quatMultiply, quatNormalize, quatInvert, rotateVecByQuat } from './math.mjs';
import { applySas } from './control.mjs';
import { stepPropulsion } from './propulsion.mjs';
import { computeAero } from './aerodynamics.mjs';

// 数值积分子步调度（滚转通道气动阻尼时间常数小, 显式积分需 ≤4ms）
export function stepPhysics(sim, P, dt) {
  const n = Math.max(1, Math.ceil(dt / P.maxStep));
  const h = dt / n;
  for (let i = 0; i < n; i++) physicsStep(sim, P, h);
}

export function physicsStep(sim, P, dt) {
  const { S, F, dyn, aero } = sim;

  // ---------- 姿态角（供 SAS 与 HUD） ----------
  const e = eulerFromQuat(S.quat);
  F.euler.x = e.phi; F.euler.y = e.theta; F.euler.z = e.psi;

  // ---------- SAS 增稳 ----------
  applySas(sim, P, dt);

  // ---------- 动力装置 ----------
  const { cf, sf, ct, st } = stepPropulsion(sim, P, dt);

  // ---------- 空气动力 ----------
  const { aX, Y, aZ } = computeAero(sim, P);

  // ---------- 重力在机体系中的分量 ----------
  const gb = rotateVecByQuat({ x: 0, y: 0, z: P.g }, quatInvert(S.quat));

  // ---------- 平动方程: v̇ = F/m − ω×v ----------
  const Fx = dyn.Fx + aX + P.m * gb.x;
  const Fy = dyn.Fy + Y + P.m * gb.y;
  const Fz = dyn.Fz + aZ + P.m * gb.z;
  const om = S.omega;
  const u = F.vel.x, v = F.vel.y, wv = F.vel.z;
  F.vel.x += (Fx / P.m - (om.y * wv - om.z * v)) * dt;
  F.vel.y += (Fy / P.m - (om.z * u - om.x * wv)) * dt;
  F.vel.z += (Fz / P.m - (om.x * v - om.y * u)) * dt;

  // ---------- 转动方程: I·ω̇ = M − ω×(I·ω) − ω×h转子 ----------
  const hv = {
    x: P.Jp * (S.wf * cf - S.wt * ct),   // 双转子角动量（前正后反, 沿摆座轴）
    y: P.Jp * S.wf * sf,
    z: P.Jp * S.wt * st,
  };
  const gx = (P.Iz - P.Iy) * om.y * om.z + (om.y * hv.z - om.z * hv.y);
  const gy = (P.Ix - P.Iz) * om.z * om.x + (om.z * hv.x - om.x * hv.z);
  const gz = (P.Iy - P.Ix) * om.x * om.y + (om.x * hv.y - om.y * hv.x);
  om.x += ((dyn.Mx + aero.Mx - gx) / P.Ix) * dt;
  om.y += ((dyn.My + aero.My - gy) / P.Iy) * dt;
  om.z += ((dyn.Mz + aero.Mz - gz) / P.Iz) * dt;

  // ---------- 姿态积分: q̇ = ½·q⊗ω_body（机体系角速度右乘） ----------
  const half = dt / 2;
  const q = S.quat;
  const wq = { x: half * om.x, y: half * om.y, z: half * om.z, w: 0 };
  const dq = quatMultiply(q, wq);
  const qn = quatNormalize({ x: q.x + dq.x, y: q.y + dq.y, z: q.z + dq.z, w: q.w + dq.w });
  S.quat.x = qn.x; S.quat.y = qn.y; S.quat.z = qn.z; S.quat.w = qn.w;

  // ---------- 位置积分（惯性系; 渲染采用载机跟随系） ----------
  const vw = rotateVecByQuat(F.vel, S.quat);
  F.vWorld.x = vw.x; F.vWorld.y = vw.y; F.vWorld.z = vw.z;
  F.pos.x += F.vWorld.x * dt;
  F.pos.y += F.vWorld.y * dt;
  F.pos.z += F.vWorld.z * dt;
  if (F.pos.z > P.groundZ && F.vWorld.z > 0) {   // 地面约束
    F.pos.z = P.groundZ; F.vWorld.z = 0;
    const vb = rotateVecByQuat(F.vWorld, quatInvert(S.quat));
    F.vel.x = vb.x; F.vel.y = vb.y; F.vel.z = vb.z;
  }
}
