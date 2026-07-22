// ============================================================
//  六自由度刚体动力学解算 — RK4 高精度积分
//  平动: m·v̇ = F推力 + F气动 + m·g − m·ω×v        （机体系）
//  转动: I·ω̇ = M推力 + M气动 − ω×(I·ω) − ω×h转子
//  姿态: q̇  = ½·q ⊗ [0, ω]^T                        （四元数运动学）
//
//  方法：经典四阶显式 Runge-Kutta (RK4)，联合积分 [v, ω, q]
//  O(h⁵) 局部截断，每子步保范投影消除浮点漂移
// ============================================================
import { eulerFromQuat, quatMultiply, quatNormalize, quatInvert, rotateVecByQuat } from './math.mjs';
import { applySas } from './control.mjs';
import { stepPropulsion } from './propulsion.mjs';
import { computeAero } from './aerodynamics.mjs';

// ============================================================
//  RK4 子步调度
// ============================================================
export function stepPhysics(sim, P, dt) {
  const n = Math.max(1, Math.ceil(dt / P.maxStep));
  const h = dt / n;
  for (let i = 0; i < n; i++) physicsStep(sim, P, h);
}

// ============================================================
//  单步推进：控制量计算（一次）→ RK4 状态积分
// ============================================================
export function physicsStep(sim, P, h) {
  const { S, F } = sim;

  // ---------- 姿态角（供 SAS 与 HUD） ----------
  const e = eulerFromQuat(S.quat);
  F.euler.x = e.phi; F.euler.y = e.theta; F.euler.z = e.psi;

  // ---------- SAS 增稳（每步一次，更新积分状态） ----------
  applySas(sim, P, h);

  // ---------- 动力装置（每步一次，更新电机状态） ----------
  const { cf, sf, ct, st } = stepPropulsion(sim, P, h);

  // ---------- 空气动力 ----------
  const { aX, Y, aZ } = computeAero(sim, P);

  // ---------- 推进力/力矩（恒定通过本子步；精确公式见 propulsion.mjs） ----------
  const dyn = sim.dyn;
  const aero = sim.aero;
  const thrustForces = {
    Fx: dyn.Fx, Fy: dyn.Fy, Fz: dyn.Fz,
    Mx: dyn.Mx, My: dyn.My, Mz: dyn.Mz,
  };
  const aeroForces = {
    aX, Y, aZ,
    Mx: aero.Mx, My: aero.My, Mz: aero.Mz,
  };

  // ---------- 转子角动量向量（恒定通过本子步） ----------
  const hv = {
    x: P.Jp * (S.wf * cf - S.wt * ct),
    y: P.Jp * S.wf * sf,
    z: P.Jp * S.wt * st,
  };

  // ---------- RK4 联合积分 [v, ω, q] ----------
  rk4Step(sim, P, h, thrustForces, aeroForces, hv);

  // ---------- 位置积分（惯性系；渲染采用载机跟随系） ----------
  const vw = rotateVecByQuat(F.vel, S.quat);
  F.vWorld.x = vw.x; F.vWorld.y = vw.y; F.vWorld.z = vw.z;
  if (S.lockXY) {
    F.vWorld.x = 0; F.vWorld.y = 0;
    const vb = rotateVecByQuat(F.vWorld, quatInvert(S.quat));
    F.vel.x = vb.x; F.vel.y = vb.y; F.vel.z = vb.z;
  } else {
    F.pos.x += F.vWorld.x * h;
    F.pos.y += F.vWorld.y * h;
  }
  F.pos.z += F.vWorld.z * h;
  if (F.pos.z > P.groundZ && F.vWorld.z > 0) {
    F.pos.z = P.groundZ; F.vWorld.z = 0;
    const vb = rotateVecByQuat(F.vWorld, quatInvert(S.quat));
    F.vel.x = vb.x; F.vel.y = vb.y; F.vel.z = vb.z;
  }
}

// ============================================================
//  状态导数: f([v, ω, q], u) → [v̇, ω̇, q̇]
// ============================================================
function stateDerivatives(v, w, q, P, thrust, aero, hv) {
  // ---------- 重力在机体系中的分量（q 的函数） ----------
  const gb = rotateVecByQuat({ x: 0, y: 0, z: P.g }, quatInvert(q));

  // ---------- 平动导数: v̇ = F/m − ω×v ----------
  const Fx = thrust.Fx + aero.aX + P.m * gb.x;
  const Fy = thrust.Fy + aero.Y  + P.m * gb.y;
  const Fz = thrust.Fz + aero.aZ + P.m * gb.z;
  const vDot = {
    x: Fx / P.m - (w.y * v.z - w.z * v.y),
    y: Fy / P.m - (w.z * v.x - w.x * v.z),
    z: Fz / P.m - (w.x * v.y - w.y * v.x),
  };

  // ---------- 转动导数: I·ω̇ = M − ω×(I·ω) − ω×h ----------
  const gx = (P.Iz - P.Iy) * w.y * w.z + (w.y * hv.z - w.z * hv.y);
  const gy = (P.Ix - P.Iz) * w.z * w.x + (w.z * hv.x - w.x * hv.z);
  const gz = (P.Iy - P.Ix) * w.x * w.y + (w.x * hv.y - w.y * hv.x);
  const wDot = {
    x: (thrust.Mx + aero.Mx - gx) / P.Ix,
    y: (thrust.My + aero.My - gy) / P.Iy,
    z: (thrust.Mz + aero.Mz - gz) / P.Iz,
  };

  // ---------- 四元数运动学: q̇ = ½·q ⊗ [0, ω] ----------
  const hw = { x: 0.5 * w.x, y: 0.5 * w.y, z: 0.5 * w.z, w: 0 };
  const qDot = quatMultiply(q, hw);

  return { vDot, wDot, qDot };
}

// ============================================================
//  经典四阶 Runge-Kutta (RK4) — 一步推进 [v, ω, q]
//  O(h⁵) 局部截断, 稳定域显著优于显式欧拉
// ============================================================
function rk4Step(sim, P, h, thrust, aero, hv) {
  const { S, F } = sim;
  const v0 = { x: F.vel.x, y: F.vel.y, z: F.vel.z };
  const w0 = { x: S.omega.x, y: S.omega.y, z: S.omega.z };
  const q0 = { x: S.quat.x, y: S.quat.y, z: S.quat.z, w: S.quat.w };

  // ---- Stage 1 ----
  const d1 = stateDerivatives(v0, w0, q0, P, thrust, aero, hv);

  // ---- Stage 2 (中点一次) ----
  const h2 = h * 0.5;
  const v1 = scaleAdd(v0, d1.vDot, h2);
  const w1 = scaleAdd(w0, d1.wDot, h2);
  const q1 = quatNormalize(scaleAddQuat(q0, d1.qDot, h2));
  const d2 = stateDerivatives(v1, w1, q1, P, thrust, aero, hv);

  // ---- Stage 3 (中点二次) ----
  const v2 = scaleAdd(v0, d2.vDot, h2);
  const w2 = scaleAdd(w0, d2.wDot, h2);
  const q2 = quatNormalize(scaleAddQuat(q0, d2.qDot, h2));
  const d3 = stateDerivatives(v2, w2, q2, P, thrust, aero, hv);

  // ---- Stage 4 (终点) ----
  const v3 = scaleAdd(v0, d3.vDot, h);
  const w3 = scaleAdd(w0, d3.wDot, h);
  const q3 = quatNormalize(scaleAddQuat(q0, d3.qDot, h));
  const d4 = stateDerivatives(v3, w3, q3, P, thrust, aero, hv);

  // ---- RK4 加权组合 ----
  const h6 = h / 6;
  F.vel.x  = v0.x + h6 * (d1.vDot.x + 2*d2.vDot.x + 2*d3.vDot.x + d4.vDot.x);
  F.vel.y  = v0.y + h6 * (d1.vDot.y + 2*d2.vDot.y + 2*d3.vDot.y + d4.vDot.y);
  F.vel.z  = v0.z + h6 * (d1.vDot.z + 2*d2.vDot.z + 2*d3.vDot.z + d4.vDot.z);

  S.omega.x = w0.x + h6 * (d1.wDot.x + 2*d2.wDot.x + 2*d3.wDot.x + d4.wDot.x);
  S.omega.y = w0.y + h6 * (d1.wDot.y + 2*d2.wDot.y + 2*d3.wDot.y + d4.wDot.y);
  S.omega.z = w0.z + h6 * (d1.wDot.z + 2*d2.wDot.z + 2*d3.wDot.z + d4.wDot.z);

  const qu = quatNormalize({
    x: q0.x + h6 * (d1.qDot.x + 2*d2.qDot.x + 2*d3.qDot.x + d4.qDot.x),
    y: q0.y + h6 * (d1.qDot.y + 2*d2.qDot.y + 2*d3.qDot.y + d4.qDot.y),
    z: q0.z + h6 * (d1.qDot.z + 2*d2.qDot.z + 2*d3.qDot.z + d4.qDot.z),
    w: q0.w + h6 * (d1.qDot.w + 2*d2.qDot.w + 2*d3.qDot.w + d4.qDot.w),
  });
  S.quat.x = qu.x; S.quat.y = qu.y; S.quat.z = qu.z; S.quat.w = qu.w;
}

// ============================================================
//  向量标量乘加: r = a + s·b
// ============================================================
function scaleAdd(a, b, s) {
  return { x: a.x + s * b.x, y: a.y + s * b.y, z: a.z + s * b.z };
}

// ============================================================
//  四元数标量乘加（用于 RK4 中间点）: r = normalize(q + s·dq)
// ============================================================
function scaleAddQuat(q, dq, s) {
  return quatNormalize({
    x: q.x + s * dq.x, y: q.y + s * dq.y,
    z: q.z + s * dq.z, w: q.w + s * dq.w,
  });
}
