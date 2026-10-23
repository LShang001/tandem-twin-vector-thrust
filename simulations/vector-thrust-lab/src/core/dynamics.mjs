/**
 * @file    dynamics.mjs
 * @module  仿真 core/六自由度动力学
 * @brief   六自由度刚体动力学解算：子步调度 + RK4 联合积分 [v, ω, q] + 地面约束
 * @details 输入：状态 S/F、参数表、控制律输出（由 control/propulsion 产生）；
 *          处理：按 maxStep 拆子步 → 每子步一次 SAS/推进更新 → RK4 四阶段
 *          （气动逐阶段用中间状态重算）→ 位置积分 → 地面钳位；
 *          输出：更新 S.omega/S.quat 与 F.vel/vWorld/pos。
 *          不变量：子步 h ≤ maxStep；每阶段与终点都做四元数保范投影；
 *          推进力/转子角动量在子步内视为常数（气动则逐阶段重算）。
 * @theory  平动 m·v̇ = F_thrust + F_aero + m·g_b − m·(ω×v) @ref MOD-001 §5.1、@ref THY-003 §1.1；
 *          转动 I·ω̇ = M_thrust + M_aero − ω×(I·ω) − ω×h_rotor @ref MOD-001 §5.2、@ref THY-003 §1.2；
 *          姿态运动学 q̇ = ½·q⊗(0,ω_b) @ref MOD-001 §5.3、@ref THY-001 §7；
 *          转子角动量 h 定义 @ref MOD-001 §3.4
 * @frame   机体系 FRD（v/ω/力矩）→ 惯性系 NED（位置、地速、重力）
 * @units   SI：m、m/s、rad、rad/s、N、N·m、s
 * @redline 子步 ≤0.004 s、四元数 RK4 + 每阶段保范投影、地面 pos.z>6.2 钳位均为行为红线
 * @note 气动逐阶段重算、推进/角动量子步内恒定，是与 Python rk4_step 对齐的口径；
 *       改动任一侧都会改变回归基线
 * @see     propulsion.mjs、aerodynamics.mjs、control.mjs、tests/dynamics.test.mjs
 */
import { eulerFromQuat, quatMultiply, quatNormalize, quatInvert, rotateVecByQuat } from './math.mjs';
import { applySas } from './control.mjs';
import { stepPropulsion } from './propulsion.mjs';
import { computeAero, computeAeroState } from './aerodynamics.mjs';

// ── 子步调度 ──────────────────────────────────────────────────────
// 公式：n = max(1, ceil(dt/maxStep))，h = dt/n
// 符号：dt 为帧间隔（已由 main.js 截到 frameCap=0.05 s）；maxStep=0.004 s
// 来源：@ref MOD-001 §9（子步长 ≤4 ms 的数值理由：滚转气动阻尼时间常数 ~12 ms）
// 实现：`stepPhysics` 两行；余数均匀摊到 n 个子步，不做变步长
// 等价变换：无；ceil 保证 h≤maxStep（红线），代价是末子步略小于 h
// 数值证据：@test tests/dynamics.test.mjs「调度结果应与手动子步完全一致」
// ─────────────────────────────────────────────────────────────────
// @formula n = max(1, ceil(dt/maxStep))，h = dt/n，重复 n 次 physicsStep(h) @ref MOD-001 §9
// @note n 由 ceil 而非 round：宁可多算一步，也不让任何子步超过 4 ms（数值稳定红线）
// @test tests/dynamics.test.mjs「子步调度」（与手动 n 子步逐位相等）
/**
 * @brief 把一帧 dt 拆成 n 个 ≤maxStep 的子步并逐个子步推进
 * @param {Object} sim 仿真状态；@param {Object} P 参数表（maxStep）
 * @param {number} dt 帧间隔 [s]（0<dt≤frameCap=0.05）
 * @returns {void}
 */
export function stepPhysics(sim, P, dt) {
  const n = Math.max(1, Math.ceil(dt / P.maxStep));
  const h = dt / n;
  for (let i = 0; i < n; i++) physicsStep(sim, P, h);
}

// ── 单子步推进（控制一次 + RK4 积分 + 位置/地面约束）──────────────
// @formula v_world = R(q)·v_body；pos += v_world·h（水平可被 lockXY 清零）
// @ref MOD-001 §5（动力学）、MOD-002 §5.2（地面约束）
// @note 控制/推进每子步只算一次（不是每个 RK4 阶段）：执行器动态远慢于 4 ms；
//       气动则必须逐阶段重算，两者口径不可互换
// @note 地面约束只在"钻入地面且仍在向下"时触发——上升穿过 groundZ 不能被拦截
// @test tests/dynamics.test.mjs（归一化、触地、上升不拦截、lockXY）
/**
 * @brief 推进一个积分子步：控制量更新一次 → RK4 积分 → 位置积分与触地钳位
 * @param {Object} sim 仿真状态；@param {Object} P 参数表；@param {number} h 子步长 [s]
 * @returns {void}（更新 F.vel/vWorld/pos 与 S.omega/quat）
 */
export function physicsStep(sim, P, h) {
  const { S, F } = sim;

  // ---------- 姿态角（供 SAS 与 HUD） ----------
  const e = eulerFromQuat(S.quat);
  F.euler.x = e.phi; F.euler.y = e.theta; F.euler.z = e.psi;

  // ---------- SAS 增稳（每步一次，更新积分状态） ----------
  applySas(sim, P, h);

  // ---------- 动力装置（每步一次，更新电机状态） ----------
  const { cf, sf, ct, st } = stepPropulsion(sim, P, h);

  // ---------- 空气动力（遥测/子步初显示；RK4 内部另行逐阶段重算） ----------
  computeAero(sim, P);

  // ---------- 推进力/力矩（恒定通过本子步；精确公式见 propulsion.mjs） ----------
  const dyn = sim.dyn;
  const thrustForces = {
    Fx: dyn.Fx, Fy: dyn.Fy, Fz: dyn.Fz,
    Mx: dyn.Mx, My: dyn.My, Mz: dyn.Mz,
  };

  // ---------- 转子角动量向量（恒定通过本子步） ----------
  const hv = {
    x: P.Jp * (S.wf * cf - S.wt * ct),
    y: P.Jp * S.wf * sf,
    z: P.Jp * S.wt * st,
  };

  // ---------- RK4 联合积分 [v, ω, q]（气动逐阶段重算，见 rk4Step） ----------
  rk4Step(sim, P, h, thrustForces, hv);

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

// ── 状态导数（牛顿–欧拉 + 四元数运动学）────────────────────────────
// 公式：v̇ = (F_thrust + F_aero)/m + g_b − ω×v
//       ω̇ = I⁻¹·(M_thrust + M_aero − ω×(I·ω) − ω×h)
//       q̇ = ½·q⊗(0,ω)
// 符号：g_b = R(q)ᵀ·(0,0,g)（重力在机体系分量，NED 下 g_z 向下为正）；
//       h = Jp·[ωf·cf−ωt·ct, ωf·sf, ωt·st] 为双转子角动量（MOD-001 §3.4）
// 来源：@ref THY-003 §1.1–1.2（体轴系绝对导数展开）、@ref MOD-001 §5.1–5.3
// 实现：`stateDerivatives` 三段（gb/vDot、欧拉方程、qDot）
// 等价变换：教材在惯性系写 m·a=F，此处用动系公式引入 −ω×v 与 −ω×(I·ω)；
//           g_b 由 Rᵀ 而非 R 变换（重力在惯性系为常数矢量）
// 数值证据：@test tests/math.test.mjs「重力变换方向」、tests/dynamics.test.mjs 配平直飞
// ─────────────────────────────────────────────────────────────────
/**
 * @brief 计算状态导数 [v̇, ω̇, q̇]（气动由调用方按 RK4 阶段传入）
 * @param {{x:number,y:number,z:number}} v 机体系速度 [m/s]
 * @param {{x:number,y:number,z:number}} w 机体角速度 [rad/s]
 * @param {{x:number,y:number,z:number,w:number}} q 姿态四元数（体系→惯性系）
 * @param {Object} P 参数表（m/g/Ix/Iy/Iz）
 * @param {Object} thrust 推进力/力矩 {Fx,Fy,Fz,Mx,My,Mz} [N]/[N·m]
 * @param {Object} aero 气动力/力矩 {aX,Y,aZ,Mx,My,Mz} [N]/[N·m]
 * @param {{x:number,y:number,z:number}} hv 双转子角动量 [kg·m²/s]
 * @returns {{vDot:Object,wDot:Object,qDot:Object}} 三个导数
 * @formula 见上方五段式
 * @ref THY-003 §1；MOD-001 §5
 * @pre q 已归一化（否则 g_b 的模长会偏离 g）
 * @note 转动方程里 ω×(I·ω) 用主轴展开避免矩阵乘；hv 项符号为 −ω×h（转子对机体的反作用）
 * @note 本函数不修改入参，纯函数便于 RK4 反复调用
 */
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

// ── 经典四阶 Runge–Kutta（一步推进 [v, ω, q]）──────────────────────
// 公式：y₊ = y + h/6·(k₁ + 2k₂ + 2k₃ + k₄)，k₂/k₃ 取中点、k₄ 取终点
// 符号：y = [v, ω, q]；h 为子步长；每个 k 阶段先归一化 q 再求导
// 来源：RK4 标准形式；一阶积分漂移与归一化见 @ref THY-001 §9
// 实现：`rk4Step` 四个 stage + 加权组合；气动用 computeAeroState 逐阶段重算
// 等价变换：教材的四元数指数映射解 q(t+h)=q⊗Exp(ωh) 是另一种（无误差）积分；
//           本实现用 RK4 加权组合 + 每次保范投影，局部截断 O(h⁵)，工程上更省
// 数值证据：@test tests/dynamics.test.mjs（长时间积分 |q|−1 < 1e-9）
// ─────────────────────────────────────────────────────────────────
/**
 * @brief 用 RK4 推进一个子步的 [v, ω, q]（推进力/角动量在子步内恒定）
 * @param {Object} sim 仿真状态（就地写 F.vel、S.omega、S.quat）
 * @param {Object} P 参数表
 * @param {number} h 子步长 [s]
 * @param {Object} thrust 推进六维力/力矩（本子步恒定）
 * @param {{x:number,y:number,z:number}} hv 双转子角动量（本子步恒定）
 * @returns {void}
 * @formula y₊ = y + (h/6)(k₁+2k₂+2k₃+k₄)；中间阶段 q 归一化
 * @ref THY-001 §9（归一化的必要性）
 * @pre h>0；q 已归一化
 * @post |q| = 1（末端再投影一次）；v/ω 为一步末值
 * @note 与显式欧拉相比，RK4 的稳定域大得多——4 ms 子步下滚转通道（阻尼时间常数 ~12 ms）
 *       才不至于数值失稳（@ref MOD-001 §9）
 * @note 每阶段 q 归一化是"保范投影"红线的落地：中间点的 q 也必须落在单位球上，
 *       否则 k₄ 阶段的四元数误差会污染 ω̇ 的 g_b 项
 * @test tests/dynamics.test.mjs（600 帧机动后范数误差 <1e-9）
 */
function rk4Step(sim, P, h, thrust, hv) {
  const { S, F } = sim;
  const v0 = { x: F.vel.x, y: F.vel.y, z: F.vel.z };
  const w0 = { x: S.omega.x, y: S.omega.y, z: S.omega.z };
  const q0 = { x: S.quat.x, y: S.quat.y, z: S.quat.z, w: S.quat.w };
  const aeroOn = S.aero;

  // 阶段导数：气动用中间状态重算（wind-axes → 机体系）
  const f = (v, w, q) => {
    const a = computeAeroState(v, w, P, aeroOn);
    return stateDerivatives(v, w, q, P, thrust, a, hv);
  };

  // ---- Stage 1 ----
  const d1 = f(v0, w0, q0);

  // ---- Stage 2 (中点一次) ----
  const h2 = h * 0.5;
  const v1 = scaleAdd(v0, d1.vDot, h2);
  const w1 = scaleAdd(w0, d1.wDot, h2);
  const q1 = quatNormalize(scaleAddQuat(q0, d1.qDot, h2));
  const d2 = f(v1, w1, q1);

  // ---- Stage 3 (中点二次) ----
  const v2 = scaleAdd(v0, d2.vDot, h2);
  const w2 = scaleAdd(w0, d2.wDot, h2);
  const q2 = quatNormalize(scaleAddQuat(q0, d2.qDot, h2));
  const d3 = f(v2, w2, q2);

  // ---- Stage 4 (终点) ----
  const v3 = scaleAdd(v0, d3.vDot, h);
  const w3 = scaleAdd(w0, d3.wDot, h);
  const q3 = quatNormalize(scaleAddQuat(q0, d3.qDot, h));
  const d4 = f(v3, w3, q3);

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
