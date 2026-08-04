// VTOL 悬停模式测试：四元数误差级联控制（control.mjs applyVtolHover）
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { P } from '../src/core/parameters.mjs';
import {
  createSimulationState, resetSimulationState, resetVtolHoverState,
  Q_HOVER, hoverThrottle,
} from '../src/core/state.mjs';
import { applySas } from '../src/core/control.mjs';
import { stepPhysics } from '../src/core/dynamics.mjs';
import { quatMultiply, quatInvert, quatNormalize, rotateVecByQuat, eulerFromQuat, quat } from '../src/core/math.mjs';

const FRAME_DT = 1 / 60;

// 当前相对目标的角度误差（rad），夹 0..π
function quatErrAngle(q, qCmd) {
  const qe = quatNormalize(quatMultiply(q, quatInvert(qCmd)));
  return 2 * Math.acos(Math.min(1, Math.abs(qe.w)));
}

// 悬停模拟：重置到悬停态 → 可选手动扰动 → 按帧步进
function runHover(durationS, perturb = () => {}, control = () => {}) {
  const sim = createSimulationState(P);
  resetVtolHoverState(sim, P);
  perturb(sim);
  const steps = Math.round(durationS / FRAME_DT);
  for (let i = 0; i < steps; i++) {
    control(sim, i);
    sim.S.time += FRAME_DT;
    stepPhysics(sim, P, FRAME_DT);
  }
  return sim;
}

// ---------- 初始化与派生量 ----------

test('悬停配平油门：双发推力恰好平衡重力', () => {
  const thr = hoverThrottle(P);
  const T2 = 2 * P.kT * (thr * P.wMax) ** 2;   // 双发总推力
  assert.ok(Math.abs(T2 - P.m * P.g) / (P.m * P.g) < 1e-9,
    `thr=${thr} 双发推力 ${T2.toFixed(4)}N ≉ 重力 ${(P.m * P.g).toFixed(4)}N`);
  assert.ok(thr > 0.4 && thr < 0.6, `悬停油门应接近 50%（实际 ${thr.toFixed(3)}）`);
});

test('Q_HOVER：机头朝天（x_b → NED −z），显示约定 theta=−90°', () => {
  const xb = rotateVecByQuat({ x: 1, y: 0, z: 0 }, Q_HOVER);
  assert.ok(Math.abs(xb.x) < 1e-12 && Math.abs(xb.y) < 1e-12 && Math.abs(xb.z + 1) < 1e-12,
    `x_b = (${xb.x.toFixed(3)}, ${xb.y.toFixed(3)}, ${xb.z.toFixed(3)}) 应为 (0,0,−1)`);
  const e = eulerFromQuat(Q_HOVER);
  assert.ok(Math.abs(e.theta * 180 / Math.PI + 90) < 1e-9, `显示 theta=${(e.theta * 180 / Math.PI).toFixed(1)}° 应为 −90°`);
});

test('resetVtolHoverState：无翼默认（aero=false）、速度/角速度/积分清零、摆角基准 0', () => {
  const sim = createSimulationState(P);
  // 先制造污染状态再切换
  sim.S.thr = 0.9; sim.S.omega.y = 5; sim.S.intTh = 0.5;
  sim.F.vel.x = 20; sim.F.vWorld.z = 30; sim.F.pos.z = 100;
  resetVtolHoverState(sim, P);
  const { S, F } = sim;
  assert.equal(S.aero, false, '悬停模式默认关掉机翼（气动力忽略）');
  assert.equal(S.lockXY, false);
  assert.ok(Math.abs(S.thr - hoverThrottle(P)) < 1e-12, '油门 = 悬停配平');
  assert.equal(S.dtAct, 0); assert.equal(S.dfAct, 0); assert.equal(S.dwAct, 0);
  assert.equal(S.intTh, 0); assert.equal(S.intPhi, 0);
  assert.equal(S.omega.x + S.omega.y + S.omega.z, 0);
  assert.equal(F.vel.x + F.vel.y + F.vel.z, 0);
  assert.equal(F.vWorld.x + F.vWorld.y + F.vWorld.z, 0);
  assert.equal(F.pos.z, 0);
  assert.ok(Math.abs(S.quat.x - Q_HOVER.x) < 1e-12 && Math.abs(S.quat.w - Q_HOVER.w) < 1e-12, '姿态 = Q_HOVER');
});

// ---------- 控制符号（数值验证，与 propulsion.mjs 效率符号交叉一致） ----------

test('控制符号：qe.y>0（绕 y_b 超转）→ dtAct>0 → My<0 → 误差收敛方向', () => {
  const sim = createSimulationState(P);
  resetVtolHoverState(sim, P);
  // 当前相对目标绕机体 y 超前 +0.1 rad
  sim.S.quat = quatNormalize(quatMultiply(Q_HOVER,
    quat(0, Math.sin(0.05), 0, Math.cos(0.05))));
  sim.F.euler = eulerFromQuat(sim.S.quat);
  applySas(sim, P, 0.004);
  assert.ok(sim.S.dtAct > 0, `dtAct=${sim.S.dtAct} 应为正（产生 My<0 低头修正）`);
  // 力矩符号链：δt>0 → My = −b·Tt·sinδt < 0（与 propulsion.mjs 一致）
  const Tt = P.kT * sim.S.wt * sim.S.wt;
  const My = -P.b * Tt * Math.sin(sim.S.dtAct);
  assert.ok(My < 0, `My=${My.toFixed(4)} 应为负`);
});

test('控制符号：qe.x>0（绕 x_b 超转=航向偏移）→ dwAct>0 → Mx<0 → 收敛方向', () => {
  const sim = createSimulationState(P);
  resetVtolHoverState(sim, P);
  sim.S.quat = quatNormalize(quatMultiply(Q_HOVER,
    quat(Math.sin(0.05), 0, 0, Math.cos(0.05))));
  sim.F.euler = eulerFromQuat(sim.S.quat);
  applySas(sim, P, 0.004);
  assert.ok(sim.S.dwAct > 0, `dwAct=${sim.S.dwAct} 应为正（产生 Mx<0 航向修正）`);
});

test('控制符号：qe.z>0（绕 z_b 超转=侧倾）→ dfAct<0 → Mz<0 → 收敛方向', () => {
  const sim = createSimulationState(P);
  resetVtolHoverState(sim, P);
  sim.S.quat = quatNormalize(quatMultiply(Q_HOVER,
    quat(0, 0, Math.sin(0.05), Math.cos(0.05))));
  sim.F.euler = eulerFromQuat(sim.S.quat);
  applySas(sim, P, 0.004);
  assert.ok(sim.S.dfAct < 0, `dfAct=${sim.S.dfAct} 应为负（产生 Mz<0 侧倾修正）`);
});

test('悬停控制无巡航配平偏置：零误差时摆角指令为 0（非 dtTrim/dfTrim）', () => {
  const sim = createSimulationState(P);
  resetVtolHoverState(sim, P);
  sim.F.euler = eulerFromQuat(sim.S.quat);
  applySas(sim, P, 0.004);
  assert.ok(Math.abs(sim.S.dtAct) < 1e-9, `dtAct=${sim.S.dtAct} 应为 0（悬停基准无配平偏置）`);
  assert.ok(Math.abs(sim.S.dfAct) < 1e-9);
  assert.ok(Math.abs(sim.S.dwAct) < 1e-9);
  assert.notEqual(P.dtTrim, 0, '前置：巡航配平偏置确实非零，基准差异有意义');
});

// ---------- 闭环行为 ----------

test('悬停自稳：初始零扰动 10s 保持姿态（无翼、气动关）', () => {
  const sim = runHover(10);
  const err = quatErrAngle(sim.S.quat, Q_HOVER);
  assert.ok(err < 0.03, `10s 后姿态误差 ${(err * 57.3).toFixed(2)}° 应 < 1.7°`);
  assert.ok(Math.hypot(sim.S.omega.x, sim.S.omega.y, sim.S.omega.z) < 0.1, '角速度应衰减');
});

test('悬停自稳：俯仰角速度扰动 0.3 rad/s 后 8s 收敛', () => {
  const sim = runHover(8, (s) => { s.S.omega.y = 0.3; });
  const err = quatErrAngle(sim.S.quat, Q_HOVER);
  assert.ok(err < 0.05, `扰动后姿态误差 ${(err * 57.3).toFixed(2)}° 应 < 2.9°`);
  assert.ok(Math.abs(sim.S.omega.y) < 0.15, `残余角速度 ${sim.S.omega.y.toFixed(3)} rad/s 应小`);
});

test('悬停自稳：航向指令 dw=20° 后 8s 收敛到目标航向（绕 x_b=世界竖直轴）', () => {
  const psiCmd = 20 * Math.PI / 180;
  const qCmd = quatNormalize(quatMultiply(Q_HOVER,
    quat(Math.sin(psiCmd / 2), 0, 0, Math.cos(psiCmd / 2))));
  const sim = runHover(8, (s) => { s.S.dw = psiCmd; });
  const err = quatErrAngle(sim.S.quat, qCmd);
  assert.ok(err < 0.05, `航向指令后误差 ${(err * 57.3).toFixed(2)}° 应 < 2.9°`);
  // 机体 x 轴应保持竖直（旋转轴不变）；航向用 y_b 水平投影指示：
  // y_b = R(Q_HOVER)·Rx(ψ)·ŷ = (sinψ, cosψ, 0) → atan2(yb.y, yb.x) = π/2 − ψ
  const xb = rotateVecByQuat({ x: 1, y: 0, z: 0 }, sim.S.quat);
  const yb = rotateVecByQuat({ x: 0, y: 1, z: 0 }, sim.S.quat);
  assert.ok(Math.abs(xb.z + 1) < 0.05, `x_b 应保持竖直（z=${xb.z.toFixed(3)}）`);
  assert.ok(Math.abs(Math.atan2(yb.y, yb.x) - (Math.PI / 2 - psiCmd)) < 0.08,
    `y_b 水平投影角 ${Math.atan2(yb.y, yb.x).toFixed(3)} rad 应 ≈ π/2 − ψ`);
});

test('悬停自稳：俯仰倾斜指令 dt=10° 后 8s 收敛（绕 y_b 倾斜产生水平推力）', () => {
  const thCmd = 10 * Math.PI / 180;
  const qCmd = quatNormalize(quatMultiply(Q_HOVER,
    quat(0, Math.sin(thCmd / 2), 0, Math.cos(thCmd / 2))));
  const sim = runHover(8, (s) => { s.S.dt = thCmd; });
  const err = quatErrAngle(sim.S.quat, qCmd);
  assert.ok(err < 0.05, `倾斜指令后误差 ${(err * 57.3).toFixed(2)}° 应 < 2.9°`);
  // 倾斜后应有水平推力（F 机体系 x 分量）与水平速度漂移
  assert.ok(Math.hypot(sim.F.vWorld.x, sim.F.vWorld.y) > 0.05,
    `倾斜后应有水平漂移（vxy=${Math.hypot(sim.F.vWorld.x, sim.F.vWorld.y).toFixed(2)} m/s）`);
});

test('悬停无自稳（sasMode=0）：扰动不收敛（对比基线）', () => {
  const sim = runHover(6,
    (s) => { s.S.omega.y = 0.5; },
    (s) => { s.S.sasMode = 0; s.S.dt = 0; s.S.df = 0; s.S.dw = 0; });
  const err = quatErrAngle(sim.S.quat, Q_HOVER);
  assert.ok(err > 0.2, `无自稳 6s 后姿态误差 ${(err * 57.3).toFixed(1)}° 应显著（>11°）`);
});

test('切换一致性：vtolMode=true 时固定翼 SAS 分支不介入（积分器不累积）', () => {
  const sim = createSimulationState(P);
  resetVtolHoverState(sim, P);
  for (let i = 0; i < 100; i++) applySas(sim, P, 0.004);
  assert.equal(sim.S.intTh, 0, '悬停控制不使用固定翼积分器');
  assert.equal(sim.S.intPhi, 0);
});

test('模式往返切换：悬停→巡航→悬停 状态完全一致（vtolMode/aero/油门/摆角基准）', () => {
  const sim = createSimulationState(P);
  resetVtolHoverState(sim, P);
  sim.S.thr = 0.8; sim.S.aero = true;       // 制造中间污染
  resetSimulationState(sim, P);             // 切回巡航
  assert.equal(sim.S.vtolMode, false, '巡航复位应清除 vtolMode');
  assert.equal(sim.S.thr, P.thrTrim, '巡航油门 = 配平油门');
  assert.ok(Math.abs(sim.S.quat.y - Math.sin(P.aTrim / 2)) < 1e-9, '巡航姿态 = 配平迎角');
  resetVtolHoverState(sim, P);              // 再进悬停
  assert.equal(sim.S.vtolMode, true);
  assert.equal(sim.S.aero, false, '再进悬停仍默认无翼');
  assert.equal(sim.S.thr, hoverThrottle(P));
  assert.equal(sim.S.dtAct, 0, '悬停摆角基准 0');
  assert.equal(sim.S.intTh, 0, '积分器清零');
});

test('悬停最短路径分支：qe.w<0（误差 >180°）时取反走短弧', () => {
  const sim = createSimulationState(P);
  resetVtolHoverState(sim, P);
  // 当前相对目标绕机体 y 超前 200°（qe.w = cos(100°) < 0）。
  // 四元数双覆盖：Ry(200°)⁻¹ ≡ Ry(+160°) —— 短弧是正转 160°（160°<200°）
  const th = 200 * Math.PI / 180;
  sim.S.quat = quatNormalize(quatMultiply(Q_HOVER,
    quat(0, Math.sin(th / 2), 0, Math.cos(th / 2))));
  sim.F.euler = eulerFromQuat(sim.S.quat);
  applySas(sim, P, 0.004);
  // 短弧 +160°：需要 q̇>0 → My>0 → δt<0 → dtAct<0
  assert.ok(sim.S.dtAct < 0, `dtAct=${sim.S.dtAct} 应为负（200° 误差走 +160° 短弧）`);
  assert.ok(Math.abs(sim.S.dtAct) <= P.dMax);
});

test('悬停直通模式（sasMode=0）：滑块摆角/差速直接执行，无配平偏置、无修正', () => {
  const sim = createSimulationState(P);
  resetVtolHoverState(sim, P);
  sim.S.sasMode = 0;
  sim.S.dt = 0.1; sim.S.df = -0.05; sim.S.dw = 0.3;
  sim.S.omega.y = 5; sim.S.omega.x = 5;    // 扰动不应进入输出
  applySas(sim, P, 0.004);
  assert.equal(sim.S.dtAct, 0.1, '直通：dtAct = 滑块指令（无 dtTrim 偏置）');
  assert.equal(sim.S.dfAct, -0.05, '直通：dfAct = 滑块指令（无 dfTrim 偏置）');
  assert.equal(sim.S.dwAct, 0.3, '直通：dwAct = 滑块指令');
});
