// ============================================================
//  仿真状态：控制/执行状态 S、飞行状态 F、遥测 dyn/aero
// ============================================================
import { vec3, quat } from './math.mjs';

// 悬停初始姿态：绕 NED y 转 +90°（机头朝天，x_b → NED −z）
// 验证：rotateVecByQuat((1,0,0), Q_HOVER) = (0,0,−1)；显示约定下 theta = −90°
export const Q_HOVER = quat(0, Math.SQRT1_2, 0, Math.SQRT1_2);

// 悬停配平油门（物理派生量，非独立参数）：
// 双发推力平衡重力：2·kT·(thr·wMax)² = m·g → thr = √(m·g / (2·kT·wMax²))
export function hoverThrottle(P) {
  return Math.sqrt(P.m * P.g / (2 * P.kT * P.wMax * P.wMax));
}

export function createSimulationState(P) {
  const sim = {
    // ---- 运行状态 ----
    S: {
      thr: P.thrTrim,      // 总油门 0..1
      df: 0,               // 前摆角指令（偏航, 绕 z）
      dt: 0,               // 尾摆增量指令（俯仰, 绕 y；实际值含 dtTrim）
      dw: 0,               // 差速指令 -1..1
      dtAct: 0, dfAct: 0, dwAct: 0,   // SAS 修正后的实际执行量
      sasMode: 1,          // SAS 模式: 0=关, 1=全SAS, 2=仅角速率阻尼, 3=角速度闭环(滑块=ω_ref)
      _prevSasMode: 1,     // 上一 SAS 模式（供 control.mjs 检测模式切换）
      aero: true,          // 气动力开关（false = 仅电机推力）
      lockXY: false,       // 水平运动学约束（true = 惯性系水平速度持续清零）
      vtolMode: false,     // 构型模式：false=固定翼巡航 / true=VTOL 悬停（机头朝天）
      useBtrue: false,     // B_true 在线 Jacobian 增量分配（仅悬停自稳模式生效）
      altHold: false,      // 定高开关（仅 VTOL 悬停模式生效）
      altRef: 5,           // 定高参考高度（m，向上为正）
      intAlt: 0,           // 定高高度外环积分器（m·s）
      wf: 0, wt: 0,        // 实际转速（一阶滞后）
      intTh: 0, intPhi: 0, // SAS 积分器（俯仰/滚转）
      omega: vec3(),       // 机体角速度 [p q r] (rad/s)
      quat: quat(),        // 姿态四元数
      time: 0,
    },
    // ---- 飞行状态（完整 6-DOF） ----
    F: {
      pos: vec3(),         // 惯性系位置（z 向下）
      vel: vec3(),         // 机体系速度 [u v w]
      vWorld: vec3(),      // 惯性系速度
      euler: vec3(),       // φ θ ψ（显示/SAS）
    },
    // ---- 推力通道遥测 ----
    dyn: { Fx: 0, Fy: 0, Fz: 0, Mx: 0, My: 0, Mz: 0, Tf: 0, Tt: 0, Qf: 0, Qt: 0 },
    // ---- 气动遥测 ----
    aero: { V: 0, qbar: 0, alpha: 0, beta: 0, Mx: 0, My: 0, Mz: 0 },
    prevWf: 0, prevWt: 0,  // 上一步转速（求 ω̇）
  };
  resetFlightState(sim, P);
  return sim;
}

// 以配平状态初始化（物理俯仰 α0；显示约定 theta0=-α0，平飞航迹）
export function resetFlightState(sim, P) {
  const a0 = P.aTrim;
  const { S, F } = sim;
  F.vel.x = P.vTrim * Math.cos(a0); F.vel.y = 0; F.vel.z = P.vTrim * Math.sin(a0);
  F.vWorld.x = P.vTrim; F.vWorld.y = 0; F.vWorld.z = 0;
  F.pos.x = 0; F.pos.y = 0; F.pos.z = 0;
  S.quat.x = 0; S.quat.y = Math.sin(a0 / 2); S.quat.z = 0; S.quat.w = Math.cos(a0 / 2);
  S.omega.x = 0; S.omega.y = 0; S.omega.z = 0;
  S.intTh = 0; S.intPhi = 0;
  if (S.lockXY) {          // 水平约束下复位：清零水平速度
    F.vel.x = 0; F.vel.y = 0;
    F.vWorld.x = 0; F.vWorld.y = 0;
  }
}

// UI/演示使用的完整动态复位；保留 SAS、气动和水平约束开关。
// 本函数设置 S.vtolMode=false（resetVtolHoverState 对称地设置 true）——
// 演示/复位按钮从悬停模式出发时会自动退出悬停，保证模式标志与状态一致。
export function resetSimulationState(sim, P) {  const { S, dyn, aero } = sim;
  S.vtolMode = false;     // ★ 模式标志（与悬停复位对称）
  S.altHold = false;      // 定高仅悬停模式有意义，一并清零
  S.intAlt = 0;
  S.thr = P.thrTrim; S.dt = 0; S.df = 0; S.dw = 0;
  S.dtAct = P.dtTrim; S.dfAct = P.dfTrim; S.dwAct = 0;
  S.time = 0;
  S._prevSasMode = S.sasMode;
  const wTrim = P.thrTrim * P.wMax;
  S.wf = wTrim; S.wt = wTrim;
  sim.prevWf = wTrim; sim.prevWt = wTrim;
  Object.assign(dyn, { Fx: 0, Fy: 0, Fz: 0, Mx: 0, My: 0, Mz: 0, Tf: 0, Tt: 0, Qf: 0, Qt: 0 });
  Object.assign(aero, { V: 0, qbar: 0, alpha: 0, beta: 0, Mx: 0, My: 0, Mz: 0 });
  resetFlightState(sim, P);
}

// 轻量复位（UI「复位」按钮）：只复位飞行状态（位置/速度/姿态/角速度/积分器/
// 执行器基准），★ 保留模式与输入（vtolMode/altHold/useBtrue/sasMode/aero/
// lockXY/thr/dt/df/dw 滑块指令均不动）。按当前构型分支复位：
//   悬停 → Q_HOVER 机头朝天静止；巡航 → 配平平飞（含 lockXY 语义）。
// 电机转速跟随当前油门 S.thr（保留输入）；时间不归零。
export function resetPoseOnly(sim, P) {
  const { S, F, dyn, aero } = sim;
  S.intTh = 0; S.intPhi = 0; S.intAlt = 0;
  const w0 = S.thr * P.wMax;             // 转速跟随当前油门（保留输入）
  if (S.vtolMode) {
    // 悬停基准：机头朝天、静止；执行器回 0 摆角基准（无配平偏置）
    F.vel.x = 0; F.vel.y = 0; F.vel.z = 0;
    F.vWorld.x = 0; F.vWorld.y = 0; F.vWorld.z = 0;
    F.pos.x = 0; F.pos.y = 0; F.pos.z = 0;
    S.quat.x = Q_HOVER.x; S.quat.y = Q_HOVER.y; S.quat.z = Q_HOVER.z; S.quat.w = Q_HOVER.w;
    S.omega.x = 0; S.omega.y = 0; S.omega.z = 0;
    S.dtAct = 0; S.dfAct = 0; S.dwAct = 0;
  } else {
    // 巡航基准：配平平飞（resetFlightState 内含 lockXY 分支）
    resetFlightState(sim, P);
    S.dtAct = P.dtTrim; S.dfAct = P.dfTrim; S.dwAct = 0;
  }
  S.wf = w0; S.wt = w0;
  sim.prevWf = w0; sim.prevWt = w0;
  S._prevSasMode = S.sasMode;
  Object.assign(dyn, { Fx: 0, Fy: 0, Fz: 0, Mx: 0, My: 0, Mz: 0, Tf: 0, Tt: 0, Qf: 0, Qt: 0 });
  Object.assign(aero, { V: 0, qbar: 0, alpha: 0, beta: 0, Mx: 0, My: 0, Mz: 0 });
}

// VTOL 悬停模式复位：机头朝天（x_b 竖直），悬停配平油门，速度/角速度/积分清零。
// ★ 无翼构型：默认关闭气动力（S.aero=false），进入悬停即"关掉机翼"，可手动再开。
//   （aero 恢复由 UI 层负责：b-vtol 切回巡航时显式 S.aero=true；直接调
//    resetSimulationState 保持用户当前的 aero 开关状态，与固定翼复位语义一致）
// 悬停控制律基准为 0 摆角（不叠加巡航配平 dtTrim/dfTrim）。
// 本函数设置 S.vtolMode=true（resetSimulationState 对称地设置 false）——
// 任何入口调用复位都会使模式标志与状态一致。
export function resetVtolHoverState(sim, P) {
  const { S, F, dyn, aero } = sim;
  const thr = hoverThrottle(P);
  const w0 = thr * P.wMax;
  S.vtolMode = true;      // ★ 模式标志（与固定翼复位对称）
  S.thr = thr; S.dt = 0; S.df = 0; S.dw = 0;
  S.dtAct = 0; S.dfAct = 0; S.dwAct = 0;
  S.intTh = 0; S.intPhi = 0;
  S.wf = w0; S.wt = w0;
  sim.prevWf = w0; sim.prevWt = w0;
  S.omega.x = 0; S.omega.y = 0; S.omega.z = 0;
  S.quat.x = Q_HOVER.x; S.quat.y = Q_HOVER.y; S.quat.z = Q_HOVER.z; S.quat.w = Q_HOVER.w;
  S.lockXY = false;
  S.aero = false;             // ★ 无翼：悬停模式默认关闭气动力
  S.useBtrue = false;         // B_true 分配默认关（由 UI 开启）
  S.altHold = false;          // 定高默认关（由 UI 开启）
  S.altRef = 5;
  S.intAlt = 0;
  S.time = 0;
  F.vel.x = 0; F.vel.y = 0; F.vel.z = 0;
  F.vWorld.x = 0; F.vWorld.y = 0; F.vWorld.z = 0;
  F.pos.x = 0; F.pos.y = 0; F.pos.z = 0;
  Object.assign(dyn, { Fx: 0, Fy: 0, Fz: 0, Mx: 0, My: 0, Mz: 0, Tf: 0, Tt: 0, Qf: 0, Qt: 0 });
  Object.assign(aero, { V: 0, qbar: 0, alpha: 0, beta: 0, Mx: 0, My: 0, Mz: 0 });
}
