// ============================================================
//  仿真状态：控制/执行状态 S、飞行状态 F、遥测 dyn/aero
// ============================================================
import { vec3, quat } from './math.mjs';

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
export function resetSimulationState(sim, P) {
  const { S, dyn, aero } = sim;
  S.thr = P.thrTrim; S.dt = 0; S.df = 0; S.dw = 0;
  S.dtAct = P.dtTrim; S.dfAct = 0; S.dwAct = 0;
  S.time = 0;
  S._prevSasMode = S.sasMode;
  const wTrim = P.thrTrim * P.wMax;
  S.wf = wTrim; S.wt = wTrim;
  sim.prevWf = wTrim; sim.prevWt = wTrim;
  Object.assign(dyn, { Fx: 0, Fy: 0, Fz: 0, Mx: 0, My: 0, Mz: 0, Tf: 0, Tt: 0, Qf: 0, Qt: 0 });
  Object.assign(aero, { V: 0, qbar: 0, alpha: 0, beta: 0, Mx: 0, My: 0, Mz: 0 });
  resetFlightState(sim, P);
}
