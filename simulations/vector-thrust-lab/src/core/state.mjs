/**
 * @file    state.mjs
 * @module  仿真 core/状态与复位
 * @brief   定义仿真状态容器（S/F/dyn/aero）与三种复位入口
 * @details 输入：参数表 P 与用户模式开关；处理：构造状态对象、按模式（巡航/悬停）
 *          初始化姿态/速度/转速/积分器；输出：可被 dynamics 直接推进的状态树。
 *          不变量：S 为控制与执行状态、F 为飞行状态、dyn/aero 为只读遥测缓存；
 *          复位入口的语义差异是 UI 契约（见各函数 @note），改动会改变演示行为。
 * @theory  悬停姿态 Q_HOVER = 绕 NED y 转 +90° @ref CTL-001 §8.1；
 *          悬停配平油门 2·kT·(thr·wMax)² = m·g @ref CTL-001 §8.1；
 *          巡航配平初始条件 @ref MOD-003 §2
 * @frame   NED（惯性系，z 向下）/ FRD（机体系，x 前 y 右 z 下）
 * @units   SI：位置 m、速度 m/s、角度 rad、角速度 rad/s、转速 rad/s
 * @redline 地面基准 groundZ=6.2 m 与空速下限 vMin=0.5 m/s 的初值语义见 behavior 红线
 * @see     dynamics.mjs、control.mjs、tests/state.test.mjs
 */
import { vec3, quat } from './math.mjs';

/**
 * @brief 悬停初始姿态：绕 NED y 轴转 +90°（机头朝天，x_b → NED −z）
 * @formula Q_HOVER = (x,y,z,w) = (0, √2/2, 0, √2/2)
 * @ref CTL-001 §8.1
 * @note 验证：rotateVecByQuat((1,0,0), Q_HOVER) = (0,0,−1)；显示约定下 theta = −90°
 *       （欧拉角在此奇异，故悬停控制必须走四元数误差链，不得用欧拉反馈）
 * @test tests/vtol.test.mjs「Q_HOVER：机头朝天」
 */
export const Q_HOVER = quat(0, Math.SQRT1_2, 0, Math.SQRT1_2);

// @formula thr = √(m·g/(2·kT·wMax²))，由 2·kT·(thr·wMax)² = m·g 解出 @ref CTL-001 §8.1
// @note 两发均分重力；T ∝ ω² 故带平方根——定高的 1/√cosγ 补偿同源
// @test tests/vtol.test.mjs「悬停配平油门：双发推力恰好平衡重力」
/**
 * @brief 悬停配平油门（物理派生量，不是独立参数）
 * @param {Object} P 参数表（需 m/g/kT/wMax）
 * @returns {number} 悬停油门 [-]（约 0.499）
 */
export function hoverThrottle(P) {
  return Math.sqrt(P.m * P.g / (2 * P.kT * P.wMax * P.wMax));
}

// @ref MOD-001 §5（六自由度状态量）、MOD-002 §8（代码变量对照）
// @note 所有字段一次给全并带默认值，避免 undefined 参与积分；dyn/aero 是"上一子步缓存"
// @note 轴语义（两套读法必须同时写）：体轴 δt→M_y(俯仰)、δf→M_z(偏航)、Δω→M_x(滚转)；
//       悬停构型（机头朝天）下同一执行器 = 下摆(俯仰)/上摆(滚转)/差速(偏航)
//       ——见 @ref AGENTS.md 踩坑记录「执行器轴映射（2026-08-07 轴置换）」
/**
 * @brief 构造一份完整的仿真状态（巡航配平初值）
 * @param {Object} P 参数表
 * @returns {Object} { S 控制/执行状态, F 飞行状态, dyn 推力遥测, aero 气动遥测 }
 * @see resetFlightState、resetSimulationState、resetVtolHoverState
 */
export function createSimulationState(P) {
  const sim = {
    // ---- 运行状态 ----
    S: {
      thr: P.thrTrim,      // 总油门 0..1（配平值来自四方程配平，见 MOD-003 §2.2）
      df: 0,               // 前摆角指令（体轴 M_z=偏航；悬停构型=上摆→滚转）
      dt: 0,               // 尾摆增量指令（体轴 M_y=俯仰；悬停构型=下摆→俯仰）
      dw: 0,               // 差速指令 -1..1（体轴 M_x=滚转；悬停构型→偏航）
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

// @formula v_b = (vTrim·cosα0, 0, vTrim·sinα0)，q = Ry(α0)，显示角 theta0 = −α0
// @ref MOD-003 §2.2（配平工作点）、MOD-002 §4（显示角符号）
// @note α0 = aTrim 是"物理迎角"，飞控显示角为 −α0；两者同时出现时以显示角注释为准
// @note lockXY=true 时清零水平速度（水平运动学约束，仅 UI 演示用，不是悬停模型）
/**
 * @brief 以配平状态初始化飞行状态（姿态、机体系速度、积分器）
 * @param {Object} sim 仿真状态（就地修改）；@param {Object} P 参数表
 * @returns {void}
 */
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

// @note 与 resetVtolHoverState 对称：本函数令 vtolMode=false，悬停复位令其为 true，
//       保证"任何复位入口后模式标志与状态一致"——演示/复位按钮据此自动退出悬停
// @note 保留 sasMode/aero/lockXY（用户模式选择），只清动力学状态
// @test tests/state.test.mjs「完整复位恢复可重复的配平动态状态并保留模式开关」
/**
 * @brief 完整动态复位到巡航配平（时间/转速/遥测全清，保留模式开关）
 * @param {Object} sim 仿真状态（就地修改）；@param {Object} P 参数表
 * @returns {void}
 */
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

// @note 保留 vtolMode/altHold/useBtrue/sasMode/aero/lockXY 与 thr/dt/df/dw 滑块指令；
//       时间不归零；电机转速跟随当前油门（用户"复位位置但不丢输入"的契约）
// @note 按当前构型分支：悬停 → Q_HOVER 静止、摆角回 0；巡航 → 配平平飞（含 lockXY）
// @test tests/state.test.mjs「轻量复位（巡航）」「轻量复位（悬停）」
/**
 * @brief 轻量复位（UI「复位」按钮）：只复位飞行状态，保留全部模式与输入
 * @param {Object} sim 仿真状态（就地修改）；@param {Object} P 参数表
 * @returns {void}
 */
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

// @note 与 resetSimulationState 对称：令 vtolMode=true；摆角基准为 0（不叠加巡航
//       dtTrim/dfTrim），否则进入悬停瞬间会带上 18.7° 尾摆配平偏置
// @note 默认关气动=悬停"拆掉机翼"；恢复 aero 由 UI 层负责（切回巡航时显式置 true）
// @test tests/vtol.test.mjs「resetVtolHoverState：无翼默认…」
/**
 * @brief 进入 VTOL 悬停构型：机头朝天、悬停配平油门、无翼（aero=false）
 * @param {Object} sim 仿真状态（就地修改）；@param {Object} P 参数表
 * @returns {void}
 */
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
