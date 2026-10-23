/**
 * @file    telemetry.mjs
 * @module  仿真 core/遥测快照
 * @brief   把仿真状态整理成浏览器层只读数据视图（深拷贝分组）
 * @details 输入：sim 状态树；处理：按 forces/aero/rotors/actuators/commands/flight/flags
 *          分组浅拷贝；输出：纯数据快照，供 HUD/示波器读取。
 *          不变量：快照与 sim 内部对象不共享引用（改快照不会改状态）。
 * @theory  N/A（数据搬运，不含物理公式）；分组字段语义沿用 @ref MOD-002 §8 代码变量对照表
 * @frame   继承 sim 约定：位置/地速为 NED，速度/角速度为机体系 FRD
 * @units   角度 rad、角速度 rad/s、转速 rad/s、力 N、力矩 N·m、时间 s
 * @note 分组名是 HUD 的接口契约（tests/telemetry.test.mjs 锁定键集合），不要随意改名
 * @see     browser/hud.mjs（消费方）、tests/telemetry.test.mjs
 */

// @note 只做一层分组拷贝（嵌套对象各自展开），避免 HUD 写到状态树
// @note flags.sas 是 sasMode>0 的派生布尔，方便 HUD 一句话判断"自稳是否在管"
// @test tests/telemetry.test.mjs（键集合、拷贝隔离、flags 映射）
/**
 * @brief 生成一帧遥测快照（分组 + 拷贝）
 * @param {Object} sim 仿真状态
 * @returns {Object} {forces, aero, rotors, actuators, commands, flight, flags, time}
 */
export function getTelemetry(sim) {
  const { S, F, dyn, aero } = sim;
  return {
    forces: { ...dyn },
    aero: { ...aero },
    rotors: { wf: S.wf, wt: S.wt },
    actuators: { dtAct: S.dtAct, dfAct: S.dfAct, dwAct: S.dwAct },
    commands: { thr: S.thr, dt: S.dt, df: S.df, dw: S.dw },
    flight: {
      pos: { ...F.pos },
      vel: { ...F.vel },
      vWorld: { ...F.vWorld },
      euler: { ...F.euler },
      quat: { ...S.quat },
      omega: { ...S.omega },
    },
    flags: { sasMode: S.sasMode, sas: S.sasMode > 0, aero: S.aero, lockXY: S.lockXY },
    time: S.time,
  };
}
