/**
 * @file    propulsion.mjs
 * @module  仿真 core/推进系统
 * @brief   电机一阶滞后、差速开方分配、反扭矩与推力矢量六维映射
 * @details 输入：油门 thr 与执行量（dtAct/dfAct/dwAct）、上一步转速；
 *          处理：差速分配求目标转速 → 一阶滞后更新实际转速 → T/Q 计算 → 六维力/力矩；
 *          输出：写入 sim.dyn 的 {Fx,Fy,Fz,Mx,My,Mz,Tf,Tt,Qf,Qt} 与摆角三角函数。
 *          不变量：六维映射只依赖当前摆角/转速，不含气动与陀螺项（后者在 dynamics）。
 * @theory  差速开方分配 ωf=ω0√(1+Δω)、ωt=ω0√(1−Δω) @ref CTL-001 §7.1；
 *          推力 T=kT·ω²、反扭矩 τ=kQ·ω²+Jp·ω̇ @ref THY-002 §6；
 *          六维映射（机体系 δf→Mz、δt→My、Δω→Mx）@ref MOD-001 §4；
 *          电机一阶滞后 τm 来源 @ref THY-002 §4
 * @frame   机体系 FRD（x 前 / y 右 / z 下）；摆座系到机体系仅绕固定轴旋转
 * @units   ω [rad/s]、T [N]、Q/τ [N·m]、力矩 [N·m]、力 [N]
 * @redline 差速公式 ωf=ω0√(1+Δω) 不可改；τm=0.28 s 不可改（见行为保持红线清单）
 * @note 执行器轴语义：体轴 δf→M_z(偏航)/δt→M_y(俯仰)/Δω→M_x(滚转)；
 *       悬停构型下同一摆分别主控滚转/俯仰，见 @ref AGENTS.md「执行器轴映射」
 * @see     control.mjs（SAS/悬停律输出 dtAct/dfAct/dwAct）、tests/propulsion.test.mjs
 */

// ── 差速开方分配（平方和不变）──────────────────────────────────────
// 公式：ωf = ω0·√(1+Δω)，ωt = ω0·√(1−Δω)，ω0 = thr·wMax
// 符号：Δω 为差速指令，正=前快尾慢；wMax 为最大转速
// 来源：@ref CTL-001 §7.1–7.2；@ref MOD-001 §3.2
// 实现：`wfT/wtT` 两行；sqrt 前 max(0,·) 兜底指令越界（|Δω|>1）
// 等价变换：平方后 ωf²+ωt²=2ω0² ⇒ T=kT·ωf² 对 Δω 严格线性（Tf=T0(1+Δω)），
//           这是"一阶不扰动总推力"的代数来源，不是小角度近似
// 数值证据：@test tests/propulsion.test.mjs「差速分配保持平方转速和不变」
// ─────────────────────────────────────────────────────────────────
// @formula ω̇≈(ω_cmd−ω)/τm；T=kT·ω²；Q=kQ·ω²+Jp·ω̇；
//          Fx=Tf·cf+Tt·ct、Fy=Tf·sf、Fz=−Tt·st；
//          Mx=−Qf·cf+Qt·ct、My=−b·Tt·st−Qf·sf、Mz=a·Tf·sf−Qt·st
// @ref THY-002 §3（反扭矩=电磁扭矩）、§4（一阶滞后）；@ref MOD-001 §2–§4
// @note ω̇ 用后向差分 ((ω−ω_prev)/dt)，dt 下限 1e-4 s 仅防除零
// @approx 一阶滞后用显式欧拉（1−e^(−dt/τm) 的一阶截断），dt=4 ms 时相对误差 ~7e-3
// @test tests/propulsion.test.mjs（T/Q 稳态、平方和、摆角符号、反扭对消、一阶滞后 63.2%）
/**
 * @brief 推进一步推进系统：更新转速/推力/反扭矩与机体系六维力/力矩
 * @param {Object} sim 仿真状态；@param {Object} P 参数表；@param {number} dt 子步长 [s]
 * @returns {{cf:number,sf:number,ct:number,st:number}} 摆角三角函数（供转子角动量）
 */
export function stepPropulsion(sim, P, dt) {
  const { S, dyn } = sim;
  const dtC = S.dtAct, dfC = S.dfAct, dwC = S.dwAct;

  // 差速分配: ω_f²+ω_t² ≈ 常数（钳位 wMax 后不再严格成立，见 CTL-001 §7.2 注）
  const w0 = S.thr * P.wMax;
  const wfT = Math.min(w0 * Math.sqrt(Math.max(0, 1 + dwC)), P.wMax);
  const wtT = Math.min(w0 * Math.sqrt(Math.max(0, 1 - dwC)), P.wMax);
  S.wf += (wfT - S.wf) * Math.min(dt / P.tauM, 1);
  S.wt += (wtT - S.wt) * Math.min(dt / P.tauM, 1);
  const dWf = (S.wf - sim.prevWf) / Math.max(dt, 1e-4);
  const dWt = (S.wt - sim.prevWt) / Math.max(dt, 1e-4);
  sim.prevWf = S.wf; sim.prevWt = S.wt;

  const Tf = P.kT * S.wf * S.wf, Tt = P.kT * S.wt * S.wt;
  const Qf = P.kQ * S.wf * S.wf + P.Jp * dWf;   // 传给机体的反扭矩 = 电磁扭矩（含转子惯性项）
  const Qt = P.kQ * S.wt * S.wt + P.Jp * dWt;
  const cf = Math.cos(dfC), sf = Math.sin(dfC);
  const ct = Math.cos(dtC), st = Math.sin(dtC);

  dyn.Tf = Tf; dyn.Tt = Tt; dyn.Qf = Qf; dyn.Qt = Qt;
  dyn.Fx = Tf * cf + Tt * ct;
  dyn.Fy = Tf * sf;
  dyn.Fz = -Tt * st;
  dyn.Mx = -Qf * cf + Qt * ct;                  // 滚转: 反扭差（零摆角时 Mx=Qt−Qf）
  dyn.My = -P.b * Tt * st - Qf * sf;            // 俯仰: 尾摆力臂 + 前摆反扭耦合
  dyn.Mz = P.a * Tf * sf - Qt * st;             // 偏航: 前摆力臂 + 尾摆反扭耦合
  return { cf, sf, ct, st };
}
