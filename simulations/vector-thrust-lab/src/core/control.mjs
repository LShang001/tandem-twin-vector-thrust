/**
 * @file    control.mjs
 * @module  仿真 core/控制律（SAS 增稳 + VTOL 悬停）
 * @brief   把滑块指令与姿态/角速度反馈合成为执行量（摆角、差速、油门）
 * @details 输入：S 中的指令与角速度、F.euler 显示角、参数表；
 *          处理：模式分支（关闭/全 SAS/仅角速率/角速度闭环）→ 反馈叠加 → 限幅；
 *          悬停模式另走误差四元数级联 + 可选定高/B_true 分配；
 *          输出：S.dtAct/dfAct/dwAct（以及定高时的 S.thr）。
 *          不变量：所有执行量出口都经 ±dMax/±dwMax 限幅；积分器出口经 ±intThMax/±intPhiMax。
 * @theory  SAS 三通道律 @ref CTL-001 §4.1；反馈极性整定 @ref CTL-001 §5；
 *          差速/摆角效率符号 @ref CTL-001 §3；悬停四元数级联 @ref CTL-001 §8.2；
 *          定高串级与 1/√cosγ 倾角补偿 @ref CTL-001 §8.4；
 *          B_true 增量分配 @ref CTL-001 §8.5
 * @frame   机体系 FRD 的力矩（Mx/My/Mz）与显示欧拉角（theta=−asin(R13) 约定）
 * @units   角度 rad、角速度 rad/s、增益单位见 parameters.mjs（sasQ [s]、sasTh [-]）
 * @redline SAS 限幅 ±25°(dMax)/±0.7(dwMax) 为仿真值；固件实机摆角 2026-08-07 已收紧
 *          ±15°，仿真未回写 —— 评审时按 @ref docs/registers/行为保持红线清单.md「SAS 摆角限幅」处理
 * @note 执行器轴语义（两套读法必须同时写）：体轴 δt→M_y(俯仰)、δf→M_z(偏航)、Δω→M_x(滚转)；
 *       悬停构型（机头朝天）下同一摆 = 下摆(俯仰)/上摆(滚转)/差速(偏航)，
 *       见 @ref AGENTS.md 踩坑记录「执行器轴映射（2026-08-07 轴置换）」
 * @see     propulsion.mjs（效率符号来源）、control-allocation.mjs、tests/control.test.mjs
 */
import { clamp, quat } from './math.mjs';
import { quatMultiply, quatInvert, quatNormalize, rotateVecByQuat } from './math.mjs';
import { Q_HOVER, hoverThrottle } from './state.mjs';
import { computeBTrue, inv3 } from './control-allocation.mjs';

// ── 固定翼 SAS（巡航构型）──────────────────────────────────────────
// 公式：δt_cmd = clamp(dtTrim + δt + sasQ·q − sasTh·Δθ − sasI·∫Δθ, ±dMax)
//       δf_cmd = clamp(dfTrim + δf − sasR·r, ±dMax)
//       Δω_cmd = clamp(Δω + sasP·p − sasPhi·φ − sasIPhi·∫φ, ±dwMax)
// 符号：Δθ = θ + aTrim（θ 为显示角，θ_ref = −aTrim）；q/p/r 为机体角速度
// 来源：@ref CTL-001 §4.1；极性推导 @ref CTL-001 §5
// 实现：`applySas` 的模式 1/2/3 三个分支；限幅在函数末尾统一执行
// 等价变换：显示约定下 theta_dot≈−q、phi_dot≈−p ⇒ 速率项取正号、姿态项取负号，
//           两类反馈符号必须分开整定（不能只看 ∂M/∂δ 的同号性）
// 数值证据：@test tests/control.test.mjs（三条极性用例 + 限幅 + 积分限幅）
// ─────────────────────────────────────────────────────────────────
// @formula 见上方五段式；模式 3 速率闭环：
//          dt=rateKq·(q−q_ref)、df=rateKr·(r_ref−r)、dw=rateKp·(p−p_ref)
// @ref CTL-001 §4.1（模式 1）、§8.2（模式 3 与悬停内环同构）
// @note 为什么模式切换要清积分：三个模式共用积分器，切回模式 1 时旧累积值会
//       直接变成配平偏置（历史曾出现切换瞬间掉高）
// @test tests/control.test.mjs（直通、极性、限幅、积分限幅、模式 2/3、切换清零）
/**
 * @brief SAS 主入口：按 S.sasMode 合成执行量（悬停模式转 applyVtolHover）
 * @param {Object} sim 仿真状态；@param {Object} P 参数表；@param {number} dt 控制周期 [s]
 * @returns {void}（写 S.dtAct/dfAct/dwAct，输出均已限幅）
 */
export function applySas(sim, P, dt) {
  const { S, F } = sim;
  // VTOL 悬停构型：机头朝天（x_b 竖直），欧拉角在 θ≈90° 奇异，
  // 固定翼 SAS 的欧拉反馈失效 → 走四元数误差级联控制分支。
  if (S.vtolMode) { applyVtolHover(sim, P, dt); return; }
  const theta = F.euler.y, phi = F.euler.x;
  const thetaError = theta + P.aTrim;
  let dtC = P.dtTrim + S.dt, dfC = P.dfTrim + S.df, dwC = S.dw;
  if (S._prevSasMode !== S.sasMode) {
    // 模式切换时重置积分器，防止旧累积值在切换回 mode 1 时产生瞬态冲击
    S.intTh = 0; S.intPhi = 0;
    S._prevSasMode = S.sasMode;
  }
  if (S.sasMode === 3) {
    // ---- 角速度闭环：滑块 = ω_ref，P 控制器追踪 ----
    // 反馈极性：效率为负的通道（俯仰/滚转）取 (ω − ω_ref)；
    //           效率为正的通道（偏航）取 (ω_ref − ω)。
    const qRef = S.dt, rRef = S.df, pRef = S.dw;
    dtC = P.dtTrim + P.rateKq * (S.omega.y - qRef);               // 俯仰 ∂My/∂δ<0
    dfC = clamp(P.dfTrim + P.rateKr * (rRef - S.omega.z), -P.dMax, P.dMax);   // 偏航 ∂Mz/∂δ>0
    dwC = clamp(P.rateKp * (S.omega.x - pRef), -P.dwMax, P.dwMax); // 滚转 ∂Mx/∂Δω<0
  } else if (S.sasMode >= 1) {
    // ---- 角速率阻尼（三通道共用，模式 1/2 均生效） ----
    dtC = dtC + P.sasQ * S.omega.y;                      // 俯仰阻尼
    dfC = dfC - P.sasR * S.omega.z;                      // 偏航阻尼
    dwC = dwC + P.sasP * S.omega.x;                      // 滚转阻尼

    if (S.sasMode === 1) {
      // ---- 全 SAS：附加姿态比例 + 积分（积分消除常值配平误差） ----
      S.intTh  = clamp(S.intTh  + thetaError * dt, -P.intThMax,  P.intThMax);
      S.intPhi = clamp(S.intPhi + phi   * dt, -P.intPhiMax, P.intPhiMax);
      dtC = dtC - P.sasTh * thetaError - P.sasI  * S.intTh;
      dwC = dwC - P.sasPhi * phi       - P.sasIPhi * S.intPhi;
    }
  }
  // 物理执行限幅对所有模式生效，尾摆限幅包含配平偏置。
  dtC = clamp(dtC, -P.dMax, P.dMax);
  dfC = clamp(dfC, -P.dMax, P.dMax);
  dwC = clamp(dwC, -P.dwMax, P.dwMax);
  S.dtAct = dtC; S.dfAct = dfC; S.dwAct = dwC;
}

// ── VTOL 悬停控制（误差四元数级联）────────────────────────────────
// 公式：qCmd = Q_HOVER ⊗ Rx(ψ_est) ⊗ Ry(θ_cmd) ⊗ Rz(φ_cmd)
//       qe = qCmd⁻¹⊗q；ωdes.yz = −2·vtolAttKp·qe.yz
//       δt = rateKq·(ω_y−ωdes.y)；δf = rateKr·(ωdes.z−ω_z)；Δω = rateKp·(ω_x−ψ̇_cmd)
// 符号：qe 为机体系误差四元数（w<0 取反走短弧）；ψ_est 由 y_b 水平投影提取
// 来源：@ref CTL-001 §8.2（控制律与符号表）、§8.3（三通道符号数值验证）
// 实现：`applyVtolHover`；定高段 `if (S.altHold)`；B_true 段 `if (S.useBtrue)`
// 等价变换：θ≈90° 时欧拉角奇异、固定翼欧拉反馈失效 ⇒ 必须改用四元数误差链；
//           内环仍复用 sasMode=3 的通道效率符号（∂My/∂δt<0、∂Mz/∂δf>0、∂Mx/∂Δω<0）
// 数值证据：@test tests/vtol.test.mjs 31 用例（收敛、航向 90°/180°、定高、B_true）
// ─────────────────────────────────────────────────────────────────
/**
 * @brief 悬停构型控制律：四元数姿态外环 + 角速度内环 + 可选定高/B_true 分配
 * @param {Object} sim 仿真状态（S.dt/df 为姿态角指令、S.dw 为航向角速度指令）
 * @param {Object} P 参数表（vtolAttKp/rateKp/rateKr/rateKp/dMax/dwMax/altKp 系列/btrueK/Ix/Iy/Iz）
 * @param {number} dt 控制周期 [s]（定高积分与 B_true 的 ω̇ 差分用）
 * @returns {void}
 * @formula 见上方五段式；定高：vZref=clamp(altKpH·h_err+altKpI·∫h_err, ±altVZMax)，
 *          thr=thrHover/√cosγ + altKpV·(vZref−vZ)
 * @ref CTL-001 §8.2/§8.4/§8.5
 * @pre S.vtolMode=true（由 applySas 分派）；dt>0
 * @post dtAct/dfAct 限幅 ±dMax，dwAct 限幅 ±dwMax（限幅只在有分支的出口生效）
 * @note qCmd 必须跟随当前航向：航向由角速度指令自由旋转（x 通道无姿态回中），
 *       若 qCmd 固定，qe 的 y/z 分量被 cosψ 调制 → ψ=90° 倾斜失效、180° 反向发散
 * @note 悬停基准是 0 摆角（不叠加 dtTrim/dfTrim）；sasMode=0 时 dt/df 直通、dw 仍为
 *       航向角速度追踪（rate 语义不因自稳关闭而消失）
 * @note 高度用真值（本仿真无传感器模型），定高只接管油门、不触碰摆角
 * @test tests/vtol.test.mjs（初始化/符号/闭环/定高/B_true 五个分组）
 */
function applyVtolHover(sim, P, dt) {
  const { S, F } = sim;

  // 滑块 → 目标姿态小角度指令（仅 y/z：俯仰/侧倾；航向为角速度指令不入指令角）
  // 四元数构造：绕 x = (sin,0,0,cos)，绕 y = (0,sin,0,cos)，绕 z = (0,0,sin,cos)
  const h = S.dt * 0.5, f = S.df * 0.5;
  const qLocal = quatMultiply(
    quat(0, Math.sin(h), 0, Math.cos(h)),   // 俯仰 θ：绕 y_b
    quat(0, 0, Math.sin(f), Math.cos(f)));  // 侧倾 φ：绕 z_b
  // ★ qCmd 必须跟随当前航向：航向由角速度指令自由旋转（x 通道无姿态回中），
  // 若 qCmd 固定，qe = qCmd⁻¹⊗q 的 y/z 分量表达在 qCmd 系，与机体系绕 x 错位 ψ
  // → qe.y = cosψ·sin(ε/2)，ψ=90° 时倾斜回中失效、180° 时反向发散（复现于 2026-08-05）。
  // 提取当前航向（机体 y 轴的水平投影方向角；悬停构型 y_b 近似水平，投影角
  // 对 ψ∈(−π,π] 精确成立，θ/φ 倾斜引入 O(θ·φ) 二阶误差）：
  // 绕机体 x 转 +ψ → atan2(yb.y, yb.x) = π/2 − ψ（与 vtol.test.mjs 航向用例一致）
  const yb = rotateVecByQuat({ x: 0, y: 1, z: 0 }, S.quat);
  const psiEst = Math.PI / 2 - Math.atan2(yb.y, yb.x);
  const qCmd = quatNormalize(quatMultiply(
    quatMultiply(Q_HOVER, quat(Math.sin(psiEst / 2), 0, 0, Math.cos(psiEst / 2))),
    qLocal));

  // ---------- 高度保持（S.altHold=true，仅悬停模式） ----------
  // 独立于姿态自稳（sasMode=0 直通时同样生效）：只接管油门，不触碰摆角。
  // 串级：高度外环（P+I）→ 目标垂直速度 → 油门内环（P）
  // 公式：h = −pos.z（NED z 向下，高度向上为正）、vZ = −vWorld.z（向上为正）
  // 公式：vZref = clamp(altKpH·h_err + altKpI·∫h_err, ±altVZMax)
  // 公式：thr = thrHover/√cosγ + altKpV·(vZref − vZ)
  // 符号：cosγ = x̂_b·(−ẑ_NED) = −R31 为机头轴竖直分量；T∝thr² ⇒ 补偿因子 1/√cosγ
  // 来源：@ref CTL-001 §8.4（串级结构、倾角补偿与积分限幅）
  if (S.altHold) {
    const h = -F.pos.z;
    const vZ = -F.vWorld.z;
    const hErr = S.altRef - h;
    S.intAlt = clamp(S.intAlt + hErr * dt, -1.5, 1.5);   // 积分限幅（派生，±1.5 m·s）
    const vZref = clamp(P.altKpH * hErr + P.altKpI * S.intAlt, -P.altVZMax, P.altVZMax);
    const cosG = Math.max(-(2 * (S.quat.x * S.quat.z - S.quat.y * S.quat.w)), 0.5); // −R31
    const thrBase = hoverThrottle(P) / Math.sqrt(cosG);   // T∝thr² → 1/√cosγ
    S.thr = clamp(thrBase + P.altKpV * (vZref - vZ), 0, 1);
  }

  if (S.sasMode === 0) {
    // 直通：dt/df 滑块 = 摆角（悬停基准 0，无自稳）；dw 仍为航向角速度指令
    S.dtAct = clamp(S.dt, -P.dMax, P.dMax);
    S.dfAct = clamp(S.df, -P.dMax, P.dMax);
  } else {
    // 机体系误差四元数：qe = qCmd⁻¹ ⊗ q（误差旋转表达在机体系），w<0 取反走最短路径
    const qe = quatNormalize(quatMultiply(quatInvert(qCmd), S.quat));
    const s = qe.w < 0 ? -1 : 1;
    // y/z 通道姿态回中（qe.x 不用于控制：航向为纯角速度指令）
    const wdy = -2 * P.vtolAttKp * s * qe.y;
    const wdz = -2 * P.vtolAttKp * s * qe.z;

    if (S.useBtrue) {
      // ---- B_true 在线控制分配（S.useBtrue=true）----
      // 内环：角速度误差 → 目标力矩（每通道独立对角）
      // 公式：M_des = diag(Ix,Iy,Iz)·btrueK·(ωdes − ω)
      // 分配层：Δu = B⁻¹·(M_des − M_cur)，u += Δu（限幅）
      // 符号：u 列序 [Δdw, Δδt, Δδf]；M_cur 为当前执行器位置的推进力矩（模型预测）
      // 来源：@ref CTL-001 §8.5；B 的解析表达式见 control-allocation.mjs
      // ★ 非 INDI：无角加速度反演（不需要 ω̇ 估计）；M_cur 用当前状态重算
      //   （sim.dyn 是上一子步滞后值——applySas 先于 stepPropulsion 执行）
      const kX = P.Ix * P.btrueK, kY = P.Iy * P.btrueK, kZ = P.Iz * P.btrueK;
      const mDes = {
        x: kX * (S.dw - S.omega.x),
        y: kY * (wdy - S.omega.y),
        z: kZ * (wdz - S.omega.z),
      };
      // 当前执行器力矩（当前摆角 + 当前转速；瞬态反扭用 prevWf 差分，同 propulsion.mjs）
      const cf = Math.cos(S.dfAct), sf = Math.sin(S.dfAct);
      const ct = Math.cos(S.dtAct), st = Math.sin(S.dtAct);
      const dWf = (S.wf - sim.prevWf) / Math.max(dt, 1e-4);
      const dWt = (S.wt - sim.prevWt) / Math.max(dt, 1e-4);
      const Tf = P.kT * S.wf * S.wf, Tt = P.kT * S.wt * S.wt;
      const Qf = P.kQ * S.wf * S.wf + P.Jp * dWf;
      const Qt = P.kQ * S.wt * S.wt + P.Jp * dWt;
      const mCur = {
        x: -Qf * cf + Qt * ct,
        y: -P.b * Tt * st - Qf * sf,
        z: P.a * Tf * sf - Qt * st,
      };
      const dM = { x: mDes.x - mCur.x, y: mDes.y - mCur.y, z: mDes.z - mCur.z };
      const Binv = inv3(computeBTrue(S.thr * P.wMax, S.dfAct, S.dtAct, S.dwAct, P));
      if (Binv) {
        const duX = Binv[0][0] * dM.x + Binv[0][1] * dM.y + Binv[0][2] * dM.z;
        const duY = Binv[1][0] * dM.x + Binv[1][1] * dM.y + Binv[1][2] * dM.z;
        const duZ = Binv[2][0] * dM.x + Binv[2][1] * dM.y + Binv[2][2] * dM.z;
        S.dwAct = clamp(S.dwAct + duX, -P.dwMax, P.dwMax);
        S.dtAct = clamp(S.dtAct + duY, -P.dMax, P.dMax);
        S.dfAct = clamp(S.dfAct + duZ, -P.dMax, P.dMax);
      } else {
        // 奇异（低油门）回退：对角映射
        S.dtAct = clamp(P.rateKq * (S.omega.y - wdy), -P.dMax, P.dMax);
        S.dfAct = clamp(P.rateKr * (wdz - S.omega.z), -P.dMax, P.dMax);
        S.dwAct = clamp(P.rateKp * (S.omega.x - S.dw), -P.dwMax, P.dwMax);
      }
      return;
    }

    S.dtAct = clamp(P.rateKq * (S.omega.y - wdy), -P.dMax, P.dMax);
    S.dfAct = clamp(P.rateKr * (wdz - S.omega.z), -P.dMax, P.dMax);
  }
  // 航向通道：纯角速度追踪（∂Mx/∂Δω<0 取 (ω.x − ψ̇_cmd)），滑块=目标角速度
  S.dwAct = clamp(P.rateKp * (S.omega.x - S.dw), -P.dwMax, P.dwMax);
}
