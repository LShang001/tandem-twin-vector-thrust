// ============================================================
//  SAS 增稳控制：角速率阻尼 + 姿态角比例/积分反馈
//  → 摆角/差速修正（积分与指令均限幅）
//
//  模式（S.sasMode）：
//    0 = 关闭（直通）
//    1 = 全 SAS（角速率阻尼 + 姿态比例 + 积分）
//    2 = 仅角速率阻尼（无姿态/积分）
//    3 = 角速度闭环（Rate Command：滑块 = 目标角速度）
// ============================================================
import { clamp, quat } from './math.mjs';
import { quatMultiply, quatInvert, quatNormalize, rotateVecByQuat } from './math.mjs';
import { Q_HOVER, hoverThrottle } from './state.mjs';
import { computeBTrue, inv3 } from './control-allocation.mjs';

// 反馈极性按各通道控制效率符号整定
// （∂My/∂δ_t<0, ∂Mx/∂Δω<0 → 正号; ∂Mz/∂δ_f>0 → 负号）
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

// ============================================================
//  VTOL 悬停控制：误差四元数姿态外环 + 角速度 P 内环（级联）
//  —— 与固件 flight_control.cpp 悬停链同构（q_hover ⊗ Rx(-Heading) 语义），
//     符号经数值验证（见 tests/vtol.test.mjs "控制符号"用例）。
//
//  目标姿态：qCmd = Q_HOVER ⊗ 滑块小角度指令（四轴式语义）
//     dt 滑块 → 俯仰倾斜 θ（绕 y_b）目标角
//     df 滑块 → 侧倾 φ（绕 z_b）目标角
//     dw 滑块 → ★ 航向角速度指令 ψ̇（绕 x_b，rad/s；rate 模式，松手回中停转，
//        不参与 qCmd —— 航向通道无姿态回中，纯角速度追踪 + 速率阻尼）
//  机体系误差：qe = qCmd⁻¹ ⊗ q（误差旋转表达在机体系，小角度 qe.xyz ≈ 误差/2；
//     ⚠ 勿用 q ⊗ qCmd⁻¹ —— 那是 NED 系表达，悬停时 x/y/z 轴错位）
//  目标角速度（y/z 通道姿态回中）：ωdes.yz = −2·vtolAttKp·qe.yz
//  角速度内环按通道效率符号（与 sasMode=3 一致）：
//     尾摆 δt（∂My/∂δt<0）: dtC = rateKq·(ω.y − ωdes.y)
//     前摆 δf（∂Mz/∂δf>0）: dfC = rateKr·(ωdes.z − ω.z)
//     差速 Δω（∂Mx/∂Δω<0）: dwC = rateKp·(ω.x − S.dw)（航向：滑块=目标角速度）
//  悬停基准为 0 摆角（不叠加巡航配平偏置 dtTrim/dfTrim）；
//  sasMode=0 时 dt/df 直通滑块摆角（无自稳），dw 仍为角速度指令。
// ============================================================
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
  //   h    = −pos.z       （NED z 向下，高度向上为正）
  //   vZ   = −vWorld.z    （垂直速度，向上为正）
  //   vZref = clamp(altKpH·h_err + altKpI·∫h_err, ±altVZMax)
  //   thr  = thrHover/√cosγ + altKpV·(vZref − vZ)   （cosγ = x̂_b·(−ẑ_NED) 倾角补偿：
  //          倾斜损失竖直推力分量，且 T∝thr² → 补偿因子 1/√cosγ 而非 1/cosγ）
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
      // 内环：角速度误差 → 目标力矩（每通道独立对角，M = I·K·(ωdes − ω)）
      //   ωdes.x = 航向角速度指令（rate 模式）、ωdes.y/z = 姿态回中（qe）
      // 分配层：Δu = B⁻¹·(M_des − M_cur)，u += Δu（限幅）
      //   M_cur = 当前执行器位置的推进力矩（模型预测，含 Jp 瞬态反扭；
      //           不含气动/gyro——分配层只负责执行器，外扰由内环闭环抑制）
      //   B = ∂[Mx,My,Mz]/∂[Δdw,Δδt,Δδf] 当前工作点 Jacobian（列序 [Δdw,Δδt,Δδf]）
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
