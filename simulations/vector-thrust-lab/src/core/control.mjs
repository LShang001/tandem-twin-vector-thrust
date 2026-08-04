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
import { quatMultiply, quatInvert, quatNormalize } from './math.mjs';
import { Q_HOVER } from './state.mjs';

// 反馈极性按各通道控制效率符号整定
// （∂My/∂δ_t<0, ∂Mx/∂Δω<0 → 正号; ∂Mz/∂δ_f>0 → 负号）
export function applySas(sim, P, dt) {
  const { S, F } = sim;
  // VTOL 悬停构型：机头朝天（x_b 竖直），欧拉角在 θ≈90° 奇异，
  // 固定翼 SAS 的欧拉反馈失效 → 走四元数误差级联控制分支。
  if (S.vtolMode) { applyVtolHover(sim, P); return; }
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
//     dw 滑块 → 航向 ψ（绕 x_b，悬停时 x_b=世界竖直轴）
//     dt 滑块 → 俯仰倾斜 θ（绕 y_b）
//     df 滑块 → 侧倾 φ（绕 z_b）
//  机体系误差：qe = qCmd⁻¹ ⊗ q（误差旋转表达在机体系，小角度 qe.xyz ≈ 误差/2；
//     ⚠ 勿用 q ⊗ qCmd⁻¹ —— 那是 NED 系表达，悬停时 x/y/z 轴错位）
//  目标角速度：ωdes = −2·vtolAttKp·qe.xyz（符号：qe.y>0 → dtAct>0 → My<0 → 收敛）
//  角速度内环按通道效率符号（与 sasMode=3 一致）：
//     尾摆 δt（∂My/∂δt<0）: dtC = rateKq·(ω.y − ωdes.y)
//     前摆 δf（∂Mz/∂δf>0）: dfC = rateKr·(ωdes.z − ω.z)
//     差速 Δω（∂Mx/∂Δω<0）: dwC = rateKp·(ω.x − ωdes.x)
//  悬停基准为 0 摆角（不叠加巡航配平偏置 dtTrim/dfTrim）；
//  sasMode=0 时直通滑块指令（无自稳），可对比演示。
// ============================================================
function applyVtolHover(sim, P) {
  const { S } = sim;

  // 滑块 → 目标姿态小角度指令（先航向后倾斜，小角度下顺序无关）
  // 四元数构造：绕 x = (sin,0,0,cos)，绕 y = (0,sin,0,cos)，绕 z = (0,0,sin,cos)
  const h = S.dt * 0.5, f = S.df * 0.5, w = S.dw * 0.5;
  const qLocal = quatMultiply(
    quatMultiply(quat(Math.sin(w), 0, 0, Math.cos(w)),   // 航向 ψ：绕 x_b（悬停时=世界竖直轴）
                 quat(0, Math.sin(h), 0, Math.cos(h))),  // 俯仰 θ：绕 y_b
    quat(0, 0, Math.sin(f), Math.cos(f)));               // 侧倾 φ：绕 z_b
  const qCmd = quatNormalize(quatMultiply(Q_HOVER, qLocal));

  if (S.sasMode === 0) {
    // 直通：滑块 = 摆角/差速指令（悬停基准 0，无自稳）
    S.dtAct = clamp(S.dt, -P.dMax, P.dMax);
    S.dfAct = clamp(S.df, -P.dMax, P.dMax);
    S.dwAct = clamp(S.dw, -P.dwMax, P.dwMax);
    return;
  }

  // 机体系误差四元数：qe = qCmd⁻¹ ⊗ q（误差旋转表达在机体系），w<0 取反走最短路径
  const qe = quatNormalize(quatMultiply(quatInvert(qCmd), S.quat));
  const s = qe.w < 0 ? -1 : 1;
  const wdx = -2 * P.vtolAttKp * s * qe.x;
  const wdy = -2 * P.vtolAttKp * s * qe.y;
  const wdz = -2 * P.vtolAttKp * s * qe.z;

  S.dtAct = clamp(P.rateKq * (S.omega.y - wdy), -P.dMax, P.dMax);
  S.dfAct = clamp(P.rateKr * (wdz - S.omega.z), -P.dMax, P.dMax);
  S.dwAct = clamp(P.rateKp * (S.omega.x - wdx), -P.dwMax, P.dwMax);
}
