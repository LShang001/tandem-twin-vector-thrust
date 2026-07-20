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
import { clamp } from './math.mjs';

// 反馈极性按各通道控制效率符号整定
// （∂My/∂δ_t<0, ∂Mx/∂Δω<0 → 正号; ∂Mz/∂δ_f>0 → 负号）
export function applySas(sim, P, dt) {
  const { S, F } = sim;
  const theta = F.euler.y, phi = F.euler.x;
  let dtC = S.dt, dfC = S.df, dwC = S.dw;
  if (S.sasMode === 3) {
    // ---- 角速度闭环：滑块 = ω_ref，P 控制器追踪 ----
    // 反馈极性：效率为负的通道（俯仰/滚转）取 (ω − ω_ref)；
    //           效率为正的通道（偏航）取 (ω_ref − ω)。
    const qRef = S.dt, rRef = S.df, pRef = S.dw;
    dtC = clamp(P.rateKq * (S.omega.y - qRef), -P.dMax, P.dMax);   // 俯仰 ∂My/∂δ<0
    dfC = clamp(P.rateKr * (rRef - S.omega.z), -P.dMax, P.dMax);   // 偏航 ∂Mz/∂δ>0
    dwC = clamp(P.rateKp * (S.omega.x - pRef), -P.dwMax, P.dwMax); // 滚转 ∂Mx/∂Δω<0
  } else if (S.sasMode >= 1) {
    // ---- 角速率阻尼（三通道共用，模式 1/2 均生效） ----
    dtC = dtC + P.sasQ * S.omega.y;                      // 俯仰阻尼
    dfC = dfC - P.sasR * S.omega.z;                      // 偏航阻尼
    dwC = dwC + P.sasP * S.omega.x;                      // 滚转阻尼

    if (S.sasMode === 1) {
      // ---- 全 SAS：附加姿态比例 + 积分（积分消除常值配平误差） ----
      S.intTh  = clamp(S.intTh  + theta * dt, -P.intThMax,  P.intThMax);
      S.intPhi = clamp(S.intPhi + phi   * dt, -P.intPhiMax, P.intPhiMax);
      dtC = dtC + P.sasTh * theta  + P.sasI  * S.intTh;
      dwC = dwC + P.sasPhi * phi    + P.sasIPhi * S.intPhi;
    }
    // 执行限幅
    dtC = clamp(dtC, -P.dMax, P.dMax);
    dfC = clamp(dfC, -P.dMax, P.dMax);
    dwC = clamp(dwC, -P.dwMax, P.dwMax);
  }
  S.dtAct = dtC; S.dfAct = dfC; S.dwAct = dwC;
}
