// ============================================================
//  SAS 增稳控制：角速率阻尼 + 姿态角比例/积分反馈
//  → 摆角/差速修正（积分与指令均限幅）
// ============================================================
import { clamp } from './math.mjs';

// 反馈极性按各通道控制效率符号整定
// （∂My/∂δ_t<0, ∂Mx/∂Δω<0 → 正号; ∂Mz/∂δ_f>0 → 负号）
export function applySas(sim, P, dt) {
  const { S, F } = sim;
  const theta = F.euler.y, phi = F.euler.x;
  let dtC = S.dt, dfC = S.df, dwC = S.dw;
  if (S.sas) {
    // 比例 + 积分（积分消除常值配平误差, 带限幅）
    S.intTh = clamp(S.intTh + theta * dt, -P.intThMax, P.intThMax);
    S.intPhi = clamp(S.intPhi + phi * dt, -P.intPhiMax, P.intPhiMax);
    dtC = clamp(dtC + P.sasQ * S.omega.y + P.sasTh * theta + P.sasI * S.intTh, -P.dMax, P.dMax);
    dfC = clamp(dfC - P.sasR * S.omega.z, -P.dMax, P.dMax);
    dwC = clamp(dwC + P.sasP * S.omega.x + P.sasPhi * phi + P.sasIPhi * S.intPhi, -P.dwMax, P.dwMax);
  }
  S.dtAct = dtC; S.dfAct = dfC; S.dwAct = dwC;
}
